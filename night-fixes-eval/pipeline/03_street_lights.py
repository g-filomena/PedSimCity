"""Step 3: compute continuous street illumination.

Inputs (searched in inputData/<City>/ then the resources folder):
- <City>_streetlights_with_radius.gpkg (step-2 output)
- <City>_edges.gpkg
- <City>_buildings.gpkg

Outputs:
- <City>_edges_illuminated_continuous.gpkg  -> src/main/resources/<City>/ (read by the sim)
- <City>_nodes_2m_densified_illuminated.gpkg -> inputData/<City>/ (intermediate for step 4)

`--city` is the city name (folder under inputData/ and resources/, and file prefix).
"""

from __future__ import annotations

import argparse
from pathlib import Path

import geopandas as gpd
import numpy as np
import pandas as pd
from scipy.spatial import cKDTree
from shapely.geometry import LineString

import lighting
import paths


SAMPLE_SPACING_M = 2.0


def remove_existing(path: Path) -> None:
    if path.exists():
        path.unlink()


def load_inputs(city: str):
    # Require the step-2 output (*_with_radius.gpkg): it carries the per-lamp physics
    # (downward_intensity_cd) this step needs. The raw inventory is intentionally NOT accepted so
    # that step 3 can never silently fabricate uniform lamp intensity — run step 2 first.
    punti_path = paths.require_input(
        city,
        "streetlights_with_radius.gpkg",
        "lamp inventory with radius (run step 2 first)",
    )
    edges_path = paths.require_input(city, "edges.gpkg", "edges layer")
    buildings_path = paths.require_input(city, "buildings.gpkg", "buildings layer")

    print("Loading datasets...")
    print(f"  city: {city}")
    print(f"  lamps    : {punti_path}")
    print(f"  edges    : {edges_path}")
    print(f"  buildings: {buildings_path}")

    punti = gpd.read_file(punti_path)
    edges = gpd.read_file(edges_path)
    buildings = gpd.read_file(buildings_path)

    if punti.empty:
        raise ValueError(f"Lamp layer is empty: {punti_path}")
    if edges.empty:
        raise ValueError(f"Edges layer is empty: {edges_path}")

    # altezza_palo_m is produced by step 2's assign_heights(), which guarantees every row gets a
    # value (measured, same-street median, support-type assumption, or the global pole median --
    # see height_source). Its absence or nulls mean step 3 was pointed at a non-step-2 file or one
    # from before that guarantee existed: fail loudly rather than silently fabricating a uniform
    # 9.0 m for every affected lamp (register finding A8 -- this used to guard inconsistently with
    # the identical situation for downward_intensity_cd immediately below, which already failed
    # loudly).
    if "altezza_palo_m" not in punti.columns:
        raise KeyError(
            f"{punti_path} has no 'altezza_palo_m' column. Run step 2 "
            "(02_street_lights_torino.py or 02_street_lights_generic.py) first."
        )
    if punti["altezza_palo_m"].isnull().any():
        n_missing = int(punti["altezza_palo_m"].isnull().sum())
        raise ValueError(
            f"{punti_path} has {n_missing} lamp(s) with a missing 'altezza_palo_m'. Step 2's "
            "assign_heights() is supposed to guarantee every row gets a value -- re-run step 2, "
            "or check height_source for how this file's heights were actually assigned."
        )

    # downward_intensity_cd is produced by step 2. Its absence means step 3 was pointed at a
    # non-step-2 file: fail loudly rather than fabricating uniform lamp intensity.
    if "downward_intensity_cd" not in punti.columns:
        raise KeyError(
            f"{punti_path} has no 'downward_intensity_cd' column. Run step 2 "
            "(02_street_lights.py) to compute the per-lamp physics before running step 3."
        )

    if punti["downward_intensity_cd"].isnull().any():
        # Impute the occasional missing lamp value within an otherwise valid dataset (by technology
        # group, then global mean). If every value is missing there is nothing to impute from, so
        # fail rather than invent one.
        if "tecnologia" in punti.columns:
            punti["downward_intensity_cd"] = punti.groupby("tecnologia")["downward_intensity_cd"].transform(
                lambda x: x.fillna(x.mean() if not x.dropna().empty else np.nan)
            )
        mean_intensity = punti["downward_intensity_cd"].mean()
        if np.isnan(mean_intensity):
            raise ValueError(
                f"{punti_path} has no usable 'downward_intensity_cd' values (all missing)."
            )
        punti["downward_intensity_cd"] = punti["downward_intensity_cd"].fillna(mean_intensity)

    target_crs = punti.crs or edges.crs or buildings.crs
    if target_crs is None:
        raise ValueError("No CRS found in lamp/edge/building layers.")

    if punti.crs is None:
        punti = punti.set_crs(target_crs)
    if edges.crs is None:
        edges = edges.set_crs(target_crs)
    elif edges.crs != target_crs:
        edges = edges.to_crs(target_crs)
    if buildings.crs is None:
        buildings = buildings.set_crs(target_crs)
    elif buildings.crs != target_crs:
        buildings = buildings.to_crs(target_crs)

    return punti, edges, buildings


def densify_edges(edges: gpd.GeoDataFrame, spacing: float) -> gpd.GeoDataFrame:
    # Stable per-edge key for the step-4 join: the edge's own edgeID when available, else the
    # positional index. parent_edge_idx (positional) is kept for this file's own aggregation.
    has_edge_id = "edgeID" in edges.columns
    rows = []
    counter = 0
    for idx, row in edges.iterrows():
        geom = row.geometry
        if geom is None or geom.is_empty:
            continue
        edge_id = row["edgeID"] if has_edge_id else idx
        length = geom.length
        distances = np.arange(0, length, spacing)
        if len(distances) == 0 or distances[-1] < length:
            distances = np.append(distances, length)
        for dist in distances:
            pt = geom.interpolate(float(dist))
            rows.append(
                {
                    "node_id": f"node_2m_{counter}",
                    "parent_edge_idx": idx,
                    "parent_edge_id": edge_id,
                    "dist_along_edge": float(dist),
                    "geometry": pt,
                    "x": pt.x,
                    "y": pt.y,
                }
            )
            counter += 1
    return gpd.GeoDataFrame(rows, crs=edges.crs)


def lamp_own_building(punti: gpd.GeoDataFrame, buildings: gpd.GeoDataFrame) -> np.ndarray:
    """Building index each lamp sits inside, or -1 for a lamp not inside any building.

    A lamp inside a building's own footprint (porticoes, facade mounts digitised at the wall) had
    every sight line from it blocked by that same building -- the lamp was permanently dark
    regardless of where it actually shines from (register finding A1). This excludes exactly that
    one building from the occlusion test for sight lines originating at this lamp; every other
    building still blocks normally.
    """
    owner = np.full(len(punti), -1, dtype="int64")
    if buildings.empty:
        return owner
    joined = gpd.sjoin(
        punti[["geometry"]].reset_index(drop=True),
        buildings[["geometry"]].reset_index(drop=True),
        predicate="within",
        how="left",
    )
    # A lamp within more than one building (overlapping footprints in the source data, which
    # shouldn't happen but isn't validated here) excludes only the first match, not both -- a
    # simplification for a case that shouldn't arise in a well-formed buildings layer.
    for lamp_idx, group in joined.groupby(joined.index):
        b_idx = group["index_right"].iloc[0]
        if pd.notna(b_idx):
            owner[lamp_idx] = int(b_idx)
    return owner


def compute_lux(points: gpd.GeoDataFrame, punti: gpd.GeoDataFrame, buildings: gpd.GeoDataFrame) -> np.ndarray:
    """Line-of-sight illuminance per sample point, occluded by buildings only.

    Register finding A4 has two halves: occlusion ignoring lamp/building height (fixed below --
    the sight line is now checked in 3D against each building's real height, not just its 2D
    footprint) and street trees being entirely absent from occlusion (NOT fixed here). Buildings
    carry real height data this inventory can use; trees would need a canopy layer -- location,
    extent, and season -- that does not exist for Torino. Left as a stated limitation rather than
    approximated, per the register's own recommendation, since a wrong canopy guess would be
    worse than no canopy model at all.
    """
    if points.empty:
        return np.array([], dtype="float64")

    lamp_coords = np.column_stack((punti.geometry.x, punti.geometry.y))
    pt_coords = np.column_stack((points["x"], points["y"]))
    intensity = punti["downward_intensity_cd"].to_numpy(dtype="float64")
    heights = punti["altezza_palo_m"].to_numpy(dtype="float64")

    # Derived, not chosen: the distance at which the strongest lamp in this inventory falls below
    # lighting.NEGLIGIBLE_LUX. The flat 40 m this replaces was undocumented and happened to sit
    # near the 0.05-0.1 lux contour of a typical 100 W lamp at 9 m - right by luck, and wrong for
    # any city whose lamps are taller or brighter.
    search_radius = lighting.summation_radius_m(intensity, heights)
    print(f"  lamp search radius: {search_radius:.1f} m "
          f"(where the strongest lamp reaches {lighting.NEGLIGIBLE_LUX} lux)")

    tree = cKDTree(lamp_coords)
    nearby = tree.query_ball_point(pt_coords, search_radius)

    has_buildings = not buildings.empty
    if has_buildings:
        b_sindex = buildings.sindex
        b_geom = buildings.geometry.values
        b_height = pd.to_numeric(buildings["height"], errors="coerce").to_numpy(dtype="float64")
        own_building = lamp_own_building(punti, buildings)
        print(f"  {int((own_building >= 0).sum())} / {len(punti)} lamps sit inside a building "
              f"footprint; excluded from occluding their own sight lines.")
    else:
        b_sindex = None
        b_geom = None
        b_height = None
        own_building = None

    lux = np.zeros(len(points), dtype="float64")
    for i, lamp_list in enumerate(nearby):
        if not lamp_list:
            continue
        px, py = pt_coords[i]
        for lamp_idx in lamp_list:
            lx, ly = lamp_coords[lamp_idx]
            lamp_height = heights[lamp_idx]
            blocked = False
            if has_buildings:
                sight = LineString([(px, py), (lx, ly)])
                sight_len = sight.length
                own = own_building[lamp_idx]
                for b in b_sindex.intersection(sight.bounds):
                    if b == own:
                        continue
                    geom = b_geom[b]
                    # crosses, not intersects: a sight line that only touches a building's edge or
                    # corner -- a graze -- shares boundary, not interior, and should not count as
                    # occlusion (register finding A1). Genuinely passing through the footprint
                    # does share interior points, which is exactly what crosses requires here.
                    if not geom.crosses(sight):
                        continue
                    # Register finding A4: does the building's height actually reach the ray at the
                    # point the ray first enters the footprint? Height rises monotonically along
                    # the ray from 0 at the sample point to lamp_height at the lamp, so the nearest
                    # crossing point is the one that matters -- if the ray already clears the roof
                    # there, it clears it for the rest of the (rising) path through the building too.
                    bh = b_height[b]
                    if sight_len > 0 and not np.isnan(bh):
                        inter = geom.exterior.intersection(sight) if geom.geom_type == "Polygon" else geom.boundary.intersection(sight)
                        near_dist = _nearest_point_distance(inter, px, py)
                        if near_dist is not None:
                            ray_height_at_entry = (near_dist / sight_len) * lamp_height
                            if ray_height_at_entry >= bh:
                                continue  # ray passes over this building's roof: not blocked
                    blocked = True
                    break
            if not blocked:
                # d is the HORIZONTAL lamp-to-point distance; the tilt is already in the formula.
                d = np.hypot(lx - px, ly - py)
                lux[i] += lighting.illuminance_lux(intensity[lamp_idx], lamp_height, d)
    return lux


def _nearest_point_distance(intersection_geom, px: float, py: float) -> float | None:
    """Distance from (px, py) to the nearest point in a line/point intersection geometry, or None
    if the intersection is empty. Handles the Point / MultiPoint / LineString / MultiLineString /
    GeometryCollection shapes a boundary-vs-line intersection can come back as."""
    if intersection_geom.is_empty:
        return None
    geoms = list(intersection_geom.geoms) if hasattr(intersection_geom, "geoms") else [intersection_geom]
    best = None
    for g in geoms:
        coords = list(g.coords) if hasattr(g, "coords") else []
        for c in coords:
            cx, cy = c[0], c[1]  # buildings may carry a Z coordinate; only X/Y matter here
            d = np.hypot(cx - px, cy - py)
            if best is None or d < best:
                best = d
    return best


def main() -> None:
    parser = argparse.ArgumentParser(description="Calculate street illumination along network edges.")
    parser.add_argument("--city", required=True,
                        help="City name: folder under inputData/ and src/main/resources/, "
                             "and the <City>_* file prefix.")
    args = parser.parse_args()
    city = args.city

    punti, edges, buildings = load_inputs(city)

    print("Densifying edges to 2 m sample points...")
    points = densify_edges(edges, SAMPLE_SPACING_M)
    print(f"  {len(points)} sample points.")

    print("Computing line-of-sight lux per sample point...")
    points["calculated_lux"] = compute_lux(points, punti, buildings)
    points["is_unlit"] = points["calculated_lux"] < lighting.MIN_LUX

    edge_stats = points.groupby("parent_edge_idx").agg(
        min_lux=("calculated_lux", "min"),
        mean_lux=("calculated_lux", "mean"),
        pct_unlit=("is_unlit", lambda x: (x.sum() / len(x)) * 100),
    )

    edges = edges.join(edge_stats)
    edges["min_lux"] = edges["min_lux"].fillna(0.0)
    edges["mean_lux"] = edges["mean_lux"].fillna(0.0)
    edges["pct_unlit"] = edges["pct_unlit"].fillna(100.0)

    # Sim-read output goes to resources; the densified nodes are an intermediate for step 4.
    edges_out = paths.resources_dir(city) / f"{city}_edges_illuminated_continuous.gpkg"
    remove_existing(edges_out)
    edges.to_file(edges_out, driver="GPKG")
    print(f"saved: {edges_out}")

    nodes_out = paths.raw_dir(city) / f"{city}_nodes_2m_densified_illuminated.gpkg"
    remove_existing(nodes_out)
    points.drop(columns=["x", "y"]).to_file(nodes_out, driver="GPKG")
    print(f"saved: {nodes_out} ({len(points)} nodes)")


if __name__ == "__main__":
    main()
