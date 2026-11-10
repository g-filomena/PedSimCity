#!/usr/bin/env python3
"""Pull the pedestrian simulation out of results.html into a GeoPackage.

results.html embeds four JavaScript constants. This reads them without loading
the page, folds the 24 hourly edge volumes into the nine Smart Locator time
bands, and writes both the street network and a cell aggregation onto the same
216 m quadkey grid the API data uses, so the two are directly comparable.

    python extract_simulation.py --html results.html --out data/torino_simulation.gpkg
"""

from __future__ import annotations

import argparse
import ast
import json
import logging
import re
from pathlib import Path

import geopandas as gpd
import numpy as np
import pandas as pd
from pyproj import Transformer
from shapely.geometry import LineString

from vfgrid import BANDS

log = logging.getLogger("sim")

# the API bands, as the hours of the day each one covers
BAND_HOURS = {
    "00_06": range(0, 6), "06_08": range(6, 8), "08_10": range(8, 10),
    "10_12": range(10, 12), "12_14": range(12, 14), "14_16": range(14, 16),
    "16_18": range(16, 18), "18_20": range(18, 20), "20_24": range(20, 24),
}
BAND_KEYS = [b[0] for b in BANDS]
HOUR_BAND = {h: k for k, hours in BAND_HOURS.items() for h in hours}
CRS = "EPSG:3003"
STEP_MINUTES = 20          # the page reads the clock as step * 20 / 60
QUADKEY_LEVEL = 17
COORD_DP = 2               # trip vertices repeat edge vertices exactly


def read_blocks(html: Path) -> dict[str, str]:
    """Grab the `const NAME = ...;` payloads a line at a time - the file is 29 MB."""
    wanted = ("ROADS_GEOJSON", "TRIPS", "AB_TRIPS", "HOURLY_VOL")
    pattern = re.compile(r"\s*const (%s) = (.*?);?\s*$" % "|".join(wanted))
    blocks: dict[str, str] = {}
    with html.open() as fh:
        for line in fh:
            m = pattern.match(line)
            if m:
                blocks[m.group(1)] = m.group(2)
    missing = set(wanted) - set(blocks)
    if missing:
        raise SystemExit(f"{html}: missing {', '.join(sorted(missing))}")
    return blocks


def build_edges(blocks: dict[str, str]) -> gpd.GeoDataFrame:
    roads = json.loads(blocks["ROADS_GEOJSON"])
    # HOURLY_VOL has bare integer keys, so it is JavaScript rather than JSON
    hourly = ast.literal_eval(blocks["HOURLY_VOL"])

    rows, geoms = [], []
    for feature in roads["features"]:
        props = feature["properties"]
        edge_id = props["edgeID"]
        counts = np.asarray(hourly.get(edge_id, [0] * 24), dtype=float)
        row = {
            "edgeID": edge_id,
            "mean_lux": props.get("mean_lux"),
            "total": float(counts.sum()),
        }
        for key in BAND_KEYS:
            row[f"band_{key}"] = float(counts[list(BAND_HOURS[key])].sum())
        rows.append(row)
        geoms.append(LineString(feature["geometry"]["coordinates"]))

    edges = gpd.GeoDataFrame(rows, geometry=geoms, crs=CRS)
    edges["length_m"] = edges.length
    log.info("%d edges, %d carrying volume, %.0f crossings total",
             len(edges), int((edges["total"] > 0).sum()), edges["total"].sum())
    return edges


def edge_segment_index(roads: dict) -> dict:
    """(vertex, vertex) -> edgeID, so a trip's path maps onto exact edges."""
    index = {}
    for feature in roads["features"]:
        eid = feature["properties"]["edgeID"]
        coords = feature["geometry"]["coordinates"]
        for a, b in zip(coords, coords[1:]):
            ka = (round(a[0], COORD_DP), round(a[1], COORD_DP))
            kb = (round(b[0], COORD_DP), round(b[1], COORD_DP))
            index[(ka, kb)] = eid
            index[(kb, ka)] = eid
    return index


_TO_MERCATOR = None


def quadkeys_for(xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
    """EPSG:3003 metres -> level-17 quadkey strings."""
    global _TO_MERCATOR
    if _TO_MERCATOR is None:
        _TO_MERCATOR = Transformer.from_crs(CRS, "EPSG:3857", always_xy=True)
    mx, my = _TO_MERCATOR.transform(xs, ys)
    extent = 20037508.342789244
    n = 1 << QUADKEY_LEVEL
    tx = np.clip(((mx + extent) / (2 * extent) * n).astype(int), 0, n - 1)
    ty = np.clip(((extent - my) / (2 * extent) * n).astype(int), 0, n - 1)
    out = np.empty(len(tx), dtype=object)
    for i, (x, y) in enumerate(zip(tx, ty)):
        digits = []
        for bit in range(QUADKEY_LEVEL, 0, -1):
            mask = 1 << (bit - 1)
            d = (1 if x & mask else 0) + (2 if y & mask else 0)
            digits.append(str(d))
        out[i] = "".join(digits)
    return out


def split_by_agent(trips: list, roads: dict) -> tuple[dict, dict]:
    """Vulnerable / normal trip counts per edge and per cell, by time band.

    A trip is credited to an edge or cell once per band it is in there. The
    band comes from interpolating the trip's start and end step along the path,
    which is how the page itself places agents in time.
    """
    index = edge_segment_index(roads)
    edge_counts: dict[tuple, list] = {}
    cell_counts: dict[tuple, list] = {}

    for trip in trips:
        coords = np.asarray(trip[3], dtype=float)
        if len(coords) < 2:
            continue
        start, end, vulnerable = trip[1], trip[2], bool(trip[4])
        steps = np.hypot(*(coords[1:] - coords[:-1]).T)
        along = np.r_[0.0, np.cumsum(steps)]
        total = along[-1] or 1.0
        midpoints = (along[:-1] + along[1:]) / 2 / total
        hours = ((start + midpoints * (end - start)) * STEP_MINUTES / 60 % 24).astype(int)
        cells = quadkeys_for(coords[:, 0], coords[:, 1])

        seen_edges, seen_cells = set(), set()
        for i, (a, b) in enumerate(zip(coords, coords[1:])):
            band = HOUR_BAND[int(hours[i])]
            key = ((round(a[0], COORD_DP), round(a[1], COORD_DP)),
                   (round(b[0], COORD_DP), round(b[1], COORD_DP)))
            eid = index.get(key)
            if eid is not None:
                seen_edges.add((eid, band))
            seen_cells.add((cells[i], band))
            seen_cells.add((cells[i + 1], band))

        slot = 0 if vulnerable else 1
        for key in seen_edges:
            edge_counts.setdefault(key, [0, 0])[slot] += 1
        for key in seen_cells:
            cell_counts.setdefault(key, [0, 0])[slot] += 1

    log.info("%d trips, %.0f%% vulnerable", len(trips),
             100 * sum(bool(t[4]) for t in trips) / len(trips))
    return edge_counts, cell_counts


def attach_shares(frame: pd.DataFrame, counts: dict, key_col: str) -> pd.DataFrame:
    """Add vulnerable / normal trip counts and their share, per band."""
    for band in BAND_KEYS:
        v = frame[key_col].map({k[0]: c[0] for k, c in counts.items() if k[1] == band})
        n = frame[key_col].map({k[0]: c[1] for k, c in counts.items() if k[1] == band})
        v = v.fillna(0.0)
        n = n.fillna(0.0)
        frame[f"vuln_{band}"] = v
        frame[f"norm_{band}"] = n
        frame[f"trips_{band}"] = v + n
        frame[f"share_v_{band}"] = np.where(v + n > 0, v / (v + n), np.nan).round(4)
    return frame


def to_cells(edges: gpd.GeoDataFrame, cells: gpd.GeoDataFrame) -> gpd.GeoDataFrame:
    """Split each edge across the cells it crosses, weighting by length inside."""
    band_cols = [f"band_{k}" for k in BAND_KEYS]
    active = edges[edges["total"] > 0].copy()

    pieces = gpd.overlay(
        active[["edgeID", "length_m", "total", *band_cols, "geometry"]],
        cells[["quadkey", "geometry"]],
        how="intersection", keep_geom_type=True,
    )
    pieces["share"] = pieces.length / pieces["length_m"]
    for col in ["total", *band_cols]:
        pieces[col] = pieces[col] * pieces["share"]

    agg = pieces.groupby("quadkey", as_index=False)[["total", *band_cols]].sum()
    out = cells[["quadkey", "geometry"]].merge(agg, on="quadkey", how="left")
    out[["total", *band_cols]] = out[["total", *band_cols]].fillna(0.0).round(2)

    # how much of the network sits in each cell, to separate "no streets here"
    # from "streets nobody walked"
    net = gpd.overlay(edges[["edgeID", "geometry"]], cells[["quadkey", "geometry"]],
                      how="intersection", keep_geom_type=True)
    net_len = net.groupby("quadkey").apply(lambda d: d.length.sum(), include_groups=False)
    out["street_m"] = out["quadkey"].map(net_len).fillna(0.0).round(1)
    log.info("%d cells, %d with streets, %d walked",
             len(out), int((out["street_m"] > 0).sum()), int((out["total"] > 0).sum()))
    return out


def read_new_format(edges_path: Path, volume_path: Path) -> gpd.GeoDataFrame:
    """Read the re-run export: edges.gpkg + edge_volume.csv.

    The model now emits the vulnerable / normal split per edge per hour, so
    nothing has to be reconstructed from trip paths.
    """
    edges = gpd.read_file(edges_path)
    if edges.crs is None or edges.crs.to_epsg() != 3003:
        raise SystemExit(f"{edges_path}: expected EPSG:3003, got {edges.crs}")

    vol = pd.read_csv(volume_path)
    vol["band"] = vol["hour"].map(HOUR_BAND)
    grouped = vol.groupby(["edgeID", "band"], as_index=False)[["vuln_count", "norm_count"]].sum()

    wide = grouped.pivot(index="edgeID", columns="band",
                         values=["vuln_count", "norm_count"]).fillna(0.0)
    frame = pd.DataFrame(index=edges["edgeID"])
    for band in BAND_KEYS:
        v = wide[("vuln_count", band)].reindex(frame.index).fillna(0.0) if             ("vuln_count", band) in wide.columns else 0.0
        n = wide[("norm_count", band)].reindex(frame.index).fillna(0.0) if             ("norm_count", band) in wide.columns else 0.0
        frame[f"vuln_{band}"] = v
        frame[f"norm_{band}"] = n
        frame[f"trips_{band}"] = v + n          # crossings by any agent
        frame[f"band_{band}"] = v + n           # same quantity, volume name
        frame[f"share_v_{band}"] = np.where(v + n > 0, v / (v + n), np.nan).round(4)

    edges = edges.join(frame, on="edgeID")
    edges["total"] = edges[[f"band_{b}" for b in BAND_KEYS]].sum(axis=1)
    edges["length_m"] = edges.length
    log.info("%d edges, %d carrying volume, %.0f crossings, %.0f%% vulnerable",
             len(edges), int((edges["total"] > 0).sum()), edges["total"].sum(),
             100 * sum(edges[f"vuln_{b}"].sum() for b in BAND_KEYS) / edges["total"].sum())
    return edges


def cells_from_edges(edges: gpd.GeoDataFrame, cells: gpd.GeoDataFrame) -> gpd.GeoDataFrame:
    """Length-weighted split of every per-edge count into the cells it crosses."""
    count_cols = [c for c in edges.columns
                  if c.startswith(("band_", "vuln_", "norm_", "trips_"))] + ["total"]
    active = edges[edges["total"] > 0]
    pieces = gpd.overlay(active[["edgeID", "length_m", *count_cols, "geometry"]],
                         cells[["quadkey", "geometry"]],
                         how="intersection", keep_geom_type=True)
    pieces["share"] = pieces.length / pieces["length_m"]
    for col in count_cols:
        pieces[col] = pieces[col] * pieces["share"]

    agg = pieces.groupby("quadkey", as_index=False)[count_cols].sum()
    out = cells[["quadkey", "geometry"]].merge(agg, on="quadkey", how="left")
    out[count_cols] = out[count_cols].fillna(0.0).round(2)
    for band in BAND_KEYS:
        v, n = out[f"vuln_{band}"], out[f"norm_{band}"]
        out[f"share_v_{band}"] = np.where(v + n > 0, v / (v + n), np.nan).round(4)

    net = gpd.overlay(edges[["edgeID", "geometry"]], cells[["quadkey", "geometry"]],
                      how="intersection", keep_geom_type=True)
    net_len = net.groupby("quadkey").apply(lambda d: d.length.sum(), include_groups=False)
    out["street_m"] = out["quadkey"].map(net_len).fillna(0.0).round(1)
    log.info("%d cells, %d with streets, %d walked",
             len(out), int((out["street_m"] > 0).sum()), int((out["total"] > 0).sum()))
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--html", default="sim/results_firstrun.html",
                        help="legacy self-contained page (used only if --edges is absent)")
    parser.add_argument("--edges", default="sim/edges.gpkg")
    parser.add_argument("--volume", default="sim/edge_volume.csv")
    parser.add_argument("--grid", default="data/torino_presence.gpkg",
                        help="GeoPackage whose quadkeys define the shared grid")
    parser.add_argument("--out", default="data/torino_simulation.gpkg")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S")

    grid = gpd.read_file(args.grid, layer="quadkeys_seasons")[["quadkey", "geometry"]]

    if Path(args.edges).exists() and Path(args.volume).exists():
        edges = read_new_format(Path(args.edges), Path(args.volume))
        cells = cells_from_edges(edges, grid)
    else:
        # legacy path: everything embedded in results.html, agent split
        # reconstructed by matching trip paths back onto edges
        log.info("no %s / %s - falling back to %s", args.edges, args.volume, args.html)
        blocks = read_blocks(Path(args.html))
        edges = build_edges(blocks)
        cells = to_cells(edges, grid)
        roads = json.loads(blocks["ROADS_GEOJSON"])
        trips = json.loads(blocks["TRIPS"])
        edge_counts, cell_counts = split_by_agent(trips, roads)
        edges = attach_shares(edges, edge_counts, "edgeID")
        cells = attach_shares(cells, cell_counts, "quadkey")

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    if out.exists():
        out.unlink()
    edges.to_file(out, layer="sim_edges", driver="GPKG")
    cells.to_file(out, layer="sim_cells", driver="GPKG")
    log.info("wrote %s (%.1f MB)", out, out.stat().st_size / 1e6)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
