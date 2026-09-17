"""Export a city's street network as web geometry, keyed by ``edgeID``.

The simulation's edge layer is a GeoPackage in the city's own projected CRS - 11.9 MB and
EPSG:3003 for Torino - which a browser can read neither format nor coordinates of. This writes
the same edges as WGS84 GeoJSON, simplified and with coordinates rounded, small enough to serve.

Only the geometry travels. Volumes arrive separately from ``aggregate_season_volumes.py`` and
are joined in the page by ``edgeID``, so one geometry file serves every season.

    python export_network_geojson.py Torino --out site_data/Torino_edges.geojson

Run it in the *activated* conda env (``conda activate pedsimcity-prep``); calling the env's
python.exe by path leaves Library\\bin off PATH and GDAL fails on a delay-loaded DLL.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import geopandas as gpd

REPO_ROOT = Path(__file__).resolve().parent.parent

# Metres in the source CRS. Below a pedestrian network's block scale, so the simplification is
# invisible at any zoom the page offers while removing the survey-grade vertices that dominate
# the file size.
DEFAULT_TOLERANCE = 4.0

# ~1.1 m at the equator, finer than the tolerance above, and it halves the text size against the
# 13-14 significant digits a float carries by default.
COORD_DECIMALS = 5


def export(city: str, out_path: Path, tolerance: float, decimals: int) -> None:
    source = REPO_ROOT / "src" / "main" / "resources" / city / f"{city}_edges.gpkg"
    if not source.exists():
        raise SystemExit(f"no edge layer at {source}")

    edges = gpd.read_file(source)
    if "edgeID" not in edges.columns:
        raise SystemExit(f"{source.name} has no edgeID column")

    original_crs = edges.crs
    edges = edges[["edgeID", "geometry"]].copy()
    # Simplify in the projected CRS, where the tolerance is metres, then convert for the browser.
    edges["geometry"] = edges.geometry.simplify(tolerance, preserve_topology=False)
    edges = edges.to_crs("EPSG:4326")

    features = []
    for edge_id, geom in zip(edges["edgeID"], edges.geometry):
        if geom is None or geom.is_empty:
            continue
        # A LineString is the normal case; a MultiLineString is emitted as its parts so the page
        # never has to branch on geometry type.
        parts = geom.geoms if geom.geom_type == "MultiLineString" else [geom]
        for part in parts:
            coords = [[round(x, decimals), round(y, decimals)] for x, y in part.coords]
            if len(coords) < 2:
                continue
            features.append(
                {
                    "type": "Feature",
                    "properties": {"e": int(edge_id)},
                    "geometry": {"type": "LineString", "coordinates": coords},
                }
            )

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w") as handle:
        json.dump({"type": "FeatureCollection", "features": features}, handle, separators=(",", ":"))

    size_mb = out_path.stat().st_size / 1e6
    print(
        f"{city}: {len(edges)} edges from {original_crs} -> {len(features)} features, "
        f"simplified at {tolerance} m, {decimals} dp -> {out_path} ({size_mb:.1f} MB)"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("city", help="city name, matching src/main/resources/<city>/")
    parser.add_argument("--out", type=Path, required=True, help="GeoJSON file to write")
    parser.add_argument("--tolerance", type=float, default=DEFAULT_TOLERANCE,
                        help=f"simplification tolerance in source-CRS metres (default {DEFAULT_TOLERANCE})")
    parser.add_argument("--decimals", type=int, default=COORD_DECIMALS,
                        help=f"decimal places kept on lon/lat (default {COORD_DECIMALS})")
    args = parser.parse_args()
    export(args.city, args.out, args.tolerance, args.decimals)


if __name__ == "__main__":
    main()
