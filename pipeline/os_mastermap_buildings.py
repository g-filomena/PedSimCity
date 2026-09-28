#!/usr/bin/env python
"""Turn a Digimap OS MasterMap order into ``<City>_officialBuildings.gpkg``.

The order is OS MasterMap Topography Layer (GeoPackage, Buildings) plus OS MasterMap Building
Height Attribute (CSV tiles). Footprints are the ``Topographicarea`` polygons whose
``descriptivegroup`` names a building; heights join on the OS TOID (``fid`` in the topography,
``OS_TOPO_TOID`` in the heights).

Written columns, as ``00_city_preparation.py`` reads an official layer:

  toid       OS TOID
  height     RelHMax, roof top above ground (m); empty where OS supplies no height
  base       empty by default, so the pipeline samples it from ``<City>_DTM.tif``, the raster
             that also gives the nodes their z; ``--base zero`` writes 0 for a city with no DTM
             (flat, matching nodes at z 0); ``--base absolute`` writes AbsHMin
  rel_h2     RelH2, roof base / eaves above ground (m)
  abs_hmin   AbsHMin, ground level above Ordnance Datum (m)
  bha_conf   BHA_Conf, OS confidence code

Usage:
  python pipeline/os_mastermap_buildings.py --city London \\
      --order inputData/London/Download_London_pedsimcity_buildings_BHA_3015943
"""
from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

import geopandas as gpd
import pandas as pd
import pyogrio

import paths

log = logging.getLogger("os_mastermap_buildings")

BHA_COLUMNS = {"RelHMax": "height", "RelH2": "rel_h2", "AbsHMin": "abs_hmin", "BHA_Conf": "bha_conf"}


def read_heights(order: Path) -> pd.DataFrame:
    """All Building Height Attribute tiles, one row per TOID (highest confidence, then latest)."""
    folder = next(order.glob("mastermap_building_heights_*"))
    header = (folder / "docs" / "osmm-building-heights-attribute-header.csv").read_text(
        encoding="utf-8-sig").strip().split(",")
    tiles = sorted((folder).rglob("*.csv"))
    tiles = [t for t in tiles if t.parent.name != "docs"]
    heights = pd.concat((pd.read_csv(t, header=None, names=header) for t in tiles), ignore_index=True)
    log.info("heights: %d rows from %d tiles", len(heights), len(tiles))
    # A TOID on a tile edge appears in both tiles.
    heights = heights.sort_values(["BHA_Conf", "BHA_ProcessDate"], ascending=[False, False])
    heights = heights.drop_duplicates("OS_TOPO_TOID")
    return heights.set_index("OS_TOPO_TOID")[list(BHA_COLUMNS)].rename(columns=BHA_COLUMNS)


def read_footprints(order: Path) -> gpd.GeoDataFrame:
    topo = next(order.glob("mastermap-topo_*.gpkg"))
    areas = pyogrio.read_dataframe(topo, layer="Topographicarea",
                                   columns=["fid", "descriptivegroup", "descriptiveterm"])
    buildings = areas[areas["descriptivegroup"].astype(str).str.contains("Building")]
    log.info("footprints: %d building polygons of %d areas", len(buildings), len(areas))
    return buildings.rename(columns={"fid": "toid"})


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--city", required=True)
    parser.add_argument("--order", required=True, type=Path,
                        help="the unzipped Digimap order folder")
    parser.add_argument("--base", choices=("dtm", "zero", "absolute"), default="dtm",
                        help="dtm: leave base empty for the pipeline to sample from the DTM "
                             "(default); zero: flat ground; absolute: AbsHMin")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s  %(levelname)s  %(message)s")

    footprints = read_footprints(args.order)
    heights = read_heights(args.order)
    buildings = footprints.join(heights, on="toid")
    buildings["base"] = {"dtm": float("nan"), "zero": 0.0}.get(args.base, buildings["abs_hmin"])
    buildings = buildings[["toid", "height", "base", "rel_h2", "abs_hmin", "bha_conf",
                           "descriptivegroup", "descriptiveterm", "geometry"]]

    with_height = buildings["height"].notna()
    log.info("heights joined: %d of %d buildings (%.1f%%), %.1f%% of footprint area",
             with_height.sum(), len(buildings), 100 * with_height.mean(),
             100 * buildings.area[with_height].sum() / buildings.area.sum())

    out = paths.raw_dir(args.city) / f"{args.city}_officialBuildings.gpkg"
    out.unlink(missing_ok=True)
    buildings.to_file(out, driver="GPKG", layer=f"{args.city}_officialBuildings")
    log.info("written %s (%d buildings)", out, len(buildings))
    return 0


if __name__ == "__main__":
    sys.exit(main())
