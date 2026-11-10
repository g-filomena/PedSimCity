#!/usr/bin/env python3
"""The Smart Locator grid and time bands, shared by preparation and validation.

Two sides depend on these: the preparation scripts in this folder, which build
the observed GeoPackage, and the validation scripts in the PedSimCity repo,
which compare the model against it. Both keep their own copy of this module —
the contract between them is the GeoPackage, not a Python import — so any change
here has to be copied across. It is deliberately dependency-light (no
matplotlib) so `build_gpkg.py` can use it without pulling in a plotting stack.
"""

from __future__ import annotations

from pathlib import Path

import geopandas as gpd
import numpy as np
from shapely.geometry import box

MODULE_DIR = Path(__file__).resolve().parent

# the day runs 06:00 -> 24:00, with the overnight band last.
# (key, label, hours covered) — the widths differ, so anything compared across
# bands has to be divided by the third element first.
BANDS = [
    ("06_08", "06:00 – 08:00", 2),
    ("08_10", "08:00 – 10:00", 2),
    ("10_12", "10:00 – 12:00", 2),
    ("12_14", "12:00 – 14:00", 2),
    ("14_16", "14:00 – 16:00", 2),
    ("16_18", "16:00 – 18:00", 2),
    ("18_20", "18:00 – 20:00", 2),
    ("20_24", "20:00 – 24:00", 4),
    ("00_06", "00:00 – 06:00", 6),
]

QUADKEY_LEVEL = 17
BLOCK_M = 216.0          # a level-17 cell at Turin's latitude
CRS = "EPSG:3003"        # Monte Mario / Italy zone 1


def quadkey_to_tile(quadkey: str) -> tuple[int, int, int]:
    """Bing/Web-Mercator quadkey -> (x, y, zoom)."""
    x = y = 0
    z = len(quadkey)
    for i, digit in enumerate(quadkey):
        mask = 1 << (z - i - 1)
        d = int(digit)
        if d & 1:
            x |= mask
        if d & 2:
            y |= mask
    return x, y, z


def tile_bounds_3857(x: int, y: int, z: int) -> tuple[float, float, float, float]:
    """Tile bounds in Web Mercator metres - a square, so no densification needed."""
    extent = 20037508.342789244
    size = 2 * extent / (1 << z)
    minx = -extent + x * size
    maxy = extent - y * size
    return minx, maxy - size, minx + size, maxy


def cell_frame(quadkeys: list[str]) -> gpd.GeoDataFrame:
    """Polygons for a list of quadkeys, at whatever level they are."""
    geoms = [box(*tile_bounds_3857(*quadkey_to_tile(qk))) for qk in quadkeys]
    frame = gpd.GeoDataFrame({"quadkey": list(quadkeys)}, geometry=geoms,
                             crs="EPSG:3857")
    return frame.to_crs(CRS)


BOUNDARY_FILE = MODULE_DIR / "Torino_boundary.gpkg"


def city_boundary(cells: gpd.GeoDataFrame, snap: float = 0.5,
                  offset: float = 70.0) -> list[np.ndarray]:
    """The municipal outline.

    Uses the surveyed boundary polygon when it is present. Otherwise falls back
    to dissolving the cell mosaic: the cells touch only at corners in places,
    which leaves the raw union pinched, so it is buffered out and back to close
    those, then pushed `offset` metres clear of the grid to stay legible where
    edge cells are brightly filled.
    """
    if BOUNDARY_FILE.exists():
        merged = gpd.read_file(BOUNDARY_FILE).to_crs(cells.crs).geometry.union_all()
    else:
        merged = cells.geometry.union_all().buffer(snap).buffer(-snap).buffer(offset)
    parts = list(getattr(merged, "geoms", [merged]))
    biggest = max(p.area for p in parts)
    return [np.asarray(p.exterior.coords) for p in parts if p.area > 0.01 * biggest]
