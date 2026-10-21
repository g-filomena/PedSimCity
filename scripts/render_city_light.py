"""Renders a city's streets by their modelled night-time illuminance, for the results site.

Reads ``src/main/resources/<City>/<City>_edges_illuminated_continuous.gpkg`` and writes, under
``<data>/<City>/light/`` (the site's data folder, ``../inclusivestreets/torino-di-notte/data``
beside this checkout unless ``--data`` says otherwise):

    <City>_light.webp          every street drawn by its mean lux, dim streets faint, lit ones warm
    <City>_light_detail.webp   a close-up, for cities with a DETAIL_WINDOW
    light.json                 street length by lux band, which the narrative page quotes

Run in the pedsimcity-prep env:

    conda run -n pedsimcity-prep python scripts/render_city_light.py Torino
"""

from __future__ import annotations

import argparse
import io
import json
from pathlib import Path

import geopandas as gpd
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402
from PIL import Image  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[1]
SITE_DATA = REPO_ROOT.parent / "inclusivestreets" / "torino-di-notte" / "data"
BACKGROUND = "#07090f"  # the site's night plates
# Lux bands the page reports: 10 lx is where reassurance plateaus, 5 lx the service level.
BANDS = [(0, 1), (1, 5), (5, 10), (10, 20), (20, None)]
# Darkness at 0 lx to sodium-warm white beyond 30 lx; positions are lux / 30.
RAMP = LinearSegmentedColormap.from_list("night", [
    (0.00, "#1b2440"), (0.10, "#2c3a66"), (0.20, "#5a5a7a"), (0.34, "#b9853f"),
    (0.60, "#f2b45c"), (1.00, "#fff3d6")])
LUX_CEILING = 30.0
# A close-up for the page, as fractions of the city's extent (x0, x1, y0, y1; y from the top):
# Turin's centre, the Po, the Valentino park and the hill beyond the river.
DETAIL_WINDOW = {"Torino": (0.40, 0.88, 0.36, 0.80)}


def segments(geometry) -> list[np.ndarray]:
    """Coordinate arrays of a LineString or MultiLineString."""
    if geometry.geom_type == "LineString":
        return [np.asarray(geometry.coords)[:, :2]]
    return [np.asarray(part.coords)[:, :2] for part in geometry.geoms]


def render(edges: gpd.GeoDataFrame, width_px: int, window: tuple | None = None,
           zoom: float = 1.0) -> bytes:
    """The streets on a dark ground. ``window`` (x0, x1, y0, y1, as fractions of the city's
    extent, y from the top) crops to a detail; ``zoom`` thickens the lines to match."""
    lux = edges["mean_lux"].fillna(0.0).to_numpy()
    order = np.argsort(lux)  # brightest drawn last, on top
    lines, values = [], []
    for i in order:
        for seg in segments(edges.geometry.iloc[i]):
            lines.append(seg)
            values.append(lux[i])
    values = np.asarray(values)
    colours = RAMP(np.clip(values / LUX_CEILING, 0, 1))

    minx, miny, maxx, maxy = edges.total_bounds
    pad = 0.02 * max(maxx - minx, maxy - miny)
    if window:
        x0, x1, y0, y1 = window
        w, h = maxx - minx, maxy - miny
        minx, maxx = minx + x0 * w, minx + x1 * w
        miny, maxy = maxy - y1 * h, maxy - y0 * h
        pad = 0.0
    aspect = (maxy - miny + 2 * pad) / (maxx - minx + 2 * pad)
    dpi = 200
    fig = plt.figure(figsize=(width_px / dpi, width_px * aspect / dpi), dpi=dpi)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_facecolor(BACKGROUND)
    fig.patch.set_facecolor(BACKGROUND)

    lit = values >= 10
    glow = colours.copy()
    glow[:, 3] = 0.10
    ax.add_collection(LineCollection([l for l, k in zip(lines, lit) if k],
                                     colors=glow[lit], linewidths=2.4 * zoom, capstyle="round"))
    ax.add_collection(LineCollection(lines, colors=colours,
                                     linewidths=np.where(lit, 0.42, 0.3) * zoom,
                                     capstyle="round"))
    ax.set_xlim(minx - pad, maxx + pad)
    ax.set_ylim(miny - pad, maxy + pad)
    ax.set_aspect("equal")
    ax.axis("off")

    buffer = io.BytesIO()
    fig.savefig(buffer, format="png", dpi=dpi, facecolor=BACKGROUND)
    plt.close(fig)
    out = io.BytesIO()
    Image.open(io.BytesIO(buffer.getvalue())).convert("RGB").save(out, "WEBP", quality=82,
                                                                 method=6)
    return out.getvalue()


def band_shares(edges: gpd.GeoDataFrame) -> dict:
    length = edges.geometry.length.to_numpy()
    lux = edges["mean_lux"].fillna(0.0).to_numpy()
    total = float(length.sum())
    bands = []
    for lo, hi in BANDS:
        mask = (lux >= lo) & (lux < hi if hi is not None else True)
        bands.append({"from": lo, "to": hi, "share": float(length[mask].sum() / total)})
    return {"streetKm": total / 1000, "streets": int(len(edges)), "bands": bands}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("city")
    parser.add_argument("--width", type=int, default=1800, help="image width in pixels")
    parser.add_argument("--data", type=Path, default=SITE_DATA,
                        help=f"the site's data folder (default: {SITE_DATA})")
    args = parser.parse_args()

    source = (REPO_ROOT / "src" / "main" / "resources" / args.city
              / f"{args.city}_edges_illuminated_continuous.gpkg")
    edges = gpd.read_file(source)
    if not edges.crs or not edges.crs.is_projected:
        raise SystemExit(f"{source.name} needs a projected CRS to measure street length")

    out_dir = args.data / args.city / "light"
    out_dir.mkdir(parents=True, exist_ok=True)
    image = out_dir / f"{args.city}_light.webp"
    image.write_bytes(render(edges, args.width))
    stats = band_shares(edges)
    stats["image"] = image.name
    window = DETAIL_WINDOW.get(args.city)
    if window:
        detail = out_dir / f"{args.city}_light_detail.webp"
        detail.write_bytes(render(edges, args.width, window, zoom=1.7))
        stats["detail"] = detail.name
    (out_dir / "light.json").write_text(json.dumps(stats, indent=1))
    print(f"{image} ({image.stat().st_size / 1e3:.0f} kB); "
          f"{stats['streetKm']:,.0f} km of street, "
          + ", ".join(f"{b['from']}-{b['to'] or ''} lx {b['share']:.1%}" for b in stats["bands"]))


if __name__ == "__main__":
    main()
