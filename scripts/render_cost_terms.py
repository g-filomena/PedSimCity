"""Draws one trip after dark as each term of the night cost is added, for the results site.

Reads the routes and street values written by ``NightCostTermRoutes.java`` and the street
geometry in ``src/main/resources/<City>/<City>_edges.gpkg``, and writes one panel per step under
``<data>/<City>/terms/`` (the site's data folder, as for ``render_city_light.py``):

    terms_<n>_<step>.webp   the route at that step, over the layer of the term just added, with
                            the previous step's route dashed
    terms.json              per step: metres walked, and metres under 10 lux, through parks or
                            along rivers, on enclosed streets and on empty ones

``--sheet`` instead draws every candidate the search kept, one row each, to choose from.

    conda run -n pedsimcity-prep python scripts/render_cost_terms.py Torino \\
        --routes cost_terms.json --edges cost_terms_edges.csv --candidate 3
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
import pandas as pd  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap, to_rgba  # noqa: E402
from PIL import Image  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[1]
SITE_DATA = REPO_ROOT.parent / "inclusivestreets" / "torino-di-notte" / "data"
BACKGROUND = "#07090f"
STREET = "#2a2e3b"
ROUTE = "#ffffff"
PREVIOUS = "#8e93a6"
# What a term adds to the cost, drawn in one cool colour; light and open venues drawn warm.
COSTLY = "#7f8ce6"
# The scale bar in the bottom-left corner of every panel; the page says how long it is.
SCALE_METRES = 250
LUX_RAMP = LinearSegmentedColormap.from_list("night", [
    (0.00, "#1b2440"), (0.10, "#2c3a66"), (0.20, "#5a5a7a"), (0.34, "#b9853f"),
    (0.60, "#f2b45c"), (1.00, "#fff3d6")])
EYES_RAMP = LinearSegmentedColormap.from_list("eyes", [STREET, "#b9853f", "#f2b45c", "#fff3d6"])
COSTLY_RAMP = LinearSegmentedColormap.from_list("costly", [STREET, COSTLY])


def segments(geometry) -> list[np.ndarray]:
    if geometry.geom_type == "LineString":
        return [np.asarray(geometry.coords)[:, :2]]
    return [np.asarray(part.coords)[:, :2] for part in geometry.geoms]


def layer_colours(step: str, streets: pd.DataFrame) -> tuple[np.ndarray, np.ndarray]:
    """Colour and width of every street under a step: the layer of the term that step adds."""
    n = len(streets)
    width = np.full(n, 0.9)
    if step == "darkness":
        colours = LUX_RAMP(np.clip(streets["lux"].to_numpy() / 30.0, 0, 1))
    elif step == "parksRivers":
        flag = streets["park_or_water"].to_numpy() == 1
        colours = np.where(flag[:, None], to_rgba(COSTLY), to_rgba(STREET))
        width = np.where(flag, 1.6, 0.9)
    elif step == "enclosure":
        value = streets["enclosure"].to_numpy()
        colours = COSTLY_RAMP(np.clip(value, 0, 1))
        width = 0.9 + 0.9 * value
    elif step == "emptyStreet":
        value = streets["eyes"].to_numpy()
        colours = EYES_RAMP(np.clip(value, 0, 1))
        width = 0.9 + 0.9 * value
    else:
        colours = np.tile(to_rgba(STREET), (n, 1))
    return colours, width


def route_lines(edges: gpd.GeoDataFrame, ids: list[int]) -> list[np.ndarray]:
    rows = edges.loc[[i for i in ids if i in edges.index]]
    return [seg for geom in rows.geometry for seg in segments(geom)]


def window(edges: gpd.GeoDataFrame, candidate: dict, pad: float = 0.18) -> tuple:
    """A square around every route of the candidate."""
    ids = {i for r in candidate["routes"] for i in r["edges"]}
    minx, miny, maxx, maxy = edges.loc[sorted(ids)].total_bounds
    side = max(maxx - minx, maxy - miny) * (1 + 2 * pad)
    cx, cy = (minx + maxx) / 2, (miny + maxy) / 2
    return cx - side / 2, cy - side / 2, cx + side / 2, cy + side / 2


def draw(ax, edges, streets, candidate, step_index, box) -> None:
    routes = candidate["routes"]
    step = routes[step_index]["step"]
    colours, width = layer_colours(step, streets)
    lines, cols, widths = [], [], []
    for (geom, colour, w) in zip(streets.geometry, colours, width):
        for seg in segments(geom):
            lines.append(seg)
            cols.append(colour)
            widths.append(w)
    ax.add_collection(LineCollection(lines, colors=cols, linewidths=widths, capstyle="round"))
    if step_index > 0:
        previous = 4 if step == "nonVulnerable" else step_index - 1
        ax.add_collection(LineCollection(route_lines(edges, routes[previous]["edges"]),
                                         colors=PREVIOUS, linewidths=1.6,
                                         linestyles=(0, (2.2, 1.8)), capstyle="butt"))
    path = route_lines(edges, routes[step_index]["edges"])
    ax.add_collection(LineCollection(path, colors=BACKGROUND, linewidths=6.0, capstyle="round"))
    ax.add_collection(LineCollection(path, colors=ROUTE, linewidths=3.2, capstyle="round"))
    first = edges.loc[routes[0]["edges"][0]].geometry
    last = edges.loc[routes[0]["edges"][-1]].geometry
    ends = endpoints(first, last, routes[0]["edges"], edges)
    for (x, y), filled in zip(ends, (True, False)):
        ax.scatter([x], [y], s=110, zorder=5, color=ROUTE if filled else BACKGROUND,
                   edgecolors=ROUTE, linewidths=2.2)
    minx, miny, maxx, maxy = box
    side = maxx - minx
    x0, y0 = minx + 0.04 * side, miny + 0.04 * side
    ax.plot([x0, x0 + SCALE_METRES], [y0, y0], color=PREVIOUS, linewidth=2.0,
            solid_capstyle="butt", zorder=6)
    ax.set_xlim(minx, maxx)
    ax.set_ylim(miny, maxy)
    ax.set_aspect("equal")
    ax.axis("off")


def endpoints(first, last, ids, edges) -> list[tuple]:
    """Origin and destination: the end of the first edge not shared with the second, and the end
    of the last edge not shared with the one before it."""
    def ends(geom):
        seg = segments(geom)
        return [tuple(seg[0][0]), tuple(seg[-1][-1])]

    def far(geom, neighbour_geom):
        a, b = ends(geom)
        na = ends(neighbour_geom)
        da = min(np.hypot(a[0] - p[0], a[1] - p[1]) for p in na)
        db = min(np.hypot(b[0] - p[0], b[1] - p[1]) for p in na)
        return a if da > db else b

    if len(ids) == 1:
        return ends(first)
    return [far(first, edges.loc[ids[1]].geometry), far(last, edges.loc[ids[-2]].geometry)]


def figure(px: int):
    dpi = 200
    fig = plt.figure(figsize=(px / dpi, px / dpi), dpi=dpi)
    fig.patch.set_facecolor(BACKGROUND)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_facecolor(BACKGROUND)
    return fig, ax, dpi


def webp(fig, dpi) -> bytes:
    buffer = io.BytesIO()
    fig.savefig(buffer, format="png", dpi=dpi, facecolor=BACKGROUND)
    plt.close(fig)
    out = io.BytesIO()
    Image.open(io.BytesIO(buffer.getvalue())).convert("RGB").save(out, "WEBP", quality=84, method=6)
    return out.getvalue()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("city")
    parser.add_argument("--routes", type=Path, required=True, help="NightCostTermRoutes' JSON")
    parser.add_argument("--edges", type=Path, required=True, help="NightCostTermRoutes' edge CSV")
    parser.add_argument("--candidate", type=int, default=0, help="which kept trip to draw")
    parser.add_argument("--sheet", type=Path, help="draw every candidate to this PNG instead")
    parser.add_argument("--width", type=int, default=900, help="panel width in pixels")
    parser.add_argument("--data", type=Path, default=SITE_DATA)
    args = parser.parse_args()

    found = json.loads(args.routes.read_text())
    values = pd.read_csv(args.edges).set_index("edgeID")
    edges = gpd.read_file(REPO_ROOT / "src" / "main" / "resources" / args.city
                          / f"{args.city}_edges.gpkg").set_index("edgeID")
    edges = edges.join(values)

    def streets_in(box):
        minx, miny, maxx, maxy = box
        return edges.cx[minx:maxx, miny:maxy]

    candidates = found["candidates"]
    if args.sheet:
        steps = len(candidates[0]["routes"])
        fig = plt.figure(figsize=(steps * 2.2, len(candidates) * 2.3), dpi=110)
        fig.patch.set_facecolor(BACKGROUND)
        for row, candidate in enumerate(candidates):
            box = window(edges, candidate)
            streets = streets_in(box)
            for col in range(steps):
                ax = fig.add_subplot(len(candidates), steps, row * steps + col + 1)
                ax.set_facecolor(BACKGROUND)
                draw(ax, edges, streets, candidate, col, box)
                r = candidate["routes"][col]
                ax.set_title(f"{row} {r['step']} {r['metres']:.0f} m", color="white", fontsize=7)
        fig.savefig(args.sheet, facecolor=BACKGROUND)
        print(args.sheet)
        return

    candidate = candidates[args.candidate]
    box = window(edges, candidate)
    streets = streets_in(box)
    out_dir = args.data / args.city / "terms"
    out_dir.mkdir(parents=True, exist_ok=True)
    for old in out_dir.glob("terms_*.webp"):
        old.unlink()
    panels = []
    for i, r in enumerate(candidate["routes"]):
        fig, ax, dpi = figure(args.width)
        draw(ax, edges, streets, candidate, i, box)
        name = f"terms_{i}_{r['step']}.webp"
        (out_dir / name).write_bytes(webp(fig, dpi))
        panels.append({k: (round(v) if isinstance(v, float) else v)
                       for k, v in r.items() if k != "edges"} | {"image": name})
    side = box[2] - box[0]
    summary = {"city": args.city, "hour": found["hour"], "weights": found["weights"],
               "origin": candidate["origin"], "destination": candidate["destination"],
               "windowMetres": round(side), "scaleMetres": SCALE_METRES, "panels": panels}
    (out_dir / "terms.json").write_text(json.dumps(summary, indent=1))
    print(f"{len(panels)} panels -> {out_dir}; window {side:.0f} m; "
          + ", ".join(f"{p['step']} {p['metres']} m" for p in panels))


if __name__ == "__main__":
    main()
