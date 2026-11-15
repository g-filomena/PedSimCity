"""The elements of the city image in Barcelona, white on black, three figures of two panels each:

    python barcelona_elements.py [--only districts_barriers landmarks sightlines_places]

    barcelona_districts_barriers.png     the 26 districts | the barriers (water, parks, railways, main roads)
    barcelona_landmarks.png              local landmarkness | global landmarkness, on the terrain
    barcelona_sight_lines_places.png     the sight lines, all of them at low opacity | the places to go to

Reads src/main/resources/Barcelona/Barcelona_*.gpkg and inputData/Barcelona/Barcelona_DEM.tif and writes figures/barcelona/*.png.
The look (black, white Bahnschrift, the palettes) is the one of the night scripts in analysis/validation/Torino/night_common.py.
Two further figures are not run unless named (EXTRA): the four components of the landmark score, and the sight lines of the three
top landmarks.
"""
from __future__ import annotations

import argparse
import colorsys
import sys
import time
from pathlib import Path

import numpy as np
import geopandas as gpd
import pyogrio
import shapely

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(REPO / "analysis" / "validation" / "Torino"))
import night_common as nc  # noqa: E402
from night_common import BG, FIG_H, FIG_W, INK, fs  # noqa: E402

import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.collections import LineCollection, PolyCollection  # noqa: E402
from matplotlib.colors import to_rgb  # noqa: E402

R = REPO / "src" / "main" / "resources" / "Barcelona" / "Barcelona_"
DEM = REPO / "inputData" / "Barcelona" / "Barcelona_DEM.tif"
OUT = HERE / "figures" / "barcelona"
STREET = "#2b2b2b"                      # streets under a coloured layer
WATER, PARK, RAIL, ROAD = nc.WATER_LINE, nc.PARK_LINE, "#ff972a", "#d9d9d9"
LANDMARK_RAMP = ["#1c1c1c", "#3b2a4f", "#7a2c6e", "#c23d5e", "#f0793a", "#ffd23f"]
VIEW_RAMP = ["#14283a", "#1d4f7a", "#2f86c9", "#6fc0f2", "#d6f0ff"]
RELIEF_RAMP = ["#000000", "#161614", "#2a2a26", "#45453f", "#64645c"]      # the terrain under the landmarks: dark enough for the buildings to stand out
SIGHT_COLOUR, SIGHT_ALPHA = "#ffb347", 0.020


def save(fig, name):
    OUT.mkdir(parents=True, exist_ok=True)
    for attempt in range(4):        # OneDrive sometimes refuses to overwrite a file it is syncing
        try:
            (OUT / f"{name}.png").unlink(missing_ok=True)
            fig.savefig(OUT / f"{name}.png", facecolor=BG)
            break
        except OSError:
            if attempt == 3:
                raise
            time.sleep(2)
    plt.close(fig)
    print("wrote", OUT / f"{name}.png")


class City:
    """The network, loaded once; map panels share its extent."""

    def __init__(self):
        self.edges = gpd.read_file(R.as_posix() + "edges.gpkg")
        self.nodes = gpd.read_file(R.as_posix() + "nodes.gpkg")
        self.segs = [np.asarray(g.coords)[:, :2] if g.geom_type == "LineString" else np.asarray(g.geoms[0].coords)[:, :2] for g in self.edges.geometry]
        minx, miny, maxx, maxy = self.nodes.total_bounds
        pad = 0.01 * max(maxx - minx, maxy - miny)
        self.box = (minx - pad, maxx + pad, miny - pad, maxy + pad)
        self.aspect = (self.box[1] - self.box[0]) / (self.box[3] - self.box[2])

    def panels(self, fig, n, y0=0.14, top=0.97, x0=0.03, gap=0.012, span=0.94):
        """n map panels in a row, as large as the row allows; returns the axes and (width, height)."""
        w = min((span - (n - 1) * gap) / n, (top - y0) * FIG_H / FIG_W * self.aspect)
        h = w * FIG_W / FIG_H / self.aspect
        axes = []
        for i in range(n):
            ax = fig.add_axes([x0 + i * (w + gap), top - h, w, h])
            ax.set_facecolor(BG); ax.set_xlim(self.box[0], self.box[1]); ax.set_ylim(self.box[2], self.box[3]); ax.set_aspect("equal"); ax.axis("off")
            axes.append(ax)
        return axes, (w, h)

    def streets(self, ax, colour=STREET, lw=0.5):
        ax.add_collection(LineCollection(self.segs, colors=colour, linewidths=lw, zorder=1))

    def relief(self, ax):
        """The terrain as a grey hillshade under everything else."""
        image, extent = terrain(self.box)
        ax.imshow(image, extent=extent, origin="upper", cmap=nc.colormap(RELIEF_RAMP), vmin=0, vmax=1, zorder=0, interpolation="bilinear")
        ax.set_xlim(self.box[0], self.box[1]); ax.set_ylim(self.box[2], self.box[3])


def terrain(box, step=2):
    """Hillshade and height of the DEM over the box (minx, maxx, miny, maxy): one array in 0..1 and its extent (left, right, bottom, top)."""
    import rasterio
    from matplotlib.colors import LightSource
    minx, maxx, miny, maxy = box
    with rasterio.open(DEM) as src:
        win = rasterio.windows.from_bounds(minx, miny, maxx, maxy, src.transform).round_offsets().round_lengths()
        z = src.read(1, window=win, out_shape=(int(win.height) // step, int(win.width) // step), resampling=rasterio.enums.Resampling.average, masked=True)
        left, bottom, right, top = src.window_bounds(win)
        cell = src.res[0] * step
    z = np.ma.filled(z.astype(float), np.nan)
    z = np.where(np.isfinite(z) & (z > -100), z, np.nanmin(z[z > -100]))
    shade = LightSource(azdeg=315, altdeg=38).hillshade(z, vert_exag=3.0, dx=cell, dy=cell)
    lo, hi = np.percentile(z, [1, 99])
    height = np.clip((z - lo) / (hi - lo), 0, 1)
    shade = np.clip((shade - np.percentile(shade, 2)) / (np.percentile(shade, 98) - np.percentile(shade, 2)), 0, 1)
    return (0.7 * shade + 0.3 * height) ** 2.0, (left, right, bottom, top)


def title(fig, text, sub=""):
    fig.text(0.03, 0.955, text, color=INK, fontsize=fs(26), ha="left", va="center")
    if sub:
        fig.text(0.03, 0.915, sub, color=INK, fontsize=fs(13), ha="left", va="center")


# ------------------------------------------------------------------------------------------------ districts | barriers
def hue_palette(n):
    """n colours well apart on black: golden-ratio hues, lightness alternating so neighbours differ."""
    return [colorsys.hsv_to_rgb((i * 0.618034) % 1, 0.62 if i % 2 else 0.8, 0.98 if i % 3 else 0.82) for i in range(n)]


def districts_barriers(city):
    district_of = dict(zip(city.nodes.nodeID, city.nodes.district))
    d = city.edges.u.map(district_of).fillna(-1).astype(int).to_numpy()
    ids = sorted(set(d) - {-1}); colours = dict(zip(ids, hue_palette(len(ids))))
    b = gpd.read_file(R.as_posix() + "barriers.gpkg")
    colour = {"water": WATER, "park": PARK, "railway": RAIL, "road": ROAD}
    width = {"water": 2.0, "park": 1.4, "railway": 1.8, "road": 0.9}
    label = {"water": "water", "park": "parks", "railway": "railways", "road": "main roads"}
    fig = plt.figure(figsize=(FIG_W, FIG_H), dpi=100, facecolor=BG)
    axes, (w, h) = city.panels(fig, 2)
    order = np.argsort(d, kind="stable")
    axes[0].add_collection(LineCollection([city.segs[i] for i in order], colors=[colours.get(d[i], STREET) for i in order], linewidths=0.9))
    city.streets(axes[1], "#2e2e2e", 0.5)
    fills = [(kind, nc.closed_outlines(b[b.barrier_type == kind].geometry), fill) for kind, fill in (("park", nc.PARK_FILL), ("water", nc.WATER_FILL))]
    for kind, polys, fill in fills:                            # what a closed outline encloses, filled
        if polys:
            axes[1].add_collection(PolyCollection([np.asarray(q.exterior.coords)[:, :2] for q in polys], facecolors=fill, edgecolors="none", zorder=2))
    for kind in ("road", "railway", "park", "water"):          # the rarest on top
        segs = []
        for g in b[b.barrier_type == kind].geometry:
            parts = g.geoms if g.geom_type == "MultiLineString" else [g]
            segs += [np.asarray(p.coords)[:, :2] for p in parts]
        axes[1].add_collection(LineCollection(segs, colors=colour[kind], linewidths=width[kind], zorder=3 + (kind != "road")))
    for ax in axes:
        ax.set_xlim(city.box[0], city.box[1]); ax.set_ylim(city.box[2], city.box[3])
    sizes = [int(np.sum(d == i)) for i in ids]
    fig.text(0.03, 0.095, f"{len(ids)} districts, from {min(sizes):,} to {max(sizes):,} streets", color=INK, fontsize=fs(14), ha="left", va="center")
    x2 = 0.03 + w + 0.012
    for i, kind in enumerate(("water", "park", "railway", "road")):
        fig.text(x2 + (i % 2) * 0.2, 0.11 - (i // 2) * 0.04, f"{label[kind]}: {int((b.barrier_type == kind).sum())}", color=colour[kind], fontsize=fs(14), ha="left", va="center")
    save(fig, "barcelona_districts_barriers")


# ------------------------------------------------------------------------------------------------ landmarks
def building_polys(b):
    return [np.asarray(g.exterior.coords)[:, :2] if g.geom_type == "Polygon" else np.asarray(max(g.geoms, key=lambda p: p.area).exterior.coords)[:, :2] for g in b.geometry]


def landmark_panel(ax, polys, score, cmap, highlight=0):
    order = np.argsort(np.nan_to_num(score), kind="stable")
    ax.add_collection(PolyCollection([polys[i] for i in order], facecolors=[cmap(float(np.nan_to_num(score[i]))) for i in order], edgecolors="none", zorder=2))
    if highlight:
        top = order[-highlight:]
        ax.add_collection(PolyCollection([polys[i] for i in top], facecolors="none", edgecolors=INK, linewidths=1.2, zorder=4))


def landmarks(city):
    b = gpd.read_file(R.as_posix() + "buildings.gpkg")
    polys = building_polys(b)
    cmap = nc.colormap(LANDMARK_RAMP)
    fig = plt.figure(figsize=(FIG_W, FIG_H), dpi=100, facecolor=BG)
    axes, (w, h) = city.panels(fig, 2)                     # the same size as the other Barcelona figures
    for ax, col in zip(axes, ("lScore_sc", "gScore_sc")):
        city.relief(ax)
        landmark_panel(ax, polys, b[col].to_numpy(), cmap, highlight=40)
    fig.text(0.03, 0.105, "local landmarkness: stands out from the buildings around it", color=INK, fontsize=fs(14), ha="left", va="center")
    fig.text(0.03 + w + 0.012, 0.105, "global landmarkness: seen from far away", color=INK, fontsize=fs(14), ha="left", va="center")
    nc.colour_bar(fig, plt.cm.ScalarMappable(norm=plt.Normalize(0, 1), cmap=cmap), 0.5, 0.3, [0, 0.5, 1], ["0  less salient", "0.5", "more salient  1"], "", "", "", y=0.04)
    save(fig, "barcelona_landmarks")


def landmarks_components(city):
    """Not run by default: the four components of the global score, each scaled 0 to 1."""
    b = gpd.read_file(R.as_posix() + "buildings.gpkg")
    polys = building_polys(b)
    cmap = nc.colormap(LANDMARK_RAMP)
    fig = plt.figure(figsize=(FIG_W, FIG_H), dpi=100, facecolor=BG)
    axes, (w, h) = city.panels(fig, 4, y0=0.2)
    parts = [("vScore_sc", "visual" + chr(10) + "facade, size, shape"), ("sScore_sc", "structural" + chr(10) + "height, area, isolation"),
             ("pScore_sc", "pragmatic" + chr(10) + "what the building is used for"), ("cScore_sc", "cultural" + chr(10) + "listed or tagged as heritage")]
    for i, (ax, (col, name)) in enumerate(zip(axes, parts)):
        city.relief(ax)
        landmark_panel(ax, polys, b[col].to_numpy(), cmap)
        fig.text(0.03 + i * (w + 0.012), 0.88 - h - 0.012, name, color=INK, fontsize=fs(15), ha="left", va="top", linespacing=1.3)
    nc.colour_bar(fig, plt.cm.ScalarMappable(norm=plt.Normalize(0, 1), cmap=cmap), 0.5, 0.32, [0, 0.5, 1], ["0", "0.5", "1"], "low", "high", "", y=0.05)
    save(fig, "barcelona_landmarks_components")


# ------------------------------------------------------------------------------------------------ sight lines | places
KINDS = [("food and drink", "#ffd23f"), ("shops", "#ff972a"), ("tourism", "#dd3eff"), ("leisure and sport", "#5cb69f"), ("offices", "#6fa8ff"), ("other amenities", "#7a7a7a")]
FOOD = {"restaurant", "cafe", "bar", "pub", "fast_food", "ice_cream", "biergarten", "food_court", "nightclub"}


def poi_kind(row):
    if isinstance(row.tourism, str):
        return "tourism"
    if isinstance(row.shop, str):
        return "shops"
    if isinstance(row.amenity, str):
        return "food and drink" if row.amenity in FOOD else "other amenities"
    if isinstance(row.leisure, str) or isinstance(row.sport, str):
        return "leisure and sport"
    if isinstance(row.office, str):
        return "offices"
    return None


def line_ends(geometry):
    """The first and last point of every line, as an (n, 2, 2) array."""
    xy = shapely.get_coordinates(geometry)
    counts = shapely.get_num_coordinates(geometry)
    first = np.cumsum(counts) - counts
    return np.stack([xy[first], xy[first + counts - 1]], axis=1)


def sight_lines_places(city):
    lines = pyogrio.read_dataframe(R.as_posix() + "sight_lines2D.gpkg")
    ends = line_ends(lines.geometry.values)
    p = gpd.read_file(R.as_posix() + "POIs.gpkg")
    p["kind"] = [poi_kind(r) for r in p.itertuples()]
    p = p.dropna(subset=["kind"])
    counts_kind = p.kind.value_counts()
    fig = plt.figure(figsize=(FIG_W, FIG_H), dpi=100, facecolor=BG)
    axes, (w, h) = city.panels(fig, 2)
    city.streets(axes[0], "#161616", 0.4)
    axes[0].add_collection(LineCollection(ends, colors=[(*to_rgb(SIGHT_COLOUR), SIGHT_ALPHA)], linewidths=0.3, zorder=2))
    city.streets(axes[1], "#1c1c1c", 0.4)
    for kind, colour in reversed(KINDS):
        sub = p[p.kind == kind]
        axes[1].scatter(sub.geometry.x, sub.geometry.y, s=3.2, color=colour, linewidths=0, alpha=0.9, zorder=3)
    for ax in axes:
        ax.set_xlim(city.box[0], city.box[1]); ax.set_ylim(city.box[2], city.box[3])
    x2 = 0.03 + w + 0.012
    for i, (kind, colour) in enumerate(KINDS):
        fig.text(x2 + (i % 2) * 0.2, 0.115 - (i // 2) * 0.04, f"{counts_kind.get(kind, 0):,}  {kind}", color=colour, fontsize=fs(14), ha="left", va="center")
    save(fig, "barcelona_sight_lines_places")


# ------------------------------------------------------------------------------------------------ sight lines of three landmarks (not run)
def sight_lines_detail(city):
    """Not run by default: the junctions that see each of the three top global landmarks, and how many buildings each junction sees."""
    attrs = pyogrio.read_dataframe(R.as_posix() + "sight_lines2D.gpkg", read_geometry=False)
    b = pyogrio.read_dataframe(R.as_posix() + "buildings.gpkg", read_geometry=False)
    bs = gpd.read_file(R.as_posix() + "buildings.gpkg")
    seen = attrs.groupby("nodeID").size()
    per_node = city.nodes.nodeID.map(seen).fillna(0).to_numpy()
    top = b.sort_values("gScore_sc", ascending=False).buildingID.head(3).tolist()
    lines = pyogrio.read_dataframe(R.as_posix() + "sight_lines2D.gpkg", where=f"buildingID IN ({','.join(map(str, top))})")
    fig = plt.figure(figsize=(FIG_W, FIG_H), dpi=100, facecolor=BG)
    axes, (w, h) = city.panels(fig, 4, y0=0.2, top=0.74, gap=0.02, span=0.94)
    cmap = nc.colormap(VIEW_RAMP)
    city.streets(axes[0], "#1c1c1c", 0.4)
    order = np.argsort(per_node, kind="stable")
    v = np.log10(1 + per_node)
    axes[0].scatter(city.nodes.geometry.x.to_numpy()[order], city.nodes.geometry.y.to_numpy()[order], c=v[order], s=3, cmap=cmap, vmin=0, vmax=np.percentile(v, 99), linewidths=0, zorder=3)
    fig.text(0.03, 0.74 - h - 0.012, "buildings in view" + chr(10) + "from each junction" + chr(10) + f"median {int(np.median(per_node[per_node > 0]))}, max {int(per_node.max())}", color=INK, fontsize=fs(14), ha="left", va="top", linespacing=1.35)
    for i, bid in enumerate(top, start=1):
        ax = axes[i]; city.streets(ax, "#1c1c1c", 0.4)
        sub = lines[lines.buildingID == bid]
        ax.add_collection(LineCollection([np.asarray(g.coords)[:, :2] for g in sub.geometry], colors="#ff972a", linewidths=0.25, alpha=0.55, zorder=2))
        row = bs[bs.buildingID == bid].iloc[0]
        ax.add_collection(PolyCollection([np.asarray(row.geometry.exterior.coords)[:, :2] if row.geometry.geom_type == "Polygon" else np.asarray(row.geometry.geoms[0].exterior.coords)[:, :2]],
                                         facecolors=INK, edgecolors=INK, linewidths=2.5, zorder=4))
        fig.text(0.03 + i * (w + 0.02), 0.74 - h - 0.012, f"global landmark #{i}" + chr(10) + f"{row.land_use}, score {row.gScore_sc:.2f}" + chr(10) + f"seen from {len(sub):,} junctions", color=INK,
                 fontsize=fs(14), ha="left", va="top", linespacing=1.35)
    save(fig, "barcelona_sight_lines")


STEPS = {"districts_barriers": districts_barriers, "landmarks": landmarks, "sightlines_places": sight_lines_places}
EXTRA = {"landmarks_components": landmarks_components, "sightlines_detail": sight_lines_detail}      # only when named


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", nargs="*", choices=list(STEPS) + list(EXTRA))
    only = ap.parse_args().only or list(STEPS)
    city = City()
    for name in only:
        (STEPS.get(name) or EXTRA[name])(city)


if __name__ == "__main__":
    main()
