"""One trip after dark, routed again as each term of the night cost is added, and the four terms across Turin.

    python night_route_terms.py                 # the trip whose terms all show most
    python night_route_terms.py --trip 4 --langs en
    python night_route_terms.py --scores        # only rank the kept trips

Reads data/terms/cost_terms.json and cost_terms_edges.csv, which scripts/NightCostTermRoutes.java writes on gdsl1:

    java -Xmx16g -cp "<overlay>:target/classes:$(cat cp.txt)" scripts/NightCostTermRoutes.java \\
      --cityName=Torino --out=cost_terms.json --edges=cost_terms_edges.csv --hour=22 --pairs=4000 --keep=12

Writes, into figures/night/images/route_terms/:
    route_terms_<lang>_1.png, _2.png     the trip's steps 1-3 and 4-6, with captions
    terms_four_maps_<lang>.png           the four terms across Turin, with legends
    terms_map_{light,parks_rivers,enclosure,open}.png    the four maps alone, no text
    panels/terms_<n>_<step>.png          the six route panels alone
The trips are kept because every term changes the route, so they show what a term can do, not how often it does.
"""
from __future__ import annotations

import argparse
import time
import json
import sys

import geopandas as gpd
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.colors
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection, PolyCollection
from matplotlib.colors import to_rgba
from matplotlib.patches import Rectangle
from PIL import Image

import night_common as nc
from night_common import AXIS, BG, FIG_H, FIG_W, GRID, HERE, IMAGES, INK, REPO, fs, style_axis

sys.path.insert(0, str(REPO / "scripts"))
import render_cost_terms as rct  # noqa: E402  (the site's renderer; its drawing is used, with our colours)

CITY = "Torino"
TERMS = HERE / "data" / "terms"
OUT = IMAGES / "route_terms"
PANELS = OUT / "panels"
PANEL_PX = 900
GREEN, BLUE = nc.PARK_LINE, nc.WATER_LINE                      # parks, rivers: shaded, as in Barcelona
ORIGINAL, AFFECTED, MARK, MAN = "#2fa7ff", "#ff233f", "#ffffff", "#ffc83d"    # the daylight route blue, the route the terms move red, the trip's ends white, the man's route amber
ENC_RAMP = matplotlib.colors.LinearSegmentedColormap.from_list("enclosure", ["#2a1a52", "#5a35a8", "#8a5cf0", "#bf9cff", "#efe3ff"])   # enclosed: violet, brighter = more enclosed
ENC_BAR = matplotlib.colors.LinearSegmentedColormap.from_list("e", ENC_RAMP(np.linspace(0.2, 1, 64)).tolist())
OPEN_RAMP = matplotlib.colors.LinearSegmentedColormap.from_list("venues", ["#14424b", "#23808c", "#3fc0c9", "#9eeee8", "#e6fffb"])    # venues open: teal, brighter = more
OPEN_BAR = matplotlib.colors.LinearSegmentedColormap.from_list("o", OPEN_RAMP(np.linspace(0.2, 1, 64)).tolist())
NL = chr(10)
STEP_MAP = {"darkness": "light", "parksRivers": "parks_rivers", "enclosure": "enclosure", "emptyStreet": "open"}   # route step -> the map of its term

rct.BACKGROUND, rct.STREET = BG, "#3a3a42"


def save(fig, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    for attempt in range(4):        # OneDrive sometimes refuses to overwrite a file it is syncing
        try:
            path.unlink(missing_ok=True)
            fig.savefig(path, facecolor=BG)
            break
        except OSError:
            if attempt == 3:
                raise
            time.sleep(2)
    plt.close(fig)
    print("wrote", path)


def streets_table():
    streets = gpd.read_file(nc.REPO / "src" / "main" / "resources" / CITY / f"{CITY}_edges.gpkg")[["edgeID", "w_parks", "a_rivers", "geometry"]].set_index("edgeID")
    streets["length_m"] = streets.geometry.length
    streets = streets.join(pd.read_csv(TERMS / "cost_terms_edges.csv").set_index("edgeID"))
    streets["park"] = (streets.w_parks.astype(str) != "[]").astype(int)
    streets["river"] = (streets.a_rivers.astype(str) != "[]").astype(int)
    eyes = streets.eyes.fillna(0).clip(0, 0.9995)
    # eyes = 1 - exp(-density / 2), the venues per 100 m of street (length floored at 50 m): invert it for the count
    streets["places_open"] = np.where(eyes <= 0, 0, np.rint(-2.0 * np.log(1 - eyes) * np.maximum(streets.length_m, 50.0) / 100.0)).astype(int)
    e = streets.enclosure
    streets["enc_class"] = np.select([e >= 0.75, e >= 0.5, e >= 0.25, e > 0], [4, 3, 2, 1], 0)
    p = streets.places_open
    streets["open_class"] = np.select([p >= 5, p >= 3, p == 2, p == 1], [4, 3, 2, 1], 0)
    return streets


_FILLS = {}


def add_fills(ax, box=None):
    """The parks and water whose outlines close, filled and shaded, under everything else (inside `box` when given)."""
    import shapely.geometry
    if not _FILLS:
        barriers = gpd.read_file(nc.REPO / "src" / "main" / "resources" / CITY / f"{CITY}_barriers.gpkg")
        for kind, fill in (("park", nc.PARK_FILL), ("water", nc.WATER_FILL)):
            _FILLS[kind] = (nc.closed_outlines(barriers[barriers.barrier_type == kind].geometry), fill)
    window = shapely.geometry.box(*box) if box is not None else None
    for polys, fill in _FILLS.values():
        verts = [np.asarray(q.exterior.coords)[:, :2] for q in polys if window is None or q.intersects(window)]
        if verts:
            ax.add_collection(PolyCollection(verts, facecolors=fill, edgecolors="none", zorder=0))


def paint_streets(ax, geoms, colours, width, glow, order, halo=4.2):
    """The streets in drawing order, the strongest with a soft halo under them."""
    lines, cols, widths, glows, glow_cols = [], [], [], [], []
    for j in order:
        for seg in geoms[j]:
            lines.append(seg); cols.append(colours[j]); widths.append(width[j])
            if glow[j]:
                glows.append(seg); glow_cols.append((*colours[j][:3], 0.16))
    ax.add_collection(LineCollection(glows, colors=glow_cols, linewidths=halo, capstyle="round"))
    ax.add_collection(LineCollection(lines, colors=cols, linewidths=widths, capstyle="round"))


def draw(ax, edges, streets, candidate, step_index, box):
    """One panel: the streets coloured by the term just added, on the scale of the whole city (the legend under the panel);
    the daylight route white on its own, blue as the reference from the second panel on, and the route the terms have moved in red."""
    routes = candidate["routes"]
    name = STEP_MAP.get(routes[step_index]["step"])
    geoms = [rct.segments(g) for g in streets.geometry]
    if name:
        colours, width, glow, order = city_colours(name, streets, city_grades(streets))
    else:
        colours, width, glow, order = np.tile(to_rgba(rct.STREET), (len(streets), 1)), np.full(len(streets), 0.9), np.zeros(len(streets), bool), np.arange(len(streets))
    if name == "parks_rivers":
        add_fills(ax, box)
    paint_streets(ax, geoms, colours, np.maximum(width, 0.9), glow, order)
    original = rct.route_lines(edges, routes[0]["edges"])
    ax.add_collection(LineCollection(original, colors=BG, linewidths=7.5, capstyle="round", zorder=5))
    if routes[step_index]["step"] == "nonVulnerable":
        woman = rct.route_lines(edges, routes[step_index - 1]["edges"])       # the route of panel 5: every weight 1.0
        man = rct.route_lines(edges, routes[step_index]["edges"])             # the same cost at half the weights
        ax.add_collection(LineCollection(woman, colors=BG, linewidths=8.5, capstyle="round", zorder=5))
        ax.add_collection(LineCollection(original, colors=ORIGINAL, linewidths=6.4, capstyle="round", zorder=6))
        ax.add_collection(LineCollection(woman, colors=AFFECTED, linewidths=4.2, capstyle="round", zorder=7))
        ax.add_collection(LineCollection(man, colors=MAN, linewidths=2.0, capstyle="round", zorder=8))             # the narrowest on top: all three stay visible where they share a street
    elif step_index > 0:
        moved = rct.route_lines(edges, routes[step_index]["edges"])
        ax.add_collection(LineCollection(moved, colors=BG, linewidths=7.5, capstyle="round", zorder=5))
        ax.add_collection(LineCollection(original, colors=ORIGINAL, linewidths=4.8, capstyle="round", zorder=6))
        ax.add_collection(LineCollection(moved, colors=AFFECTED, linewidths=2.6, capstyle="round", zorder=7))     # on top: where they share a street it shows red with a blue edge
    else:
        ax.add_collection(LineCollection(original, colors=MARK, linewidths=3.4, capstyle="round", zorder=6))      # the daylight route on its own: white
    first, last = edges.loc[routes[0]["edges"][0]].geometry, edges.loc[routes[0]["edges"][-1]].geometry
    for (x, y), filled in zip(rct.endpoints(first, last, routes[0]["edges"], edges), (True, False)):
        ax.scatter([x], [y], s=110, zorder=9, color=MARK if filled else BG, edgecolors=MARK, linewidths=2.2)
    minx, miny, maxx, maxy = box
    x0, y0 = minx + 0.04 * (maxx - minx), miny + 0.04 * (maxx - minx)
    ax.plot([x0, x0 + rct.SCALE_METRES], [y0, y0], color="#bdbdbd", linewidth=2.0, solid_capstyle="butt", zorder=10)
    ax.set_xlim(minx, maxx); ax.set_ylim(miny, maxy); ax.set_aspect("equal"); ax.axis("off")


rct.draw = draw


def route_change(streets, a, b):
    """The share of the metres of two routes that only one of them uses."""
    length = streets.length_m
    sa, sb = set(a["edges"]), set(b["edges"])
    union = length.reindex(sorted(sa | sb)).sum()
    return float(length.reindex(sorted(sa ^ sb)).sum() / union) if union else 0.0


def trip_scores(found, streets):
    """Per kept trip: how much each added term moves the route (smallest of the four), how far the last route is from the first, and
    the share of the first route that is dark (counted double: the darkness panel needs dark streets on the way). The best trip is the one where every term shows."""
    rows = []
    for k, c in enumerate(found["candidates"]):
        r = c["routes"]
        step = min(route_change(streets, r[i - 1], r[i]) for i in range(1, 5))
        far = min(route_change(streets, r[0], r[i]) for i in range(1, 5))
        dark = r[0]["under10lux"] / max(r[0]["metres"], 1)
        rows.append({"trip": k, "each_term_moves": step, "from_the_first": far, "dark_share": dark, "score": step + 0.5 * far + 2 * dark})
    return pd.DataFrame(rows).sort_values("score", ascending=False)


def choose_trip(found, streets, trip):
    scores = trip_scores(found, streets)
    print(scores.round(2).to_string(index=False))
    return int(scores.trip.iloc[0]) if trip is None else trip


TEXT = {
    "en": dict(title="One trip after dark, one term of the cost at a time", guide="How to read it",
               names=["1  The daylight route", "2  + Darkness", "3  + Parks and rivers", "4  + Enclosure", "5  + An empty street", "6  The same trip, a man"],
               dark="under 10 lx", park="in parks or along rivers", encl="on enclosed streets", empty="on streets with nothing open", halved="the whole cost, every weight 0.5 instead of 1.0",
               notes=["Turin, {h}:00", "white: the daylight route, on its own", "blue: the same route as the reference in the other panels", "red: the route once the terms shown so far are added",
                      "panel 6: red is the woman's route of panel 5 (weights 1.0)", "panel 6: amber is the man's route (weights 0.5)", "reference light: 17.5 lx (middle of 8.9-26)", "the bar in each panel: {s} m"],
               cost=["cost of a street =", "length", "x (1 + darkness)", "x (1 + park or river)", "x (1 + enclosure)", "x (1 + empty street)"],
               avg_title="What each term does, on average over the trips", steps=["shortest", "+ dark", "+ parks", "+ enclosure", "+ empty", "a man"],
               metrics=["Length of the route (m)", "Metres under 10 lx", "Metres in parks or along rivers", "Metres on enclosed streets", "Metres on streets with nothing open"],
               legend=["grey: shortest route", "red: woman (weights 1.0)", "blue: man (weights 0.5)"],
               caveat="{n} trips chosen because every term moves the route:" + NL + "they show what a term does when it acts," + NL + "not how often it acts.",
               maps_title="Street by street: the four terms across Turin", maps=["Light", "Parks and rivers", "Enclosure", "Open at {h}:00"],
               stats=["{lit10:.0%} of street length" + NL + "gets 10 lux or more", "{park_river:.0%} of street length" + NL + "in a park or along a river",
                      "{enclosed:.0%} of street length" + NL + "is enclosed", "{nothing} streets in 20" + NL + "have nothing open"],
               legends=[["0", "5", "10", "20", "30+ lux"], ["park", "river", "other"], ["view < 5,000 m2", "< 3,750", "< 2,500", "< 1,250"], ["1 venue", "2", "3-4", "5+", "none"]]),
}


def trip_figures(found, streets, trip, langs):
    cand = found["candidates"][trip]
    box = rct.window(streets, cand)
    inside = streets.cx[box[0]:box[2], box[1]:box[3]]
    P = []
    for i, r in enumerate(cand["routes"]):
        fig, ax, dpi = rct.figure(PANEL_PX)
        rct.draw(ax, streets, inside, cand, i, box)
        name = f"terms_{i}_{r['step']}.png"
        PANELS.mkdir(parents=True, exist_ok=True)
        fig.savefig(PANELS / name, dpi=dpi, facecolor=BG)
        plt.close(fig)
        P.append({k: (round(v) if isinstance(v, float) else v) for k, v in r.items() if k != "edges"} | {"image": name})
    hour, scale = found["hour"], rct.SCALE_METRES

    def change(a, b):
        return f"{(b - a) / a:+.0%}"

    for lang in langs:
        T = TEXT[lang]
        captions = [f"{P[0]['metres']} m" + NL + f"{P[0]['under10lux']} m {T['dark']}",
                    f"{P[1]['metres']} m ({change(P[0]['metres'], P[1]['metres'])})" + NL + f"{T['dark']}: {P[0]['under10lux']} -> {P[1]['under10lux']} m",
                    f"{P[2]['metres']} m ({change(P[0]['metres'], P[2]['metres'])})" + NL + f"{T['park']}: {P[1]['parksRivers']} -> {P[2]['parksRivers']} m",
                    f"{P[3]['metres']} m ({change(P[0]['metres'], P[3]['metres'])})" + NL + f"{T['encl']}: {P[2]['enclosed']} -> {P[3]['enclosed']} m",
                    f"{P[4]['metres']} m ({change(P[0]['metres'], P[4]['metres'])})" + NL + f"{T['empty']}: {P[3]['empty']} -> {P[4]['empty']} m",
                    f"{P[5]['metres']} m ({change(P[0]['metres'], P[5]['metres'])}), a woman's: {P[4]['metres']} m ({change(P[0]['metres'], P[4]['metres'])})" + NL + T["halved"]]
        for part in (0, 1):
            fig = plt.figure(figsize=(FIG_W, FIG_H), dpi=100, facecolor=BG)
            pw, ph = 0.3, 0.3 * FIG_W / FIG_H
            for col in range(3):
                i = part * 3 + col
                x = 0.03 + col * (pw + 0.025)
                ax = fig.add_axes([x, 0.36, pw, ph]); ax.axis("off")
                ax.imshow(Image.open(PANELS / P[i]["image"]))
                fig.text(x, 0.345, T["names"][i], color=INK, fontsize=fs(19), ha="left", va="top")
                fig.text(x, 0.285, captions[i], color=INK, fontsize=fs(14), ha="left", va="top", linespacing=1.4)
                factor_legend(fig, cand["routes"][i]["step"], x, 0.13, pw * 0.8, P)
            save(fig, OUT / f"route_terms_{lang}_{part + 1}.png")
        # the notes and the cost formula on their own line, bottom of the second image is already full: one more small page
        fig = plt.figure(figsize=(FIG_W, FIG_H * 0.5), dpi=100, facecolor=BG)
        fig.text(0.03, 0.85, T["guide"], color=INK, fontsize=fs(24), ha="left", va="center")
        for k, line in enumerate(T["notes"]):
            fig.text(0.03, 0.72 - k * 0.088, line.format(h=hour, s=scale), color=INK, fontsize=fs(16), ha="left", va="center")
        fig.text(0.55, 0.85, T["cost"][0], color=INK, fontsize=fs(22), ha="left", va="center")
        for k, line in enumerate(T["cost"][1:]):
            fig.text(0.55, 0.68 - k * 0.12, line, color=INK, fontsize=fs(18), ha="left", va="center")
        save(fig, OUT / f"route_terms_{lang}_notes.png")
    return P


LUX_TOP = 30.0
VENUES_TOP = 8.0


def city_grades(streets):
    """Continuous 0..1 grades of the four terms over the whole city, and the drawing order of each."""
    lux = streets.lux.fillna(0).to_numpy()
    e = streets.enclosure.fillna(0).to_numpy()
    p = streets.places_open.to_numpy(float)
    return {"light": np.sqrt(np.clip(lux / LUX_TOP, 0, 1)), "enclosure": np.clip(e, 0, 1), "open": np.clip(np.log1p(p) / np.log1p(VENUES_TOP), 0, 1)}


def city_colours(name, streets, t):
    n = len(streets)
    colours, width, glow = np.tile(to_rgba(rct.STREET), (n, 1)), np.full(n, 0.55), np.zeros(n, bool)
    if name == "light":
        colours = rct.LUX_RAMP(t["light"]); width = 0.5 + 1.0 * t["light"]; glow = t["light"] > 0.55
        order = np.argsort(t["light"], kind="stable")
    elif name == "parks_rivers":
        park, river = streets.park.to_numpy() == 1, streets.river.to_numpy() == 1
        colours[park] = to_rgba(GREEN); colours[river] = to_rgba(BLUE)
        width = np.where(park | river, 1.5, 0.55); glow = park | river
        order = np.argsort((park | river).astype(int), kind="stable")
    elif name == "enclosure":
        on = t["enclosure"] > 0
        colours[on] = ENC_RAMP(0.2 + 0.8 * t["enclosure"][on]); width = np.where(on, 0.8 + 1.2 * t["enclosure"], 0.55); glow = t["enclosure"] > 0.5
        order = np.argsort(t["enclosure"], kind="stable")
    else:
        on = t["open"] > 0
        colours[on] = OPEN_RAMP(0.2 + 0.8 * t["open"][on]); width = np.where(on, 0.8 + 1.2 * t["open"], 0.55); glow = t["open"] > 0.6
        order = np.argsort(t["open"], kind="stable")
    return colours, width, glow, order


def gradient_bar(fig, x, y, w, cmap, ticks, labels, title, h=0.016):
    """A horizontal colour bar with labels at ticks (0..1 along the bar) and a title above it."""
    ax = fig.add_axes([x, y, w, h]); ax.axis("off")
    ax.imshow(np.linspace(0, 1, 256)[None, :], aspect="auto", cmap=cmap, extent=(0, 1, 0, 1))
    fig.text(x, y + h + 0.012, title, color=INK, fontsize=fs(13), ha="left", va="bottom")
    for tk, lb in zip(ticks, labels):
        fig.text(x + tk * w, y - 0.012, lb, color=INK, fontsize=fs(12), ha="center", va="top")


def factor_legend(fig, step, x, y, w, routes=None):
    """The legend of the term a panel shows: a colour bar with its numbers, or the swatches of parks and rivers;
    under the last panel, the three routes it draws with their lengths."""
    if step == "nonVulnerable" and routes:
        for k, (label, colour) in enumerate(((f"daylight route, {routes[0]['metres']} m", ORIGINAL), (f"woman, {routes[4]['metres']} m", AFFECTED), (f"man, {routes[5]['metres']} m", MAN))):
            fig.patches.append(Rectangle((x, y + 0.03 - k * 0.03), 0.012, 0.016, transform=fig.transFigure, facecolor=colour, edgecolor="none"))
            fig.text(x + 0.018, y + 0.038 - k * 0.03, label, color=INK, fontsize=fs(13), ha="left", va="center")
    elif step == "darkness":
        gradient_bar(fig, x, y, w, rct.LUX_RAMP, [np.sqrt(v / LUX_TOP) for v in (0, 5, 10, 20, 30)], ["0", "5", "10", "20", "30+"], "mean lux of the street")
    elif step == "parksRivers":
        for k, (label, colour) in enumerate((("park", GREEN), ("river", BLUE), ("other street", rct.STREET))):
            fig.patches.append(Rectangle((x + k * 0.07, y), 0.012, 0.016, transform=fig.transFigure, facecolor=colour, edgecolor="none"))
            fig.text(x + k * 0.07 + 0.016, y + 0.008, label, color=INK, fontsize=fs(12), ha="left", va="center")
    elif step == "enclosure":
        gradient_bar(fig, x, y, w, ENC_BAR, [0, 0.5, 1], ["5,000", "2,500", "0"], "area in view (m2)")
    elif step == "emptyStreet":
        gradient_bar(fig, x, y, w, OPEN_BAR, [np.log1p(v) / np.log1p(VENUES_TOP) for v in (1, 3, 8)], ["1", "3", "8+"], "venues open")


def four_maps(found, streets, langs):
    total_m = streets.length_m.sum()
    lit = gpd.read_file(nc.RESOURCES / f"{nc.CITY}_edges_illuminated_continuous.gpkg")    # the same layer and rule as turin_light, so the two figures agree (70% / 30%)
    share = {"lit10": 1 - float(lit.length[lit.mean_lux < 10].sum() / lit.length.sum()), "park_river": streets.length_m[(streets.park + streets.river) > 0].sum() / total_m,
             "enclosed": streets.length_m[streets.enclosure > 0].sum() / total_m, "nothing": 19 if (streets.places_open == 0).mean() > 0.94 else round(1 / (1 - (streets.places_open == 0).mean()))}
    print({k: (f"{v:.0%}" if k != "nothing" else v) for k, v in share.items()}, f"{total_m / 1000:,.0f} km of street")
    minx, miny, maxx, maxy = streets.total_bounds
    pad = 0.02 * max(maxx - minx, maxy - miny)
    w, h = (maxx - minx) + 2 * pad, (maxy - miny) + 2 * pad
    geoms = [rct.segments(g) for g in streets.geometry]
    t = city_grades(streets)
    names = ["light", "parks_rivers", "enclosure", "open"]
    for name in names:
        colours, width, glow, order = city_colours(name, streets, t)
        fig = plt.figure(figsize=(6.6, 6.6 * h / w), dpi=200, facecolor=BG)
        ax = fig.add_axes([0, 0, 1, 1]); ax.set_facecolor(BG)
        if name == "parks_rivers":
            add_fills(ax)
        paint_streets(ax, geoms, colours, width, glow, order)
        ax.set_xlim(minx - pad, maxx + pad); ax.set_ylim(miny - pad, maxy + pad); ax.set_aspect("equal"); ax.axis("off")
        fig.savefig(OUT / f"terms_map_{name}.png", dpi=200, facecolor=BG)
        plt.close(fig)
    for lang in langs:
        T = TEXT[lang]
        fig = plt.figure(figsize=(FIG_W, FIG_H), dpi=100, facecolor=BG)
        pw = 0.225
        for i, name in enumerate(names):
            x = 0.03 + i * (pw + 0.0125)
            img = Image.open(OUT / f"terms_map_{name}.png")
            ph = pw * FIG_W / FIG_H * img.height / img.width
            ax = fig.add_axes([x, 0.36, pw, ph]); ax.axis("off"); ax.imshow(img)
            fig.text(x, 0.36 + ph + 0.012, T["maps"][i].format(h=found["hour"]), color=INK, fontsize=fs(21), ha="left", va="bottom")
            fig.text(x, 0.335, T["stats"][i].format(**share), color=INK, fontsize=fs(14), ha="left", va="top", linespacing=1.4)
        for i, step in enumerate(("darkness", "parksRivers", "enclosure", "emptyStreet")):
            factor_legend(fig, step, 0.03 + i * (pw + 0.0125), 0.17, pw * 0.86)
        save(fig, OUT / f"terms_four_maps_{lang}.png")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--trip", type=int, help="which kept trip to draw (default: the one whose smallest term effect is largest)")
    parser.add_argument("--scores", action="store_true", help="only rank the kept trips")
    parser.add_argument("--langs", nargs="*", default=["en"], choices=["en"])
    parser.add_argument("--only", nargs="*", choices=["trip", "maps"], default=["trip", "maps"])
    args = parser.parse_args()
    found = json.loads((TERMS / "cost_terms.json").read_text())
    streets = streets_table()
    trip = choose_trip(found, streets, args.trip)
    print("trip drawn:", trip)
    if args.scores:
        return
    if "trip" in args.only:
        trip_figures(found, streets, trip, args.langs)
    if "maps" in args.only:
        four_maps(found, streets, args.langs)


if __name__ == "__main__":
    main()
