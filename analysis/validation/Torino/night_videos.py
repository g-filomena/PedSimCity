"""Every night video, into figures/night/videos/<observed|simulated|validation_cells|validation_streets>/ (no still is written next to a video).

    python night_videos.py                       # all of them, 4 at a time
    python night_videos.py --only observed_byGender validation_cells_women_winter
    python night_videos.py --list
    python night_videos.py --only simulated_volume_cells --still C:/tmp      # one still of a video's frame, to check a design

Videos (names are the file names):
    observed_byGender            share of women among Vodafone's passers-by, cell by cell, four seasons, band by band
    simulated_byGender_cells     the same for the model (vulnerable agents = women), on the Vodafone cells
    simulated_byGender_network   the model on the street network: colour = share of women against the average, width = pedestrians per hour, the busiest on top
    observed_volume              Vodafone passers-by, share of the band's total per cell (green)
    simulated_volume_cells       the model's share of the volume per cell (green)
    simulated_volume_network     the model's volume per hour on each street (green)
    simulated_volume_change      each season against winter: the change in pedestrians per hour on each street
    validation_cells_<variant>_<season>    observed | simulated | simulated over observed in nine classes, the ten largest differences ringed
    validation_streets_<variant>_<season>  each street segment coloured by the discrepancy of the cell it crosses
with variant = total (green), women (red) or men (blue), made for the winter only (VALIDATION_SEASONS).
"""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
import pandas as pd
from matplotlib.collections import LineCollection, PolyCollection
from matplotlib.colors import LogNorm, Normalize, PowerNorm

import night_common as nc
from night_common import (BAND_KEYS, BAND_LABEL, BG, BOUNDARY, DIVERGING, INK, MIN_OBS, MIN_SIM, MIN_SIM_FLOOR, NO_DATA, RESIDUAL,
                          SEASON_LABEL, SEASON_ORDER, STREET, VARIANTS, colormap, colour_bar, fs, new_figure, video_path, write_video)

STILL_BAND = "20_24"
VALIDATION_SEASONS = ["winter"]       # the validation videos are made for these seasons
SPREAD = 0.09           # share of women: the colour scale spans +-9 points around the average
EDGE_SPREAD = 0.12      # ... and +-12 on the streets
RINGED = 10             # cells ringed in the discrepancy panel: where observed and simulated shares differ most
WIDTH = (0.5, 3.6)      # street widths, thinnest to widest
CHANGE_TOP = 40.0      # pedestrians per hour of change at the ends of the season-against-winter scale
VOLUME_TOP = 500.0      # pedestrians per hour on a street at the top of the volume scale, the same in every band so that the bands compare
NL = chr(10)


def width_of(volume):
    """Street width from its volume per hour: it rises fast for the quiet streets and slowly for the busiest, up to VOLUME_TOP."""
    return WIDTH[0] + (WIDTH[1] - WIDTH[0]) * np.clip(np.asarray(volume, float) / VOLUME_TOP, 0, 1) ** 0.4


def width_key(fig, x, y):
    """Three sample widths with the volumes they stand for."""
    from matplotlib.lines import Line2D
    fig.text(x, y + 0.045, "street width: pedestrians per hour", color=INK, fontsize=fs(15), ha="left", va="bottom")
    for k, v in enumerate((10, 100, int(VOLUME_TOP))):
        xk = x + k * 0.062
        fig.add_artist(Line2D([xk, xk + 0.022], [y, y], transform=fig.transFigure, linewidth=float(width_of(v)), color="#bdbdbd", solid_capstyle="butt"))
        fig.text(xk + 0.027, y, f"{v:,}", color=INK, fontsize=fs(15), ha="left", va="center")


def short(v: float) -> str:
    """A legend number: two significant figures below ten, whole numbers above."""
    return f"{v:,.0f}" if v >= 10 else f"{v:.2g}"


def pct(v: float) -> str:
    """A share in percent, two decimals below 10 and one above."""
    return f"{v:.2f}%" if v < 10 else f"{v:.1f}%"


def women_at(base: float, shift: float) -> str:
    """What a point of the share-of-women scale means in absolute terms: the share of women there."""
    return f"{100 * (base + shift):.1f}% women"


def count(n: float, unit: str) -> str:
    n = max(round(n), 1)
    return f"about {n:,} {unit if n != 1 else unit.rstrip('s')}"


def segments(geometry):
    if geometry.geom_type == "LineString":
        return [np.asarray(geometry.coords)[:, :2]]
    return [np.asarray(part.coords)[:, :2] for part in geometry.geoms]


class Job:
    """One video: builds its figure, and either writes the video or, with a still directory, one frame."""

    def __init__(self, name, still=None):
        self.name, self.still = name, still
        self.cache = nc.load_cache()
        self.grid = self.cache["grid"]
        self.outline = nc.city_boundary(self.grid)
        self.verts = [np.asarray(g.exterior.coords) for g in self.grid.geometry]

    def finish(self, fig, draw):
        if self.still:
            draw(BAND_KEYS.index(STILL_BAND))
            Path(self.still).mkdir(parents=True, exist_ok=True)
            fig.savefig(Path(self.still) / f"{self.name}.png", facecolor=BG)
            print("still", self.name)
        else:
            write_video(fig, draw, len(BAND_KEYS), video_path(self.name))

    # ------------------------------------------------------------------------ four seasons side by side
    def season_panels(self, bounds, outline):
        fig, axes, headline, geometry = new_figure(bounds, 4, outline=outline)
        x0, w, gap, top, h = geometry
        for i, (ax, season) in enumerate(zip(axes, SEASON_ORDER)):
            fig.text(x0 + i * (w + gap) + 0.002, top + 0.012, SEASON_LABEL[season], ha="left", va="bottom", color=INK, fontsize=fs(19))
        return fig, axes, headline, geometry

    def obs_table(self, season, prefix):
        obs = self.cache["obs"][season]
        return obs[[f"{prefix}_{b}" for b in BAND_KEYS]].set_axis(BAND_KEYS, axis=1).fillna(0)

    def by_gender_cells(self, side):
        cache, base = self.cache, (self.cache["women_share_obs"] if side == "obs" else self.cache["women_share_sim"])
        women = {s: self.obs_table(s, "passF") if side == "obs" else cache["sim"][s]["cell_vuln"] for s in SEASON_ORDER}
        men = {s: self.obs_table(s, "passM") if side == "obs" else cache["sim"][s]["cell_norm"] for s in SEASON_ORDER}
        fig, axes, headline, (x0, w, gap, top, h) = self.season_panels(self.grid.total_bounds, self.outline)
        cmap, norm = colormap(DIVERGING), Normalize(vmin=-SPREAD, vmax=SPREAD)
        colls, readouts = {}, {}
        for i, (ax, season) in enumerate(zip(axes, SEASON_ORDER)):
            colls[season] = PolyCollection(self.verts, cmap=cmap, norm=norm, linewidths=0)
            ax.add_collection(colls[season])
            readouts[season] = fig.text(x0 + i * (w + gap) + 0.002, top - h - 0.012, "", ha="left", va="top", color=INK, fontsize=fs(17))
        low, high = nc.end_bar(fig, 0.5, 0.52, cmap, y=0.20, above=("more men", "more women"),
                               caption=f"share of women compared with the average ({base:.1%})" + ("  -  observed" if side == "obs" else "  -  model"))

        def draw(i):
            key = BAND_KEYS[i]
            headline.set_text(BAND_LABEL[key])
            values, totals = {}, {}
            for season in SEASON_ORDER:
                f, m = women[season][key].to_numpy(float), men[season][key].to_numpy(float)
                total = f + m
                bar = MIN_OBS if side == "obs" else (min(MIN_SIM, max(MIN_SIM_FLOOR, 0.5 * total[total > 0].mean())) if (total > 0).any() else MIN_SIM)
                share = f / np.where(total > 0, total, np.nan)
                values[season], totals[season] = np.where(total >= bar, share - base, np.nan), total
            spread = nc.spread_of(np.concatenate(list(values.values())))        # the scale follows the band: its ends are the largest differences shown
            norm.vmin, norm.vmax = -spread, spread
            low.set_text(f"{-100 * spread:+.1f}%" + chr(10) + women_at(base, -spread)); high.set_text(f"{100 * spread:+.1f}%" + chr(10) + women_at(base, spread))
            for season in SEASON_ORDER:
                value, total = values[season], totals[season]
                colls[season].set_array(np.ma.masked_invalid(value))
                shown = np.isfinite(value)
                overall = np.average(value[shown], weights=total[shown]) + base if shown.any() else np.nan
                readouts[season].set_text(f"{overall:.1%} women")
        self.finish(fig, draw)

    # ------------------------------------------------------------------------ the street network
    def by_gender_network(self):
        """Colour = share of women against the model average; width = pedestrians per hour on the street, on one scale for every band.
        Only the busier streets are drawn (a share needs people), the busiest on top; a line under each map gives the share of women over the whole network."""
        cache = self.cache
        edges = cache["edges"]
        segs = [np.asarray(g.coords)[:, :2] for g in edges.geometry]
        index = edges.edgeID.to_numpy()
        fig, axes, headline, (x0, w, gap, top, h) = self.season_panels(edges.total_bounds, self.outline)
        cmap, norm = colormap(DIVERGING), Normalize(vmin=-EDGE_SPREAD, vmax=EDGE_SPREAD)
        flows, readouts = {}, {}
        for i, (ax, season) in enumerate(zip(axes, SEASON_ORDER)):
            ax.add_collection(LineCollection(segs, colors=STREET, linewidths=0.35, zorder=1))
            flows[season] = LineCollection([], cmap=cmap, norm=norm, zorder=2, capstyle="round", joinstyle="round")
            ax.add_collection(flows[season])
            readouts[season] = fig.text(x0 + i * (w + gap) + 0.002, top - h - 0.012, "", ha="left", va="top", color=INK, fontsize=fs(17))
        base = cache["women_share_sim"]
        low, high = nc.end_bar(fig, 0.4, 0.52, cmap, y=0.20, above=("more men", "more women"),
                               caption=f"share of women compared with the model average ({base:.1%})")
        width_key(fig, 0.72, 0.20)

        def draw(i):
            key = BAND_KEYS[i]
            headline.set_text(BAND_LABEL[key])
            hours = nc.BAND_HOURS[key]
            drawn = {}
            for season in SEASON_ORDER:
                f = cache["sim"][season]["edge_vuln"][key].reindex(index).fillna(0).to_numpy() / 7.0 / hours
                m = cache["sim"][season]["edge_norm"][key].reindex(index).fillna(0).to_numpy() / 7.0 / hours
                total = f + m
                shown = np.flatnonzero(total >= np.quantile(total[total > 0], 0.6))      # busy enough for a share to mean something
                order = shown[np.argsort(total[shown], kind="stable")]                   # the busiest last, on top
                drawn[season] = (order, f[order] / total[order] - base, total[order], f.sum() / (f.sum() + m.sum()))
            spread = nc.spread_of(np.concatenate([d[1] for d in drawn.values()]), minimum=0.03)
            norm.vmin, norm.vmax = -spread, spread
            low.set_text(f"{-100 * spread:+.1f}%" + NL + women_at(base, -spread)); high.set_text(f"{100 * spread:+.1f}%" + NL + women_at(base, spread))
            for season in SEASON_ORDER:
                order, value, total, women = drawn[season]
                flows[season].set_segments([segs[j] for j in order])
                flows[season].set_array(value)
                flows[season].set_linewidths(width_of(total))
                readouts[season].set_text(f"{women:.1%} women")
        self.finish(fig, draw)

    def volume_network(self):
        """Pedestrians per hour on every street (mean day): colour and width grow with it on a square-root scale from 0 to VOLUME_TOP,
        the same in every band and season, so that the bands compare."""
        cache = self.cache
        edges = cache["edges"]
        segs = [np.asarray(g.coords)[:, :2] for g in edges.geometry]
        index = edges.edgeID.to_numpy()
        cmap = colormap(nc.GREEN_RAMP)
        norm = PowerNorm(gamma=0.5, vmin=0.0, vmax=VOLUME_TOP, clip=True)
        fig, axes, headline, (x0, w, gap, top, h) = self.season_panels(edges.total_bounds, self.outline)
        flows = {}
        for ax, season in zip(axes, SEASON_ORDER):
            ax.add_collection(LineCollection(segs, colors=STREET, linewidths=0.35, zorder=1))
            flows[season] = LineCollection([], cmap=cmap, norm=norm, zorder=2, capstyle="round", joinstyle="round")
            ax.add_collection(flows[season])
        low, high = nc.end_bar(fig, 0.5, 0.40, cmap, y=0.20, caption="simulated volume on each street (pedestrians per hour, mean day)" + NL + "the same scale in every band")
        low.set_text("0" + NL + "per hour"); high.set_text(f"{VOLUME_TOP:,.0f}" + NL + "per hour")

        def draw(i):
            key = BAND_KEYS[i]
            headline.set_text(BAND_LABEL[key])
            for season in SEASON_ORDER:
                total = (cache["sim"][season]["edge_vuln"][key] + cache["sim"][season]["edge_norm"][key]).reindex(index).fillna(0).to_numpy() / 7.0 / nc.BAND_HOURS[key]
                order = np.flatnonzero(total > 0)
                order = order[np.argsort(total[order], kind="stable")]           # the busiest last, on top
                flows[season].set_segments([segs[j] for j in order])
                flows[season].set_array(total[order])
                flows[season].set_linewidths(width_of(total[order]))
        self.finish(fig, draw)

    def volume_change(self):
        """Each season against winter, street by street: the change in pedestrians per hour (mean day), colour on a square-root
        scale clipped at +-CHANGE_TOP, width growing with the size of the change, the largest on top."""
        cache = self.cache
        edges = cache["edges"]
        segs = [np.asarray(g.coords)[:, :2] for g in edges.geometry]
        index = edges.edgeID.to_numpy()
        cmap = colormap(RESIDUAL)
        norm = Normalize(vmin=-1.0, vmax=1.0, clip=True)
        compared = [s for s in SEASON_ORDER if s != "winter"]
        fig, axes, headline, (x0, w, gap, top, h) = new_figure(edges.total_bounds, len(compared), x0=0.13, span=0.74, gap=0.004,
                                                                outline=self.outline)      # panels the size of the four-season videos
        flows, readouts = {}, {}
        for i, (ax, season) in enumerate(zip(axes, compared)):
            fig.text(x0 + i * (w + gap) + 0.002, top + 0.012, f"{SEASON_LABEL[season]} vs winter", ha="left", va="bottom", color=INK, fontsize=fs(19))
            ax.add_collection(LineCollection(segs, colors=STREET, linewidths=0.35, zorder=1))
            flows[season] = LineCollection([], cmap=cmap, norm=norm, zorder=2, capstyle="round", joinstyle="round")
            ax.add_collection(flows[season])
            readouts[season] = fig.text(x0 + i * (w + gap) + 0.002, top - h - 0.012, "", ha="left", va="top", color=INK, fontsize=fs(17))
        ticks = [-1.0, -0.5, 0.0, 0.5, 1.0]
        labels = [f"{np.sign(t) * CHANGE_TOP * t * t:+,.0f}" if t else "0" for t in ticks]
        colour_bar(fig, flows[compared[0]], 0.5, 0.40, ticks, labels, "fewer than in winter", "more than in winter",
                   "change in pedestrians per hour on each street (mean day), the same scale in every band", y=0.12)

        def per_hour(season, key):
            sim = cache["sim"][season]
            return (sim["edge_vuln"][key] + sim["edge_norm"][key]).reindex(index).fillna(0).to_numpy() / 7.0 / nc.BAND_HOURS[key]

        def draw(i):
            key = BAND_KEYS[i]
            headline.set_text(BAND_LABEL[key])
            winter = per_hour("winter", key)
            for season in compared:
                change = per_hour(season, key) - winter
                order = np.flatnonzero(change != 0)
                order = order[np.argsort(np.abs(change[order]), kind="stable")]        # the largest changes last, on top
                size = np.clip(np.abs(change[order]) / CHANGE_TOP, 0, 1)
                flows[season].set_segments([segs[j] for j in order])
                flows[season].set_array(np.sign(change[order]) * np.sqrt(size))
                flows[season].set_linewidths(WIDTH[0] + (WIDTH[1] - WIDTH[0]) * size ** 0.5)
                readouts[season].set_text(f"{100 * (change.sum() / winter.sum()):+.1f}% in the band" + NL
                                          + f"median change {np.median(np.abs(change)):.1f} per hour")
        self.finish(fig, draw)

    def volume_cells(self, side):
        """Share of the band per cell, coloured from a quiet cell to the busiest of the band (square-root scale); the two ends are numbers."""
        cache = self.cache
        cmap = colormap(nc.GREEN_RAMP)
        norm = PowerNorm(gamma=0.5, vmin=0.001, vmax=0.4, clip=True)
        fig, axes, headline, (x0, w, gap, top, h) = self.season_panels(self.grid.total_bounds, self.outline)
        colls = {}
        for ax, season in zip(axes, SEASON_ORDER):
            colls[season] = PolyCollection(self.verts, cmap=cmap, norm=norm, linewidths=0)
            ax.add_collection(colls[season])
        low, high = nc.end_bar(fig, 0.5, 0.40, cmap, y=0.20, above=("a quiet cell", "the busiest cell"),
                               caption=f"share of {'observed' if side == 'obs' else 'simulated'} volumes per cell")
        equivalent = fig.text(0.5, 0.04, "", color=INK, fontsize=fs(17), ha="center", va="center")

        def draw(i):
            key = BAND_KEYS[i]
            headline.set_text(BAND_LABEL[key])
            shares, totals = {}, []
            for season in SEASON_ORDER:
                if side == "obs":
                    v = cache["obs"][season][f"passing_{key}"].fillna(0).to_numpy(float)
                else:
                    v = (cache["sim"][season]["cell_vuln"][key] + cache["sim"][season]["cell_norm"][key]).to_numpy(float)
                shares[season] = 100 * v / v.sum()
                if side == "obs":
                    totals.append(v.sum())
                else:       # the legend counts true crossings of the streets (mean day), not the cell values, which are in units of a mean street
                    totals.append(float((cache["sim"][season]["edge_vuln"][key].sum() + cache["sim"][season]["edge_norm"][key].sum()) / 7.0))
            nonzero = np.concatenate([x[x > 0] for x in shares.values()])
            lo, hi = float(np.percentile(nonzero, 5)), float(nonzero.max())      # the bottom of the scale is the 5th percentile: the cells below it are all "quiet"
            norm.vmin, norm.vmax = lo, hi
            mean_total = float(np.mean(totals))
            unit = "passers-by" if side == "obs" else "crossings"
            low.set_text(pct(lo) + chr(10) + f"fewer than {max(round(mean_total * lo / 100), 2):,} {unit}"); high.set_text(pct(hi) + chr(10) + count(mean_total * hi / 100, unit))
            for season in SEASON_ORDER:
                colls[season].set_array(np.ma.masked_invalid(np.where(shares[season] > 0, shares[season], np.nan)))
            equivalent.set_text(f"1% = about {mean_total / 100:,.0f} {'observed passers-by' if side == 'obs' else 'simulated crossings'} ({BAND_LABEL[key]}{'' if side == 'obs' else ', mean day'})")
        self.finish(fig, draw)

    # ------------------------------------------------------------------------ validation, one season
    def readout(self, season, band, variant):
        one = nc.indicators(nc.cell_frame(self.cache, season, band, variant))
        return f"rank correlation {one['rho']:.2f}  |  {one['misallocated']:.0%} in the wrong cell"

    def validation_cells(self, variant, season):
        label, ramp = VARIANTS[variant]
        cmap_share, norm_share = colormap(ramp), PowerNorm(gamma=0.5, vmin=0.001, vmax=0.4, clip=True)
        cmap_res, norm_res, ticks, labels = nc.ratio_classes()
        fig, axes, headline, (x0, w, gap, top, h) = new_figure(self.grid.total_bounds, 3, top=0.78, gap=0.004, outline=self.outline)
        colls = []
        for ax, cmap, norm in zip(axes, (cmap_share, cmap_share, cmap_res), (norm_share, norm_share, norm_res)):
            coll = PolyCollection(self.verts, cmap=cmap, norm=norm, linewidths=0)
            ax.add_collection(coll)
            colls.append(coll)
        centres = np.array([(g.centroid.x, g.centroid.y) for g in self.grid.geometry])
        rings = axes[2].scatter([], [], s=330, facecolors="none", edgecolors=INK, linewidths=1.6, zorder=6)    # the cells that differ most
        noun = {"total": "pedestrians", "women": "women", "men": "men"}[variant]
        for i, title in enumerate(["Observed", "Simulated", "Simulated vs observed"]):
            fig.text(x0 + i * (w + gap) + w / 2, top + 0.04, title, ha="center", va="center", color=INK, fontsize=fs(21))
        mid = [x0 + i * (w + gap) + w / 2 for i in range(3)]
        low, high = nc.end_bar(fig, (mid[0] + mid[1]) / 2, 0.30, cmap_share, y=0.15, above=(f"fewer {noun}", f"more {noun}"), size=15)
        colour_bar(fig, colls[2], mid[2], 0.30, ticks, labels, "simulated lower", "simulated higher",
                   f"simulated vs observed  (x2 = twice){chr(10)}ringed: the {RINGED} cells that differ most", y=0.15)
        line = fig.text(0.006, 0.955, "", color=INK, fontsize=fs(21), ha="left", va="center")

        def draw(i):
            band = BAND_KEYS[i]
            obs, sim, _ = nc.cell_arrays(self.cache, season, band, variant)
            res = nc.log_ratio(obs, sim)
            headline.set_text(f"{SEASON_LABEL[season]}  {BAND_LABEL[band]}")
            nonzero = np.concatenate([obs[np.isfinite(obs) & (obs > 0)], sim[np.isfinite(sim) & (sim > 0)]])
            norm_share.vmin, norm_share.vmax = float(np.percentile(nonzero, 2)), float(nonzero.max())
            low.set_text(pct(norm_share.vmin)); high.set_text(pct(norm_share.vmax))
            colls[0].set_array(np.ma.masked_invalid(np.where(obs != 0, obs, np.nan)))
            colls[1].set_array(np.ma.masked_invalid(np.where(sim != 0, sim, np.nan)))
            colls[2].set_array(np.ma.masked_invalid(res))
            rings.set_offsets(centres[nc.most_different(obs, sim, RINGED)])
            line.set_text(self.readout(season, band, variant))
        self.finish(fig, draw)

    def validation_streets(self, variant, season):
        label, _ = VARIANTS[variant]
        pieces = self.cache["pieces"]
        position = {q: i for i, q in enumerate(self.grid.quadkey)}
        cell_of = pieces.quadkey.map(position).to_numpy()
        segs = [segments(g) for g in pieces.geometry]
        flat = [(s, k) for k, group in enumerate(segs) for s in group]
        lines = [s for s, _ in flat]
        owner = np.array([k for _, k in flat])
        fig, axes, headline, (x0, w, gap, top, h) = new_figure(pieces.total_bounds, 1, top=0.89, x0=0.315, span=0.37, outline=self.outline)
        ax = axes[0]
        ax.add_collection(LineCollection(lines, colors=STREET, linewidths=0.35, zorder=1))
        cmap_res, norm_res, ticks, labels = nc.ratio_classes()
        coll = LineCollection([], cmap=cmap_res, norm=norm_res, zorder=2, linewidths=1.4, capstyle="round")
        ax.add_collection(coll)
        colour_bar(fig, coll, 0.5, 0.30, ticks, labels, "simulated lower", "simulated higher", nc.RATIO_CAPTION + ", painted on the streets of the cell", y=0.115)
        line = fig.text(0.02, 0.55, "", color=INK, fontsize=fs(20), ha="left", va="center", linespacing=1.7)

        def draw(i):
            band = BAND_KEYS[i]
            obs, sim, _ = nc.cell_arrays(self.cache, season, band, variant)
            res = nc.log_ratio(obs, sim)
            headline.set_text(f"{SEASON_LABEL[season]}  {BAND_LABEL[band]}")
            value = res[cell_of][owner]
            shown = np.flatnonzero(np.isfinite(value))
            order = shown[np.argsort(np.abs(value[shown]), kind="stable")]          # the largest discrepancies last
            coll.set_segments([lines[j] for j in order])
            coll.set_array(value[order])
            line.set_text(self.readout(season, band, variant).replace("  |  ", chr(10)))
        self.finish(fig, draw)


def all_jobs() -> list[str]:
    names = ["observed_byGender", "simulated_byGender_cells", "simulated_byGender_network", "observed_volume", "simulated_volume_cells",
             "simulated_volume_network", "simulated_volume_change"]
    for kind in ("cells", "streets"):
        for variant in VARIANTS:
            names += [f"validation_{kind}_{variant}_{season}" for season in VALIDATION_SEASONS]
    return names


def run(name: str, still: str | None = None) -> str:
    job = Job(name, still)
    if name == "observed_byGender":
        job.by_gender_cells("obs")
    elif name == "simulated_byGender_cells":
        job.by_gender_cells("sim")
    elif name == "simulated_byGender_network":
        job.by_gender_network()
    elif name == "observed_volume":
        job.volume_cells("obs")
    elif name == "simulated_volume_cells":
        job.volume_cells("sim")
    elif name == "simulated_volume_network":
        job.volume_network()
    elif name == "simulated_volume_change":
        job.volume_change()
    else:
        _, kind, variant, season = name.split("_")
        (job.validation_cells if kind == "cells" else job.validation_streets)(variant, season)
    return name


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--only", nargs="*", help="video names, or prefixes of them")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--still", help="write one still of each selected video's frame to this folder instead of a video")
    args = parser.parse_args()
    names = all_jobs()
    if args.only:
        names = [n for n in names if any(n == o or n.startswith(o) for o in args.only)]
    if args.list:
        print(chr(10).join(names))
        return
    if len(names) == 1 or args.workers <= 1:
        for n in names:
            run(n, args.still)
        return
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for done in pool.map(run, names, [args.still] * len(names)):
            print("done", done)


if __name__ == "__main__":
    main()
