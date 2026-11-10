#!/usr/bin/env python3
"""Animate who walks where in the pedestrian simulation, against the API data.

Three panels, stepping through the nine Smart Locator time bands:
  1. street network - colour is the share of vulnerable agents, width is volume
  2. the same, aggregated onto the 216 m quadkeys
  3. simulated against observed: where the model puts more or fewer people

Panels 1 and 2 read red where vulnerable agents are over-represented against
the simulation's own average, blue where under, dark neutral at the average.

Panel 3 cannot subtract the two datasets - the model counts edge crossings by
sampled agents, the API counts unique visitors - so each side is first turned
into that cell's share of the band's city total, and the panel shows the log2
ratio of those shares: red where the model puts more people than the data does,
blue where fewer.

    python animate_sim.py --poster 18_20 --no-video
    python animate_sim.py --out figures/torino_sim.mp4
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

import geopandas as gpd
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection, PolyCollection
from matplotlib.colors import LinearSegmentedColormap, LogNorm, Normalize

from vfgrid import BANDS, city_boundary
from vfstyle import (BOUNDARY, DIVERGING, FPS, HOLD_FRAMES, INK, INK_MUTED,
                     NO_DATA, SURFACE)

log = logging.getLogger("sim-anim")

BAND_KEYS = [b[0] for b in BANDS]
NETWORK_GREY = "#3d3d39"
# volume is carried by luminance alone, so hue stays reserved for the two
# diverging meanings in the panels either side of it
VOLUME_RAMP = ["#262624", "#3f3f3c", "#5c5c58", "#7d7d77", "#a2a29a",
               "#c6c6be", "#e4e4dd", "#ffffff"]
# Per hour, not per band: the bands are 2 h except 20-24 (4 h) and 00-06 (6 h),
# so totals made the wide bands look like the busiest. Per hour the frames are
# comparable and 18-20 reads as the peak it is.
VOL_MIN, VOL_MAX = 15.0, 1500.0
CAPTION = "#ffffff"

# The residual needs a pair that cannot be read as the blue/red carrying
# vulnerability in the panels beside it. Purple <-> orange: distinct from both,
# CVD-safe, and warm still reads as "too many".
RESIDUAL = [
    (0.000, "#dd3eff"),
    (0.125, "#bf1ae2"),
    (0.250, "#932da9"),
    (0.375, "#663271"),
    (0.500, "#333330"),
    (0.625, "#6b4829"),
    (0.750, "#9e5f21"),
    (0.875, "#d37811"),
    (1.000, "#ff972a"),
]

# the simulation's own overall share of vulnerable agents, and a spread wide
# enough for a 5 779-trip sample split nine ways
VULN_SHARE = 0.75        # the re-run's own overall share of vulnerable agents
VULN_SPREAD = 0.12       # 21x more trips than the first run, so a tighter ramp
MIN_EDGE_TRIPS = 25      # below this an edge is both noisy and clutter
MIN_CELL_TRIPS = 66      # ~ +/-10 points at 95% on a share near 0.75
RESID_SPREAD = 0.15      # percentage points of the city's walking; +/-0.15
                         # holds ~97% of cells. Kept as a share throughout: the
                         # two datasets are never differenced as counts, only
                         # their shares of their own citywide totals

EDGE_LW = (0.3, 4.2)     # width follows each edge's rank within the band, so the
                         # range is used evenly however skewed the volumes are

FIG_W, FIG_H = 19.2, 10.8
PANEL_TOP = 0.860
PANEL_X0, PANEL_SPAN, PANEL_GAP = 0.006, 0.988, 0.004


def load(sim_gpkg: Path, api_gpkg: Path, season: str):
    edges = gpd.read_file(sim_gpkg, layer="sim_edges")
    cells = gpd.read_file(sim_gpkg, layer="sim_cells")
    api = gpd.read_file(api_gpkg, layer="quadkeys_long")
    api = api[api["season"] == season].set_index("quadkey").reindex(cells["quadkey"])

    def share(frame, prefix, min_trips):
        vals = np.column_stack([frame[f"share_v_{k}"].to_numpy(float) for k in BAND_KEYS])
        trips = np.column_stack([frame[f"trips_{k}"].to_numpy(float) for k in BAND_KEYS])
        return np.where(trips >= min_trips, vals - VULN_SHARE, np.nan)

    edge_share = share(edges, "edge", MIN_EDGE_TRIPS)
    cell_share = share(cells, "cell", MIN_CELL_TRIPS)
    edge_vol = np.column_stack([edges[f"band_{k}"].to_numpy(float) for k in BAND_KEYS])

    # panel 3: where the model puts more or fewer people than the data does.
    # Counts are not comparable (crossings vs unique visitors), so both sides
    # become each cell's share of that band's city total. The residual is their
    # difference, not their ratio: a difference is self-weighting, so a busy
    # cell that is slightly wrong outweighs a quiet one that is wildly wrong,
    # and the values sum to the total misallocated share. A ratio did the
    # opposite - its worst values sat in the quietest cells (rho -0.22 against
    # observed volume) - so it overstated error where little was at stake.
    sim_vol = np.column_stack([cells[f"band_{k}"].to_numpy(float) for k in BAND_KEYS])
    obs_vol = np.column_stack([api[f"passing_{k}"].to_numpy(float) for k in BAND_KEYS])
    on_street = (cells["street_m"].to_numpy() > 0)[:, None]
    sim_share = sim_vol / np.nansum(np.where(on_street, sim_vol, np.nan), axis=0, keepdims=True)
    obs_share = obs_vol / np.nansum(np.where(on_street, obs_vol, np.nan), axis=0, keepdims=True)
    api_share = np.where(np.broadcast_to(on_street, sim_share.shape),
                         (sim_share - obs_share) * 100, np.nan)

    log.info("%d edges, %d cells, season %s", len(edges), len(cells), season)
    return edges, cells, edge_vol, edge_share, cell_share, api_share


def build_figure(edges, cells, edge_vol, edge_share, cell_share, api_share):
    cmap = LinearSegmentedColormap.from_list("vf_div", DIVERGING).copy()
    cmap.set_bad(NO_DATA)
    vuln_norm = Normalize(vmin=-VULN_SPREAD, vmax=VULN_SPREAD)
    resid_cmap = LinearSegmentedColormap.from_list("vf_resid", RESIDUAL).copy()
    resid_cmap.set_bad(NO_DATA)
    vol_cmap = LinearSegmentedColormap.from_list("vf_vol", VOLUME_RAMP).copy()
    vol_cmap.set_bad(NO_DATA)
    vol_norm = LogNorm(vmin=VOL_MIN, vmax=VOL_MAX)
    resid_norm = Normalize(vmin=-RESID_SPREAD, vmax=RESID_SPREAD)

    fig = plt.figure(figsize=(FIG_W, FIG_H), dpi=100, facecolor=SURFACE)
    headline = fig.text(1 - PANEL_X0, 0.945, "", ha="right", va="center",
                        color=INK, fontsize=27, fontweight="bold")

    minx, miny, maxx, maxy = cells.total_bounds
    pad = 0.005 * max(maxx - minx, maxy - miny)
    xlim, ylim = (minx - pad, maxx + pad), (miny - pad, maxy + pad)
    panel_w = (PANEL_SPAN - 2 * PANEL_GAP) / 3
    panel_h = panel_w * (FIG_W / FIG_H) / ((xlim[1] - xlim[0]) / (ylim[1] - ylim[0]))
    panel_y = PANEL_TOP - panel_h
    log.info("panel %.0f x %.0f px", panel_w * FIG_W * 100, panel_h * FIG_H * 100)

    outline = city_boundary(cells)
    axes, readouts = [], []
    for i in range(3):
        ax = fig.add_axes([PANEL_X0 + i * (panel_w + PANEL_GAP), panel_y, panel_w, panel_h])
        ax.set_facecolor(SURFACE)
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        ax.set_aspect("equal")
        ax.axis("off")
        ax.add_collection(LineCollection(outline, colors=BOUNDARY, linewidths=1.1,
                                         zorder=4, capstyle="round"))
        axes.append(ax)

    for i, title in enumerate(["Simulated: who walks",
                               "Simulated: how many walk",
                               "Simulated vs observed"]):
        fig.text(PANEL_X0 + i * (panel_w + PANEL_GAP) + panel_w / 2, PANEL_TOP + 0.035,
                 title, ha="center", va="center", color=INK, fontsize=19,
                 fontweight="bold")

    segments = [np.asarray(geom.coords) for geom in edges.geometry]
    axes[0].add_collection(LineCollection(segments, colors=NETWORK_GREY,
                                          linewidths=0.5, zorder=1))
    # round caps stop each segment ending in a visible stub at every junction
    flow = LineCollection([], cmap=cmap, norm=vuln_norm, zorder=2,
                          capstyle="round", joinstyle="round")
    axes[0].add_collection(flow)

    verts = [np.asarray(geom.exterior.coords) for geom in cells.geometry]
    polys = []
    for ax, norm, cm in ((axes[1], vol_norm, vol_cmap), (axes[2], resid_norm, resid_cmap)):
        coll = PolyCollection(verts, cmap=cm, norm=norm, linewidths=0)
        ax.add_collection(coll)
        polys.append(coll)

    bar_w = 0.26
    mid = [PANEL_X0 + i * (panel_w + PANEL_GAP) + panel_w / 2 for i in range(3)]
    for x, coll, ticks, tick_labels, pole_lo, pole_hi, caption, pole_colors in (
        (mid[0] - bar_w / 2, flow, [-0.12, -0.06, 0.0, 0.06, 0.12],
         ["+12%", "+6%", "0%", "+6%", "+12%"],
         "non-vulnerable", "vulnerable", "", ("#2fa7ff", "#ff233f")),
        (mid[1] - bar_w / 2, polys[0], [15, 50, 150, 500, 1500],
         ["15", "50", "150", "500", "1500"],
         "", "", "simulated crossings per hour per cell", (INK, INK)),
        (mid[2] - bar_w / 2, polys[1], [-0.15, -0.075, 0.0, 0.075, 0.15],
         ["0.15%", "0.075%", "0%", "0.075%", "0.15%"],
         "underestimation", "overestimation",
         "difference in % share of total pedestrian counts\n"
         "(ends are about 750 people in a 2-hour band)",
         ("#dd3eff", "#ff972a")),
    ):
        cax = fig.add_axes([x, 0.175, bar_w, 0.016])
        bar = fig.colorbar(coll, cax=cax, orientation="horizontal")
        bar.set_ticks(ticks)
        bar.set_ticklabels(tick_labels)
        bar.ax.tick_params(colors=CAPTION, labelsize=15, length=0)
        bar.outline.set_visible(False)
        if pole_lo:
            fig.text(x, 0.222, pole_lo, color=CAPTION, fontsize=19,
                     ha="left", va="center")
        if pole_hi:
            fig.text(x + bar_w, 0.222, pole_hi, color=CAPTION, fontsize=19,
                     ha="right", va="center")
        if caption:
            fig.text(x + bar_w / 2, 0.112, caption, color=CAPTION, fontsize=21,
                     ha="center", va="top", linespacing=1.35)

    return fig, flow, polys, readouts, headline, segments


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--sim", default="data/torino_simulation.gpkg")
    parser.add_argument("--api", default="data/torino_presence.gpkg")
    parser.add_argument("--season", default="autumn")
    parser.add_argument("--out", default="figures/torino_sim.mp4")
    parser.add_argument("--hold", type=int, default=HOLD_FRAMES)
    parser.add_argument("--poster")
    parser.add_argument("--no-video", action="store_true")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S")

    edges, cells, edge_vol, edge_share, cell_share, api_share = load(
        Path(args.sim), Path(args.api), args.season)
    fig, flow, polys, readouts, headline, segments = build_figure(
        edges, cells, edge_vol, edge_share, cell_share, api_share)

    vol_ceiling = np.percentile(edge_vol[edge_vol > 0], 99)
    trips = np.column_stack([edges[f"trips_{k}"].to_numpy(float) for k in BAND_KEYS])
    cell_trips = np.column_stack([cells[f"trips_{k}"].to_numpy(float) for k in BAND_KEYS])
    on_street_cells = cells["street_m"].to_numpy() > 0
    band_hours = np.array([b[2] for b in BANDS], dtype=float)
    cell_vol = np.column_stack([cells[f"band_{k}"].to_numpy(float) for k in BAND_KEYS])
    cell_vol = np.where(on_street_cells[:, None], cell_vol, np.nan) / band_hours

    def draw_band(i: int) -> None:
        headline.set_text(BANDS[i][1])

        shown = np.isfinite(edge_share[:, i])
        vol = edge_vol[:, i]
        # busiest last, so heavy streets sit on top of the quiet ones they cross
        order = np.argsort(vol[shown], kind="stable")
        idx = np.flatnonzero(shown)[order]
        flow.set_segments([segments[j] for j in idx])
        flow.set_array(edge_share[idx, i])
        rank = np.arange(len(idx)) / max(len(idx) - 1, 1)   # idx is volume-sorted
        flow.set_linewidths(EDGE_LW[0] + (EDGE_LW[1] - EDGE_LW[0]) * rank ** 2.6)

        polys[0].set_array(np.ma.masked_invalid(
            np.where(cell_vol[:, i] > 0, cell_vol[:, i], np.nan)))
        polys[1].set_array(np.ma.masked_invalid(api_share[:, i]))



    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    if args.poster:
        draw_band(BAND_KEYS.index(args.poster))
        fig.savefig(out.with_suffix(".png"), facecolor=SURFACE)
        log.info("wrote %s", out.with_suffix(".png"))
    if args.no_video:
        return 0

    writer = animation.FFMpegWriter(fps=FPS, codec="h264", bitrate=8000,
                                    extra_args=["-pix_fmt", "yuv420p", "-preset", "slow"])
    total = len(BANDS) * args.hold
    with writer.saving(fig, str(out), dpi=100):
        for frame in range(total):
            if frame % args.hold == 0:
                draw_band(frame // args.hold)
                log.info("band %d/%d %s", frame // args.hold + 1, len(BANDS),
                         BANDS[frame // args.hold][1])
            writer.grab_frame()
    log.info("wrote %s (%.1f MB, %.1f s)", out, out.stat().st_size / 1e6, total / FPS)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
