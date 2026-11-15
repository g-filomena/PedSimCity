"""The night stills, into figures/night/images/.

    python night_images.py                      # all of them
    python night_images.py --only validation_charts turin_light

    simulated_volume_day     where the model's pedestrians walk, women and men together, over the day (green)
    validation_summary       the validation in four charts, white on black: when (share of the day), where (cell by cell; centre against ring), who (women's share)
    turin_light              the measured light on every street, on the terrain
    validation_charts        only on request (--only validation_charts): the indicators against the street-length and residents baselines, black on white
                             (validation_charts, validation_rank, validation_wrong_cell, validation_cell_by_cell, validation_centre)
"""
from __future__ import annotations

import argparse
import time

import geopandas as gpd
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import LinearSegmentedColormap, PowerNorm
from matplotlib.ticker import FixedLocator, FuncFormatter, NullFormatter

import night_common as nc
from night_common import (RESIDUAL, AXIS, BAND_KEYS, BAND_LABEL, BG, CENTRE, CENTRE_KM, DEM, FIG_H, FIG_W, GRID, IMAGES, INK, NO_DATA, RING_KM,
                          SEASONS, SEASON_LABEL, SEASON_ORDER, STREET, TABLES, cell_frame, colormap, colour_bar, fs, indicators,
                          new_figure, shares, style_axis)

SHOW_SEASON, SHOW_BAND = "winter", "20_24"           # the band of the cell-by-cell chart
MODEL, STREET_LINE, PEOPLE, OBSERVED = "#ff972a", "#6f6f6f", "#a8a8a8", "#000000"
BG_V, INK_V, AXIS_V, GRID_V = "#ffffff", "#000000", "#444444", "#e4e4e4"            # the validation charts: black on white
RESIDUAL_LIGHT = [(0.0, "#a800d6"), (0.25, "#b46bd0"), (0.5, "#d4d4d4"), (0.75, "#eda761"), (1.0, "#e8730a")]   # too few ... too many, light centre for a white ground
WIDTH = (0.25, 3.8)


def style_axis_w(ax, size: float = 1.0):
    """style_axis for a white ground."""
    ax.set_facecolor(BG_V)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(AXIS_V)
    ax.tick_params(colors=INK_V, labelsize=fs(13) * size, length=4, color=AXIS_V)
    ax.grid(axis="y", color=GRID_V, linewidth=0.9)
    ax.set_axisbelow(True)


def save(fig, name):
    IMAGES.mkdir(parents=True, exist_ok=True)
    for attempt in range(4):        # OneDrive sometimes refuses to overwrite a file it is syncing
        try:
            (IMAGES / f"{name}.png").unlink(missing_ok=True)
            fig.savefig(IMAGES / f"{name}.png", facecolor=fig.get_facecolor())
            break
        except OSError:
            if attempt == 3:
                raise
            time.sleep(2)
    plt.close(fig)
    print("wrote", IMAGES / f"{name}.png")


# ------------------------------------------------------------------------------------------------ volumes
def simulated_volume_day(cache):
    edges, sim = cache["edges"], cache["sim"]
    index = edges.edgeID.to_numpy()
    segs = [np.asarray(g.coords)[:, :2] for g in edges.geometry]
    length = edges.length_m.to_numpy()
    day = sum((sim[s]["edge_vuln"] + sim[s]["edge_norm"]).sum(axis=1).reindex(index).fillna(0) for s in SEASON_ORDER).to_numpy() / len(SEASON_ORDER) / 7.0   # a mean day
    classes = nc.VOLUME_CLASSES_DAY
    k = nc.class_index(day, classes)
    colours, widths = nc.class_colours(len(classes)), nc.class_widths(len(classes))
    order = np.flatnonzero(k >= 0)
    order = order[np.argsort(day[order], kind="stable")]                    # the busiest last, on top
    fig, axes, headline, (x0, w, gap, top, h) = new_figure(edges.total_bounds, 1, top=0.99, x0=0.24, span=0.46, outline=nc.city_boundary(cache["grid"]))
    headline.set_text("")
    ax = axes[0]
    ax.add_collection(LineCollection(segs, colors=STREET, linewidths=0.35, zorder=1))
    ax.add_collection(LineCollection([segs[j] for j in order], colors=[colours[k[j]] for j in order], linewidths=[widths[k[j]] for j in order],
                                     capstyle="round", joinstyle="round", zorder=2))
    fig.text(0.72, 0.74, "crossings of a street", color=INK, fontsize=fs(22), ha="left", va="center")
    fig.text(0.72, 0.70, "in a mean day", color=INK, fontsize=fs(22), ha="left", va="center")
    from matplotlib.patches import Rectangle
    rows = [("none", "#3a3a3a", 100 * length[k == -1].sum() / length.sum())] + [(c[2], colours[i], 100 * length[k == i].sum() / length.sum()) for i, c in enumerate(classes)]
    for r, (label, colour, share) in enumerate(rows):
        y = 0.63 - r * 0.062
        fig.patches.append(Rectangle((0.72, y - 0.012), 0.012, 0.024, transform=fig.transFigure, facecolor=colour, edgecolor="none"))
        fig.text(0.74, y, label, color=INK, fontsize=fs(19), ha="left", va="center")
        fig.text(0.835, y, f"{share:.0f}% of the length", color=INK, fontsize=fs(15), ha="left", va="center")
    fig.text(0.72, 0.2, "mean of the four seasons," + chr(10) + "weekday and Sunday weighted 5 to 2;" + chr(10) + "the model's sample is half of" + chr(10) + "Turin's residents, plus commuters" + chr(10) + "and tourists",
             color=INK, fontsize=fs(12), ha="left", va="top", linespacing=1.4)
    save(fig, "simulated_volume_day")


# ------------------------------------------------------------------------------------------------ validation charts
def cell_distance_km(cache) -> pd.Series:
    """Distance of every cell's centre from Piazza Castello, in km, indexed by quadkey."""
    grid = cache["grid"]
    centre = gpd.GeoSeries(gpd.points_from_xy([CENTRE[0]], [CENTRE[1]]), crs=4326).to_crs(grid.crs).iloc[0]
    return pd.Series(grid.geometry.centroid.distance(centre).to_numpy() / 1000, index=grid.quadkey)


def chart_data(cache):
    table = pd.read_csv(TABLES / "validation_indicators.csv")
    cells = table[table.scale == "216 m cells"]
    km = cell_distance_km(cache)
    ratio = {}
    for kind in ("obs", "sim"):
        ratio[kind] = pd.DataFrame({s: [centre_ring(cache, km, s, b, kind) for b in BAND_KEYS] for s in SEASONS}, index=BAND_KEYS)
    return table, cells, ratio


def centre_ring(cache, km, season, band, kind):
    df = cell_frame(cache, season, band)
    s = shares(df[kind])
    k = km.reindex(df.index)
    return s[k <= CENTRE_KM].mean() / s[k >= RING_KM].mean()


X = np.arange(len(BAND_KEYS))
XLABELS = [BAND_LABEL[b].replace(":00", "").replace(" – ", "–") for b in BAND_KEYS]


def heading(ax, k, title, subtitle):
    ax.text(0, 1.17, title, transform=ax.transAxes, color=INK_V, fontsize=fs(23) * k, ha="left", va="bottom")
    ax.text(0, 1.045, subtitle, transform=ax.transAxes, color=INK_V, fontsize=fs(14) * k, ha="left", va="bottom")


def band_axis(ax, k):
    ax.axvspan(len(BAND_KEYS) - 2.5, len(BAND_KEYS) - 0.5, color="#eeeeee", zorder=0, lw=0)
    ax.set_xticks(X)
    ax.set_xticklabels(XLABELS, fontsize=fs(13) * k)
    ax.set_xlim(-0.5, len(BAND_KEYS) + 0.6)
    ax.set_xlabel("time of day (the shaded two bands are evening and night)", fontsize=fs(14) * k, labelpad=8)


def line(ax, y, colour, label, k, ls="-", lw=2.6, marker=None, at=(4, 0.02, "bottom")):
    ax.plot(X, y, color=colour, ls=ls, lw=lw * max(k, 1), marker=marker, ms=7 * k, zorder=3)
    ax.text(at[0], y[at[0]] + at[1], label, color=colour, fontsize=fs(16) * k, ha="center", va=at[2])


SEASON_LINE = {"autumn": "#8b5a2b", "winter": "#5bb8f5", "spring": "#f08a3a", "summer": "#f2c200"}   # autumn brown, winter light blue, spring orange, summer yellow


def season_lines(ax, k, frame, column, at=None):
    """One line per season of an indicator; the key is season_key()."""
    d = frame.pivot_table(index="band", columns="season", values=column).reindex(BAND_KEYS)
    for s in SEASON_ORDER:
        ax.plot(X, d[s].to_numpy(), color=SEASON_LINE[s], lw=2.8 * max(k, 1), marker="o", ms=5 * k, zorder=3)


def season_key(fig, x, y, size=17):
    """The four season names in their colours, one row, centred on x."""
    names = [SEASON_LABEL[s].lower() for s in SEASON_ORDER]
    step = 0.075
    for i, (s, name) in enumerate(zip(SEASON_ORDER, names)):
        fig.text(x + (i - 1.5) * step, y, name, color=SEASON_LINE[s], fontsize=fs(size), ha="center", va="center")


def across(frame, col):
    d = frame.pivot_table(index="band", columns="season", values=col).reindex(BAND_KEYS)
    return d.mean(axis=1).to_numpy(), d.min(axis=1).to_numpy(), d.max(axis=1).to_numpy()


def panel_rank(ax, k, ctx):
    cells = ctx["cells"]
    season_lines(ax, k, cells, "rho", len(BAND_KEYS) - 1)
    line(ax, cells.groupby("band").rho_street.mean().reindex(BAND_KEYS).to_numpy(), STREET_LINE, "street length", k, ls="--", at=(3, 0.012, "bottom"))
    line(ax, cells.groupby("band").rho_residents.mean().reindex(BAND_KEYS).to_numpy(), PEOPLE, "residents", k, ls=":", at=(3, -0.012, "top"))
    ax.set_ylim(0.5, 0.85); ax.set_yticks([0.5, 0.6, 0.7, 0.8])
    ax.set_ylabel("rank correlation", fontsize=fs(14) * k)
    band_axis(ax, k); style_axis_w(ax, k)


def panel_wrong(ax, k, ctx):
    cells, table = ctx["cells"], ctx["table"]
    season_lines(ax, k, cells, "misallocated", len(BAND_KEYS) - 1)
    line(ax, cells.groupby("band").misallocated_street.mean().reindex(BAND_KEYS).to_numpy(), STREET_LINE, "street length", k, ls="--", at=(4, 0.008, "bottom"))
    line(ax, cells.groupby("band").misallocated_residents.mean().reindex(BAND_KEYS).to_numpy(), PEOPLE, "residents", k, ls=":", at=(4, 0.008, "bottom"))
    ax.set_ylim(0.25, 0.5); ax.set_yticks([0.3, 0.4, 0.5])
    ax.yaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:.0%}"))
    ax.set_ylabel("walking in the wrong cell", fontsize=fs(14) * k)
    band_axis(ax, k); style_axis_w(ax, k)
    skill_c = table[table.scale == "216 m cells"].skill_street.mean()
    skill_b = table[table.scale == "864 m blocks"].skill_street.mean()
    ax.text(0.0, 0.04, f"against street length, the model removes {skill_c:.0%} of the error on 216 m cells and {skill_b:.0%} on 864 m blocks",
            transform=ax.transAxes, color=INK_V, fontsize=fs(13) * k, ha="left")


LOG_SPAN = 2.0                                  # colours run from x0.25 to x4 (log2 of simulated over observed)
RATIO_TICKS = [-2, -1, 0, 1, 2]
RATIO_LABELS = ["x0.25", "x0.5", "x1", "x2", "x4"]


def scatter(ax, df, k, size):
    """Observed against simulated share of each cell, a dot coloured by simulated over observed (purple too few, orange too many)."""
    s_sim, s_obs = 100 * shares(df.sim), 100 * shares(df.obs)
    both = (s_sim > 0) & (s_obs > 0)
    x, y = s_obs[both].to_numpy(), s_sim[both].to_numpy()
    log_ratio = np.log2(y / x)
    typical = 2 ** np.median(np.abs(log_ratio))                      # half of the cells are off by less than this factor
    within2 = float(np.mean(np.abs(log_ratio) < 1))
    lo, hi = 1e-4, 1.5
    g = np.array([lo, hi])
    ax.fill_between(g, g / 2, g * 2, color="#000000", alpha=0.07, lw=0, zorder=1)
    ax.plot(g, g, color=INK_V, lw=1.6 * max(k, 1), zorder=2)
    order = np.argsort(np.abs(log_ratio), kind="stable")             # the largest discrepancies on top
    ax.scatter(x[order], y[order], s=size, c=log_ratio[order], cmap=colormap(RESIDUAL_LIGHT), vmin=-LOG_SPAN, vmax=LOG_SPAN, alpha=0.85, linewidths=0, zorder=3)
    ax.set_xscale("log"); ax.set_yscale("log"); ax.set_xlim(3e-4, hi); ax.set_ylim(lo, hi)
    for axis in (ax.xaxis, ax.yaxis):
        axis.set_major_locator(FixedLocator([1e-4, 1e-3, 1e-2, 1e-1, 1]))
        axis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:g}%"))
        axis.set_minor_formatter(NullFormatter())
    style_axis_w(ax, k); ax.grid(axis="x", color=GRID_V, linewidth=0.9)
    return typical, within2, indicators(df)


def ratio_bar(fig, centre, width, y):
    colour_bar(fig, matplotlib.cm.ScalarMappable(norm=matplotlib.colors.Normalize(-LOG_SPAN, LOG_SPAN), cmap=colormap(RESIDUAL_LIGHT)), centre, width, RATIO_TICKS, RATIO_LABELS,
               "too few", "too many", "simulated over observed share of the cell  (x2 = twice the observed)", y=y, ink=INK_V)


def panel_cells(ax, k, ctx):
    typical, within2, one = scatter(ax, cell_frame(ctx["cache"], SHOW_SEASON, SHOW_BAND), k, 16 * k * k)
    ax.set_xlabel("observed share of the band's walking", fontsize=fs(14) * k)
    ax.set_ylabel("simulated share", fontsize=fs(14) * k)
    ax.text(0.03, 0.97, f"half of the cells are within x{typical:.1f} of the observed" + chr(10) + f"{within2:.0%} within x2 (grey band)" + chr(10) + f"{SEASON_LABEL[SHOW_SEASON].lower()}, {BAND_LABEL[SHOW_BAND]}: each dot a 216 m cell", transform=ax.transAxes, color=INK_V,
            fontsize=fs(14) * k, ha="left", va="top", linespacing=1.5)


def cell_by_cell_seasons(cache):
    """The four seasons together: one small scatter each, same axes, same colours as the validation maps."""
    fig, axes = plt.subplots(2, 2, figsize=(FIG_W, FIG_H), dpi=100, facecolor=BG_V)
    fig.subplots_adjust(left=0.07, right=0.97, top=0.95, bottom=0.30, hspace=0.38, wspace=0.16)
    for ax, season in zip(axes.ravel(), SEASON_ORDER):
        typical, within2, one = scatter(ax, cell_frame(cache, season, SHOW_BAND), 0.8, 9)
        ax.set_title(f"{SEASON_LABEL[season]}", color=INK_V, fontsize=fs(19), loc="left", pad=8)
        ax.text(0.03, 0.97, f"half of the cells within x{typical:.1f}", transform=ax.transAxes, color=INK_V, fontsize=fs(12), ha="left", va="top")
        ax.tick_params(labelsize=fs(11))
    fig.text(0.07, 0.255, "x: observed share of the band's walking   y: simulated share   dot = one 216 m cell", color=INK_V, fontsize=fs(14), ha="left", va="center")
    ratio_bar(fig, 0.5, 0.3, 0.115)
    save(fig, "validation_cell_by_cell")


def panel_centre(ax, k, ctx):
    obs, sim = ctx["ratio"]["obs"], ctx["ratio"]["sim"]
    ax.fill_between(X, obs.min(axis=1), obs.max(axis=1), color=OBSERVED, alpha=0.18, lw=0, zorder=2)
    line(ax, obs.mean(axis=1).to_numpy(), OBSERVED, "observed", k, lw=3.4, marker="o", at=(5, -0.8, "top"))
    season_lines(ax, k, sim.stack().rename("v").reset_index().rename(columns={"level_0": "band", "level_1": "season"}), "v", len(BAND_KEYS) - 1)
    ax.set_ylim(0, 17); ax.set_ylabel("centre / ring", fontsize=fs(14) * k)
    band_axis(ax, k); style_axis_w(ax, k)


PANELS = [(panel_rank, "validation_rank"), (panel_wrong, "validation_wrong_cell"), (panel_cells, "validation_cell_by_cell"), (panel_centre, "validation_centre")]


def validation_charts(cache):
    with plt.rc_context({"text.color": INK_V, "axes.labelcolor": INK_V, "xtick.color": INK_V, "ytick.color": INK_V}):
        _validation_charts(cache)


def _validation_charts(cache):
    table, cells, ratio = chart_data(cache)
    ctx = {"cache": cache, "table": table, "cells": cells, "ratio": ratio}
    fig = plt.figure(figsize=(FIG_W, FIG_H), dpi=100, facecolor=BG_V)
    gs = fig.add_gridspec(2, 2, left=0.065, right=0.95, top=0.92, bottom=0.095, hspace=0.55, wspace=0.22)
    for (fn, _), spot in zip(PANELS, gs):
        fn(fig.add_subplot(spot), 0.74, ctx)
    season_key(fig, 0.5, 0.975)
    save(fig, "validation_charts")
    cell_by_cell_seasons(cache)
    for fn, name in PANELS:
        if name == "validation_cell_by_cell":
            continue
        fig = plt.figure(figsize=(FIG_W, FIG_H), dpi=100, facecolor=BG_V)
        fn(fig.add_axes([0.08, 0.17, 0.80, 0.56]), 1.0, ctx)
        season_key(fig, 0.48, 0.86, 20)
        save(fig, name)


# ------------------------------------------------------------------------------------------------ the four-chart summary
NL = "\n"
DARK = {"bg": BG, "ink": INK, "axis": AXIS, "grid": GRID, "shade": "#1b1b1b", "observed": "#ffffff", "key": "#b8b8b8",
        "seasons": {"autumn": "#c8864a", "winter": "#5bb8f5", "spring": "#f08a3a", "summer": "#f2c200"}}      # white on black, like the videos


def women_rho(cache) -> pd.Series:
    """Rank correlation, cell by cell, of the share of women among observed passers-by with the simulated share of women; one value per season and band."""
    out = {}
    for season in SEASON_ORDER:
        obs, sim = cache["obs"][season], cache["sim"][season]
        for band in BAND_KEYS:
            df = pd.DataFrame({"of": obs[f"passF_{band}"].to_numpy(), "om": obs[f"passM_{band}"].to_numpy(), "sf": sim["cell_vuln"][band].to_numpy(),
                               "sm": sim["cell_norm"][band].to_numpy(), "street": cache["street_m"].to_numpy()}).dropna()
            df = df[(df.street > 0) & (df.of + df.om >= nc.MIN_OBS) & (df.sf + df.sm >= nc.MIN_SIM_FLOOR)]
            out[(season, band)] = (df.of / (df.of + df.om)).corr(df.sf / (df.sf + df.sm), method="spearman")
    return pd.Series(out)


def by_season(volumes, column):
    """One column of volumes_by_band_sex.csv as a bands x seasons frame."""
    return pd.DataFrame({s: volumes[volumes.season == s][column].to_numpy() for s in SEASON_ORDER}, index=BAND_KEYS)


def style_summary_axis(ax, k, t):
    """Axis of one summary chart: time bands on x, the evening and night bands on a lighter ground."""
    ax.set_facecolor(t["bg"])
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(t["axis"])
    ax.tick_params(colors=t["ink"], labelsize=fs(13) * k, length=4, color=t["axis"])
    ax.grid(axis="y", color=t["grid"], linewidth=0.9)
    ax.set_axisbelow(True)
    ax.axvspan(len(BAND_KEYS) - 2.5, len(BAND_KEYS) - 0.5, color=t["shade"], zorder=0, lw=0)
    ax.set_xticks(X)
    ax.set_xticklabels(XLABELS, fontsize=fs(13) * k)
    ax.set_xlim(-0.5, len(BAND_KEYS) - 0.5)
    ax.set_xlabel("time of day", fontsize=fs(14) * k, labelpad=8)


def note(ax, k, t, text, where):
    x, y, ha, va = where
    ax.text(x, y, text, transform=ax.transAxes, color=t["ink"], fontsize=fs(13) * k, ha=ha, va=va, linespacing=1.4)


def observed_line(ax, k, t, frame):
    """The observed series as a thick line over the range of the four seasons."""
    ax.fill_between(X, frame.min(axis=1), frame.max(axis=1), color=t["observed"], alpha=0.22, lw=0, zorder=2)
    ax.plot(X, frame.mean(axis=1).to_numpy(), color=t["observed"], lw=3.4 * k, marker="o", ms=7 * k, zorder=4)


def model_lines(ax, k, t, frame):
    for s in SEASON_ORDER:
        ax.plot(X, frame[s].to_numpy(), color=t["seasons"][s], lw=2.8 * k, marker="o", ms=5 * k, zorder=3)


def percent_axis(ax, k, low, high, ticks, label):
    ax.set_ylim(low, high); ax.set_yticks(ticks)
    ax.yaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:.0f}%"))
    ax.set_ylabel(label, fontsize=fs(14) * k)


def panel_when(ax, k, ctx, t):
    obs, sim = by_season(ctx["volumes"], "obs_passers_share_of_day_pct"), by_season(ctx["volumes"], "sim_share_of_day_pct")
    observed_line(ax, k, t, obs); model_lines(ax, k, t, sim)
    percent_axis(ax, k, 0, 20, [0, 5, 10, 15, 20], "share of the day's walking")
    night = len(BAND_KEYS) - 1
    note(ax, k, t, f"after midnight ({XLABELS[night]}):{NL}{obs.iloc[night].mean():.1f}% observed, {sim.iloc[night].mean():.1f}% in the model", (0.99, 0.99, "right", "top"))
    style_summary_axis(ax, k, t)


CENTRE_COLOUR, RING_COLOUR, MID_COLOUR = "#ffffff", "#3fa38f", "#6a6a6a"      # cells of the centre, of the ring, and in between


def panel_scatter(ax, k, ctx, t):
    """Observed against simulated share of every 216 m cell, in one band; the cells of the centre in white, those of the ring in green."""
    df = cell_frame(ctx["cache"], SHOW_SEASON, SHOW_BAND)
    s_sim, s_obs = 100 * shares(df.sim), 100 * shares(df.obs)
    distance = ctx["km"].reindex(df.index)
    both = (s_sim > 0) & (s_obs > 0)
    x, y, d = s_obs[both].to_numpy(), s_sim[both].to_numpy(), distance[both].to_numpy()
    ratio = y / x
    g = np.array([1e-4, 1.5])
    ax.fill_between(g, g / 2, g * 2, color="#ffffff", alpha=0.09, lw=0, zorder=1)
    ax.plot(g, g, color=t["ink"], lw=1.4, zorder=2)
    centre, ring = d <= CENTRE_KM, d >= RING_KM
    middle = ~centre & ~ring
    ax.scatter(x[ring], y[ring], s=10, color=RING_COLOUR, alpha=0.7, linewidths=0, zorder=3)
    ax.scatter(x[middle], y[middle], s=12, color=MID_COLOUR, alpha=0.9, linewidths=0, zorder=4)
    ax.scatter(x[centre], y[centre], s=34, color=CENTRE_COLOUR, alpha=1.0, linewidths=0, zorder=5)
    ax.set_xscale("log"); ax.set_yscale("log"); ax.set_xlim(3e-4, 1.5); ax.set_ylim(1e-4, 1.5)
    for axis in (ax.xaxis, ax.yaxis):
        axis.set_major_locator(FixedLocator([1e-4, 1e-3, 1e-2, 1e-1, 1]))
        axis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:g}%"))
        axis.set_minor_formatter(NullFormatter())
    ax.set_xlabel("observed share of the walking", fontsize=fs(14) * k, labelpad=8)
    ax.set_ylabel("simulated share", fontsize=fs(14) * k)
    style_plain_axis(ax, k, t)
    one = indicators(df)
    rows = [(f"rank correlation {one['rho']:.2f}, {one['misallocated']:.0%} of the walking in the wrong cell", t["ink"]),
            (f"centre cells: the model has x{np.median(ratio[centre]):.1f} the observed (median)", CENTRE_COLOUR),
            (f"ring cells: x{np.median(ratio[ring]):.1f}", RING_COLOUR)]
    for r, (text, colour) in enumerate(rows):
        ax.text(0.01, 0.985 - r * 0.115, text, transform=ax.transAxes, color=colour, fontsize=fs(13) * k, ha="left", va="top")


def style_plain_axis(ax, k, t):
    """Axis of the scatter: both grids, no time bands."""
    ax.set_facecolor(t["bg"])
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(t["axis"])
    ax.tick_params(colors=t["ink"], labelsize=fs(13) * k, length=4, color=t["axis"])
    ax.grid(color=t["grid"], linewidth=0.9)
    ax.set_axisbelow(True)


def panel_ring(ax, k, ctx, t):
    obs, sim = ctx["ratio"]["obs"], ctx["ratio"]["sim"]
    observed_line(ax, k, t, obs); model_lines(ax, k, t, sim)
    ax.set_ylim(0, 11); ax.set_yticks([0, 3, 6, 9])
    ax.set_ylabel("centre / ring", fontsize=fs(14) * k)
    o, m = obs.mean(axis=1), sim.mean(axis=1)
    peak, night = int(m.to_numpy().argmax()), len(BAND_KEYS) - 1
    note(ax, k, t, f"{XLABELS[peak]}: {m.iloc[peak]:.1f} in the model, {o.iloc[peak]:.1f} observed{NL}"
         f"{XLABELS[night]}: {m.iloc[night]:.1f} in the model, {o.iloc[night]:.1f} observed", (0.01, 0.99, "left", "top"))
    style_summary_axis(ax, k, t)


def panel_who(ax, k, ctx, t):
    obs, sim = by_season(ctx["volumes"], "obs_passers_pct_women"), by_season(ctx["volumes"], "sim_pct_women")
    observed_line(ax, k, t, obs); model_lines(ax, k, t, sim)
    percent_axis(ax, k, 44, 54, [45, 48, 51, 54], "women among those walking")
    rho = ctx["women_rho"]
    note(ax, k, t, f"cell by cell the share of women is unrelated to the observed one:{NL}rank correlation {rho.min():+.2f} to {rho.max():+.2f} over {len(rho)} times of day and seasons",
         (0.01, 0.03, "left", "bottom"))
    style_summary_axis(ax, k, t)


def key_row(fig, x, y, t, size=17):
    """observed, then the model with one colour per season, one row centred on x."""
    names = [("observed", t["observed"]), ("model:", t["key"])] + [(SEASON_LABEL[s].lower(), t["seasons"][s]) for s in SEASON_ORDER]
    texts = [fig.text(0, y, name, color=colour, fontsize=fs(size), ha="left", va="center") for name, colour in names]
    renderer = fig.canvas.get_renderer()
    widths = [text.get_window_extent(renderer).width / fig.bbox.width for text in texts]
    gap = 0.022
    x_left = x - (sum(widths) + gap * (len(texts) - 1)) / 2
    for text, width in zip(texts, widths):
        text.set_x(x_left)
        x_left += width + gap


def validation_summary(cache, t=DARK):
    """Four charts: when people walk, where (ranking of cells; centre against ring), and who."""
    with plt.rc_context({"text.color": t["ink"], "axes.labelcolor": t["ink"], "xtick.color": t["ink"], "ytick.color": t["ink"]}):
        table, cells, ratio = chart_data(cache)
        ctx = {"cache": cache, "cells": cells, "ratio": ratio, "km": cell_distance_km(cache), "volumes": pd.read_csv(TABLES / "volumes_by_band_sex.csv"),
               "women_rho": women_rho(cache)}
        fig = plt.figure(figsize=(FIG_W, FIG_H), dpi=100, facecolor=t["bg"])
        gs = fig.add_gridspec(2, 2, left=0.065, right=0.97, top=0.82, bottom=0.115, hspace=0.9, wspace=0.2)
        k = 0.74
        panels = [(panel_when, "When", "share of the day's walking at each time of day"),
                  (panel_scatter, "Where: cell by cell", f"observed against simulated share of each 216 m cell, {SEASON_LABEL[SHOW_SEASON].lower()} {BAND_LABEL[SHOW_BAND]}"),
                  (panel_ring, "Where: centre against ring", f"share per cell in the centre (within {CENTRE_KM:g} km of Piazza Castello) over the ring (beyond {RING_KM:g} km)"),
                  (panel_who, "Who", "women among those walking at each time of day")]
        for (fn, title, subtitle), spot in zip(panels, gs):
            ax = fig.add_subplot(spot)
            fn(ax, k, ctx, t)
            ax.text(0, 1.17, title, transform=ax.transAxes, color=t["ink"], fontsize=fs(23) * k, ha="left", va="bottom")
            ax.text(0, 1.045, subtitle, transform=ax.transAxes, color=t["ink"], fontsize=fs(14) * k, ha="left", va="bottom")
        key_row(fig, 0.5, 0.955, t)
        fig.text(0.065, 0.025, "Observed: Vodafone passers-by (dwell under 30 minutes), unique people. Model: crossings of the street network, a mean day "
                 "(weekday and Sunday weighted 5 to 2).", color=t["ink"], fontsize=fs(12) * k * 1.15, ha="left", va="center")
        save(fig, "validation_summary")


# ------------------------------------------------------------------------------------------------ light
def terrain(bounds, pad=400.0, step=2):
    import rasterio
    from matplotlib.colors import LightSource
    minx, miny, maxx, maxy = bounds
    with rasterio.open(DEM) as src:
        win = rasterio.windows.from_bounds(minx - pad, miny - pad, maxx + pad, maxy + pad, src.transform).round_offsets().round_lengths()
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


def scale_bar(ax, metres, x, y, ink=INK):
    """A plain scale bar on a map in metres: a line `metres` long from (x, y) in axes fractions, ticked at its ends and middle."""
    (x0, x1), (y0, y1) = ax.get_xlim(), ax.get_ylim()
    left, base = x0 + x * (x1 - x0), y0 + y * (y1 - y0)
    tick = 0.008 * (y1 - y0)
    ax.plot([left, left + metres], [base, base], color=ink, lw=1.6, solid_capstyle="butt", zorder=6)
    for k, label in ((0, "0"), (0.5, f"{metres / 2000:g}"), (1, f"{metres / 1000:g} km")):
        ax.plot([left + k * metres] * 2, [base, base + tick], color=ink, lw=1.2, zorder=6)
        ax.text(left + k * metres, base + 1.8 * tick, label, color=ink, fontsize=fs(13), ha="center", va="bottom", zorder=6)


def turin_light(cache):
    from matplotlib.patches import Polygon
    lit = gpd.read_file(nc.RESOURCES / f"{nc.CITY}_edges_illuminated_continuous.gpkg")
    share = {t: float(lit.length[lit.mean_lux < t].sum() / lit.length.sum()) for t in (5, 10)}
    cmap = colormap(["#262a45", "#4b3a5a", "#8a4a3a", "#d9822b", "#ffc85a", "#fff1c9"], bad="#262a45")
    norm = PowerNorm(gamma=0.5, vmin=0, vmax=40)
    outline = nc.city_boundary(cache["grid"])
    ring = np.vstack(outline)                      # the frame holds the municipal outline as well as the streets, so the terrain is not cut
    lb = lit.total_bounds
    bounds = (min(lb[0], ring[:, 0].min()), min(lb[1], ring[:, 1].min()), max(lb[2], ring[:, 0].max()), max(lb[3], ring[:, 1].max()))
    fig, axes, headline, (x0, w, gap, top, h) = new_figure(bounds, 1, top=0.99, x0=0.28, span=0.44, outline=outline)
    headline.set_text("")
    ax = axes[0]
    relief, extent = terrain(bounds)
    image = ax.imshow(relief, extent=extent, origin="upper", cmap=colormap(["#000000", "#262622", "#4a4a44", "#74746c", "#9b9b92"]), vmin=0, vmax=1, zorder=0, interpolation="bilinear")
    clip = Polygon(np.asarray(max(outline, key=len)), transform=ax.transData, fc="none", ec="none")
    ax.add_patch(clip); image.set_clip_path(clip)
    by_lux = lit.assign(lux=lit.mean_lux.clip(0, 40)).sort_values("lux")
    ax.add_collection(LineCollection([np.asarray(g.coords) for g in by_lux.geometry], cmap=cmap, norm=norm, array=by_lux.lux.to_numpy(),
                                     linewidths=0.6, capstyle="round", zorder=2))
    scale_bar(ax, 2000, 0.72, 0.05)       # bottom right, below the hill, clear of the outline
    colour_bar(fig, matplotlib.cm.ScalarMappable(norm=norm, cmap=cmap), 0.5, 0.26, [0, 5, 10, 20, 40], ["0", "5", "10", "20", "40+ lx"], caption="mean lux of the street", y=0.175)
    fig.text(0.715, 0.5, f"{share[10]:.0%} of street length" + chr(10) + f"under 10 lx" + chr(10) + f"{share[5]:.0%} under 5 lx" + chr(10) + chr(10) + "shading: terrain (10 m DEM)",
             color=INK, fontsize=fs(20), ha="left", va="center", linespacing=1.5)       # right of the map, centred on its height
    save(fig, "turin_light")


JOBS = {"simulated_volume_day": simulated_volume_day, "validation_summary": validation_summary, "turin_light": turin_light, "validation_charts": validation_charts}
ON_REQUEST = {"validation_charts"}       # the summary is the validation figure; the baseline charts are written only when asked for


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--only", nargs="*", choices=sorted(JOBS))
    args = parser.parse_args()
    cache = nc.load_cache()
    for name, fn in JOBS.items():
        if (args.only and name in args.only) or (not args.only and name not in ON_REQUEST):
            fn(cache)


if __name__ == "__main__":
    main()
