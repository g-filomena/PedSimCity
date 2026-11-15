"""Shared by the night scripts: palette (black background, white text), loaders, indicators, video writer.

    night_prepare_data.py   reads the runs and the Vodafone data, writes data/night_cache.pkl and the tables
    night_videos.py         every video, into figures/night/videos/
    night_images.py         the stills, into figures/night/images/
    night_tables_pdf.py     the summary tables as a PDF, into figures/night/tables/
    night_route_terms.py    one trip routed term by term, and the four term maps of Turin

Change a colour, a size or a band here and every script follows.
"""
from __future__ import annotations

import glob
import os
import pickle
import shutil
import sys
from pathlib import Path

import numpy as np
import pandas as pd

import matplotlib

matplotlib.use("Agg")
import matplotlib.animation as animation  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
sys.path.insert(0, str(HERE))
from vfgrid import BANDS, city_boundary  # noqa: E402,F401
from vfstyle import FIG_H, FIG_W, FPS  # noqa: E402,F401

# ---------------------------------------------------------------------------------------------- paths
CITY = "Torino_sidewalks"                           # the network the validation runs were made on
RESOURCES = REPO / "src" / "main" / "resources" / CITY
RUNS = REPO / "outputs" / "PedSimCityNight"         # val_<month>_<wed|sun>[_s<seed>]/data/edge_volume.csv
VODAFONE = HERE / "data" / "torino_presence.gpkg"
CENSUS = REPO / "src" / "main" / "resources" / "Torino" / "Torino_censusData.gpkg"
DEM = REPO / "inputData" / "Torino" / "Torino_DEM.tif"
OUT = HERE / "figures" / "night"
VIDEOS, IMAGES, TABLES = OUT / "videos", OUT / "images", OUT / "tables"
CACHE = HERE / "data" / "night_cache.pkl"


def video_path(name: str) -> Path:
    """Where a video goes: observed/, simulated/, validation_cells/ or validation_streets/ under videos/."""
    folder = name.rsplit("_", 2)[0] if name.startswith("validation_") else name.split("_")[0]
    return VIDEOS / folder / f"{name}.mp4"

# ---------------------------------------------------------------------------------------------- the days
SEASONS = {"winter": "feb", "spring": "apr", "summer": "jul", "autumn": "oct"}   # Vodafone window -> month of the runs
SEASON_ORDER = ["autumn", "winter", "spring", "summer"]
SEASON_LABEL = {"autumn": "Autumn", "winter": "Winter", "spring": "Spring", "summer": "Summer"}
WEEKDAY_WEIGHT, SUNDAY_WEIGHT = 5, 2                # the month's mean day: five weekdays to two Sundays
BAND_KEYS = [b[0] for b in BANDS]
BAND_LABEL = {b[0]: b[1] for b in BANDS}
BAND_HOURS = {b[0]: b[2] for b in BANDS}
MIN_OBS = 50           # a cell needs this many observed passers-by in a band for its share to be drawn
MIN_SIM = 2000         # ... and this many simulated crossings (the runs are a sample: scale it with their size)
MIN_SIM_FLOOR = 100    # a quiet band (the night) lowers that bar to half its mean cell, but not below this
HOLD = 50              # frames each band is held in a video (25 frames per second): 2 seconds per band
CENTRE, CENTRE_KM, RING_KM = (7.6869, 45.0711), 1.0, 3.0   # Piazza Castello; the centre and the ring, in km


def band_of(hour: int) -> str:
    for key in BAND_KEYS:
        start, end = map(int, key.split("_"))
        if start <= hour < end:
            return key
    raise ValueError(hour)


HOUR_BAND = {h: band_of(h) for h in range(24)}

# ---------------------------------------------------------------------------------------------- look
BG = "#000000"          # background of every figure and video
INK = "#ffffff"         # every piece of text
AXIS = "#b8b8b8"        # axis lines and grids
GRID = "#2e2e2e"
NO_DATA = "#151515"     # a cell with nothing to draw, on the black
BOUNDARY = "#9a9a9a"    # municipal outline
STREET = "#3a3a3a"      # the street network under a coloured layer
FONT = "Bahnschrift"    # a condensed face that stays readable in white on black
FONT_SCALE = 1.35       # every font size in the scripts goes through fs()

try:
    font_manager.findfont(FONT, fallback_to_default=False)
except ValueError:
    FONT = "Arial"
matplotlib.rcParams["font.family"] = FONT
matplotlib.rcParams["font.weight"] = "normal"
matplotlib.rcParams["text.color"] = INK
matplotlib.rcParams["axes.labelcolor"] = INK
matplotlib.rcParams["xtick.color"] = INK
matplotlib.rcParams["ytick.color"] = INK


def fs(size: float) -> float:
    return size * FONT_SCALE


# palettes: women red, men blue, anything not split by sex green; the discrepancy purple to orange
GREEN_RAMP = ["#0b3a3a", "#0e4a47", "#125a55", "#176b63", "#1f7d71", "#2a9080", "#3fa38f", "#5cb69f", "#7ec9b2", "#a3dac6", "#c6eadc"]
DIVERGING = [(0.0, "#2fa7ff"), (0.125, "#2088d3"), (0.25, "#346a9d"), (0.375, "#374e68"), (0.5, "#333330"),
             (0.625, "#73312e"), (0.75, "#a72a2f"), (0.875, "#d91833"), (1.0, "#ff233f")]   # more men ... more women
RESIDUAL = [(0.0, "#dd3eff"), (0.125, "#bf1ae2"), (0.25, "#932da9"), (0.375, "#663271"), (0.5, "#333330"),
            (0.625, "#6b4829"), (0.75, "#9e5f21"), (0.875, "#d37811"), (1.0, "#ff972a")]   # too few ... too many
# the sequential ramps of women and men are the two arms of the diverging scale of the share of women (neutral centre outwards)
RED_RAMP = [c for _, c in DIVERGING if _ >= 0.5]
BLUE_RAMP = [c for _, c in reversed(DIVERGING) if _ <= 0.5]
VARIANTS = {"total": ("All pedestrians", GREEN_RAMP), "women": ("Women", RED_RAMP), "men": ("Men", BLUE_RAMP)}


# parks and water, the same in Turin and Barcelona: shaded, the outline lighter than the fill
PARK_LINE, PARK_FILL = "#4f9a6f", "#1d3b2c"
WATER_LINE, WATER_FILL = "#3f75c4", "#15305a"


def closed_outlines(geoms) -> list:
    """The polygons that closed outlines among these lines enclose (a park, a dock); open lines, a coast or a river bank cut by the map edge, enclose none."""
    from shapely.ops import polygonize, unary_union
    try:
        return list(polygonize(unary_union(list(geoms))))
    except Exception:
        return []


def colormap(stops_or_colours, bad: str = NO_DATA):
    cmap = LinearSegmentedColormap.from_list("c", stops_or_colours).copy()
    cmap.set_bad(bad)
    return cmap


def style_axis(ax, size: float = 1.0):
    ax.set_facecolor(BG)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(AXIS)
    ax.tick_params(colors=INK, labelsize=fs(13) * size, length=4, color=AXIS)
    ax.grid(axis="y", color=GRID, linewidth=0.9)
    ax.set_axisbelow(True)


def new_figure(bounds, n: int, top: float = 0.775, x0: float = 0.006, span: float = 0.988, gap: float = 0.003, outline=None):
    """n equal map panels side by side; returns the figure, the axes, the headline text and the geometry."""
    minx, miny, maxx, maxy = bounds
    pad = 0.005 * max(maxx - minx, maxy - miny)
    xlim, ylim = (minx - pad, maxx + pad), (miny - pad, maxy + pad)
    w = (span - (n - 1) * gap) / n
    h = w * (FIG_W / FIG_H) / ((xlim[1] - xlim[0]) / (ylim[1] - ylim[0]))
    fig = plt.figure(figsize=(FIG_W, FIG_H), dpi=100, facecolor=BG)
    headline = fig.text(1 - x0, 0.945, "", ha="right", va="center", color=INK, fontsize=fs(27))
    axes = []
    for i in range(n):
        ax = fig.add_axes([x0 + i * (w + gap), top - h, w, h])
        ax.set_facecolor(BG)
        ax.set_xlim(*xlim); ax.set_ylim(*ylim); ax.set_aspect("equal"); ax.axis("off")
        if outline is not None:
            from matplotlib.collections import LineCollection
            ax.add_collection(LineCollection(outline, colors=BOUNDARY, linewidths=1.1, zorder=4))
        axes.append(ax)
    return fig, axes, headline, (x0, w, gap, top, h)


def colour_bar(fig, mappable, centre, width, ticks, labels, lo="", hi="", caption="", y=0.175, ink=INK):
    cax = fig.add_axes([centre - width / 2, y, width, 0.016])
    bar = fig.colorbar(mappable, cax=cax, orientation="horizontal")
    bar.set_ticks(ticks); bar.set_ticklabels(labels)
    bar.ax.tick_params(colors=ink, labelsize=fs(14), length=0)
    bar.outline.set_visible(False)
    if lo:
        fig.text(centre - width / 2, y + 0.05, lo, color=ink, fontsize=fs(16), ha="left", va="center")
    if hi:
        fig.text(centre + width / 2, y + 0.05, hi, color=ink, fontsize=fs(16), ha="right", va="center")
    if caption:
        fig.text(centre, y - 0.065, caption, color=ink, fontsize=fs(17), ha="center", va="top", linespacing=1.3)


# ---------------------------------------------------------------------------------------------- video
def find_ffmpeg():
    for rel in ("Library/bin/ffmpeg.exe", "bin/ffmpeg", "ffmpeg.exe"):
        mine = Path(sys.prefix) / rel
        if mine.exists():
            return str(mine)
    found = shutil.which("ffmpeg")
    if found:
        return found
    home = os.path.expanduser("~")
    for pattern in (home + "/AppData/Local/miniconda3/envs/*/Library/bin/ffmpeg.exe", home + "/miniconda3/envs/*/bin/ffmpeg"):
        hits = sorted(glob.glob(pattern))
        if hits:
            return hits[-1]
    raise SystemExit("ffmpeg not found")


def write_video(fig, draw, n_steps: int, path: Path, hold: int = HOLD):
    """draw(i) sets step i; every step is held for `hold` frames at FPS. No still is written.
    Each step is drawn once and handed to ffmpeg at one frame per `hold` frames of the output, which repeats it to FPS."""
    ffmpeg = find_ffmpeg()
    os.environ["PATH"] = os.path.dirname(ffmpeg) + os.pathsep + os.environ["PATH"]
    matplotlib.rcParams["animation.ffmpeg_path"] = ffmpeg
    path.parent.mkdir(parents=True, exist_ok=True)
    writer = animation.FFMpegWriter(fps=FPS / hold, codec="h264", bitrate=8000, extra_args=["-r", str(FPS), "-pix_fmt", "yuv420p", "-preset", "slow"])
    with writer.saving(fig, str(path), dpi=100):
        for step in range(n_steps):
            draw(step)
            writer.grab_frame()
    plt.close(fig)
    print("wrote", path)


# ---------------------------------------------------------------------------------------------- data
def load_cache() -> dict:
    if not CACHE.exists():
        raise SystemExit(f"{CACHE} is missing: run night_prepare_data.py first")
    with open(CACHE, "rb") as handle:
        return pickle.load(handle)


def shares(x):
    return x / x.sum()


def indicators(df: pd.DataFrame) -> dict:
    """Shape indicators of a frame with the columns sim and obs (one row per cell)."""
    s_sim, s_obs = shares(df.sim), shares(df.obs)
    both = (df.sim > 0) & (df.obs > 0)
    log_error = np.log10(s_sim[both] / s_obs[both])
    return {"cells": len(df), "rho": s_sim.rank().corr(s_obs.rank()),
            "misallocated": 0.5 * (s_sim - s_obs).abs().sum(),
            "error_factor": 10 ** np.sqrt((log_error ** 2).mean()),
            "rmse_pp": 100 * np.sqrt(((s_sim - s_obs) ** 2).mean())}


def variant_columns(variant: str, band: str) -> tuple[str, str]:
    """The observed column and the simulated table of a variant: all pedestrians, women (vulnerable agents) or men."""
    return {"total": (f"passing_{band}", "both"), "women": (f"passF_{band}", "vuln"), "men": (f"passM_{band}", "norm")}[variant]


def cell_frame(cache: dict, season: str, band: str, variant: str = "total", level: int | None = None) -> pd.DataFrame:
    """Per cell that holds street and has an observed value: observed, simulated, street metres, residents."""
    obs_col, kind = variant_columns(variant, band)
    sim = cache["sim"][season]
    simulated = (sim["cell_vuln"][band] + sim["cell_norm"][band]) if kind == "both" else sim[f"cell_{kind}"][band]
    grid = cache["grid"]
    df = pd.DataFrame({"sim": simulated, "obs": cache["obs"][season][obs_col], "street": cache["street_m"],
                       "residents": cache["residents"]}, index=grid.quadkey)
    df = df[df.street > 0].dropna(subset=["obs"])
    if level:
        df = df.groupby(df.index.str[:level]).sum()
    return df


LOG_SPAN = 3.5                                  # log2 of simulated over observed, clipped here; the two outer classes are open-ended
RATIO_EDGES = [-3.0, -2.0, -1.0, -0.5, 0.5, 1.0, 2.0, 3.0]          # class limits: x1/8, x1/4, x1/2, x0.7 | x1.4, x2, x4, x8
RATIO_LABELS = ["x1/8", "x1/4", "x1/2", "x0.7", "x1.4", "x2", "x4", "x8"]
RATIO_CAPTION = "simulated vs observed share  (x2 = twice the observed)"


def ratio_classes():
    """The discrepancy scale as nine classes, from the nine stops of RESIDUAL: colour map, norm, and the ticks of the class limits."""
    from matplotlib.colors import BoundaryNorm, ListedColormap
    cmap = ListedColormap([c for _, c in RESIDUAL]).with_extremes(bad=NO_DATA)
    norm = BoundaryNorm([-LOG_SPAN] + RATIO_EDGES + [LOG_SPAN], cmap.N)
    return cmap, norm, RATIO_EDGES, RATIO_LABELS


def most_different(obs, sim, n: int = 10) -> np.ndarray:
    """Indices of the n cells where the two shares (percent of the band) differ most in points: where the model moves the most volume."""
    gap = np.abs(np.nan_to_num(np.asarray(sim, float)) - np.nan_to_num(np.asarray(obs, float)))
    return np.argsort(gap, kind="stable")[::-1][:n]


def log_ratio(obs, sim, floor: float = 1e-4):
    """log2 of simulated over observed share (both in percent), clipped to +-LOG_SPAN; a cell empty on one side counts at the floor."""
    obs, sim = np.asarray(obs, float), np.asarray(sim, float)
    out = np.log2((np.nan_to_num(sim) + floor) / (np.nan_to_num(obs) + floor))
    out[np.isnan(obs) | np.isnan(sim) | ((obs <= 0) & (sim <= 0))] = np.nan
    return np.clip(out, -LOG_SPAN, LOG_SPAN)


def cell_arrays(cache: dict, season: str, band: str, variant: str):
    """Observed share, simulated share and their difference (percentage points of the band), one value per grid cell."""
    grid = cache["grid"]
    df = cell_frame(cache, season, band, variant).reindex(grid.quadkey)
    on = df.street.notna()
    sim_share = 100 * shares(df.sim[on]).reindex(grid.quadkey)
    obs_share = 100 * shares(df.obs[on]).reindex(grid.quadkey)
    return obs_share.to_numpy(), sim_share.to_numpy(), (sim_share - obs_share).to_numpy()


def spread_of(values, q: float = 98, step: float = 0.005, minimum: float = 0.02) -> float:
    """The half-width of a diverging scale for these values: their q-th percentile of |value|, rounded up to `step`."""
    v = np.abs(np.asarray(values, float))
    v = v[np.isfinite(v)]
    high = float(np.percentile(v, q)) if len(v) else minimum
    return max(minimum, float(np.ceil(high / step) * step))


def end_bar(fig, centre, width, cmap, y=0.14, above=("", ""), caption="", height=0.016, size=16):
    """A horizontal colour bar with a number at each end, as texts the caller changes frame by frame; returns (low, high)."""
    ax = fig.add_axes([centre - width / 2, y, width, height]); ax.axis("off")
    ax.imshow(np.linspace(0, 1, 256)[None, :], aspect="auto", cmap=cmap, extent=(0, 1, 0, 1))
    low = fig.text(centre - width / 2, y - 0.012, "", color=INK, fontsize=fs(size), ha="left", va="top")
    high = fig.text(centre + width / 2, y - 0.012, "", color=INK, fontsize=fs(size), ha="right", va="top")
    if above[0]:
        fig.text(centre - width / 2, y + height + 0.012, above[0], color=INK, fontsize=fs(size), ha="left", va="bottom")
    if above[1]:
        fig.text(centre + width / 2, y + height + 0.012, above[1], color=INK, fontsize=fs(size), ha="right", va="bottom")
    if caption:
        fig.text(centre, y - 0.095, caption, color=INK, fontsize=fs(size + 1), ha="center", va="top", linespacing=1.3)
    return low, high


# ---------------------------------------------------------------------------------------------- numbers on the legends
VOLUME_CLASSES_DAY = [(0, 10, "<10"), (10, 50, "10-50"), (50, 200, "50-200"), (200, 1000, "200-1,000"), (1000, np.inf, ">1,000")]
VOLUME_CLASSES_HOUR = [(0, 1, "<1"), (1, 5, "1-5"), (5, 20, "5-20"), (20, 50, "20-50"), (50, 150, "50-150"), (150, np.inf, ">150")]


def class_index(values, classes) -> np.ndarray:
    """Index of the class of every value; -1 for a street nobody crosses."""
    index = np.digitize(values, [c[1] for c in classes[:-1]])
    return np.where(values > 0, index, -1)


def class_colours(n: int) -> list[str]:
    cmap = colormap(GREEN_RAMP)
    return [matplotlib.colors.to_hex(cmap(v)) for v in np.linspace(0.12, 1.0, n)]


def class_widths(n: int) -> np.ndarray:
    return np.linspace(0.5, 3.8, n)


def legend_row(fig, x: float, y: float, labels, colours, size: float = 14.0, centre: bool = False):
    """Swatch and label, left to right from x (or centred on x): a horizontal legend of classes."""
    from matplotlib.patches import Rectangle
    widths = [0.013 + 0.0066 * len(label) * size / 14.0 + 0.016 for label in labels]
    if centre:
        x = x - (sum(widths) - 0.016) / 2
    for label, colour, width in zip(labels, colours, widths):
        fig.patches.append(Rectangle((x, y - 0.009), 0.009, 0.018, transform=fig.transFigure, facecolor=colour, edgecolor="none"))
        fig.text(x + 0.013, y, label, color=INK, fontsize=fs(size), ha="left", va="center")
        x += width


def gender_ticks(base: float, spread: float) -> list[str]:
    """Tick labels of the share-of-women scale: the difference in points and what it makes in percent of women."""
    out = []
    for v in (-spread, -spread / 2, 0.0, spread / 2, spread):
        top = "average" if v == 0 else f"{100 * v:+.1f} pts"
        out.append(f"{top}{chr(10)}{100 * (base + v):.1f}% women")
    return out
