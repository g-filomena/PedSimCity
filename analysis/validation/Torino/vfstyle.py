#!/usr/bin/env python3
"""Shared map styling for the Turin figures — palette, typeface, timing.

Imported by the preparation figures in this folder and by the validation figure
in the PedSimCity repo, so the two read as one set. Like `vfgrid`, each side
keeps its own copy; the font is looked for in a `fonts/` directory beside this
module. Importing this sets matplotlib's default family as a side effect.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib
from matplotlib import font_manager

MODULE_DIR = Path(__file__).resolve().parent

FONT_FILE = MODULE_DIR / "fonts" / "BarlowSemiCondensed-Light.ttf"
if FONT_FILE.exists():
    font_manager.fontManager.addfont(str(FONT_FILE))
    matplotlib.rcParams["font.family"] = font_manager.FontProperties(
        fname=str(FONT_FILE)).get_name()
    matplotlib.rcParams["font.weight"] = "light"

SURFACE = "#1a1a19"
NO_DATA = "#232322"
INK = "#ffffff"
INK_2 = "#c3c2b7"
INK_MUTED = "#898781"
CAPTION = "#ffffff"    # legend captions and tick labels
BOUNDARY = "#8a8a80"   # municipal outline, sits just clear of the grid

# Diverging, built in CIE LCh: one hue per arm, with chroma AND lightness both
# rising from a near-black neutral at the centre out to the ends. Chroma is what
# reads as "strong", so it has to peak at the extremes - an earlier version
# peaked mid-arm and faded to pale tints at the ends, which made the largest
# differences look like the weakest ones.
DIVERGING = [
    (0.000, "#2fa7ff"),
    (0.125, "#2088d3"),
    (0.250, "#346a9d"),
    (0.375, "#374e68"),
    (0.500, "#333330"),
    (0.625, "#73312e"),
    (0.750, "#a72a2f"),
    (0.875, "#d91833"),
    (1.000, "#ff233f"),
]
# sequential single hue, darkest step nearest the surface
BLUE_RAMP = [
    "#0d366b", "#104281", "#184f95", "#1c5cab", "#256abf", "#2a78d6",
    "#3987e5", "#5598e7", "#6da7ec", "#86b6ef", "#9ec5f4", "#b7d3f6", "#cde2fb",
]

FPS = 25
HOLD_FRAMES = 35

# panel geometry, in figure fractions of a 1920 x 1080 canvas. The panel height
# is derived from the study area's aspect so the city fills its panel edge to
# edge - no dead margin around the map.
FIG_W, FIG_H = 19.2, 10.8
