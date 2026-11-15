"""The night summary tables as one PDF (figures/night/tables/night_tables.pdf), white on black.

    python night_tables_pdf.py

Reads the CSVs night_prepare_data.py wrote (and the cache, for the whole-day indicators). Pages: the volumes recorded by band and sex for
each season (Vodafone passers-by, Vodafone everyone, and the model's crossings), the four seasons together, and the shape indicators of
the validation. Also night_table_winter (counts by band) and night_table_validation_winter (16:9, the validation by time of day).
"""
from __future__ import annotations

import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle

from night_common import (BAND_KEYS, BAND_LABEL, BG, FIG_H, FIG_W, INK, SEASON_LABEL, SEASON_ORDER, TABLES, cell_frame, fs, indicators,
                          load_cache)

PAGE = (16.5, 9.3)
HEADER, ZEBRA, EDGE = "#262626", "#0c0c0c", "#555555"


def page(pdf, df: pd.DataFrame, title: str, subtitle: str, columns: list[tuple[str, str]], widths: list[float] | None = None):
    """One table page; columns are (heading, format) pairs applied left to right to the frame's columns."""
    fig = plt.figure(figsize=PAGE, facecolor=BG)
    fig.text(0.04, 0.95, title, color=INK, fontsize=fs(22), ha="left", va="center")
    fig.text(0.04, 0.895, subtitle, color=INK, fontsize=fs(11), ha="left", va="center", linespacing=1.4)
    ax = fig.add_axes([0.03, 0.04, 0.94, 0.78]); ax.axis("off")
    cells = []
    for _, row in df.iterrows():
        out = []
        for value, (_, fmt) in zip(row.tolist(), columns):
            out.append(value if fmt == "text" else ("-" if pd.isna(value) else format(value, fmt)))
        cells.append(out)
    table = ax.table(cellText=cells, colLabels=[c[0] for c in columns], loc="upper center", cellLoc="right", colWidths=widths)
    table.auto_set_font_size(False)
    table.set_fontsize(fs(12))
    table.scale(1, 2.7)
    for (r, c), cell in table.get_celld().items():
        cell.set_edgecolor(EDGE); cell.set_linewidth(0.6)
        cell.get_text().set_color(INK)
        if r == 0:
            cell.set_facecolor(HEADER); cell.get_text().set_fontweight("bold"); cell.get_text().set_wrap(True)
            cell.set_height(cell.get_height() * 1.5)
        else:
            cell.set_facecolor(ZEBRA if r % 2 else BG)
        if c == 0:
            cell._loc = "left"
            cell.get_text().set_ha("left")
    pdf.savefig(fig, facecolor=BG)
    plt.close(fig)


NL = chr(10)
VOLUME_COLUMNS = [("Band", "text"),
                  ("Passers-by" + NL + "women", ",.0f"), ("Passers-by" + NL + "men", ",.0f"), ("Passers-by" + NL + "% women", ".1f"),
                  ("Everyone" + NL + "women", ",.0f"), ("Everyone" + NL + "men", ",.0f"), ("Everyone" + NL + "% women", ".1f"),
                  ("Model" + NL + "women", ",.0f"), ("Model" + NL + "men", ",.0f"), ("Model" + NL + "% women", ".1f"),
                  ("Passers-by" + NL + "day share %", ".1f"), ("Model" + NL + "day share %", ".1f")]
VOLUME_NOTE = ("Passers-by and Everyone: observed, unique phones summed over the 216 m cells, with a dwell under 30 min or with any dwell." + NL +
               "Model: crossings of a street, vulnerable agents = women, weekday and Sunday weighted 5 to 2. They count different things: compare the shares, not the sizes.")


def winter_table():
    """One compact table for the winter: what the data and the model count in each band, and who they count."""
    v = pd.read_csv(TABLES / "volumes_by_band_sex.csv")
    v = v[v.season == "winter"].reset_index(drop=True)
    obs = v.obs_passers_women + v.obs_passers_men
    sim = v.sim_women + v.sim_men
    rows = [[b, f"{o / 1e3:,.0f}", f"{w:.1f}", f"{m / 1e6:,.2f}", f"{sw:.1f}", f"{od:.1f}", f"{sd:.1f}"]
            for b, o, w, m, sw, od, sd in zip(v.band, obs, v.obs_passers_pct_women, sim, v.sim_pct_women, v.obs_passers_share_of_day_pct, v.sim_share_of_day_pct)]
    ow, sw_ = v.obs_passers_women.sum(), v.sim_women.sum()
    rows.append(["All day", f"{obs.sum() / 1e3:,.0f}", f"{100 * ow / obs.sum():.1f}", f"{sim.sum() / 1e6:,.2f}", f"{100 * sw_ / sim.sum():.1f}", "100", "100"])
    heads = ["Winter", "Observed" + NL + "passers-by, k", "Observed" + NL + "% women", "Model" + NL + "crossings, M", "Model" + NL + "% women",
             "Observed" + NL + "% of the day", "Model" + NL + "% of the day"]
    fig = plt.figure(figsize=(12.5, 5.6), facecolor=BG)
    ax = fig.add_axes([0.01, 0.02, 0.98, 0.96]); ax.axis("off")
    table = ax.table(cellText=rows, colLabels=heads, loc="center", cellLoc="right", colWidths=[0.15, 0.17, 0.12, 0.17, 0.11, 0.14, 0.14])
    table.auto_set_font_size(False); table.set_fontsize(fs(13)); table.scale(1, 2.3)
    for (r, c), cell in table.get_celld().items():
        cell.set_edgecolor(EDGE); cell.set_linewidth(0.6); cell.get_text().set_color(INK)
        if r == 0:
            cell.set_facecolor(HEADER); cell.get_text().set_fontweight("bold"); cell.set_height(cell.get_height() * 1.5)
        else:
            cell.set_facecolor(ZEBRA if r % 2 else BG)
            if r == len(rows):
                cell.get_text().set_fontweight("bold")
        if c == 0:
            cell.get_text().set_ha("left"); cell._loc = "left"
    for ext in ("pdf", "png"):
        fig.savefig(TABLES / f"night_table_winter.{ext}", facecolor=BG, dpi=200)
        print("wrote", TABLES / f"night_table_winter.{ext}")
    plt.close(fig)


LOWER, HIGHER, MUTED = "#dd3eff", "#ff972a", "#9a9a9a"    # model 2 points or more below / above the observed share of the day; the street-length reference


def whole_day(cache, season):
    """Indicators on the day's totals (each square's walking summed over the bands), for the model and for street length alone."""
    frames = [cell_frame(cache, season, b) for b in BAND_KEYS]
    common = frames[0].index
    for f in frames[1:]:
        common = common.intersection(f.index)
    day = sum(f.loc[common, ["sim", "obs"]] for f in frames)
    day["street"] = frames[0].loc[common, "street"]
    return indicators(day), indicators(day.assign(sim=day.street)), len(common)


def validation_table(season: str = "winter"):
    """One 16:9 table of the validation by time of day: when (share of the day), who (share of women), where (216 m squares: rank correlation
    and walking in the wrong square, for the model and for street length alone)."""
    v = pd.read_csv(TABLES / "volumes_by_band_sex.csv")
    v = v[v.season == season].reset_index(drop=True)
    t = pd.read_csv(TABLES / "validation_indicators.csv")
    t = t[(t.season == season) & (t.scale == "216 m cells")].set_index("band").reindex(BAND_KEYS)
    day, day_street, n_cells = whole_day(load_cache(), season)
    rows = [[BAND_LABEL[b], v.obs_passers_share_of_day_pct[i], v.sim_share_of_day_pct[i], v.obs_passers_pct_women[i], v.sim_pct_women[i],
             t.rho[b], t.rho_street[b], t.misallocated[b], t.misallocated_street[b]] for i, b in enumerate(BAND_KEYS)]
    obs_w, obs_all = v.obs_passers_women.sum(), (v.obs_passers_women + v.obs_passers_men).sum()
    sim_w, sim_all = v.sim_women.sum(), (v.sim_women + v.sim_men).sum()
    rows.append(["All day", 100.0, 100.0, 100 * obs_w / obs_all, 100 * sim_w / sim_all, day["rho"], day_street["rho"], day["misallocated"], day_street["misallocated"]])

    fig = plt.figure(figsize=(FIG_W, FIG_H), dpi=100, facecolor=BG)
    fig.text(0.05, 0.92, f"{SEASON_LABEL[season]}: the model against Vodafone, by time of day", color=INK, fontsize=fs(26), ha="left", va="center")
    centres = [0.26, 0.34, 0.44, 0.52, 0.62, 0.71, 0.81, 0.90]
    for label, a, b in (("When", 0, 1), ("Who", 2, 3), ("Where: 216 m squares", 4, 7)):
        fig.text((centres[a] + centres[b]) / 2, 0.845, label, color=INK, fontsize=fs(18), ha="center", va="center")
    for label, a, b in (("% of the day", 0, 1), ("% women", 2, 3), ("rank correlation", 4, 5), ("in the wrong square", 6, 7)):
        fig.text((centres[a] + centres[b]) / 2, 0.80, label, color=INK, fontsize=fs(15), ha="center", va="center")
        fig.add_artist(Line2D([centres[a] - 0.035, centres[b] + 0.035], [0.78, 0.78], transform=fig.transFigure, color=EDGE, linewidth=1.0))
    subs = ["observed", "model", "observed", "model", "model", "street length", "model", "street length"]
    for x, sub in zip(centres, subs):
        fig.text(x, 0.75, sub, color=MUTED if sub == "street length" else INK, fontsize=fs(14), ha="center", va="center")
    fig.text(0.05, 0.75, "time of day", color=INK, fontsize=fs(14), ha="left", va="center")
    step, y0 = 0.054, 0.69
    for r, row in enumerate(rows):
        y = y0 - r * step
        last = r == len(rows) - 1
        if last:
            fig.add_artist(Line2D([0.04, 0.95], [y + step / 2, y + step / 2], transform=fig.transFigure, color=INK, linewidth=1.0))
        elif r % 2 == 0:
            fig.patches.append(Rectangle((0.04, y - step / 2), 0.91, step, transform=fig.transFigure, facecolor=ZEBRA, edgecolor="none", zorder=0))
        weight = "bold" if last else "normal"
        fig.text(0.05, y, row[0], color=INK, fontsize=fs(17), ha="left", va="center", fontweight=weight)
        values = [f"{row[1]:.1f}", f"{row[2]:.1f}", f"{row[3]:.1f}", f"{row[4]:.1f}", f"{row[5]:.2f}", f"{row[6]:.2f}", f"{row[7]:.0%}", f"{row[8]:.0%}"]
        if last:
            values[0] = values[1] = "100"
        colours = [INK] * 8
        if not last and abs(row[2] - row[1]) >= 2:
            colours[1] = HIGHER if row[2] > row[1] else LOWER
        colours[5] = colours[7] = MUTED
        for x, value, colour in zip(centres, values, colours):
            fig.text(x, y, value, color=colour, fontsize=fs(17), ha="center", va="center", fontweight=weight)
    import textwrap
    paragraphs = ["When: each time of day's share of the day. Observed = Vodafone phones staying under 30 minutes (any mode); model = street crossings, "
                  "a weekday and a Sunday weighted 5 to 2. Orange / purple: the model is 2 points or more above / below the observed share.",
                  "Who: women among them; in the model, its vulnerable agents.",
                  f"Where: the {n_cells:,} 216 m squares with street, each side as a share of the city's total at that time. Rank correlation: 1 = the same "
                  "order of the squares. In the wrong square: the share of the walking that would have to move to another square to match the data. "
                  "Street length: the score if the walking were spread in proportion to the metres of street in each square, the yardstick the model has to beat."]
    note = NL.join(textwrap.fill(p, 175) for p in paragraphs)
    fig.text(0.05, 0.03, note, color=MUTED, fontsize=fs(11), ha="left", va="bottom", linespacing=1.45)
    for ext in ("pdf", "png"):
        fig.savefig(TABLES / f"night_table_validation_{season}.{ext}", facecolor=BG, dpi=150)
        print("wrote", TABLES / f"night_table_validation_{season}.{ext}")
    plt.close(fig)


def main():
    winter_table()
    validation_table("winter")
    volumes = pd.read_csv(TABLES / "volumes_by_band_sex.csv")
    whole = pd.read_csv(TABLES / "volumes_by_band_sex_all_seasons.csv")
    table = pd.read_csv(TABLES / "validation_indicators.csv")
    path = TABLES / "night_tables.pdf"
    with PdfPages(path) as pdf:
        order = ["band", "obs_passers_women", "obs_passers_men", "obs_passers_pct_women", "obs_all_women", "obs_all_men", "obs_all_pct_women",
                 "sim_women", "sim_men", "sim_pct_women", "obs_passers_share_of_day_pct", "sim_share_of_day_pct"]
        for season in SEASON_ORDER:
            page(pdf, volumes[volumes.season == season][order], f"Volumes recorded, by band and sex  -  {SEASON_LABEL[season]}", VOLUME_NOTE, VOLUME_COLUMNS, [0.12] + [0.08] * 11)
        w = whole.rename(columns={"band": "band"})
        w["obs_passers_share_of_day_pct"] = 100 * (w.obs_passers_women + w.obs_passers_men) / (w.obs_passers_women + w.obs_passers_men)[w.band != "all bands"].sum()
        w["sim_share_of_day_pct"] = 100 * (w.sim_women + w.sim_men) / (w.sim_women + w.sim_men)[w.band != "all bands"].sum()
        page(pdf, w[order], "Volumes recorded, by band and sex  -  the four seasons together", VOLUME_NOTE, VOLUME_COLUMNS, [0.12] + [0.08] * 11)

        def indicators_page(scale):
            d = table[table.scale == scale]
            by_band = d.groupby("band", sort=False)[["rho", "rho_street", "rho_residents", "misallocated", "misallocated_street", "misallocated_residents",
                                                    "skill_street", "skill_residents", "error_factor", "rmse_pp"]].mean().reset_index()
            by_band["band"] = by_band.band.str.replace("_", "-")
            mean = d[["rho", "rho_street", "rho_residents", "misallocated", "misallocated_street", "misallocated_residents", "skill_street", "skill_residents",
                      "error_factor", "rmse_pp"]].mean()
            by_band.loc[len(by_band)] = ["all bands", *mean.tolist()]
            cols = [("Band", "text"), ("Rank corr." + chr(10) + "model", ".2f"), ("Rank corr." + chr(10) + "street length", ".2f"), ("Rank corr." + chr(10) + "residents", ".2f"),
                    ("Wrong cell" + chr(10) + "model", ".1%"), ("Wrong cell" + chr(10) + "street length", ".1%"), ("Wrong cell" + chr(10) + "residents", ".1%"),
                    ("Skill vs" + chr(10) + "street length", ".1%"), ("Skill vs" + chr(10) + "residents", ".1%"), ("Typical error" + chr(10) + "factor", ".1f"), ("RMSE" + chr(10) + "(points)", ".3f")]
            page(pdf, by_band, f"Validation indicators by band  -  {scale}",
                 "Mean over the four seasons, one run per day. Rank corr.: Spearman, 1 = same order. Wrong cell: share of the walking in a different cell from the data (0 = perfect)." + NL +
                 "Skill: how much of a naive guess's error the model removes; the naive guesses are shares proportional to street length, or to residents.", cols, [0.10] + [0.09] * 10)
        indicators_page("216 m cells")
        indicators_page("864 m blocks")
    print("wrote", path)


if __name__ == "__main__":
    main()
