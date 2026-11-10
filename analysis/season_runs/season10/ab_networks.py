"""Season10 A/B: the same runs on Torino and on Torino_sidewalks (centrelines with a sidewalk drawn
on both sides removed). Compares where the walked metres go: by distance from Piazza Castello
against Vodafone passing presence, and by street, with every edge assigned to the street it
belongs to on the Torino network (Torino_streets.csv; Torino_sidewalks keeps Torino's edge IDs).

Inputs: report/<tag>_edge_volume.csv for the 40 runs, both networks in src/main/resources, and
the Vodafone presence layer. Writes ab_networks.md and the per-street table beside this file.
"""

import glob
from pathlib import Path

import geopandas as gpd
import numpy as np
import pandas as pd
from pyproj import Transformer

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
RES = REPO / "src/main/resources"
SEASONS = ["winter", "spring", "summer", "autumn"]
ARMS = {"base": "Torino", "sidewalks": "Torino_sidewalks"}
RINGS = [0, 500, 1000, 1500, 2000, 3000, 4000, 5000, 6000, 8000, 20000]
REFERENCE_RING = 6  # 4-5 km


def edges(city):
    e = gpd.read_file(RES / city / f"{city}_edges.gpkg", columns=["edgeID"])
    e["edgeID"] = e.edgeID.astype(int)
    e["length"] = e.geometry.length
    return e


def metres(tag, e):
    v = pd.read_csv(REPO / f"analysis/end_of_project/season10/report/{tag}_edge_volume.csv")
    n = (v.vuln_count + v.norm_count).groupby(v.edgeID).sum()
    return e.edgeID.map(n).fillna(0) * e.length.values


def main():
    net = {arm: edges(city) for arm, city in ARMS.items()}
    streets = pd.read_csv(RES / "Torino/Torino_streets.csv").set_index("edgeID")
    vf = gpd.read_file(REPO / "analysis/validation/Torino/data/torino_presence.gpkg",
                       layer="quadkeys_long")
    band_cols = [c for c in vf.columns if c.startswith("passing_")]
    vf["passing"] = vf[band_cols].sum(axis=1)
    cells = vf[vf.season == "winter"][["quadkey", "geometry"]].reset_index(drop=True)
    x, y = Transformer.from_crs(4326, cells.crs, always_xy=True).transform(7.6858, 45.0712)
    cells["d"] = cells.geometry.centroid.distance(gpd.points_from_xy([x], [y], crs=cells.crs)[0])
    cells["ring"] = pd.cut(cells.d, RINGS)
    pieces = {arm: gpd.overlay(e[["edgeID", "length", "geometry"]], cells[["quadkey", "geometry"]],
                               how="intersection", keep_geom_type=True)
              for arm, e in net.items()}
    for p in pieces.values():
        p["w"] = p.geometry.length / p["length"]

    ring_rows, rho_rows, street_rows, role_rows = [], [], [], []
    for arm, e in net.items():
        role = e.edgeID.map(streets.street_role).fillna("path")
        sid = e.edgeID.map(streets.street_id)
        for t in sorted(glob.glob(str(HERE / f"report/season10_{arm}_*_edge_volume.csv"))):
            tag = Path(t).name[: -len("_edge_volume.csv")]
            season, seed = tag.split("_")[-2], tag.split("_")[-1][1:]
            m = pd.Series(metres(tag, e).values, index=e.edgeID.values)
            # walked metres per cell: each edge's metres split by its length inside the cell
            p = pieces[arm]
            cell_m = (p.edgeID.map(m) * p.w).groupby(p.quadkey).sum()
            obs = vf[vf.season == season].set_index("quadkey").passing
            c = cells.assign(sim=cells.quadkey.map(cell_m).fillna(0), obs=cells.quadkey.map(obs))
            c = c[c.obs.notna()]
            r = c.groupby("ring", observed=True)[["sim", "obs"]].mean()
            r = r / r.iloc[REFERENCE_RING]
            r["arm"], r["season"], r["seed"] = arm, season, seed
            ring_rows.append(r.reset_index())
            rho_rows.append({"arm": arm, "season": season, "seed": seed,
                             "rho": c.sim.corr(c.obs, method="spearman")})
            sm = m.groupby(sid.values).sum()
            street_rows.append(pd.DataFrame({"street_id": sm.index, "m": sm.values, "arm": arm,
                                             "season": season, "seed": seed}))
            rm = m.groupby(role.values).sum()
            role_rows.append(pd.DataFrame({"role": rm.index, "m": rm.values, "arm": arm,
                                           "season": season, "seed": seed}))

    rings = pd.concat(ring_rows)
    rho = pd.DataFrame(rho_rows)
    st = pd.concat(street_rows)
    roles = pd.concat(role_rows)

    out = ["# Season10: Torino against Torino_sidewalks", "",
           "Same seeds, dates and size (20% of residents, 3 days, 5 seeds x 4 seasons) on both "
           "networks. Figures are means over the 20 runs of each network unless stated.", ""]

    # 1. walked metres by distance from Piazza Castello, per 216 m cell, relative to 4-5 km
    g = rings.groupby(["ring", "arm"], observed=True)[["sim", "obs"]].mean().unstack("arm")
    out += ["## 1. Walked metres per 216 m cell by distance from Piazza Castello, "
            "relative to the 4-5 km ring", "",
            "| ring (m) | Torino | Torino_sidewalks | Vodafone passing |", "|---|---|---|---|"]
    for ring, r in g.iterrows():
        out.append(f"| {ring.left:.0f}-{ring.right:.0f} | {r[('sim', 'base')]:.2f} "
                   f"| {r[('sim', 'sidewalks')]:.2f} | {r[('obs', 'base')]:.2f} |")
    out += ["", "Spearman, walked metres per cell against Vodafone passing presence (whole day), "
            "mean ± sd over the 5 seeds:", "", "| season | Torino | Torino_sidewalks |",
            "|---|---|---|"]
    for s in SEASONS:
        cell = []
        for arm in ARMS:
            v = rho[(rho.arm == arm) & (rho.season == s)].rho
            cell.append(f"{v.mean():.3f} ± {v.std(ddof=1):.3f}")
        out.append(f"| {s} | {cell[0]} | {cell[1]} |")

    # 2. where the metres are, by the role of the edge on the Torino network
    rt = roles.groupby(["role", "arm"]).m.sum().unstack("arm") / 20 / 1000
    rt = rt.reindex(["carriageway", "side_road", "sidewalk", "pedestrian_street", "crossing",
                     "path"]).fillna(0)
    total = rt.sum()
    out += ["", "## 2. Walked km per run by the role of the edge (roles from Torino_streets.csv)",
            "", "| role | Torino | share | Torino_sidewalks | share |", "|---|---|---|---|---|"]
    for role, r in rt.iterrows():
        out.append(f"| {role} | {r.base:,.0f} | {100 * r.base / total.base:.1f}% "
                   f"| {r.sidewalks:,.0f} | {100 * r.sidewalks / total.sidewalks:.1f}% |")
    out.append(f"| total | {total.base:,.0f} | | {total.sidewalks:,.0f} | |")

    # 3. streets
    per = st.groupby(["street_id", "arm"]).m.sum().unstack("arm").fillna(0) / 20 / 1000
    names = streets.groupby("street_id").street_name.first()
    removed = set(net["base"].edgeID) - set(net["sidewalks"].edgeID)
    lost = streets.loc[streets.index.isin(removed)]
    lost = lost[lost.street_role.isin(["carriageway", "side_road"])]
    per["name"] = names
    per["lost_km"] = (net["base"].set_index("edgeID").length.reindex(lost.index)
                      .groupby(lost.street_id).sum() / 1000)
    per["lost_km"] = per.lost_km.fillna(0)
    per["ratio"] = per.sidewalks / per.base
    per.to_csv(HERE / "ab_streets.csv")
    busy = per[per.base >= 20]
    rho_st = busy.base.corr(busy.sidewalks, method="spearman")
    hit = busy[busy.lost_km >= 0.3]
    rest = busy[busy.lost_km == 0]
    out += ["", "## 3. Walked km per run by street (carriageways, side roads and sidewalks "
            "together)", "",
            f"{len(busy)} streets with at least 20 km a run on Torino. Spearman between the two "
            f"networks: {rho_st:.3f}. Median ratio Torino_sidewalks / Torino: "
            f"{hit.ratio.median():.2f} on the {len(hit)} streets that lost at least 300 m of "
            f"carriageway, {rest.ratio.median():.2f} on the {len(rest)} that lost none.", "",
            "The 15 busiest streets on Torino:", "",
            "| street | Torino | Torino_sidewalks | ratio | carriageway removed (km) |",
            "|---|---|---|---|---|"]
    for _, r in busy.sort_values("base", ascending=False).head(15).iterrows():
        out.append(f"| {r['name']} | {r.base:,.0f} | {r.sidewalks:,.0f} | {r.ratio:.2f} "
                   f"| {r.lost_km:.2f} |")
    out += ["", "Largest losses and gains among streets that lost at least 300 m of carriageway:",
            "", "| street | Torino | Torino_sidewalks | ratio | carriageway removed (km) |",
            "|---|---|---|---|---|"]
    h = hit.sort_values("ratio")
    for _, r in pd.concat([h.head(6), h.tail(6)]).iterrows():
        out.append(f"| {r['name']} | {r.base:,.0f} | {r.sidewalks:,.0f} | {r.ratio:.2f} "
                   f"| {r.lost_km:.2f} |")

    (HERE / "ab_networks.md").write_text("\n".join(out) + "\n", encoding="utf-8")
    print("\n".join(out))


if __name__ == "__main__":
    main()
