"""A city whose road centrelines are removed wherever sidewalks are drawn on both sides.

Where OSM draws a street's sidewalks as ways of their own, a pedestrian network built with
cityImage's ``keep_both`` policy carries the street two or three times: the centreline, which
nobody walks, and the sidewalks. This writes a copy of a built city without those centrelines,
editing its layers in place instead of rebuilding them, so every edge and node that stays keeps
its ``edgeID`` / ``nodeID`` and the two cities can be compared edge for edge:

    python drop_duplicate_centrelines.py --city Torino --out Torino_sidewalks

1. **Candidates**: roads with a walkway drawn along both sides over ``--min-coverage`` of their
   length (cityImage's ``sidewalk_coverage``: parallel within 20 degrees, within 25 m, crossings
   excluded). Tags are not read: ``sidewalk=separate`` is on a minority of such roads.
2. **Removal**, longest first: a centreline goes only if its two ends stay joined by a route at
   most ``--detour-ratio`` times its length or ``--detour-slack`` metres longer, whichever is more,
   so no part of the network is cut off or sent round the block.
3. **Stubs**: a dead end the removal created, shorter than ``--max-stub``, is the piece of a side
   street or crossing that ran into the carriageway, and goes too, repeatedly.
4. **Pseudo-nodes** the removal created are dissolved where the two edges meeting there carry the
   same attributes; the joined edge keeps the ``edgeID`` of the longer one. Where they differ (a
   sidewalk and a crossing) the node stays, so no edge loses what it is.

Layers keyed on edges or nodes are carried over: the lit edges (``mean_lux`` and ``pct_unlit``
length-weighted over a joined edge, ``min_lux`` its minimum), the directional lighting lookup (a
joined edge is entered from each end through its end pieces' records), the street isovists
(means weighted by observation points, ``iso_area_min`` the minimum). Layers that do not depend
on the network are copied. Re-run ``street_corridors.py --city <out>`` afterwards.
"""

from __future__ import annotations

import argparse
import json
import shutil

import cityImage as ci
import geopandas as gpd
import networkx as nx
import numpy as np
import pandas as pd
from shapely.geometry import LineString

import paths
import provenance

# Layers that do not depend on the network, copied under the new city's name.
COPIED = ["POIs.gpkg", "barriers.gpkg", "buildings.gpkg", "censusData.gpkg", "employment.gpkg",
          "obstructions.gpkg"]
# Two edges meeting at a pseudo-node are joined only when all of these agree.
SAME_EDGE = ["name", "highway", "footway", "foot", "cycleway", "access", "ped", "lit", "bridge",
             "a_rivers", "w_parks", "p_barr", "n_barr", "sep_barr"]
ISOVIST_MEANS = ["iso_area_mean", "iso_compactness_mean", "iso_occlusions_mean",
                 "iso_occlusivity_mean"]


def equal(a, b) -> bool:
    return (pd.isna(a) and pd.isna(b)) or a == b


def oriented(geometry: LineString, start) -> list:
    """The line's coordinates, starting from its end nearest ``start``."""
    coords = list(geometry.coords)
    if np.hypot(*np.subtract(coords[-1], start)) < np.hypot(*np.subtract(coords[0], start)):
        coords.reverse()
    return coords


def remove_centrelines(edges, graph, candidates, ratio, slack):
    removed = []
    for row in candidates.sort_values("length", ascending=False).itertuples():
        if row.u == row.v:
            continue
        graph.remove_edge(row.u, row.v, key=row.edgeID)
        allowed = max(ratio * row.length, row.length + slack)
        reach = nx.single_source_dijkstra_path_length(graph, row.u, cutoff=allowed,
                                                      weight="length")
        if row.v in reach:
            removed.append(row.edgeID)
        else:
            graph.add_edge(row.u, row.v, key=row.edgeID, length=row.length)
    return removed


def trim_stubs(graph, degree_before, lengths, max_stub):
    trimmed = []
    while True:
        stubs = [(n, next(iter(graph.edges(n, keys=True)))) for n, d in graph.degree()
                 if d == 1 and degree_before[n] > 1]
        stubs = [(n, e) for n, e in stubs if lengths[e[2]] < max_stub]
        if not stubs:
            return trimmed
        for _, (a, b, key) in stubs:
            if graph.has_edge(a, b, key):
                graph.remove_edge(a, b, key)
                trimmed.append(key)


def join_pseudo_nodes(edges, graph, degree_before):
    """Join the edges meeting at each pseudo-node the removal created, where they agree.

    Returns the joined edges and, for each, its pieces in order from its ``u`` to its ``v``.
    """
    rows = {row["edgeID"]: row for row in edges.to_dict("records")}
    pieces = {key: [key] for key in rows}  # the original edges, in order from u to v
    changed = True
    while changed:
        changed = False
        for node in [n for n, d in graph.degree() if d == 2 and degree_before[n] != 2]:
            if node not in graph or graph.degree(node) != 2:
                continue
            (_, a, k1), (_, b, k2) = list(graph.edges(node, keys=True))
            if a == node or b == node or a == b:
                continue
            r1, r2 = rows[k1], rows[k2]
            if not all(equal(r1[c], r2[c]) for c in SAME_EDGE if c in r1):
                continue
            keep, drop = (k1, k2) if r1["length"] >= r2["length"] else (k2, k1)
            # The joined edge runs from a, through node, to b.
            first, second = (rows[k1], rows[k2])
            coords = oriented(first["geometry"], graph.nodes[a]["xy"])[:-1] + \
                oriented(second["geometry"], graph.nodes[node]["xy"])
            joined = dict(rows[keep])
            joined.update(u=a, v=b, length=first["length"] + second["length"],
                          geometry=LineString(coords))
            pieces[keep] = _ordered(pieces[k1], rows[k1], a) + _ordered(pieces[k2], rows[k2], node)
            rows[keep] = joined
            del rows[drop], pieces[drop]
            graph.remove_edge(node, a, k1)
            graph.remove_edge(node, b, k2)
            graph.add_edge(a, b, key=keep, length=joined["length"])
            graph.remove_node(node)
            changed = True
    return rows, pieces


def _ordered(chain, row, start):
    """``chain`` (pieces from ``row['u']`` to ``row['v']``) as walked from ``start``."""
    return chain if row["u"] == start else chain[::-1]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--city", required=True, help="the built city, e.g. Torino")
    parser.add_argument("--out", required=True, help="the new city, e.g. Torino_sidewalks")
    parser.add_argument("--min-coverage", type=float, default=0.8,
                        help="share of a road's length with a walkway on each side")
    parser.add_argument("--detour-ratio", type=float, default=1.5)
    parser.add_argument("--detour-slack", type=float, default=50.0, help="metres")
    parser.add_argument("--max-stub", type=float, default=25.0, help="metres")
    args = parser.parse_args()

    source, target = paths.resources_dir(args.city), paths.resources_dir(args.out)
    target.mkdir(parents=True, exist_ok=True)

    def src(name):
        return source / f"{args.city}_{name}"

    def out(name):
        return target / f"{args.out}_{name}"

    edges = gpd.read_file(src("edges.gpkg"))
    nodes = gpd.read_file(src("nodes.gpkg"))
    edges["length"] = edges.geometry.length

    coverage = ci.sidewalk_coverage(edges)
    both = coverage.min(axis=1) >= args.min_coverage
    candidates = edges[both.values]

    graph = nx.MultiGraph()
    for row in nodes.itertuples():
        graph.add_node(row.nodeID, xy=(row.geometry.x, row.geometry.y))
    for row in edges.itertuples():
        graph.add_edge(row.u, row.v, key=row.edgeID, length=row.length)
    degree_before = dict(graph.degree())
    lengths = dict(zip(edges.edgeID, edges.length, strict=True))

    removed = remove_centrelines(edges, graph, candidates, args.detour_ratio, args.detour_slack)
    trimmed = trim_stubs(graph, degree_before, lengths, args.max_stub)
    gone = set(removed) | set(trimmed)
    remaining = edges[~edges.edgeID.isin(gone)]
    rows, pieces = join_pseudo_nodes(remaining, graph, degree_before)
    joined = {key: chain for key, chain in pieces.items() if len(chain) > 1}

    new_edges = gpd.GeoDataFrame(list(rows.values()), geometry="geometry", crs=edges.crs)
    new_edges = new_edges[edges.columns].sort_values("edgeID").reset_index(drop=True)
    new_edges["length"] = new_edges.geometry.length
    live = set(new_edges.u) | set(new_edges.v)
    new_nodes = nodes[nodes.nodeID.isin(live)].reset_index(drop=True)
    assert nx.number_connected_components(graph.subgraph(live)) == 1, "the network split"

    # Lit edges.
    lit = gpd.read_file(src("edges_illuminated_continuous.gpkg"), ignore_geometry=True)
    lit = lit.set_index("edgeID")
    lux = lit.loc[new_edges.edgeID, ["min_lux", "mean_lux", "pct_unlit"]].copy()
    for key, chain in joined.items():
        ids = chain
        w = edges.set_index("edgeID").loc[ids, "length"].values
        lux.loc[key, "min_lux"] = lit.loc[ids, "min_lux"].min()
        for c in ("mean_lux", "pct_unlit"):
            lux.loc[key, c] = np.average(lit.loc[ids, c].values, weights=w)
    new_lit = new_edges.merge(lux, left_on="edgeID", right_index=True)

    # Directional lighting: entering an edge from each end.
    lookup = pd.read_csv(src("directional_lighting_lookup.csv"))
    entry = {(r.current_node_id, r.target_node_id): r for r in lookup.itertuples(index=False)}
    original = edges.set_index("edgeID")
    records = []
    for idx, row in enumerate(new_edges.itertuples()):
        chain = joined.get(row.edgeID, [row.edgeID])
        first, last = original.loc[chain[0]], original.loc[chain[-1]]
        for start, piece in ((row.u, first), (row.v, last)):
            far = piece.v if piece.u == start else piece.u
            record = entry.get((start, far))
            if record is not None:
                records.append({"current_node_id": start,
                                "target_node_id": row.v if start == row.u else row.u,
                                "chosen_edge_idx": idx,
                                "visibility_mean_lux": record.visibility_mean_lux,
                                "visibility_min_lux": record.visibility_min_lux})
    new_lookup = pd.DataFrame(records, columns=lookup.columns)

    # Street isovists.
    iso = pd.read_csv(src("street_isovists.csv")).set_index("edgeID")
    new_iso = iso.loc[new_edges.edgeID].copy()
    for key, chain in joined.items():
        part = iso.loc[chain]
        new_iso.loc[key, "iso_points"] = part.iso_points.sum()
        new_iso.loc[key, "iso_area_min"] = part.iso_area_min.min()
        for c in ISOVIST_MEANS:
            new_iso.loc[key, c] = np.average(part[c], weights=part.iso_points)

    for path in [out(n) for n in ("edges.gpkg", "nodes.gpkg", "edges_illuminated_continuous.gpkg")]:
        path.unlink(missing_ok=True)
    new_edges.to_file(out("edges.gpkg"), layer=f"{args.out}_edges", driver="GPKG")
    new_nodes.to_file(out("nodes.gpkg"), layer=f"{args.out}_nodes", driver="GPKG")
    new_lit.to_file(out("edges_illuminated_continuous.gpkg"),
                    layer=f"{args.out}_edges_illuminated_continuous", driver="GPKG")
    new_lookup.to_csv(out("directional_lighting_lookup.csv"), index=False)
    new_iso.reset_index().to_csv(out("street_isovists.csv"), index=False)
    for name in COPIED:
        if src(name).exists():
            shutil.copyfile(src(name), out(name))
    shutil.copyfile(source / f"{args.city}.properties", target / f"{args.out}.properties")
    roads = paths.raw_dir(args.city) / f"{args.city}_osm_roads.gpkg"
    if roads.exists():
        paths.raw_dir(args.out).mkdir(parents=True, exist_ok=True)
        shutil.copyfile(roads, paths.raw_dir(args.out) / f"{args.out}_osm_roads.gpkg")

    km = lambda ids: float(original.loc[list(ids), "length"].sum() / 1000)  # noqa: E731
    summary = {
        "candidates": {"edges": int(len(candidates)), "km": round(km(candidates.edgeID), 1)},
        "removed": {"edges": len(removed), "km": round(km(removed), 1)},
        "kept_for_detour": {"edges": int(len(candidates) - len(removed)),
                            "km": round(km(set(candidates.edgeID) - set(removed)), 1)},
        "stubs_trimmed": {"edges": len(trimmed), "km": round(km(trimmed), 2)},
        "pseudo_nodes_dissolved": int(sum(len(c) - 1 for c in joined.values())),
        "edges": [int(len(edges)), int(len(new_edges))],
        "nodes": [int(len(nodes)), int(len(new_nodes))],
    }
    print(json.dumps(summary, indent=2))
    record = json.loads((source / f"{args.city}_provenance.json").read_text(encoding="utf-8"))
    record["drop_duplicate_centrelines"] = {
        "written": provenance.now(),
        "from_city": args.city,
        "settings": provenance.settings(args),
        "environment": provenance.environment(),
        "summary": summary,
    }
    (target / f"{args.out}_provenance.json").write_text(json.dumps(record, indent=2),
                                                         encoding="utf-8")


if __name__ == "__main__":
    main()
