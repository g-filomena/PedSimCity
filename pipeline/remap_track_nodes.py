#!/usr/bin/env python
"""Point a track layer's origin and destination node IDs at a rebuilt network.

A layer of walked tracks (London's GPS tracks) names each track's origin and destination by the
``nodeID`` of the network it was matched to. A rebuilt network has new IDs, so each end takes the
node nearest to the track's first point, walking in from that end, within ``--tolerance`` of a
node: a track that starts outside the network takes the node where it enters it. The previous IDs
are kept as ``<column>_old``, the snap distance as ``<column>_snap_m``, and the length of track
walked in from the end as ``<column>_outside_m``; ends with no point within tolerance anywhere
take the nearest node and are listed.

  python pipeline/remap_track_nodes.py --city London --tracks London_GPSIES_tracks.gpkg
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import geopandas as gpd
import numpy as np
from scipy.spatial import cKDTree

import paths


def remap(tracks: gpd.GeoDataFrame, nodes: gpd.GeoDataFrame, origin: str, destination: str,
          tolerance: float) -> gpd.GeoDataFrame:
    nodes = nodes.to_crs(tracks.crs)
    tree = cKDTree(np.column_stack([nodes.geometry.x, nodes.geometry.y]))
    ids = nodes["nodeID"].to_numpy()
    out = tracks.copy()
    for column, reverse in ((origin, False), (destination, True)):
        chosen, snapped, outside = [], [], []
        for line in tracks.geometry:
            coords = np.array(line.coords)[:, :2]
            if reverse:
                coords = coords[::-1]
            distance, index = tree.query(coords)
            inside = np.flatnonzero(distance <= tolerance)
            k = inside[0] if len(inside) else 0
            chosen.append(ids[index[k]])
            snapped.append(distance[k])
            steps = np.hypot(*np.diff(coords[: k + 1], axis=0).T) if k else np.array([0.0])
            outside.append(float(steps.sum()))
        out[f"{column}_old"] = tracks[column]
        out[column] = np.array(chosen).astype(int)
        out[f"{column}_snap_m"] = np.round(snapped, 1)
        out[f"{column}_outside_m"] = np.round(outside, 1)
    walked_in = out[(out[f"{origin}_outside_m"] > 0) | (out[f"{destination}_outside_m"] > 0)]
    print(f"{len(walked_in)} tracks start or end outside the network and take the node they enter by")
    far = out[(out[f"{origin}_snap_m"] > tolerance) | (out[f"{destination}_snap_m"] > tolerance)]
    print(f"{len(out)} tracks; snap median {np.median(out[f'{origin}_snap_m']):.1f} m (origin), "
          f"{np.median(out[f'{destination}_snap_m']):.1f} m (destination); "
          f"{len(far)} beyond {tolerance:.0f} m")
    if len(far):
        print(far[[c for c in ("uniqueID", "name", f"{origin}_snap_m", f"{destination}_snap_m")
                   if c in far.columns]].to_string(index=False))
    if len(walked_in):
        print(walked_in[[c for c in ("uniqueID", "name", f"{origin}_outside_m",
                                     f"{destination}_outside_m") if c in walked_in.columns]]
              .to_string(index=False))
    return out


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--city", required=True)
    parser.add_argument("--tracks", required=True, help="the track layer in resources/<City>/")
    parser.add_argument("--nodes", type=Path, default=None,
                        help="the rebuilt nodes (default resources/<City>/<City>_nodes.gpkg)")
    parser.add_argument("--origin", default="origin")
    parser.add_argument("--destination", default="destinatio")
    parser.add_argument("--tolerance", type=float, default=25.0)
    parser.add_argument("--out", type=Path, default=None, help="default: overwrite --tracks")
    args = parser.parse_args(argv)

    resources = paths.resources_dir(args.city)
    tracks_path = resources / args.tracks
    nodes = gpd.read_file(args.nodes or resources / f"{args.city}_nodes.gpkg")
    tracks = gpd.read_file(tracks_path)
    out = remap(tracks, nodes, args.origin, args.destination, args.tolerance)
    target = args.out or tracks_path
    Path(target).unlink(missing_ok=True)
    out.to_file(target, driver="GPKG", layer=Path(target).stem)
    print(f"written {target}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
