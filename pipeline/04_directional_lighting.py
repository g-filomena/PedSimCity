"""Step 4: build directional lighting lookup.

Reads:
- <city>_edges.gpkg
- <city>_nodes_2m_densified_illuminated.gpkg

Writes:
- <city>_directional_lighting_lookup.csv

`--input_dir` is the resources folder path.
`--city_name` is the filename prefix; if omitted, the input folder name is used.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import geopandas as gpd
import pandas as pd


VISIBILITY_HORIZON_M = 12.0


def first_existing(paths: list[Path], label: str) -> Path:
    for path in paths:
        if path.exists():
            return path
    joined = "\n  ".join(str(p) for p in paths)
    raise FileNotFoundError(f"Could not find {label}. Tried:\n  {joined}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Build directional lighting lookup CSV.")
    parser.add_argument("--input_dir", required=True)
    parser.add_argument(
        "--city_name",
        default=None,
        help="Filename prefix before _edges.gpkg. Defaults to input folder name.",
    )
    args = parser.parse_args()

    base_dir = Path(os.path.abspath(args.input_dir))
    city = args.city_name or base_dir.name

    edges_path = first_existing([base_dir / f"{city}_edges.gpkg"], "edges layer")
    nodes_2m_path = first_existing(
        [base_dir / f"{city}_nodes_2m_densified_illuminated.gpkg"],
        "densified illuminated nodes",
    )
    output_path = base_dir / f"{city}_directional_lighting_lookup.csv"

    print("Loading datasets...")
    print(f"  resources folder: {base_dir}")
    print(f"  city file prefix: {city}")
    print(f"  edges: {edges_path}")
    print(f"  nodes: {nodes_2m_path}")

    edges = gpd.read_file(edges_path)
    nodes_2m = gpd.read_file(nodes_2m_path)

    required_node_cols = {"dist_along_edge", "calculated_lux"}
    missing = required_node_cols - set(nodes_2m.columns)
    if missing:
        raise KeyError(f"Missing required columns in {nodes_2m_path}: {sorted(missing)}")

    # Join sample points to edges by a stable key: the edge's edgeID (matching the parent_edge_id
    # written by step 3) when available, else the positional index. Avoids relying on both files
    # being read in the same row order.
    use_edge_id = "parent_edge_id" in nodes_2m.columns and "edgeID" in edges.columns
    node_key_col = "parent_edge_id" if use_edge_id else "parent_edge_idx"
    if node_key_col not in nodes_2m.columns:
        raise KeyError(f"Nodes file lacks a parent-edge key column: {nodes_2m_path}")

    required_edge_cols = {"u", "v"}
    missing_edge_cols = sorted(required_edge_cols - set(edges.columns))
    if missing_edge_cols:
        raise KeyError(
            f"Missing required edge topology columns in {edges_path}: {missing_edge_cols}. "
            "Directional lighting CSV requires integer node IDs matching NodeGraph.getID(); "
            "refusing to generate node_start_<idx>/node_end_<idx> fallback IDs."
        )

    def require_int_node_id(value, column_name: str, edge_idx: int) -> int:
        if pd.isna(value):
            raise ValueError(
                f"Missing node ID in column '{column_name}' for edge row {edge_idx}."
            )
        try:
            return int(value)
        except Exception as exc:
            raise ValueError(
                f"Invalid node ID in column '{column_name}' for edge row {edge_idx}: {value!r}"
            ) from exc

    directional_records = []
    grouped_points = nodes_2m.groupby(node_key_col)

    print("Calculating directional lightness profiles...")
    for idx, row in edges.iterrows():
        key = row["edgeID"] if use_edge_id else idx
        if key not in grouped_points.groups:
            continue

        edge_pts = grouped_points.get_group(key)
        edge_length = row.geometry.length if row.geometry is not None else 0.0

        from_node = require_int_node_id(row["u"], "u", idx)
        to_node = require_int_node_id(row["v"], "v", idx)

        start_slice = edge_pts[edge_pts["dist_along_edge"] <= VISIBILITY_HORIZON_M]
        directional_records.append(
            {
                "current_node_id": from_node,
                "target_node_id": to_node,
                "chosen_edge_idx": idx,
                "visibility_mean_lux": start_slice["calculated_lux"].mean() if not start_slice.empty else 0.0,
                "visibility_min_lux": start_slice["calculated_lux"].min() if not start_slice.empty else 0.0,
            }
        )

        end_slice = edge_pts[edge_pts["dist_along_edge"] >= (edge_length - VISIBILITY_HORIZON_M)]
        directional_records.append(
            {
                "current_node_id": to_node,
                "target_node_id": from_node,
                "chosen_edge_idx": idx,
                "visibility_mean_lux": end_slice["calculated_lux"].mean() if not end_slice.empty else 0.0,
                "visibility_min_lux": end_slice["calculated_lux"].min() if not end_slice.empty else 0.0,
            }
        )

    lookup_df = pd.DataFrame(
        directional_records,
        columns=[
            "current_node_id",
            "target_node_id",
            "chosen_edge_idx",
            "visibility_mean_lux",
            "visibility_min_lux",
        ],
    )
    lookup_df["visibility_mean_lux"] = lookup_df["visibility_mean_lux"].fillna(0.0)
    lookup_df["visibility_min_lux"] = lookup_df["visibility_min_lux"].fillna(0.0)

    lookup_df.to_csv(output_path, index=False)
    print(f"saved: {output_path} ({len(lookup_df)} rows)")


if __name__ == "__main__":
    main()
