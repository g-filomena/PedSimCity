"""Street isovists for a city: how open each street is and how many corners along it hide what is
beyond them, from cityImage.street_isovists over the city's building footprints.

Writes ``src/main/resources/<City>/<City>_street_isovists.csv`` (the measures by ``edgeID``, which
the night module reads), and for looking at them ``<out>/<City>_street_isovists.gpkg`` and two
maps, ``<City>_isovist_area.png`` and ``<City>_isovist_occlusions.png``. The measures are keyed by
``edgeID``, so they must be recomputed after every network rebuild.
Run in the pedsimcity-prep environment, with cityImage installed from ../cityImage:

    python pipeline/street_isovists.py Torino --workers 8
"""

import argparse
import time
from pathlib import Path

import geopandas as gpd
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import cityImage as ci  # noqa: E402

REPO = Path(__file__).resolve().parent.parent


def draw(edges, column, title, path, log=False):
    values = edges[column].to_numpy(dtype=float)
    if log:
        values = np.log10(np.clip(values, 1.0, None))
    lo, hi = np.nanpercentile(values, [2, 98])
    fig, ax = plt.subplots(figsize=(14, 14), facecolor="#0b0b0c")
    ax.set_facecolor("#0b0b0c")
    edges.assign(v=values).plot(ax=ax, column="v", cmap="magma", vmin=lo, vmax=hi, linewidth=0.5)
    ax.set_title(title, color="white", fontsize=14)
    ax.set_axis_off()
    fig.savefig(path, dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("city")
    ap.add_argument("--workers", type=int, default=1)
    ap.add_argument("--spacing", type=float, default=20.0)
    ap.add_argument("--rays", type=int, default=64)
    ap.add_argument("--max-distance", type=float, default=100.0)
    ap.add_argument("--out", type=Path, default=REPO / "outputs" / "isovists")
    args = ap.parse_args()

    res = REPO / "src" / "main" / "resources" / args.city
    edges = gpd.read_file(res / f"{args.city}_edges.gpkg", columns=["edgeID", "highway", "name"])
    buildings = gpd.read_file(res / f"{args.city}_buildings.gpkg", columns=[])
    t0 = time.time()
    measures = ci.street_isovists(
        edges, buildings, spacing=args.spacing, n_rays=args.rays,
        max_distance=args.max_distance, num_workers=args.workers)
    print(f"{len(edges)} edges, {int(measures.iso_points.sum())} observer points, "
          f"{time.time() - t0:.0f} s")
    out = edges.join(measures)
    table = res / f"{args.city}_street_isovists.csv"
    out[["edgeID", *ci.visibility2d.ISOVIST_COLUMNS]].astype({"edgeID": int}).round(3).to_csv(
        table, index=False)
    print(f"-> {table}")
    args.out.mkdir(parents=True, exist_ok=True)
    target = args.out / f"{args.city}_street_isovists.gpkg"
    target.unlink(missing_ok=True)
    out.to_file(target, driver="GPKG")
    print(out[ci.visibility2d.ISOVIST_COLUMNS].describe().round(2).to_string())
    draw(out, "iso_area_mean", f"{args.city}: mean isovist area along each street (log10 m2)",
         args.out / f"{args.city}_isovist_area.png", log=True)
    draw(out, "iso_occlusions_mean", f"{args.city}: corners per isovist along each street",
         args.out / f"{args.city}_isovist_occlusions.png")
    print(f"-> {target}")


if __name__ == "__main__":
    main()
