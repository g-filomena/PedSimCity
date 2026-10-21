"""Inbound commuters for PedSimCity — ISTAT adapter (Italian cities).

Writes ``src/main/resources/<City>/<City>_inbound_commuters.csv``: the people who live outside the
city and travel into it every day to work or study, by where they live, how they travel, when they
leave home and how long the journey takes. The activity model builds them as agents that arrive in
the city by that mode and walk from where they arrive.

Source: ISTAT, Matrice del pendolarismo, 15th Censimento generale della popolazione (9 October
2011), ``matrix_pendo2011_10112014.txt``. The detailed records (``L``) carry mode, departure band
and duration; they cover residents in households and, in municipalities over 20,000 residents,
are estimates from the census sample, hence the decimals. Every flow whose usual place of study
or work is the city and whose residence is another Italian municipality is kept.

The archive is downloaded once into ``inputData/<City>/istat_pendolarismo_2011/``. Where each
residence municipality is, is the centroid of its 2011 census sections, from the same
``R<RR>_11_WGS84.zip`` the employment adapter uses; municipalities outside that region get none.

The output schema is the contract the Java side reads, and any national adapter must produce it:

- ``origin_id``          where the commuters live (here the ISTAT municipality code)
- ``origin_x, origin_y`` its centroid in the CRS of ``<City>_edges.gpkg``; empty when unknown
- ``motive``             ``work`` or ``study``
- ``mode``               ``train``, ``tram``, ``metro``, ``bus``, ``coach``, ``works_bus``,
                         ``car_driver``, ``car_passenger``, ``motorbike``, ``bicycle``, ``walk``
                         or ``other``
- ``leave_from_min, leave_to_min``        minutes of the day within which they leave home
- ``journey_from_min, journey_to_min``    how long the journey takes, in minutes
- ``female``             1 for women, 0 for men
- ``people``             how many

ISTAT gives the departure in four bands (before 7:15, 7:15-8:14, 8:15-9:14, after 9:14) and the
duration in four (up to 15 minutes, 16-30, 31-60, over 60). The open ends are this adapter's
assumptions, written here once: leaving before 7:15 is taken from 6:00, after 9:14 up to 10:30,
and a journey over an hour as up to 90 minutes.

    python inbound_commuters_istat.py --city Torino --procom 1272
"""

from __future__ import annotations

import argparse
import collections
import csv
import io
import urllib.request
import zipfile

import geopandas as gpd

import paths
import provenance

MATRIX_URL = ("https://www.istat.it/storage/cartografia/matrici_pendolarismo/"
              "matrici_pendolarismo_2011.zip")
GEO_URL = ("https://www.istat.it/storage/cartografia/basi_territoriali/WGS_84_UTM/2011/"
           "R{region:02d}_11_WGS84.zip")
MATRIX_MEMBER = "matrix_pendo2011_10112014.txt"
MOTIVES = {"1": "study", "2": "work"}
MODES = {"01": "train", "02": "tram", "03": "metro", "04": "bus", "05": "coach", "06": "works_bus",
         "07": "car_driver", "08": "car_passenger", "09": "motorbike", "10": "bicycle",
         "11": "other", "12": "walk"}
LEAVE = {"1": (360, 435), "2": (435, 495), "3": (495, 555), "4": (555, 630)}
JOURNEY = {"1": (5, 15), "2": (16, 30), "3": (31, 60), "4": (61, 90)}


def fetch(url: str, target) -> None:
    if target.exists():
        return
    print(f"downloading {url}")
    request = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
    with urllib.request.urlopen(request, timeout=600) as response:
        target.write_bytes(response.read())


def member(archive: zipfile.ZipFile, name: str) -> str:
    for entry in archive.namelist():
        if entry.replace("\\", "/").split("/")[-1] == name:
            return entry
    raise FileNotFoundError(f"{name} not in {archive.filename}")


def inbound_flows(matrix_zip, procom: int) -> collections.Counter:
    """People per (residence, motive, mode, departure, duration, female) commuting into procom."""
    province, municipality = f"{procom // 1000:03d}", f"{procom % 1000:03d}"
    flows = collections.Counter()
    with zipfile.ZipFile(matrix_zip) as archive:
        with archive.open(member(archive, MATRIX_MEMBER)) as raw:
            for line in io.TextIOWrapper(raw, encoding="latin-1"):
                p = line.split()
                # L record, another Italian municipality, usually studying or working in procom.
                if (len(p) < 14 or p[0] != "L" or p[6] != "2" or p[7] != province
                        or p[8] != municipality or (p[2] == province and p[3] == municipality)):
                    continue
                residence = int(p[2]) * 1000 + int(p[3])
                key = (residence, MOTIVES[p[5]], p[10], p[11], p[12], 1 if p[4] == "2" else 0)
                flows[key] += float(p[13])
    return flows


def centroids(geo_zip, region: int, folder, crs) -> dict:
    shapefile = folder / f"R{region:02d}_11_WGS84" / f"R{region:02d}_11_WGS84.shp"
    if not shapefile.exists():
        with zipfile.ZipFile(geo_zip) as archive:
            archive.extractall(folder)
    sections = gpd.read_file(shapefile, columns=["PRO_COM"]).to_crs(crs)
    municipalities = sections.dissolve(by="PRO_COM")
    points = municipalities.geometry.centroid
    return {int(code): (point.x, point.y) for code, point in points.items()}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--city", required=True, help="folder and file prefix, e.g. Torino")
    parser.add_argument("--procom", type=int, required=True,
                        help="ISTAT municipality code without the region, e.g. 1272 for Torino")
    parser.add_argument("--region", type=int, default=1,
                        help="ISTAT region code whose municipalities get a centroid (1, Piedmont)")
    args = parser.parse_args()

    folder = paths.raw_dir(args.city) / "istat_pendolarismo_2011"
    folder.mkdir(parents=True, exist_ok=True)
    matrix_zip = folder / "matrici_pendolarismo_2011.zip"
    fetch(MATRIX_URL, matrix_zip)
    geo_folder = paths.raw_dir(args.city) / "istat_employment_2011"
    geo_folder.mkdir(parents=True, exist_ok=True)
    geo_zip = geo_folder / f"R{args.region:02d}_11_WGS84.zip"
    fetch(GEO_URL.format(region=args.region), geo_zip)

    edges = paths.require_input(args.city, "edges.gpkg", "the street network")
    crs = gpd.read_file(edges, rows=1).crs
    flows = inbound_flows(matrix_zip, args.procom)
    where = centroids(geo_zip, args.region, geo_folder, crs)

    target = paths.resources_dir(args.city) / f"{args.city}_inbound_commuters.csv"
    with open(target, "w", newline="", encoding="utf-8") as out:
        writer = csv.writer(out)
        writer.writerow(["origin_id", "origin_x", "origin_y", "motive", "mode", "leave_from_min",
                         "leave_to_min", "journey_from_min", "journey_to_min", "female", "people"])
        for key in sorted(flows):
            residence, motive, mode, departure, duration, female = key
            x, y = where.get(residence, ("", ""))
            writer.writerow([residence, round(x, 1) if x != "" else "", round(y, 1) if y != "" else "",
                             motive, MODES.get(mode, "other"), *LEAVE[departure], *JOURNEY[duration],
                             female, round(flows[key], 2)])

    by_motive = collections.Counter()
    placed = 0.0
    for key, people in flows.items():
        by_motive[key[1]] += people
        if key[0] in where:
            placed += people
    total = sum(by_motive.values())
    print(f"{target.name}: {len(flows)} flows, {total:,.0f} people "
          f"(work {by_motive['work']:,.0f}, study {by_motive['study']:,.0f}); "
          f"{100 * placed / total:.1f}% from municipalities with a centroid")
    provenance.update(args.city, "inbound_commuters", {
        "written": provenance.now(),
        "source": "ISTAT, Matrice del pendolarismo 2011, L records into the city",
        "urls": [MATRIX_URL, GEO_URL.format(region=args.region)],
        "settings": provenance.settings(args),
        "environment": provenance.environment(),
        "inputs": [provenance.file_info(matrix_zip), provenance.file_info(geo_zip)],
        "output": provenance.file_info(target),
        "people": round(total),
    })


if __name__ == "__main__":
    main()
