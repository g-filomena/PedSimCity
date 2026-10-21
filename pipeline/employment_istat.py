"""Employment layer for PedSimCity — ISTAT adapter (Italian cities).

Writes ``src/main/resources/<City>/<City>_employment.gpkg``: one polygon per census section with
the number of people working there. The activity model reads it as the WORK attraction (where
jobs are), in place of the footprint of buildings tagged as offices or industry.

Source: ISTAT, 9th Censimento dell'industria e dei servizi (2011), employees (``ADDETTI``) of the
local units in each census section, summed over firms (``IM``), public institutions (``IP``) and
non-profit institutions (``NP``), with the 2011 census-section boundaries. 2011 is the latest year
ISTAT publishes at section level; the 2021 permanent census has no section-level employment.

Both archives are downloaded once into ``inputData/<City>/istat_employment_2011/``:

- ``dati-sce_2011.zip``: ``<RR>_AttEcon_SCE_2011.txt``, one row per section, legal type and
  ATECO group (``TIPO_SOGGETTO;CODREG;PROCOM;NSEZ;ATECO3;NUM_UNITA;ADDETTI;...``)
- ``R<RR>_11_WGS84.zip``: section polygons (``PRO_COM``, ``SEZ``), EPSG:32632

The output schema is the contract the Java side reads, and any national adapter must produce it:

- ``zoneID``     the section's identifier
- ``employees``  people working in the section
- ``employees_firms``, ``employees_public``, ``employees_nonprofit``  the same by legal type

The layer is written in the CRS of ``<City>_edges.gpkg``.

    python employment_istat.py --city Torino --procom 1272
"""

from __future__ import annotations

import argparse
import io
import urllib.request
import zipfile

import geopandas as gpd
import pandas as pd

import paths
import provenance

SCE_URL = "https://www.istat.it/storage/cartografia/variabili-censuarie/dati-sce_2011.zip"
GEO_URL = ("https://www.istat.it/storage/cartografia/basi_territoriali/WGS_84_UTM/2011/"
           "R{region:02d}_11_WGS84.zip")
LEGAL_TYPES = {"IM": "employees_firms", "IP": "employees_public", "NP": "employees_nonprofit"}


def fetch(url: str, target) -> None:
    if target.exists():
        return
    print(f"downloading {url}")
    with urllib.request.urlopen(url, timeout=600) as response:
        target.write_bytes(response.read())


def member(archive: zipfile.ZipFile, name: str) -> str:
    """The archive member whose file name is ``name``; ISTAT's zips use backslash separators."""
    for entry in archive.namelist():
        if entry.replace("\\", "/").split("/")[-1] == name:
            return entry
    raise FileNotFoundError(f"{name} not in {archive.filename}")


def employees_by_section(sce_zip, region: int, procom: int) -> pd.DataFrame:
    with zipfile.ZipFile(sce_zip) as archive:
        raw = archive.read(member(archive, f"{region:02d}_AttEcon_SCE_2011.txt"))
    table = pd.read_csv(io.BytesIO(raw), sep=";", usecols=["TIPO_SOGGETTO", "PROCOM", "NSEZ",
                                                           "ADDETTI"])
    table = table[table.PROCOM == procom]
    if table.empty:
        raise SystemExit(f"no employment rows for PROCOM {procom} in region {region}")
    by_type = table.pivot_table(index="NSEZ", columns="TIPO_SOGGETTO", values="ADDETTI",
                                aggfunc="sum", fill_value=0).rename(columns=LEGAL_TYPES)
    for column in LEGAL_TYPES.values():
        if column not in by_type:
            by_type[column] = 0
    by_type = by_type[list(LEGAL_TYPES.values())]
    by_type["employees"] = by_type.sum(axis=1)
    return by_type


def sections(geo_zip, region: int, procom: int, folder) -> gpd.GeoDataFrame:
    shapefile = folder / f"R{region:02d}_11_WGS84" / f"R{region:02d}_11_WGS84.shp"
    if not shapefile.exists():
        with zipfile.ZipFile(geo_zip) as archive:
            archive.extractall(folder)
    polygons = gpd.read_file(shapefile)
    polygons = polygons[polygons.PRO_COM == procom]
    return polygons[["SEZ2011", "SEZ", "geometry"]]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--city", required=True, help="folder and file prefix, e.g. Torino")
    parser.add_argument("--procom", type=int, required=True,
                        help="ISTAT municipality code without the region, e.g. 1272 for Torino")
    parser.add_argument("--region", type=int, default=1,
                        help="ISTAT region code (1, Piedmont, by default)")
    args = parser.parse_args()
    region = args.region

    folder = paths.raw_dir(args.city) / "istat_employment_2011"
    folder.mkdir(parents=True, exist_ok=True)
    sce_zip = folder / "dati-sce_2011.zip"
    geo_zip = folder / f"R{region:02d}_11_WGS84.zip"
    fetch(SCE_URL, sce_zip)
    fetch(GEO_URL.format(region=region), geo_zip)

    employees = employees_by_section(sce_zip, region, args.procom)
    polygons = sections(geo_zip, region, args.procom, folder)
    out = polygons.merge(employees, left_on="SEZ", right_index=True, how="left")
    unmatched = set(employees.index) - set(polygons.SEZ)
    if unmatched:
        lost = int(employees.loc[sorted(unmatched), "employees"].sum())
        print(f"warning: {len(unmatched)} sections with {lost} employees have no polygon")
    out[list(employees.columns)] = out[list(employees.columns)].fillna(0).astype(int)
    out = out.rename(columns={"SEZ2011": "zoneID"}).drop(columns="SEZ")
    out["zoneID"] = out.zoneID.astype("int64")

    edges = paths.require_input(args.city, "edges.gpkg", "the street network")
    out = out.to_crs(gpd.read_file(edges, rows=1).crs)

    target = paths.resources_dir(args.city) / f"{args.city}_employment.gpkg"
    if target.exists():
        target.unlink()
    out.to_file(target, layer=f"{args.city}_employment", driver="GPKG")

    total = int(out.employees.sum())
    print(f"{target.name}: {len(out)} sections, {total} employees "
          f"({int((out.employees > 0).sum())} sections with any)")
    provenance.update(args.city, "employment", {
        "written": provenance.now(),
        "source": "ISTAT, Censimento dell'industria e dei servizi 2011, ADDETTI per sezione",
        "urls": [SCE_URL, GEO_URL.format(region=region)],
        "settings": provenance.settings(args),
        "environment": provenance.environment(),
        "inputs": [provenance.file_info(sce_zip), provenance.file_info(geo_zip)],
        "output": provenance.file_info(target),
        "employees": total,
    })


if __name__ == "__main__":
    main()
