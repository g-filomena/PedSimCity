import sqlite3
import glob

# The exact WKT string that GeoMason expects for EPSG:3003
wkt = 'PROJCS["Monte Mario / Italy zone 1",GEOGCS["Monte Mario",DATUM["Monte_Mario",SPHEROID["International 1924",6378388,297,AUTHORITY["EPSG","7022"]],AUTHORITY["EPSG","6265"]],PRIMEM["Greenwich",0,AUTHORITY["EPSG","8901"]],UNIT["degree",0.0174532925199433,AUTHORITY["EPSG","9122"]],AUTHORITY["EPSG","4265"]],PROJECTION["Transverse_Mercator"],PARAMETER["latitude_of_origin",0],PARAMETER["central_meridian",9],PARAMETER["scale_factor",0.9996],PARAMETER["false_easting",1500000],PARAMETER["false_northing",0],UNIT["metre",1,AUTHORITY["EPSG","9001"]],AXIS["Easting",EAST],AXIS["Northing",NORTH],AUTHORITY["EPSG","3003"]]'

for f in glob.glob('src/main/resources/Torino/*.gpkg'):
    try:
        with sqlite3.connect(f) as conn:
            conn.execute("UPDATE gpkg_spatial_ref_sys SET definition = ? WHERE srs_id = 3003", (wkt,))
            conn.commit()
            print(f"Fixed CRS in {f}")
    except Exception as e:
        print(f"Failed to fix {f}: {e}")
