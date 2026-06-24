import os
import argparse
import geopandas as gpd
import pandas as pd
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument('--input_dir', required=True)
parser.add_argument('--prefix', required=True)
args = parser.parse_args()

punti_path = os.path.join(args.input_dir, f"{args.prefix}puntiLuce.gpkg")
output_path = os.path.join(args.input_dir, f"{args.prefix}puntiLuce_with_radius.gpkg")

if not os.path.exists(punti_path):
    print(f"File not found: {punti_path}")
    exit(1)

print("Loading raw puntiLuce data...")
punti = gpd.read_file(punti_path)

print("Cleaning data...")
punti["potenza_w_max"] = pd.to_numeric(punti["potenza_w_max"], errors="coerce")
punti["altezza_palo_m"] = pd.to_numeric(punti["altezza_palo_m"], errors="coerce")
punti["braccio_l_m_max"] = pd.to_numeric(punti["braccio_l_m_max"], errors="coerce")

punti.loc[punti["potenza_w_max"] > 500, "potenza_w_max"] = np.nan
if "tecnologia" in punti.columns:
    punti["potenza_w_max"] = punti["potenza_w_max"].fillna(punti.groupby("tecnologia")["potenza_w_max"].transform("median"))
punti["potenza_w_max"] = punti["potenza_w_max"].fillna(punti["potenza_w_max"].median() if not pd.isna(punti["potenza_w_max"].median()) else 100.0)

if "uso_ottica" in punti.columns:
    punti["altezza_palo_m"] = punti["altezza_palo_m"].fillna(punti.groupby("uso_ottica")["altezza_palo_m"].transform("median"))
punti["altezza_palo_m"] = punti["altezza_palo_m"].fillna(9.0)
punti["braccio_l_m_max"] = punti["braccio_l_m_max"].fillna(0.0)

luminous_efficacy_map = {"LED / probabile LED": 120, "scarica / HID-CDM-HQL": 90, "non determinata": 70}
utilization_factor_map = {"stradale": 0.6, "viali": 0.6, "sospeso": 0.4, "storico/decorativo; sospeso": 0.35, "storico/decorativo": 0.3, "non determinato": 0.4}

if "tecnologia" in punti.columns:
    punti["luminous_efficacy"] = punti["tecnologia"].map(luminous_efficacy_map).fillna(70)
else:
    punti["luminous_efficacy"] = 70

if "uso_ottica" in punti.columns:
    punti["utilization_factor"] = punti["uso_ottica"].map(utilization_factor_map).fillna(0.4)
else:
    punti["utilization_factor"] = 0.4

print("Calculating physics (lumens, intensity, radius)...")
punti["total_lumens"] = punti["potenza_w_max"] * punti["luminous_efficacy"]
punti["downward_intensity_cd"] = (punti["total_lumens"] * punti["utilization_factor"]) / np.pi

E_min = 5.0
I = punti["downward_intensity_cd"]
h = punti["altezza_palo_m"]
radius_term = np.power((I * h) / E_min, 2.0 / 3.0) - np.power(h, 2.0)
punti["radius_m"] = np.sqrt(np.clip(radius_term, a_min=0.0, a_max=None))
punti["radius_m"] = punti["radius_m"].fillna(0.0)

# Backward compatibility
punti["radius"] = punti["radius_m"]
punti["lux"] = punti["downward_intensity_cd"]

print(f"Saving to {output_path}...")
punti.to_file(output_path, driver="GPKG")
print("Done.")
