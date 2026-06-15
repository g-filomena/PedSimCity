import os
import sys
import subprocess
import argparse

def main():
    parser = argparse.ArgumentParser(description="Build city data pipeline for PedSimCity.")
    parser.add_argument('--input_dir', required=True, help="Path to the city folder (e.g. src/main/resources/Torino__)")
    parser.add_argument('--prefix', required=True, help="Prefix for city files (e.g. Torino_)")
    args = parser.parse_args()

    input_dir = os.path.abspath(args.input_dir)
    prefix = args.prefix

    pipeline_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "src", "main", "python", "pipeline")

    print(f"============================================================")
    print(f"PedSimCity Data Pipeline Orchestrator")
    print(f"Target Directory: {input_dir}")
    print(f"File Prefix: {prefix}")
    print(f"============================================================\n")

    # Step 4: Street Lights Simple Prep
    punti_physics = os.path.join(input_dir, f"{prefix}puntiLuce_with_radius.gpkg")
    if os.path.exists(punti_physics):
        print("[SKIP] 04_street_lights_simple.py - Output file already exists.")
    else:
        print("[RUN] 04_street_lights_simple.py - Calculating base light physics...")
        script_path = os.path.join(pipeline_dir, "04_street_lights_simple.py")
        subprocess.run([sys.executable, script_path, "--input_dir", input_dir, "--prefix", prefix], check=True)

    # Step 5: Street Lights
    edges_illuminated = os.path.join(input_dir, "edges_illuminated_continuous.gpkg")
    nodes_2m = os.path.join(input_dir, "nodes_2m_densified_illuminated.gpkg")
    if os.path.exists(edges_illuminated) and os.path.exists(nodes_2m):
        print("[SKIP] 05_street_lights.py - Output files already exist.")
    else:
        print("[RUN] 05_street_lights.py - Calculating street illumination...")
        script_path = os.path.join(pipeline_dir, "05_street_lights.py")
        subprocess.run([sys.executable, script_path, "--input_dir", input_dir, "--prefix", prefix], check=True)

    # Step 6: Directional Lighting
    directional_csv = os.path.join(input_dir, f"{prefix}directional_lighting_lookup.csv")
    if os.path.exists(directional_csv):
        print("[SKIP] 06_directional_lighting.py - Output file already exists.")
    else:
        print("[RUN] 06_directional_lighting.py - Calculating directional lighting...")
        script_path = os.path.join(pipeline_dir, "06_directional_lighting.py")
        subprocess.run([sys.executable, script_path, "--input_dir", input_dir, "--prefix", prefix], check=True)

    print("\n============================================================")
    print(f"Pipeline completed successfully!")
    print(f"All required data components are ready in {input_dir}.")
    print(f"============================================================")

if __name__ == "__main__":
    main()
