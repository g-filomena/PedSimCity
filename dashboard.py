import streamlit as st
import requests
import pandas as pd
import json
import time

st.set_page_config(layout="wide", page_title="PedSimCity Dashboard")

# --- CUSTOM CSS ---
st.markdown("""
    <style>
    .main { background-color: #000000; color: #f8fafc; }
    .stMetric { background-color: #111111; padding: 15px; border-radius: 10px; border: 1px solid #222222; }
    </style>
    """, unsafe_allow_html=True)

st.title("PedSimCity Live Dashboard")

# --- CONFIG ---
BASE_URL = "http://localhost:8081"
API_URL = f"{BASE_URL}/api/state"

# --- SESSION STATE ---
if 'roads' not in st.session_state:
    st.session_state.roads = None

# --- SIDEBAR ---
st.sidebar.header("Simulation Settings")

# Module selector — fetches available modules from /api/modules if reachable
def fetch_available_modules():
    try:
        r = requests.get(f"{BASE_URL}/api/modules", timeout=1)
        if r.status_code == 200:
            data = r.json()
            return [m["id"] for m in data.get("modules", [])]
    except Exception:
        pass
    return ["core", "night"]  # fallback if server not yet up

available_modules = fetch_available_modules()
module = st.sidebar.selectbox("Module", available_modules, index=0)

city_name = st.sidebar.selectbox("City Name", ["TorinoCentre", "Muenster"], index=0)
duration = st.sidebar.number_input("Duration (days)", 1, 30, 7)
population = st.sidebar.number_input("Actual Population", 1000, 500000, 100000)
percentage = st.sidebar.slider("% Represented", 0.001, 1.0, 0.01, format="%.3f")
jobs = st.sidebar.number_input("Parallel Jobs", 1, 16, 1)

# Night-specific parameters (shown only when night module is selected)
enable_ab = False
if module == "night":
    st.sidebar.divider()
    st.sidebar.subheader("Night Parameters")
    enable_ab = st.sidebar.checkbox("Enable A/B Light Testing", value=False)

if st.sidebar.button("Run Simulation", use_container_width=True):
    try:
        params = {
            "module": module,
            "cityName": city_name,
            "days": duration,
            "actualPopulation": population,
            "percentage": percentage,
            "jobs": jobs,
        }
        if module == "night":
            params["enableAB"] = enable_ab
        r = requests.post(f"{BASE_URL}/api/start", json=params)
        if r.status_code == 200:
            st.sidebar.success(f"Simulation started — module={module}, city={city_name}")
        elif r.status_code == 400:
            st.sidebar.error(f"Bad request: {r.text}")
        elif r.status_code == 503:
            st.sidebar.error("Server not ready (no module registered).")
        else:
            st.sidebar.error(f"Failed to start simulation (HTTP {r.status_code}).")
    except Exception as e:
        st.sidebar.error(f"Error: {e}")

st.sidebar.divider()
st.sidebar.header("View Controls")
refresh_rate = st.sidebar.slider("Refresh Rate (seconds)", 0.1, 5.0, 1.0)
show_roads = st.sidebar.checkbox("Show Edges (Roads)", True)

stats_container = st.sidebar.empty()
log_container = st.sidebar.empty()

# --- MAIN PANEL ---
map_placeholder = st.empty()

def fetch_data():
    try:
        r = requests.get(API_URL, timeout=1)
        if r.status_code == 200:
            return r.json()
    except Exception:
        pass
    return None

# --- MAIN LOOP ---
while True:
    data = fetch_data()

    if data:
        # Module-specific state — read from moduleState with backward compat for older server
        module_state = data.get("moduleState") or {}
        enable_ab_live = module_state.get("enableAB", data.get("enableAB", False))
        active_module = data.get("module", "core")

        # Update Stats
        with stats_container.container():
            st.subheader("Live Stats")
            st.caption(f"Module: **{active_module}**")
            st.metric("Time", data.get('simulationTime', '—'))
            st.metric("Step", data.get('currentStep', 0))

            c1, c2 = st.columns(2)
            c1.metric("Walking", data.get('walkingCount', 0))
            c2.metric("Home", data.get('atHomeCount', 0))
            st.metric("At Destination", data.get('atDestCount', 0))

            if active_module == "night":
                st.divider()
                st.caption("Night module state")
                st.write(f"A/B testing: {'ON' if enable_ab_live else 'OFF'}")
                if "crowdednessPercentile" in module_state:
                    st.write(f"Crowdedness percentile: {module_state['crowdednessPercentile']}")
                avg_vuln = data.get('avgVulnTripM', -1)
                avg_norm = data.get('avgNormalTripM', -1)
                if avg_vuln >= 0:
                    st.metric("Avg vuln trip (m)", f"{avg_vuln:.0f}")
                if avg_norm >= 0:
                    st.metric("Avg normal trip (m)", f"{avg_norm:.0f}")

        # --- MAP ---
        with map_placeholder.container():
            import matplotlib.pyplot as plt
            import geopandas as gpd
            from shapely.geometry import shape

            fig, ax = plt.subplots(figsize=(12, 8), dpi=100)
            ax.set_facecolor('#000000')
            fig.patch.set_facecolor('#000000')
            ax.set_xticks([])
            ax.set_yticks([])
            for spine in ax.spines.values():
                spine.set_visible(False)

            # Roads
            if st.session_state.roads is None and data.get('roadsGeoJson'):
                try:
                    st.session_state.roads = json.loads(data['roadsGeoJson'])
                except Exception:
                    pass

            if st.session_state.roads is None:
                try:
                    r = requests.get(f"{BASE_URL}/api/roads", timeout=2)
                    if r.status_code == 200:
                        st.session_state.roads = r.json()
                except Exception:
                    pass

            if show_roads and st.session_state.roads:
                try:
                    features = st.session_state.roads.get('features', [])
                    if features:
                        geoms = [shape(f['geometry']) for f in features]
                        roads_gdf = gpd.GeoDataFrame(geometry=geoms)
                        roads_gdf.plot(ax=ax, color='#475569', linewidth=1, alpha=0.7)
                except Exception as e:
                    st.warning(f"Failed to plot roads: {e}")

            # Agents
            agents = data.get('agents', [])
            if agents:
                df = pd.DataFrame(agents)
                non_vuln = df[df['vulnerable'] == False]
                if not non_vuln.empty:
                    ax.scatter(non_vuln['lon'], non_vuln['lat'],
                               c='#3b82f6', s=15, zorder=5, label='Agent')
                vuln = df[df['vulnerable'] == True]
                if not vuln.empty:
                    ax.scatter(vuln['lon'], vuln['lat'],
                               c='#ef4444', s=15, zorder=6, label='Vulnerable')
                if active_module == "night" and not vuln.empty:
                    ax.legend(facecolor='#111111', labelcolor='white', loc='lower right')

            st.pyplot(fig)
            plt.close(fig)

    else:
        with map_placeholder.container():
            st.warning(
                "Waiting for simulation data… "
                "Make sure the Java simulation is running with REST API enabled (Option 2)."
            )
            st.info(f"Polling {API_URL}")

    time.sleep(refresh_rate)
