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
API_URL = "http://localhost:8081/api/state"

# --- SESSION STATE ---
if 'roads' not in st.session_state:
    st.session_state.roads = None

# --- SIDEBAR ---
st.sidebar.header("Simulation Settings")

city_name = st.sidebar.selectbox("City Name", ["TorinoCentre", "Muenster"], index=0)
duration = st.sidebar.number_input("Duration (days)", 1, 30, 7)
population = st.sidebar.number_input("Actual Population", 1000, 500000, 100000)
percentage = st.sidebar.slider("% Represented", 0.001, 1.0, 0.01, format="%.3f")
jobs = st.sidebar.number_input("Parallel Jobs", 1, 16, 1)

if st.sidebar.button("Run Simulation", use_container_width=True):
    try:
        params = {
            "cityName": city_name,
            "days": duration,
            "actualPopulation": population,
            "percentage": percentage,
            "jobs": jobs
        }
        r = requests.post("http://localhost:8081/api/start", json=params)
        if r.status_code == 200:
            st.sidebar.success(f"Simulation started for {city_name}!")
        else:
            st.sidebar.error("Failed to start simulation.")
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
    except Exception as e:
        return None
    return None

# --- MAIN LOOP ---
while True:
    data = fetch_data()
    
    if data:
        # Update Stats
        with stats_container.container():
            st.subheader("Live Stats")
            st.metric("Time", data.get('simulationTime', '00:00'))
            st.metric("Step", data.get('currentStep', 0))
            
            c1, c2 = st.columns(2)
            c1.metric("Walking", data.get('walkingCount', 0))
            c2.metric("Home", data.get('atHomeCount', 0))
            st.metric("At Destination", data.get('atDestCount', 0))

        # --- STATIC IMAGE GENERATION ---
        with map_placeholder.container():
            st.info("Generating static image for the current step...")
            
            import matplotlib.pyplot as plt
            import geopandas as gpd
            from shapely.geometry import shape
            
            # Create a Matplotlib figure
            fig, ax = plt.subplots(figsize=(12, 8), dpi=100)
            ax.set_facecolor('#000000')
            fig.patch.set_facecolor('#000000')
            
            # Hide axes
            ax.set_xticks([])
            ax.set_yticks([])
            for spine in ax.spines.values():
                spine.set_visible(False)
            
            # 1. Plot Roads (Edges)
            if st.session_state.roads is None and data.get('roadsGeoJson'):
                try:
                    st.session_state.roads = json.loads(data['roadsGeoJson'])
                except:
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
 
            # 2. Plot Agents (Nodes)
            agents = data.get('agents', [])
            if agents:
                df = pd.DataFrame(agents)
                
                # Plot non-vulnerable agents
                non_vuln = df[df['vulnerable'] == False]
                if not non_vuln.empty:
                    ax.scatter(non_vuln['lon'], non_vuln['lat'], c='#3b82f6', s=15, zorder=5, label='Agent')
                    
                # Plot vulnerable agents
                vuln = df[df['vulnerable'] == True]
                if not vuln.empty:
                    ax.scatter(vuln['lon'], vuln['lat'], c='#ef4444', s=15, zorder=6, label='Vulnerable Agent')
            
            # Render the static image in Streamlit
            st.pyplot(fig)
            plt.close(fig)
            
    else:
        with map_placeholder.container():
            st.warning("Waiting for simulation data... Make sure the Java simulation is running with the REST API enabled (Option 2).")
            st.info(f"Polling {API_URL}...")

    time.sleep(refresh_rate)
