import streamlit as st
import requests
import pandas as pd
import pydeck as pdk
import json
import time

st.set_page_config(layout="wide", page_title="PedSimCity Dashboard", page_icon="🏙️")

# --- CUSTOM CSS ---
st.markdown("""
    <style>
    .main { background-color: #0f172a; color: #f8fafc; }
    .stMetric { background-color: #1e293b; padding: 15px; border-radius: 10px; border: 1px solid #334155; }
    </style>
    """, unsafe_allow_html=True)

st.title("🏙️ PedSimCity Live Dashboard")

# --- CONFIG ---
API_URL = "http://localhost:8081/api/state"

# --- SESSION STATE ---
if 'roads' not in st.session_state:
    st.session_state.roads = None
if 'view_state' not in st.session_state:
    st.session_state.view_state = pdk.ViewState(latitude=0, longitude=0, zoom=12, pitch=0)

# --- SIDEBAR ---
st.sidebar.header("Simulation Control")

if st.sidebar.button("▶️ Run Simulation", use_container_width=True):
    try:
        r = requests.post("http://localhost:8081/api/start")
        if r.status_code == 200:
            st.sidebar.success("Simulation started!")
        else:
            st.sidebar.error("Failed to start simulation.")
    except Exception as e:
        st.sidebar.error(f"Error: {e}")

refresh_rate = st.sidebar.slider("Refresh Rate (seconds)", 0.1, 5.0, 1.0)
show_roads = st.sidebar.checkbox("Show Road Network", True)

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

        # Handle Road Network
        if st.session_state.roads is None and data.get('roadsGeoJson'):
            try:
                st.session_state.roads = json.loads(data['roadsGeoJson'])
                # Calculate initial view state from roads
                features = st.session_state.roads.get('features', [])
                if features:
                    # Get center of the first feature as a starting point
                    coords = features[0]['geometry']['coordinates']
                    if isinstance(coords[0], list): # LineString
                        lon, lat = coords[0]
                    else:
                        lon, lat = coords
                    st.session_state.view_state = pdk.ViewState(latitude=lat, longitude=lon, zoom=14, pitch=0)
            except:
                pass

        # Prepare Map Layers
        layers = []
        
        # 1. Road Layer
        if show_roads and st.session_state.roads:
            layers.append(pdk.Layer(
                "GeoJsonLayer",
                st.session_state.roads,
                stroke_width_min_pixels=1,
                get_line_color=[71, 85, 105, 150], # #475569
                get_fill_color=[71, 85, 105, 150],
            ))

        # 2. Agent Layer
        agents = data.get('agents', [])
        if agents:
            df = pd.DataFrame(agents)
            # Add color column based on vulnerability
            df['color'] = df['vulnerable'].apply(lambda v: [239, 68, 68] if v else [59, 130, 246])
            
            layers.append(pdk.Layer(
                "ScatterplotLayer",
                df,
                get_position='[lon, lat]',
                get_color='color',
                get_radius=8,
                pickable=True,
                opacity=0.8,
            ))

        # Render Map
        with map_placeholder.container():
            st.pydeck_chart(pdk.Deck(
                map_style='mapbox://styles/mapbox/dark-v10',
                initial_view_state=st.session_state.view_state,
                layers=layers,
                tooltip={"text": "Agent ID: {id}\nWalking: {walking}"}
            ))
            
    else:
        with map_placeholder.container():
            st.warning("⚠️ Waiting for simulation data... Make sure the Java simulation is running with the REST API enabled (Option 2).")
            st.info(f"Polling {API_URL}...")

    time.sleep(refresh_rate)
