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
st.sidebar.header("Simulation Settings")

city_name = st.sidebar.selectbox("City Name", ["TorinoCentre", "Muenster"], index=0)
duration = st.sidebar.number_input("Duration (days)", 1, 30, 7)
population = st.sidebar.number_input("Actual Population", 1000, 500000, 100000)
percentage = st.sidebar.slider("% Represented", 0.001, 1.0, 0.01, format="%.3f")
jobs = st.sidebar.number_input("Parallel Jobs", 1, 16, 1)

if st.sidebar.button("▶️ Run Simulation", use_container_width=True):
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

        # --- COORDINATE DETECTION ---
        is_geospatial = True
        
        # Check Agent coords
        agents = data.get('agents', [])
        if agents:
            first_agent = agents[0]
            lat, lon = first_agent.get('lat', 0), first_agent.get('lon', 0)
            if not (-90 <= lat <= 90 and -180 <= lon <= 180):
                is_geospatial = False

        # Handle Road Network
        if st.session_state.roads is None and data.get('roadsGeoJson'):
            try:
                st.session_state.roads = json.loads(data['roadsGeoJson'])
                features = st.session_state.roads.get('features', [])
                if features:
                    coords = features[0]['geometry']['coordinates']
                    # Handle both LineString and MultiLineString
                    first_coord = coords[0][0] if isinstance(coords[0][0], list) else coords[0]
                    lon, lat = first_coord[0], first_coord[1]
                    
                    if not (-90 <= lat <= 90 and -180 <= lon <= 180):
                        is_geospatial = False
                    
                    # Update view state to center on the data
                    st.session_state.view_state = pdk.ViewState(
                        latitude=lat if is_geospatial else 0, 
                        longitude=lon if is_geospatial else 0,
                        target=[lon, lat, 0] if not is_geospatial else None,
                        zoom=14 if is_geospatial else -2, 
                        pitch=0
                    )
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
                get_line_color=[71, 85, 105, 150],
                get_fill_color=[71, 85, 105, 150],
                coordinate_system=0 if is_geospatial else 1, # 0=LNGLAT, 1=CARTESIAN
            ))

        # 2. Agent Layer
        if agents:
            df = pd.DataFrame(agents)
            df['color'] = df['vulnerable'].apply(lambda v: [239, 68, 68] if v else [59, 130, 246])
            
            layers.append(pdk.Layer(
                "ScatterplotLayer",
                df,
                get_position='[lon, lat]',
                get_color='color',
                get_radius=8 if is_geospatial else 50, # Scale radius for local coords
                pickable=True,
                opacity=0.8,
            ))

        # Render Map only if we have data
        if st.session_state.roads or agents:
            with map_placeholder.container():
                if not is_geospatial:
                    st.info("💡 Local Coordinate System detected (Meters). Base map disabled.")
                
                view = pdk.View(type="MapView", controller=True) if is_geospatial else pdk.View(type="OrthographicView", controller=True)

                st.pydeck_chart(pdk.Deck(
                    views=[view],
                    map_style='mapbox://styles/mapbox/dark-v10' if is_geospatial else None,
                    initial_view_state=st.session_state.view_state,
                    layers=layers,
                    tooltip={"text": "Agent ID: {id}\nWalking: {walking}"}
                ))
        else:
            with map_placeholder.container():
                st.info("Waiting for data from the Java Backend...")
            
    else:
        with map_placeholder.container():
            st.warning("⚠️ Waiting for simulation data... Make sure the Java simulation is running with the REST API enabled (Option 2).")
            st.info(f"Polling {API_URL}...")

    time.sleep(refresh_rate)
