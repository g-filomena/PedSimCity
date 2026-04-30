package pedsim.core.website;

/**
 * Builds the self-contained Leaflet HTML block embedded in the Javelit dashboard
 * via {@code Jt.html(...).use()}.
 *
 * <p>The generated HTML:
 * <ul>
 *   <li>Loads Leaflet from CDN.</li>
 *   <li>Renders a CartoDB Dark Matter basemap.</li>
 *   <li>Fetches the road network as GeoJSON on first load and draws it as
 *       dark-grey polylines.</li>
 *   <li>Polls {@code /api/state} every 500 ms and re-renders agent dots:
 *     <ul>
 *       <li>Blue {@code #3b82f6} — normal agents.</li>
 *       <li>Red {@code #ef4444} — vulnerable agents.</li>
 *     </ul>
 *   </li>
 *   <li>Updates inline stat labels for simulation time, walking count and
 *       at-home count.</li>
 * </ul>
 */
public final class MapHtmlBuilder {

  private MapHtmlBuilder() {}

  /**
   * Builds the Leaflet map HTML string for embedding in the Javelit dashboard.
   *
   * @param centreLatLon two-element array {@code [lat, lon]} for the initial map centre.
   * @param zoom         initial zoom level.
   * @param apiPort      port of the {@link SimulationRestApi} endpoint (typically 8081).
   * @return an HTML string ready for {@code Jt.html(...).use()}.
   */
  public static String build(double[] centreLatLon, int zoom, int apiPort) {
    double lat = centreLatLon[0];
    double lon = centreLatLon[1];
    String apiUrl = "http://localhost:" + apiPort + "/api/state";

    return """
        <style>
          #sim-map {
            height: 560px;
            border-radius: 12px;
            border: 1px solid #333;
            box-shadow: 0 4px 24px rgba(0,0,0,0.5);
          }
          .sim-stats {
            display: flex;
            gap: 24px;
            margin-top: 10px;
            padding: 10px 16px;
            background: #1a1a2e;
            border-radius: 8px;
            font-family: 'Inter', sans-serif;
            color: #e2e8f0;
            font-size: 13px;
          }
          .sim-stat { display: flex; flex-direction: column; align-items: center; }
          .sim-stat-label { color: #94a3b8; font-size: 11px; text-transform: uppercase; letter-spacing: .05em; }
          .sim-stat-value { font-size: 18px; font-weight: 600; color: #f8fafc; }
          .sim-legend {
            display: flex; gap: 16px; margin-top: 6px;
            font-family: sans-serif; font-size: 12px; color: #94a3b8;
          }
          .legend-dot {
            display: inline-block; width: 10px; height: 10px;
            border-radius: 50%%; margin-right: 4px; vertical-align: middle;
          }
        </style>

        <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
        <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>

        <div id="sim-map"></div>

        <div class="sim-stats">
          <div class="sim-stat">
            <span class="sim-stat-label">Sim Time</span>
            <span class="sim-stat-value" id="stat-time">—</span>
          </div>
          <div class="sim-stat">
            <span class="sim-stat-label">Step</span>
            <span class="sim-stat-value" id="stat-step">—</span>
          </div>
          <div class="sim-stat">
            <span class="sim-stat-label">Walking</span>
            <span class="sim-stat-value" id="stat-walking">—</span>
          </div>
          <div class="sim-stat">
            <span class="sim-stat-label">At Home</span>
            <span class="sim-stat-value" id="stat-home">—</span>
          </div>
          <div class="sim-stat">
            <span class="sim-stat-label">At Dest</span>
            <span class="sim-stat-value" id="stat-dest">—</span>
          </div>
        </div>

        <div class="sim-legend">
          <span><span class="legend-dot" style="background:#3b82f6"></span>Normal agent</span>
          <span><span class="legend-dot" style="background:#ef4444"></span>Vulnerable agent</span>
        </div>

        <script>
          // Initialise Leaflet map
          var map = L.map('sim-map', { preferCanvas: true })
                     .setView([%s, %s], %d);

          L.tileLayer(
            'https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png',
            { attribution: '&copy; CartoDB', maxZoom: 19 }
          ).addTo(map);

          var agentLayer = L.layerGroup().addTo(map);
          var roadLayerAdded = false;

          // Fetch state and refresh agents every 500 ms
          function refreshState() {
            fetch('%s')
              .then(r => r.json())
              .then(data => {
                // Draw roads once when GeoJSON is available
                if (data.roadsGeoJson && !roadLayerAdded) {
                  try {
                    L.geoJSON(JSON.parse(data.roadsGeoJson), {
                      style: { color: '#475569', weight: 1.2, opacity: 0.7 }
                    }).addTo(map);
                    roadLayerAdded = true;
                  } catch (e) { console.warn('Road GeoJSON parse error', e); }
                }

                // Update agent dots
                agentLayer.clearLayers();
                (data.agents || []).forEach(a => {
                  var color = a.vulnerable ? '#ef4444' : '#3b82f6';
                  L.circleMarker([a.lat, a.lon], {
                    radius: 4,
                    fillColor: color,
                    color: '#ffffff',
                    weight: 0.5,
                    opacity: 1,
                    fillOpacity: 0.9
                  }).bindTooltip('Agent ' + a.id + ' — ' + a.status).addTo(agentLayer);
                });

                // Update stat labels
                document.getElementById('stat-time').textContent    = data.simulationTime || '—';
                document.getElementById('stat-step').textContent    = data.currentStep    || '—';
                document.getElementById('stat-walking').textContent = data.walkingCount   || '0';
                document.getElementById('stat-home').textContent    = data.atHomeCount    || '0';
                document.getElementById('stat-dest').textContent    = data.atDestCount    || '0';
              })
              .catch(() => {}); // silently ignore if sim hasn't started yet
          }

          setInterval(refreshState, 500);
          refreshState();
        </script>
        """.formatted(lat, lon, zoom, apiUrl);
  }
}
