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

  /**
   * High-performance Canvas renderer implementation.
   * Handles GeoJSON road projection and glowing agent dots.
   */
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

    // We use an iframe with srcdoc to isolate the canvas engine from Javelit's lifecycle.
    // This ensures scripts run reliably and allows for high-performance rendering.
    return """
        <div class="map-container" style="height: 600px; border-radius: 16px; overflow: hidden; background: #0f172a; box-shadow: 0 10px 40px rgba(0,0,0,0.4); border: 1px solid #334155;">
          <iframe id="map-iframe" style="width:100%%; height:100%%; border:none;" srcdoc='
            <!DOCTYPE html>
            <html>
            <head>
              <style>
                body { margin: 0; padding: 0; background: #0f172a; color: #f8fafc; font-family: system-ui, -apple-system, sans-serif; overflow: hidden; }
                #canvas { width: 100vw; height: 100vh; cursor: crosshair; }
                .overlay { position: absolute; top: 16px; left: 16px; pointer-events: none; }
                .stat-pill { background: rgba(30, 41, 59, 0.8); backdrop-filter: blur(8px); padding: 6px 12px; border-radius: 99px; border: 1px solid #475569; font-size: 12px; font-weight: 500; display: inline-flex; align-items: center; gap: 8px; margin-bottom: 8px; }
                .pulse { width: 8px; height: 8px; border-radius: 50%%; background: #10b981; box-shadow: 0 0 8px #10b981; animation: pulse 2s infinite; }
                @keyframes pulse { 0%% { opacity: 0.4; } 50%% { opacity: 1; } 100%% { opacity: 0.4; } }
              </style>
            </head>
            <body>
              <canvas id="canvas"></canvas>
              <div class="overlay">
                <div class="stat-pill"><div class="pulse"></div> Live View</div>
              </div>
              <script>
                const canvas = document.getElementById("canvas");
                const ctx = canvas.getContext("2d");
                let roads = null;
                let agents = [];
                let stats = {};
                let bounds = null;

                function resize() {
                  canvas.width = window.innerWidth * window.devicePixelRatio;
                  canvas.height = window.innerHeight * window.devicePixelRatio;
                  ctx.scale(window.devicePixelRatio, window.devicePixelRatio);
                }
                window.onresize = resize;
                resize();

                function project(lat, lon) {
                  if (!bounds) return { x: 0, y: 0 };
                  const x = (lon - bounds.minX) / (bounds.maxX - bounds.minX) * window.innerWidth;
                  const y = (1 - (lat - bounds.minY) / (bounds.maxY - bounds.minY)) * window.innerHeight;
                  return { x, y };
                }

                function draw() {
                  ctx.fillStyle = "#0f172a";
                  ctx.fillRect(0, 0, window.innerWidth, window.innerHeight);

                  if (roads && bounds) {
                    ctx.strokeStyle = "#334155";
                    ctx.lineWidth = 1;
                    ctx.beginPath();
                    roads.features.forEach(f => {
                      if (f.geometry.type === "LineString") {
                        const coords = f.geometry.coordinates;
                        const p0 = project(coords[0][1], coords[0][0]);
                        ctx.moveTo(p0.x, p0.y);
                        for (let i = 1; i < coords.length; i++) {
                          const p = project(coords[i][1], coords[i][0]);
                          ctx.lineTo(p.x, p.y);
                        }
                      }
                    });
                    ctx.stroke();
                  }

                  agents.forEach(a => {
                    const p = project(a.lat, a.lon);
                    const color = a.vulnerable ? "#ef4444" : "#3b82f6";
                    
                    // Glow
                    ctx.shadowBlur = 10;
                    ctx.shadowColor = color;
                    ctx.fillStyle = color;
                    ctx.beginPath();
                    ctx.arc(p.x, p.y, 4, 0, Math.PI * 2);
                    ctx.fill();
                    
                    // Core
                    ctx.shadowBlur = 0;
                    ctx.fillStyle = "#ffffff";
                    ctx.beginPath();
                    ctx.arc(p.x, p.y, 1.5, 0, Math.PI * 2);
                    ctx.fill();
                  });

                  requestAnimationFrame(draw);
                }

                async function update() {
                  try {
                    const r = await fetch("%s");
                    const data = await r.json();
                    agents = data.agents || [];
                    
                    if (data.roadsGeoJson && !roads) {
                      roads = JSON.parse(data.roadsGeoJson);
                      // Calculate bounds from roads
                      let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
                      roads.features.forEach(f => {
                        f.geometry.coordinates.forEach(c => {
                          minX = Math.min(minX, c[0]); maxX = Math.max(maxX, c[0]);
                          minY = Math.min(minY, c[1]); maxY = Math.max(maxY, c[1]);
                        });
                      });
                      // Add padding
                      const dx = (maxX - minX) * 0.1;
                      const dy = (maxY - minY) * 0.1;
                      bounds = { minX: minX - dx, maxX: maxX + dx, minY: minY - dy, maxY: maxY + dy };
                    }
                  } catch (e) { console.error(e); }
                  setTimeout(update, 200);
                }

                update();
                draw();
              </script>
            </html>
          '></iframe>
        </div>
        """.formatted(apiUrl);
  }
}
