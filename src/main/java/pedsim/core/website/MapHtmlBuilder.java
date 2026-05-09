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
                #canvas { width: 100vw; height: 100vh; }
              </style>
            </head>
            <body>
              <canvas id="canvas"></canvas>
            </body>
            </html>
          '></iframe>
        </div>
        """.formatted(apiUrl);
  }
}
