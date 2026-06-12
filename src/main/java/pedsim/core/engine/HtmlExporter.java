package pedsim.core.engine;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.logging.Logger;
import pedsim.core.parameters.Pars;
import pedsim.core.parameters.TimePars;
import pedsim.core.utilities.LoggerUtil;
import pedsim.core.website.GeoJsonExporter;

/**
 * Generates a self-contained, single-file HTML dashboard that embeds:
 * <ul>
 *   <li>The road network as a GeoJSON FeatureCollection (with edgeID + cumulative volume per edge).</li>
 *   <li>All agent trips with exact street coordinates traversed.</li>
 *   <li>A second tab showing hourly pedestrian volumes per street (24h animation with sky gradient).</li>
 *   <li>The full interactive visualizer engine inline.</li>
 * </ul>
 *
 * <p>Output path:
 * {@code C:\Users\<username>\PedSimCity\Output\results\results_<city>_day<day>_job<job>.html}
 */
public class HtmlExporter {

  private static final Logger logger = LoggerUtil.getLogger();
  private static final String USER = System.getProperty("user.name");

  private static final String OUTPUT_ROOT =
      "C:" + File.separator + "Users" + File.separator + USER
      + File.separator + "PedSimCity" + File.separator + "Output"
      + File.separator + "results";

  /**
   * Exports a complete, self-contained HTML dashboard.
   */
  public static String export(int day, int job,
      List<TripRouteRecorder.TripRecord> trips,
      Map<Integer, Map<String, Integer>> volumesMap) {

    try {
      Files.createDirectories(Paths.get(OUTPUT_ROOT));

      String city = Pars.cityName;
      String filename = "results_" + city + "_day" + day + "_job" + job + ".html";
      String outputPath = OUTPUT_ROOT + File.separator + filename;

      String roadsGeoJson = GeoJsonExporter.exportRoadsWithVolumes(
          PedSimCity.roads, volumesMap);

      String tripsJs = buildTripsJs(trips);
      if (isABTestingEnabled()) {
        saveAbTestTrips(tripsJs);
      }
      String abTripsJs = loadAbTestTrips();

      String hourlyVolJs = buildHourlyVolumesJs(trips);

      double[] centre = computeCentre();

      String html = renderHtml(city, day, job, roadsGeoJson, tripsJs, abTripsJs, hourlyVolJs, centre);
      try (FileWriter fw = new FileWriter(outputPath)) {
        fw.write(html);
      }

      logger.info("[HtmlExporter] Dashboard written → " + outputPath);
      return outputPath;

    } catch (IOException e) {
      logger.severe("[HtmlExporter] Failed to write dashboard: " + e.getMessage());
      return null;
    }
  }

  private static void saveAbTestTrips(String tripsJs) {
    String path = OUTPUT_ROOT + File.separator + "ab_test_trips.json";
    try {
      Files.createDirectories(Paths.get(OUTPUT_ROOT));
      try (FileWriter fw = new FileWriter(path)) {
        fw.write(tripsJs);
      }
      logger.info("[HtmlExporter] Saved A/B test trips data to " + path);
    } catch (IOException e) {
      logger.severe("[HtmlExporter] Failed to save A/B test trips: " + e.getMessage());
    }
  }

  private static String loadAbTestTrips() {
    String path = OUTPUT_ROOT + File.separator + "ab_test_trips.json";
    File file = new File(path);
    if (file.exists()) {
      try {
        return new String(Files.readAllBytes(Paths.get(path)));
      } catch (IOException e) {
        logger.severe("[HtmlExporter] Failed to read A/B test trips: " + e.getMessage());
      }
    }
    return "[]";
  }

  // -------------------------------------------------------------------------
  // Private helpers
  // -------------------------------------------------------------------------

  /**
   * Converts the trip records list into a compact JavaScript array literal.
   */
  private static String buildTripsJs(List<TripRouteRecorder.TripRecord> trips) {
    StringBuilder sb = new StringBuilder("[");
    boolean first = true;
    for (TripRouteRecorder.TripRecord trip : trips) {
      if (!first) sb.append(',');
      first = false;
      
      sb.append('[')
        .append(trip.agentId).append(',')
        .append(String.format("%.2f", trip.startStep)).append(',')
        .append(String.format("%.2f", trip.endStep)).append(',')
        .append('[');
      
      boolean firstCoord = true;
      for (org.locationtech.jts.geom.Coordinate c : trip.pathCoords) {
        if (!firstCoord) sb.append(',');
        firstCoord = false;
        sb.append(String.format("[%.6f,%.6f]", c.x, c.y));
      }
      sb.append("],")
        .append(trip.vulnerable ? "true" : "false")
        .append(",[");
      
      boolean firstSpook = true;
      for (org.locationtech.jts.geom.Coordinate s : trip.spookLocations) {
        if (!firstSpook) sb.append(',');
        firstSpook = false;
        sb.append(String.format("[%.6f,%.6f]", s.x, s.y));
      }
      sb.append("]]");
    }
    sb.append(']');
    return sb.toString();
  }

  /**
   * Builds a JS object: { edgeId: [vol_h0, vol_h1, ..., vol_h23], ... }
   * Each trip is attributed to the hour-of-day bucket of its startStep.
   * Steps per day = 72 (each step = 20 min), so step mod 72 / 3 = hour.
   */
  private static String buildHourlyVolumesJs(List<TripRouteRecorder.TripRecord> trips) {
    // edgeId -> int[24]
    Map<Integer, int[]> hourlyMap = new HashMap<>();

    for (TripRouteRecorder.TripRecord trip : trips) {
      // Determine hour-of-day (0-23) from startStep
      long totalMinutes = (long) (trip.startStep * (TimePars.STEP_DURATION / 60));
      int hourOfDay = (int) ((totalMinutes / 60) % 24);
      for (int edgeId : trip.edgeIds) {
        hourlyMap.computeIfAbsent(edgeId, k -> new int[24])[hourOfDay]++;
      }
    }

    StringBuilder sb = new StringBuilder("{");
    boolean first = true;
    for (Map.Entry<Integer, int[]> entry : hourlyMap.entrySet()) {
      if (!first) sb.append(',');
      first = false;
      sb.append(entry.getKey()).append(":[");
      int[] counts = entry.getValue();
      for (int h = 0; h < 24; h++) {
        if (h > 0) sb.append(',');
        sb.append(counts[h]);
      }
      sb.append(']');
    }
    sb.append('}');
    return sb.toString();
  }

  /** Returns [centreLatitude, centreLongitude] from the road layer MBR. */
  private static double[] computeCentre() {
    if (PedSimCity.MBR == null) {
      return new double[]{45.07, 7.68};
    }
    double cx = (PedSimCity.MBR.getMinX() + PedSimCity.MBR.getMaxX()) / 2.0;
    double cy = (PedSimCity.MBR.getMinY() + PedSimCity.MBR.getMaxY()) / 2.0;
    return new double[]{cy, cx};
  }

  // -------------------------------------------------------------------------
  // HTML template
  // -------------------------------------------------------------------------

  private static String renderHtml(String city, int day, int job,
      String roadsGeoJson, String tripsJs, String abTripsJs, String hourlyVolJs, double[] centre) {

    String isNight = String.valueOf(Pars.isNight);
    String enableAB = String.valueOf(isABTestingEnabled());

    return "<!DOCTYPE html>\n"
        + "<html lang=\"en\">\n"
        + "<head>\n"
        + "<meta charset=\"UTF-8\"/>\n"
        + "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\"/>\n"
        + "<title>Night Pedestrian Simulation: Torino — March 12</title>\n"
        + "<link rel=\"preconnect\" href=\"https://fonts.googleapis.com\">\n"
        + "<link href=\"https://fonts.googleapis.com/css2?family=Inter:wght@300;400;600;700&display=swap\" rel=\"stylesheet\">\n"
        + "<style>\n"
        + "  *{box-sizing:border-box;margin:0;padding:0}\n"
        + "  body{font-family:'Inter',sans-serif;background:#000000;color:#f1f5f9;display:flex;flex-direction:column;height:100vh;overflow:hidden}\n"
        + "  #header{padding:12px 20px;background:rgba(0,0,0,0.95);border-bottom:1px solid #1e293b;display:flex;align-items:center;justify-content:space-between;gap:16px;flex-shrink:0}\n"
        + "  #header h1{font-size:1.05rem;font-weight:700;color:#818cf8;letter-spacing:0.02em}\n"
        + "  .metrics{display:flex;gap:12px}\n"
        + "  .card{background:rgba(30,41,59,0.45);border:1px solid #1e293b;border-radius:10px;padding:10px 18px;min-width:120px}\n"
        + "  .card .label{font-size:.65rem;color:#94a3b8;text-transform:uppercase;letter-spacing:.08em}\n"
        + "  .card .value{font-size:1.4rem;font-weight:700;color:#818cf8;margin-top:2px}\n"
        + "  .card .value.red{color:#ef4444}\n"
        + "  .card .value.orange{color:#fb923c}\n"
        + "  .card .value.yellow{color:#fde047}\n"
        // Tabs
        + "  #tab-bar{flex-shrink:0;display:flex;gap:0;background:rgba(0,0,0,0.98);border-bottom:1px solid #1e293b;padding:0 20px}\n"
        + "  .tab-btn{padding:10px 22px;font-size:.8rem;font-weight:600;color:#64748b;background:none;border:none;border-bottom:2px solid transparent;cursor:pointer;transition:all .2s;letter-spacing:.03em}\n"
        + "  .tab-btn.active{color:#818cf8;border-bottom-color:#6366f1}\n"
        + "  .tab-btn:hover:not(.active){color:#a5b4fc}\n"
        // Panels
        + "  .tab-panel{display:none;flex:1;position:relative;overflow:hidden}\n"
        + "  .tab-panel.active{display:flex;flex-direction:column}\n"
        + "  #container, #ab-container, #hv-container, #vol-container {position:relative;flex:1;width:100%;overflow:hidden;background:#000000}\n"
        + "  #canvas, #ab-canvas, #hv-canvas, #vol-canvas {width:100%;height:100%;display:block;cursor:grab}\n"
        + "  #canvas:active, #ab-canvas:active, #hv-canvas:active, #vol-canvas:active {cursor:grabbing}\n"
        + "  #floating-controls{position:absolute;top:20px;right:20px;display:flex;flex-direction:column;gap:8px}\n"
        + "  .btn-float{background:rgba(30,41,59,0.85);border:1px solid #334155;border-radius:8px;color:#f1f5f9;padding:8px 14px;font-size:.75rem;font-weight:600;cursor:pointer;backdrop-filter:blur(4px);transition:all .2s;display:flex;align-items:center;gap:6px}\n"
        + "  .btn-float:hover{background:#6366f1;color:#f1f5f9;border-color:#6366f1}\n"
        + "  .toggle-label{display:flex;align-items:center;gap:10px;font-size:0.8rem;background:rgba(30,41,59,0.85);padding:8px 14px;border-radius:8px;border:1px solid #334155;backdrop-filter:blur(4px);cursor:pointer;color:#f1f5f9;font-weight:600;user-select:none;}\n"
        + "  .toggle-switch { position: relative; width: 34px; height: 18px; background: #0f172a; border-radius: 18px; border: 1px solid #334155; transition: 0.3s; flex-shrink: 0; }\n"
        + "  .toggle-switch::after { content: ''; position: absolute; top: 1px; left: 1px; width: 14px; height: 14px; background: #94a3b8; border-radius: 50%; transition: 0.3s; }\n"
        + "  input[type=\"checkbox\"]:checked + .toggle-switch { background: #6366f1; border-color: #6366f1; }\n"
        + "  input[type=\"checkbox\"]:checked + .toggle-switch::after { transform: translateX(16px); background: #fff; }\n"
        + "  input[type=\"checkbox\"] { display: none; }\n"
        + "  #timeline-panel{flex-shrink:0;padding:14px 20px 16px;background:rgba(0,0,0,0.98);border-top:1px solid #1e293b}\n"
        + "  #timeline-panel .row{display:flex;align-items:center;gap:14px}\n"
        + "  #hv-timeline-panel{flex-shrink:0;padding:14px 20px 16px;background:rgba(0,0,0,0.98);border-top:1px solid #1e293b}\n"
        + "  #hv-timeline-panel .row{display:flex;align-items:center;gap:14px}\n"
        + "  #hv-time-label{font-size:1rem;font-weight:700;color:#f8c56d;min-width:90px;text-align:center}\n"
        + "  #hv-hour-slider{flex:1;-webkit-appearance:none;height:5px;border-radius:3px;background:#1e293b;outline:none;cursor:pointer}\n"
        + "  #hv-hour-slider::-webkit-slider-thumb{-webkit-appearance:none;width:16px;height:16px;border-radius:50%;background:#f8c56d;cursor:pointer;box-shadow:0 0 6px rgba(248,197,109,0.6)}\n"
        + "  .hv-btn{width:42px;height:42px;border-radius:50%;border:2px solid #f8c56d;background:transparent;color:#f8c56d;cursor:pointer;font-size:1.2rem;display:flex;align-items:center;justify-content:center;flex-shrink:0;transition:all .2s;padding-left:3px;}\n"
        + "  .hv-btn.playing{padding-left:0;}\n"
        + "  .hv-btn:hover{background:#f8c56d;color:#0a0c14}\n"
        + "  .hv-speed{background:rgba(30,41,59,0.85);color:#f1f5f9;border:1px solid #334155;border-radius:8px;padding:6px 12px;font-size:.75rem;font-weight:600;cursor:pointer;outline:none;transition:all .2s;}\n"
        + "  .hv-speed:hover{background:#f8c56d;color:#0a0c14;border-color:#f8c56d}\n"
        + "  #speed-select{background:rgba(30,41,59,0.85);color:#f1f5f9;border:1px solid #334155;border-radius:8px;padding:6px 12px;font-size:.75rem;font-weight:600;cursor:pointer;outline:none;transition:all .2s;display:inline-flex;align-items:center}\n"
        + "  #speed-select:hover{background:#6366f1;color:#f1f5f9;border-color:#6366f1}\n"
        + "  #play-btn{width:42px;height:42px;border-radius:50%;border:2px solid #6366f1;background:transparent;color:#6366f1;cursor:pointer;font-size:1.2rem;display:flex;align-items:center;justify-content:center;flex-shrink:0;transition:all .2s;padding-left:3px;}\n"
        + "  #play-btn.playing{padding-left:0;}\n"
        + "  #play-btn:hover{background:#6366f1;color:#f1f5f9}\n"
        + "  #step-slider{flex:1;-webkit-appearance:none;height:5px;border-radius:3px;background:#1e293b;outline:none;cursor:pointer}\n"
        + "  #step-slider::-webkit-slider-thumb{-webkit-appearance:none;width:16px;height:16px;border-radius:50%;background:#6366f1;cursor:pointer;box-shadow:0 0 6px rgba(99,102,241,0.5)}\n"
        + "  #time-label{font-size:.8rem;color:#a5b4fc;min-width:48px;text-align:right;font-weight:600}\n"
        + "  .legend{display:flex;gap:16px;margin-top:10px;font-size:.7rem;color:#94a3b8}\n"
        + "  .leg-item{display:flex;align-items:center;gap:6px}\n"
        + "  .leg-dot{width:8px;height:8px;border-radius:50%}\n"
        + "  .leg-line{width:20px;height:3px;border-radius:2px}\n"
        + "  .hv-legend{position:absolute;bottom:16px;left:20px;display:flex;gap:6px;align-items:center;font-size:0.72rem;color:#94a3b8;background:rgba(0,0,0,0.7);padding:6px 12px;border-radius:8px;border:1px solid #1e293b;backdrop-filter:blur(4px);}\n"
        + "  .hv-legend-bar{width:80px;height:8px;border-radius:4px;background:linear-gradient(to right,#161c2e,#3bbaf8,#ffaa00,#ff3300)}\n"
        + "  .hv-floating{position:absolute;top:20px;right:20px;}\n"
        + "  #hv-hour-grid{display:flex;gap:4px;overflow-x:auto;padding-bottom:4px;scrollbar-width:thin}\n"
        + "  .hour-btn{background:rgba(30,41,59,0.7);border:1px solid #1e293b;border-radius:4px;color:#94a3b8;padding:4px 8px;font-size:0.7rem;font-weight:600;cursor:pointer;transition:all 0.2s;flex-shrink:0}\n"
        + "  .hour-btn:hover{background:#334155;color:#f1f5f9}\n"
        + "  .hour-btn.active{background:#6366f1 !important;color:#ffffff !important;border-color:#6366f1 !important}\n"
        + "</style>\n"
        + "</head>\n"
        + "<body>\n"
        + "<div id=\"header\">\n"
        + "  <h1>Night Pedestrian Simulation: Torino &nbsp;—&nbsp; March 12</h1>\n"
        + "  <div class=\"metrics\" id=\"metrics-trips\">\n"
        + "    <div class=\"card\"><div class=\"label\">Active Agents</div><div class=\"value\" id=\"m-active\">-</div></div>\n"
        + "    <div class=\"card\"><div class=\"label\">% Vulnerable</div><div class=\"value red\" id=\"m-vuln\">-</div></div>\n"
        + "    <div class=\"card\"><div class=\"label\">Avg Vuln Trip</div><div class=\"value orange\" id=\"m-vuln-d\">-</div></div>\n"
        + "    <div class=\"card\"><div class=\"label\">Avg Normal Trip</div><div class=\"value\" id=\"m-norm-d\">-</div></div>\n"
        + "    <div class=\"card\"><div class=\"label\">Avg Lux</div><div class=\"value yellow\" id=\"m-lux\">-</div></div>\n"
        + "    <div class=\"card\"><div class=\"label\">Sim Time</div><div class=\"value\" id=\"m-time\">-</div></div>\n"
        + "  </div>\n"
        + "  <div class=\"metrics\" id=\"metrics-hourly\" style=\"display:none\">\n"
        + "    <div class=\"card\"><div class=\"label\">Hour</div><div class=\"value yellow\" id=\"m-hv-hour\">00:00</div></div>\n"
        + "    <div class=\"card\"><div class=\"label\">Active Roads</div><div class=\"value\" id=\"m-hv-roads\">-</div></div>\n"
        + "    <div class=\"card\"><div class=\"label\">Peak Vol</div><div class=\"value red\" id=\"m-hv-peak\">-</div></div>\n"
        + "    <div class=\"card\"><div class=\"label\">Total Peds</div><div class=\"value\" id=\"m-hv-total\">-</div></div>\n"
        + "    <div class=\"card\"><div class=\"label\">Sky</div><div class=\"value\" id=\"m-hv-sky\">🌙 Night</div></div>\n"
        + "  </div>\n"
        + "  <div class=\"metrics\" id=\"metrics-volumes\" style=\"display:none\">\n"
        + "    <div class=\"card\"><div class=\"label\">Flow Type</div><div class=\"value yellow\" id=\"m-vol-type\">Day Flow</div></div>\n"
        + "    <div class=\"card\"><div class=\"label\">Peak Period Vol</div><div class=\"value red\" id=\"m-vol-peak\">-</div></div>\n"
        + "    <div class=\"card\"><div class=\"label\">Total Traversed</div><div class=\"value\" id=\"m-vol-total\">-</div></div>\n"
        + "  </div>\n"
        + "</div>\n"
        + "<div id=\"tab-bar\">\n"
        + "  <button class=\"tab-btn active\" onclick=\"switchTab('trips')\">Agent Trips</button>\n"
        + "  <button class=\"tab-btn\" onclick=\"switchTab('hourly')\">Hourly Volumes</button>\n"
        + "  <button class=\"tab-btn\" onclick=\"switchTab('volumes')\">Simulation Volumes</button>\n"
        + "  <button class=\"tab-btn\" onclick=\"switchTab('ab')\">A/B Testing</button>\n"
        + "</div>\n"
        + "<div class=\"tab-panel active\" id=\"panel-trips\">\n"
        + "  <div id=\"container\">\n"
        + "    <canvas id=\"canvas\"></canvas>\n"
        + "    <div id=\"floating-controls\">\n"
        + "      <label class=\"toggle-label\" id=\"lbl-light\"><input type=\"checkbox\" id=\"tg-light\" /><div class=\"toggle-switch\"></div> Light Level Map</label>\n"
        + "      <label class=\"toggle-label\" id=\"lbl-tethers\"><input type=\"checkbox\" id=\"tg-tethers\" checked /><div class=\"toggle-switch\"></div> Show A/B Tethers</label>\n"
        + "      <button class=\"btn-float\" id=\"reset-btn\">Reset Zoom</button>\n"
        + "    </div>\n"
        + "  </div>\n"
        + "  <div id=\"timeline-panel\">\n"
        + "    <div class=\"row\">\n"
        + "      <button id=\"play-btn\" title=\"Play/Pause\">▶</button>\n"
        + "      <input type=\"range\" id=\"step-slider\" min=\"0\" value=\"0\"/>\n"
        + "      <span id=\"time-label\">March 12 00:00</span>\n"
        + "      <select id=\"speed-select\" title=\"Playback Speed\">\n"
        + "        <option value=\"0.1\">0.1x Speed</option>\n"
        + "        <option value=\"0.5\">0.5x Speed</option>\n"
        + "        <option value=\"1\">1x Speed</option>\n"
        + "        <option value=\"2\" selected>2x Speed</option>\n"
        + "        <option value=\"5\">5x Speed</option>\n"
        + "        <option value=\"10\">10x Speed</option>\n"
        + "      </select>\n"
        + "    </div>\n"
        + "    <div class=\"legend\">\n"
        + "      <div class=\"leg-item\"><div class=\"leg-dot\" style=\"background:#ef4444\"></div> Vulnerable agent</div>\n"
        + "      <div class=\"leg-item\"><div class=\"leg-dot\" style=\"background:#38bdf8\"></div> Normal agent</div>\n"
        + "    </div>\n"
        + "  </div>\n"
        + "</div>\n"
        + "<div class=\"tab-panel\" id=\"panel-hourly\">\n"
        + "  <div id=\"hv-container\">\n"
        + "    <canvas id=\"hv-canvas\"></canvas>\n"
        + "    <div class=\"hv-legend\">\n"
        + "      <span>Low</span><div class=\"hv-legend-bar\"></div><span>High</span>\n"
        + "      &nbsp;&nbsp; Pedestrian volume/hour\n"
        + "    </div>\n"
        + "    <div class=\"hv-floating\">\n"
        + "      <button class=\"btn-float\" id=\"hv-reset-btn\">Reset Zoom</button>\n"
        + "    </div>\n"
        + "  </div>\n"
        + "  <div id=\"hv-timeline-panel\">\n"
        + "    <div class=\"row\">\n"
        + "      <button class=\"hv-btn\" id=\"hv-play-btn\" title=\"Play/Pause\">▶</button>\n"
        + "      <div style=\"flex:1;display:flex;flex-direction:column;gap:8px\">\n"
        + "        <div id=\"hv-hour-grid\"></div>\n"
        + "        <input type=\"range\" id=\"hv-hour-slider\" min=\"0\" max=\"23\" value=\"0\"/>\n"
        + "      </div>\n"
        + "      <span id=\"hv-time-label\">00:00 – 01:00</span>\n"
        + "      <select class=\"hv-speed\" id=\"hv-speed-select\">\n"
        + "        <option value=\"0.5\">0.5x</option>\n"
        + "        <option value=\"1\" selected>1x</option>\n"
        + "        <option value=\"2\">2x</option>\n"
        + "        <option value=\"4\">4x</option>\n"
        + "      </select>\n"
        + "      <select class=\"hv-speed\" id=\"hv-theme-select\" title=\"Color Theme\">\n"
        + "        <option value=\"consistency\" selected>Sim Consistency</option>\n"
        + "        <option value=\"heatmap\">Classic Heatmap</option>\n"
        + "      </select>\n"
        + "    </div>\n"
        + "    <div class=\"legend\" style=\"margin-top:8px\">\n"
        + "      <div class=\"leg-item\"><div style=\"width:20px;height:3px;border-radius:2px;background:#161c2e\"></div> No traffic</div>\n"
        + "      <div class=\"leg-item\"><div id=\"hv-leg-light\" style=\"width:20px;height:3px;border-radius:2px;background:#38bdf8\"></div> Light traffic</div>\n"
        + "      <div class=\"leg-item\"><div id=\"hv-leg-heavy\" style=\"width:20px;height:3px;border-radius:2px;background:#6366f1\"></div> Heavy traffic</div>\n"
        + "      <div class=\"leg-item\" style=\"margin-left:auto;font-style:italic;color:#f8c56d\">Sunrise 06:51 · Sunset 18:30</div>\n"
        + "    </div>\n"
        + "  </div>\n"
        + "</div>\n"
        + "<div class=\"tab-panel\" id=\"panel-volumes\">\n"
        + "  <div id=\"vol-container\">\n"
        + "    <canvas id=\"vol-canvas\"></canvas>\n"
        + "    <div class=\"hv-legend\">\n"
        + "      <span>Low</span><div class=\"hv-legend-bar\" id=\"vol-legend-bar\"></div><span>High</span>\n"
        + "      &nbsp;&nbsp; Average flow/hour\n"
        + "    </div>\n"
        + "    <div class=\"hv-floating\">\n"
        + "      <button class=\"btn-float\" id=\"vol-reset-btn\">Reset Zoom</button>\n"
        + "    </div>\n"
        + "  </div>\n"
        + "  <div id=\"vol-timeline-panel\" style=\"flex-shrink:0;padding:14px 20px 16px;background:rgba(0,0,0,0.98);border-top:1px solid #1e293b\">\n"
        + "    <div class=\"row\" style=\"display:flex;align-items:center;gap:14px\">\n"
        + "      <span style=\"font-size:0.85rem;font-weight:600;color:#818cf8\">Flow Period:</span>\n"
        + "      <select class=\"hv-speed\" id=\"vol-period-select\" style=\"background:rgba(30,41,59,0.85);color:#f1f5f9;border:1px solid #334155;border-radius:8px;padding:6px 12px;font-size:.75rem;font-weight:600;cursor:pointer;outline:none;transition:all .2s;display:inline-flex;align-items:center\">\n"
        + "        <option value=\"day\" selected>☀️ Day Flow Average (07:00 – 19:00)</option>\n"
        + "        <option value=\"night\">🌙 Night Flow Average (19:00 – 07:00)</option>\n"
        + "      </select>\n"
        + "      <select class=\"hv-speed\" id=\"vol-theme-select\" title=\"Color Theme\" style=\"background:rgba(30,41,59,0.85);color:#f1f5f9;border:1px solid #334155;border-radius:8px;padding:6px 12px;font-size:.75rem;font-weight:600;cursor:pointer;outline:none;transition:all .2s;display:inline-flex;align-items:center\">\n"
        + "        <option value=\"consistency\" selected>Sim Consistency</option>\n"
        + "        <option value=\"heatmap\">Classic Heatmap</option>\n"
        + "      </select>\n"
        + "    </div>\n"
        + "    <div class=\"legend\" style=\"margin-top:8px\">\n"
        + "      <div class=\"leg-item\"><div style=\"width:20px;height:3px;border-radius:2px;background:#161c2e\"></div> No traffic</div>\n"
        + "      <div class=\"leg-item\"><div id=\"vol-leg-light\" style=\"width:20px;height:3px;border-radius:2px;background:#38bdf8\"></div> Light traffic</div>\n"
        + "      <div class=\"leg-item\"><div id=\"vol-leg-heavy\" style=\"width:20px;height:3px;border-radius:2px;background:#6366f1\"></div> Heavy traffic</div>\n"
        + "      <div class=\"leg-item\" style=\"margin-left:auto;font-style:italic;color:#f8c56d\">Sunrise 06:51 · Sunset 18:30</div>\n"
        + "    </div>\n"
        + "  </div>\n"
        + "</div>\n"
        + "<div class=\"tab-panel\" id=\"panel-ab\">\n"
        + "  <div id=\"ab-container\">\n"
        + "    <canvas id=\"ab-canvas\"></canvas>\n"
        + "    <div id=\"ab-empty-overlay\" style=\"position:absolute;top:0;left:0;width:100%;height:100%;background:rgba(15,23,42,0.95);display:flex;flex-direction:column;align-items:center;justify-content:center;color:#94a3b8;font-size:1rem;font-weight:600;padding:20px;text-align:center;z-index:10\">\n"
        + "      <p style=\"margin-bottom:12px;color:#f1f5f9;font-size:1.2rem\">No A/B Test Data Found</p>\n"
        + "      <p style=\"max-width:450px;font-size:0.85rem;line-height:1.6\">Please run the simulation with A/B testing enabled (enableLightABTesting = true in parameters) once. The simulation will save the 72-pair hourly release test data, which will then sit here permanently.</p>\n"
        + "    </div>\n"
        + "    <div id=\"ab-floating-controls\" style=\"position:absolute;top:20px;right:20px;display:none;flex-direction:column;gap:8px\">\n"
        + "      <label class=\"toggle-label\" id=\"ab-lbl-light\"><input type=\"checkbox\" id=\"ab-tg-light\" checked /><div class=\"toggle-switch\"></div> Light Level Map</label>\n"
        + "      <label class=\"toggle-label\"><input type=\"checkbox\" id=\"ab-tg-tethers\" checked /><div class=\"toggle-switch\"></div> Show A/B Tethers</label>\n"
        + "      <button class=\"btn-float\" id=\"ab-reset-btn\">Reset Zoom</button>\n"
        + "    </div>\n"
        + "  </div>\n"
        + "  <div id=\"ab-timeline-panel\" style=\"display:none;flex-shrink:0;padding:14px 20px 16px;background:rgba(0,0,0,0.98);border-top:1px solid #1e293b\">\n"
        + "    <div class=\"row\" style=\"display:flex;align-items:center;gap:14px\">\n"
        + "      <button id=\"ab-play-btn\" style=\"width:42px;height:42px;border-radius:50%;border:2px solid #6366f1;background:transparent;color:#6366f1;cursor:pointer;font-size:1.2rem;display:flex;align-items:center;justify-content:center;flex-shrink:0;transition:all .2s;padding-left:3px;\" title=\"Play/Pause\">▶</button>\n"
        + "      <input type=\"range\" id=\"ab-step-slider\" style=\"flex:1;-webkit-appearance:none;height:5px;border-radius:3px;background:#1e293b;outline:none;cursor:pointer\" min=\"0\" value=\"0\"/>\n"
        + "      <span id=\"ab-time-label\" style=\"font-size:.8rem;color:#a5b4fc;min-width:48px;text-align:right;font-weight:600\">March 12 00:00</span>\n"
        + "      <select id=\"ab-speed-select\" style=\"background:rgba(30,41,59,0.85);color:#f1f5f9;border:1px solid #334155;border-radius:8px;padding:6px 12px;font-size:.75rem;font-weight:600;cursor:pointer;outline:none;transition:all .2s;display:inline-flex;align-items:center\" title=\"Playback Speed\">\n"
        + "        <option value=\"0.1\">0.1x Speed</option>\n"
        + "        <option value=\"0.5\">0.5x Speed</option>\n"
        + "        <option value=\"1\">1x Speed</option>\n"
        + "        <option value=\"2\" selected>2x Speed</option>\n"
        + "        <option value=\"5\">5x Speed</option>\n"
        + "        <option value=\"10\">10x Speed</option>\n"
        + "      </select>\n"
        + "    </div>\n"
        + "    <div class=\"legend\" style=\"display:flex;gap:16px;margin-top:10px;font-size:.7rem;color:#94a3b8\">\n"
        + "      <div class=\"leg-item\"><div class=\"leg-dot\" style=\"width:8px;height:8px;border-radius:50%;background:#ef4444\"></div> Vulnerable agent</div>\n"
        + "      <div class=\"leg-item\"><div class=\"leg-dot\" style=\"width:8px;height:8px;border-radius:50%;background:#38bdf8\"></div> Normal agent</div>\n"
        + "      <div class=\"leg-item\"><div style=\"width:20px;height:3px;border-radius:2px;border-top:1.5px dashed rgba(255,255,255,0.6)\"></div> Tether</div>\n"
        + "    </div>\n"
        + "  </div>\n"
        + "</div>\n"
        + "<script>\n"
        + "const ROADS_GEOJSON = " + roadsGeoJson + ";\n"
        + "const TRIPS = " + tripsJs + ";\n"
        + "const AB_TRIPS = " + abTripsJs + ";\n"
        + "const HOURLY_VOL = " + hourlyVolJs + ";\n"
        + "const isNight = " + isNight + ";\n"
        + "const enableAB = " + enableAB + ";\n"
        + "if (!isNight) {\n"
        + "  const lbl = document.getElementById('lbl-light'); if (lbl) lbl.style.display = 'none';\n"
        + "  const abLbl = document.getElementById('ab-lbl-light'); if (abLbl) abLbl.style.display = 'none';\n"
        + "}\n"
        + "if (!enableAB) {\n"
        + "  const lbl = document.getElementById('lbl-tethers'); if (lbl) lbl.style.display = 'none';\n"
        + "  const tg = document.getElementById('tg-tethers'); if (tg) tg.checked = false;\n"
        + "}\n"
        + "if (AB_TRIPS && AB_TRIPS.length > 0) {\n"
        + "  document.getElementById('ab-empty-overlay').style.display = 'none';\n"
        + "  document.getElementById('ab-floating-controls').style.display = 'flex';\n"
        + "  document.getElementById('ab-timeline-panel').style.display = 'block';\n"
        + "}\n"
        + "function switchTab(name) {\n"
        + "  if (playing) { playing = false; playBtn.textContent = '▶'; playBtn.classList.remove('playing'); cancelAnimationFrame(animId); }\n"
        + "  if (hvPlaying) { hvPlaying = false; hvPlayBtn.textContent = '▶'; hvPlayBtn.classList.remove('playing'); cancelAnimationFrame(hvAnimId); }\n"
        + "  if (abPlaying) { abPlaying = false; abPlayBtn.textContent = '▶'; abPlayBtn.classList.remove('playing'); cancelAnimationFrame(abAnimId); }\n"
        + "  document.querySelectorAll('.tab-panel').forEach(p => p.classList.remove('active'));\n"
        + "  document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));\n"
        + "  document.getElementById('panel-' + name).classList.add('active');\n"
        + "  const btns = document.querySelectorAll('.tab-btn');\n"
        + "  if (name === 'trips') btns[0].classList.add('active');\n"
        + "  else if (name === 'hourly') btns[1].classList.add('active');\n"
        + "  else if (name === 'volumes') btns[2].classList.add('active');\n"
        + "  else if (name === 'ab') btns[3].classList.add('active');\n"
        + "  document.getElementById('metrics-trips').style.display  = (name === 'trips' || name === 'ab') ? '' : 'none';\n"
        + "  document.getElementById('metrics-hourly').style.display = name === 'hourly' ? '' : 'none';\n"
        + "  document.getElementById('metrics-volumes').style.display = name === 'volumes' ? '' : 'none';\n"
        + "  if (name === 'trips') { canvas.width = canvas.clientWidth; canvas.height = canvas.clientHeight; roadsDirty = true; draw(); }\n"
        + "  if (name === 'hourly') {\n"
        + "    setTimeout(() => { hvCanvas.width = hvCanvas.clientWidth; hvCanvas.height = hvCanvas.clientHeight; resetHvView(); }, 50);\n"
        + "  }\n"
        + "  if (name === 'volumes') {\n"
        + "    setTimeout(() => { volCanvas.width = volCanvas.clientWidth; volCanvas.height = volCanvas.clientHeight; resetVolView(); }, 50);\n"
        + "  }\n"
        + "  if (name === 'ab') {\n"
        + "    setTimeout(() => { abCanvas.width = abCanvas.clientWidth; abCanvas.height = abCanvas.clientHeight; abResetView(); if (AB_TRIPS && AB_TRIPS.length > 0) abUpdateMetrics(abCurrentFloatStep); }, 50);\n"
        + "  }\n"
        + "}\n"
        + "let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;\n"
        + "function getPoints(geom) {\n"
        + "  let pts = [];\n"
        + "  if (geom.type === 'LineString') { pts = geom.coordinates; }\n"
        + "  else if (geom.type === 'MultiLineString') { geom.coordinates.forEach(line => { pts = pts.concat(line); }); }\n"
        + "  return pts;\n"
        + "}\n"
        + "ROADS_GEOJSON.features.forEach(f => {\n"
        + "  if (f.geometry) {\n"
        + "    getPoints(f.geometry).forEach(pt => {\n"
        + "      if (pt[0] < minX) minX = pt[0]; if (pt[0] > maxX) maxX = pt[0];\n"
        + "      if (pt[1] < minY) minY = pt[1]; if (pt[1] > maxY) maxY = pt[1];\n"
        + "    });\n"
        + "  }\n"
        + "});\n"
        + "if (minX === Infinity) { minX = 0; maxX = 1000; minY = 0; maxY = 1000; }\n"
        + "const edgeCoords = {};\n"
        + "ROADS_GEOJSON.features.forEach(f => {\n"
        + "  if (!f.geometry || !f.properties || f.properties.edgeID == null) return;\n"
        + "  edgeCoords[f.properties.edgeID] = getPoints(f.geometry);\n"
        + "});\n"
        + "const maxHourlyVol = (()=>{ let mx=0; Object.values(HOURLY_VOL).forEach(arr=>{ arr.forEach(v=>{ if(v>mx) mx=v; }); }); return mx||1; })();\n"
        + "function getSkyColor(hour) {\n"
        + "  const SUNRISE = 6.85, SUNSET = 18.5;\n"
        + "  const night    = [8,  10, 25];\n"
        + "  const preDawn  = [20, 25, 45];\n"
        + "  const dawn     = [45, 55, 80];\n"
        + "  const day      = [90, 105, 130];\n"
        + "  const dusk     = [60, 70, 95];\n"
        + "  const postDusk = [25, 30, 55];\n"
        + "  function cl(t,mn,mx) { return Math.max(mn, Math.min(mx, t)); }\n"
        + "  function lerp(a, b, t) { const f=cl(t,0,1); return a.map((v,i)=>Math.round(v+(b[i]-v)*f)); }\n"
        + "  function rgb(c) { return `rgb(${c[0]},${c[1]},${c[2]})`; }\n"
        + "  if (hour < SUNRISE - 2) return rgb(night);\n"
        + "  if (hour < SUNRISE - 0.5) return rgb(lerp(night, preDawn, (hour-(SUNRISE-2))/1.5));\n"
        + "  if (hour < SUNRISE) return rgb(lerp(preDawn, dawn, (hour-(SUNRISE-0.5))/0.5));\n"
        + "  if (hour < SUNRISE + 1) return rgb(lerp(dawn, day, (hour-SUNRISE)/1));\n"
        + "  if (hour < SUNSET - 1) return rgb(day);\n"
        + "  if (hour < SUNSET) return rgb(lerp(day, dusk, (hour-(SUNSET-1))/1));\n"
        + "  if (hour < SUNSET + 1.5) return rgb(lerp(dusk, postDusk, (hour-SUNSET)/1.5));\n"
        + "  if (hour < SUNSET + 2.5) return rgb(lerp(postDusk, night, (hour-(SUNSET+1.5))/1));\n"
        + "  return rgb(night);\n"
        + "}\n"
        + "const maxVol = (()=>{ let mx=0; ROADS_GEOJSON.features.forEach(f=>{ const v=f.properties.volume||0; if(v>mx) mx=v; }); return mx||1; })();\n"
        + "function getVolColor(f, useLight) {\n"
        + "  if (useLight) {\n"
        + "    const lux = f.properties.mean_lux || 0;\n"
        + "    if (lux < 5) return '#1e1b4b';\n"
        + "    const t = Math.min(lux / 30, 1);\n"
        + "    const r = Math.round(30 + t * (253 - 30));\n"
        + "    const g = Math.round(27 + t * (224 - 27));\n"
        + "    const b = Math.round(75 + t * (71 - 75));\n"
        + "    return `rgb(${r},${g},${b})`;\n"
        + "  }\n"
        + "  const v = f.properties.volume || 0;\n"
        + "  if (v < 1) return '#161c2e';\n"
        + "  const t = Math.min(Math.sqrt(v) / Math.sqrt(maxVol), 1);\n"
        + "  const r = Math.round(56 + t * (99 - 56));\n"
        + "  const g = Math.round(189 + t * (102 - 189));\n"
        + "  const b = Math.round(248 + t * (241 - 248));\n"
        + "  return `rgb(${r},${g},${b})`;\n"
        + "}\n"
        + "function getVolWeight(f) { const v = f.properties.volume || 0; if (v < 1) return 2.0; return 3.5 + Math.min(Math.sqrt(v) / Math.sqrt(maxVol), 1) * 6.5; }\n"
        + "function getPointAlongPath(coords, segs, totalLength, progress) {\n"
        + "  if (!coords || coords.length === 0) return null;\n"
        + "  if (progress <= 0 || totalLength === 0) return { x: coords[0][0], y: coords[0][1] };\n"
        + "  if (progress >= 1) return { x: coords[coords.length-1][0], y: coords[coords.length-1][1] };\n"
        + "  const targetDist = progress * totalLength; let acc = 0;\n"
        + "  for (let i = 0; i < segs.length; i++) {\n"
        + "    const l = segs[i];\n"
        + "    if (acc + l >= targetDist) { const f = (targetDist - acc) / (l || 1); return { x: coords[i][0]+(coords[i+1][0]-coords[i][0])*f, y: coords[i][1]+(coords[i+1][1]-coords[i][1])*f }; }\n"
        + "    acc += l;\n"
        + "  }\n"
        + "  return { x: coords[coords.length-1][0], y: coords[coords.length-1][1] };\n"
        + "}\n"
        + "let luxGrid = {}, GRID_SZ = 15;\n"
        + "ROADS_GEOJSON.features.forEach(f => {\n"
        + "  if (!f.geometry) return; const lx = f.properties.mean_lux || 0; if (lx === 0) return;\n"
        + "  getPoints(f.geometry).forEach(pt => {\n"
        + "    const gx = Math.floor(pt[0]/GRID_SZ), gy = Math.floor(pt[1]/GRID_SZ), k = gx+','+gy;\n"
        + "    if(!luxGrid[k] || lx > luxGrid[k]) luxGrid[k] = lx;\n"
        + "  });\n"
        + "});\n"
        + "let maxStep = 0;\n"
        + "const tripLengthById = {};\n"
        + "TRIPS.forEach(t => {\n"
        + "  if (t[2] > maxStep) maxStep = t[2];\n"
        + "  const coords = t[3]; const segs = []; let tot = 0;\n"
        + "  for (let i = 0; i < coords.length - 1; i++) {\n"
        + "    const dx = coords[i+1][0]-coords[i][0], dy = coords[i+1][1]-coords[i][1];\n"
        + "    const l = Math.sqrt(dx*dx+dy*dy); segs.push(l); tot += l;\n"
        + "  }\n"
        + "  t[6] = segs; t[7] = tot;\n"
        + "  tripLengthById[t[0]] = tot;\n"
        + "});\n"
        + "TRIPS.sort((a, b) => a[1] - b[1]);\n"
        + "const canvas = document.getElementById('canvas'); const ctx = canvas.getContext('2d');\n"
        + "let currentFloatStep = 0, scale = 1.0, panX = 0, panY = 0;\n"
        + "function toScreen(wx, wy) { return { x: (wx - minX) * scale + panX, y: canvas.height - ((wy - minY) * scale + panY) }; }\n"
        + "function resetView() {\n"
        + "  const dx = maxX - minX, dy = maxY - minY, pad = 40;\n"
        + "  scale = Math.min((canvas.width - pad*2) / (dx || 1), (canvas.height - pad*2) / (dy || 1));\n"
        + "  panX = canvas.width / 2 - (minX + dx / 2 - minX) * scale; panY = canvas.height / 2 - (minY + dy / 2 - minY) * scale;\n"
        + "  roadsDirty = true; draw();\n"
        + "}\n"
        + "let activeList = [], nextTripIdx = 0;\n"
        + "function updateActiveTrips(floatStep) {\n"
        + "  while(nextTripIdx < TRIPS.length && TRIPS[nextTripIdx][1] <= floatStep) {\n"
        + "    activeList.push(TRIPS[nextTripIdx]); nextTripIdx++;\n"
        + "  }\n"
        + "  for (let i = activeList.length - 1; i >= 0; i--) {\n"
        + "    if (activeList[i][2] < floatStep) activeList.splice(i, 1);\n"
        + "  }\n"
        + "}\n"
        + "function getLiveAgents(floatStep) {\n"
        + "  const res = [];\n"
        + "  for (let i = 0; i < activeList.length; i++) {\n"
        + "    const t = activeList[i];\n"
        + "    const duration = t[2] - t[1]; const progress = duration > 0 ? (floatStep - t[1]) / duration : 1.0;\n"
        + "    const pt = getPointAlongPath(t[3], t[6], t[7], progress);\n"
        + "    if (pt) res.push({ id: t[0], vuln: t[4], x: pt.x, y: pt.y, progress });\n"
        + "  }\n"
        + "  return res;\n"
        + "}\n"
        + "const offscreenVol = document.createElement('canvas'); const offCtxVol = offscreenVol.getContext('2d');\n"
        + "const offscreenLight = document.createElement('canvas'); const offCtxLight = offscreenLight.getContext('2d');\n"
        + "let roadsDirty = true;\n"
        + "const tgLight = document.getElementById('tg-light');\n"
        + "const tgTethers = document.getElementById('tg-tethers');\n"
        + "tgLight.addEventListener('change', () => { draw(); });\n"
        + "tgTethers.addEventListener('change', () => { draw(); });\n"
        + "function buildRoadLayers() {\n"
        + "  offscreenVol.width = canvas.width; offscreenVol.height = canvas.height;\n"
        + "  offscreenLight.width = canvas.width; offscreenLight.height = canvas.height;\n"
        + "  offCtxVol.clearRect(0, 0, canvas.width, canvas.height); offCtxLight.clearRect(0, 0, canvas.width, canvas.height);\n"
        + "  ROADS_GEOJSON.features.forEach(f => {\n"
        + "    if (!f.geometry) return; const pts = getPoints(f.geometry); if (pts.length < 2) return;\n"
        + "    offCtxVol.strokeStyle = getVolColor(f, false); offCtxVol.lineWidth = getVolWeight(f);\n"
        + "    offCtxVol.lineCap = 'round'; offCtxVol.lineJoin = 'round'; offCtxVol.beginPath();\n"
        + "    let p0 = toScreen(pts[0][0], pts[0][1]); offCtxVol.moveTo(p0.x, p0.y);\n"
        + "    for (let i = 1; i < pts.length; i++) { const pi = toScreen(pts[i][0], pts[i][1]); offCtxVol.lineTo(pi.x, pi.y); }\n"
        + "    offCtxVol.stroke();\n"
        + "    offCtxLight.strokeStyle = getVolColor(f, true); offCtxLight.lineWidth = getVolWeight(f);\n"
        + "    offCtxLight.lineCap = 'round'; offCtxLight.lineJoin = 'round'; offCtxLight.beginPath();\n"
        + "    p0 = toScreen(pts[0][0], pts[0][1]); offCtxLight.moveTo(p0.x, p0.y);\n"
        + "    for (let i = 1; i < pts.length; i++) { const pi = toScreen(pts[i][0], pts[i][1]); offCtxLight.lineTo(pi.x, pi.y); }\n"
        + "    offCtxLight.stroke();\n"
        + "  });\n"
        + "  roadsDirty = false;\n"
        + "}\n"
        + "function draw(agents) {\n"
        + "  if (!agents) agents = getLiveAgents(currentFloatStep);\n"
        + "  if (roadsDirty) buildRoadLayers();\n"
        + "  const simHour = (currentFloatStep * 20 / 60) % 24;\n"
        + "  ctx.fillStyle = getSkyColor(simHour);\n"
        + "  ctx.fillRect(0, 0, canvas.width, canvas.height);\n"
        + "  ctx.drawImage(tgLight.checked ? offscreenLight : offscreenVol, 0, 0);\n"
        + "  const posById = {};\n"
        + "  agents.forEach(a => { posById[a.id] = toScreen(a.x, a.y); });\n"
        + "  if (tgTethers.checked) {\n"
        + "    ctx.strokeStyle = 'rgba(255, 255, 255, 0.4)'; ctx.lineWidth = 1.5; ctx.setLineDash([4, 4]);\n"
        + "    ctx.beginPath();\n"
        + "    agents.forEach(a => {\n"
        + "      if (a.vuln && posById[a.id + 1]) {\n"
        + "        const p1 = posById[a.id], p2 = posById[a.id + 1];\n"
        + "        ctx.moveTo(p1.x, p1.y); ctx.lineTo(p2.x, p2.y);\n"
        + "      }\n"
        + "    });\n"
        + "    ctx.stroke(); ctx.setLineDash([]);\n"
        + "  }\n"
        + "  agents.forEach(a => {\n"
        + "    const p = posById[a.id];\n"
        + "    ctx.beginPath();\n"
        + "    ctx.arc(p.x, p.y, a.vuln ? 7 : 5, 0, Math.PI * 2);\n"
        + "    ctx.fillStyle = a.vuln ? '#ef4444' : '#38bdf8';\n"
        + "    ctx.fill();\n"
        + "  });\n"
        + "}\n"
        + "let isDragging = false, startX = 0, startY = 0;\n"
        + "canvas.addEventListener('mousedown', e => { isDragging = true; startX = e.clientX; startY = e.clientY; });\n"
        + "window.addEventListener('mousemove', e => { if (!isDragging) return; panX += e.clientX - startX; panY -= e.clientY - startY; startX = e.clientX; startY = e.clientY; roadsDirty = true; draw(); });\n"
        + "window.addEventListener('mouseup', () => { isDragging = false; });\n"
        + "canvas.addEventListener('wheel', e => {\n"
        + "  e.preventDefault(); const rect = canvas.getBoundingClientRect(), mouseX = e.clientX - rect.left, mouseY = e.clientY - rect.top;\n"
        + "  const worldX = (mouseX - panX) / scale + minX, worldY = (canvas.height - mouseY - panY) / scale + minY;\n"
        + "  const factor = e.deltaY < 0 ? 1.12 : 1/1.12;\n"
        + "  scale *= factor;\n"
        + "  panX = mouseX - (worldX - minX) * scale; panY = (canvas.height - mouseY) - (worldY - minY) * scale;\n"
        + "  roadsDirty = true; draw();\n"
        + "}, { passive: false });\n"
        + "document.getElementById('reset-btn').addEventListener('click', resetView);\n"
        + "function updateMetrics(step, agents) {\n"
        + "  if (!agents) agents = getLiveAgents(step);\n"
        + "  const total = agents.length, vuln = agents.filter(a=>a.vuln).length;\n"
        + "  document.getElementById('m-active').textContent = total;\n"
        + "  document.getElementById('m-vuln').textContent = total > 0 ? (vuln/total*100).toFixed(1)+'%' : '0%';\n"
        + "  const simHour = (step * 20 / 60) % 24;\n"
        + "  const hh = String(Math.floor(simHour)).padStart(2,'0');\n"
        + "  const mm = String(Math.floor(step*20)%60).padStart(2,'0');\n"
        + "  document.getElementById('m-time').textContent = `March 12 ${hh}:${mm}`;\n"
        + "  document.getElementById('time-label').textContent = document.getElementById('m-time').textContent;\n"
        + "  if (isNight) {\n"
        + "    const shouldLight = simHour >= 18.5 || simHour < 6.85;\n"
        + "    if (tgLight.checked !== shouldLight) {\n"
        + "      tgLight.checked = shouldLight;\n"
        + "      draw(agents);\n"
        + "    }\n"
        + "  }\n"
        + "  const vulnActive = agents.filter(a => a.vuln);\n"
        + "  const normActive = agents.filter(a => !a.vuln);\n"
        + "  const avgVulnActiveDist = vulnActive.length > 0 ? Math.round(vulnActive.reduce((s, a) => s + (tripLengthById[a.id] || 0), 0) / vulnActive.length) : 0;\n"
        + "  const avgNormActiveDist = normActive.length > 0 ? Math.round(normActive.reduce((s, a) => s + (tripLengthById[a.id] || 0), 0) / normActive.length) : 0;\n"
        + "  document.getElementById('m-vuln-d').textContent = avgVulnActiveDist > 0 ? avgVulnActiveDist + ' m' : '-';\n"
        + "  document.getElementById('m-norm-d').textContent = avgNormActiveDist > 0 ? avgNormActiveDist + ' m' : '-';\n"
        + "  if (tgLight.checked && total > 0) {\n"
        + "    let sumLux = 0;\n"
        + "    agents.forEach(a => {\n"
        + "      const gx = Math.floor(a.x/GRID_SZ), gy = Math.floor(a.y/GRID_SZ);\n"
        + "      sumLux += luxGrid[gx+','+gy] || 0;\n"
        + "    });\n"
        + "    document.getElementById('m-lux').textContent = (sumLux / total).toFixed(2);\n"
        + "  } else {\n"
        + "    document.getElementById('m-lux').textContent = '-';\n"
        + "  }\n"
        + "}\n"
        + "const slider = document.getElementById('step-slider'); slider.min = 0; slider.max = (Math.max(1, maxStep)) * 10; slider.value = 0;\n"
        + "slider.addEventListener('input', () => {\n"
        + "  let val = parseInt(slider.value); currentFloatStep = val / 10;\n"
        + "  nextTripIdx = 0; activeList = []; updateActiveTrips(currentFloatStep);\n"
        + "  const agents = getLiveAgents(currentFloatStep);\n"
        + "  draw(agents); updateMetrics(currentFloatStep, agents);\n"
        + "});\n"
        + "let playing = false, animId = null, lastTs = 0; const playBtn = document.getElementById('play-btn'), speedSelect = document.getElementById('speed-select');\n"
        + "playBtn.addEventListener('click', () => { playing = !playing; playBtn.textContent = playing ? '⏸' : '▶'; if (playing) { playBtn.classList.add('playing'); animate(); } else { playBtn.classList.remove('playing'); cancelAnimationFrame(animId); } });\n"
        + "function animate(ts) {\n"
        + "  if (!playing) return;\n"
        + "  if (ts - lastTs > 30) {\n"
        + "    lastTs = ts; let sliderFloatVal = parseFloat(slider.value) + (parseFloat(speedSelect.value) || 2);\n"
        + "    if (sliderFloatVal > slider.max) { sliderFloatVal = 0; nextTripIdx = 0; activeList = []; }\n"
        + "    slider.value = Math.floor(sliderFloatVal); currentFloatStep = sliderFloatVal / 10;\n"
        + "    updateActiveTrips(currentFloatStep);\n"
        + "    const agents = getLiveAgents(currentFloatStep);\n"
        + "    draw(agents); updateMetrics(currentFloatStep, agents);\n"
        + "  }\n"
        + "  animId = requestAnimationFrame(animate);\n"
        + "}\n"
        + "const hvCanvas = document.getElementById('hv-canvas'); const hvCtx = hvCanvas.getContext('2d');\n"
        + "let hvScale = 1.0, hvPanX = 0, hvPanY = 0, hvDragging = false, hvDX = 0, hvDY = 0;\n"
        + "function toHvScreen(wx, wy) { return { x: (wx - minX) * hvScale + hvPanX, y: hvCanvas.height - ((wy - minY) * hvScale + hvPanY) }; }\n"
        + "function resetHvView() {\n"
        + "  const dx = maxX - minX, dy = maxY - minY, pad = 40;\n"
        + "  hvScale = Math.min((hvCanvas.width - pad*2) / (dx || 1), (hvCanvas.height - pad*2) / (dy || 1));\n"
        + "  hvPanX = hvCanvas.width / 2 - (dx / 2) * hvScale; hvPanY = hvCanvas.height / 2 - (dy / 2) * hvScale;\n"
        + "  hvDraw(currentHvHour);\n"
        + "}\n"
        + "function getHvRoadColor(vol, maxV, isDay) {\n"
        + "  if (vol === 0) return '#161c2e';\n"
        + "  const t = Math.min(Math.sqrt(vol) / Math.sqrt(maxV), 1);\n"
        + "  const theme = document.getElementById('hv-theme-select') ? document.getElementById('hv-theme-select').value : 'consistency';\n"
        + "  if (theme === 'consistency') {\n"
        + "    const r = Math.round(56 + t * (99 - 56));\n"
        + "    const g = Math.round(189 + t * (102 - 189));\n"
        + "    const b = Math.round(248 + t * (241 - 248));\n"
        + "    return `rgb(${r},${g},${b})`;\n"
        + "  } else {\n"
        + "    let r, g, b;\n"
        + "    if (t < 0.5) { const u = t * 2; r = Math.round(245 + u * (239 - 245)); g = Math.round(158 + u * (68 - 158)); b = Math.round(11 + u * (68 - 11)); }\n"
        + "    else { const u = (t - 0.5) * 2; r = Math.round(239 + u * (162 - 239)); g = Math.round(68 + u * (28 - 68)); b = Math.round(68 + u * (175 - 68)); }\n"
        + "    return `rgb(${r},${g},${b})`;\n"
        + "  }\n"
        + "}\n"
        + "function hvDraw(hour) {\n"
        + "  const hrInt = Math.floor(hour) % 24;\n"
        + "  const isDay = hour >= 6.85 && hour < 18.5;\n"
        + "  hvCtx.clearRect(0, 0, hvCanvas.width, hvCanvas.height); hvCtx.fillStyle = getSkyColor(hour); hvCtx.fillRect(0, 0, hvCanvas.width, hvCanvas.height);\n"
        + "  let hvRoadsActive = 0, hvPeakVol = 0, hvTotalPeds = 0;\n"
        + "  ROADS_GEOJSON.features.forEach(f => {\n"
        + "    if (!f.geometry) return; const pts = getPoints(f.geometry); if (pts.length < 2) return;\n"
        + "    const eid = f.properties && f.properties.edgeID; const vol = (eid != null && HOURLY_VOL[eid]) ? HOURLY_VOL[eid][hrInt] : 0;\n"
        + "    if (vol > 0) { hvRoadsActive++; hvTotalPeds += vol; if (vol > hvPeakVol) hvPeakVol = vol; }\n"
        + "    hvCtx.strokeStyle = getHvRoadColor(vol, maxHourlyVol, isDay); hvCtx.lineWidth = vol > 0 ? (2.5 + Math.min(Math.sqrt(vol) / Math.sqrt(maxHourlyVol), 1) * 6) : 1.5;\n"
        + "    hvCtx.lineCap = 'round'; hvCtx.lineJoin = 'round'; hvCtx.beginPath();\n"
        + "    let p0 = toHvScreen(pts[0][0], pts[0][1]); hvCtx.moveTo(p0.x, p0.y);\n"
        + "    for (let i = 1; i < pts.length; i++) { let pi = toHvScreen(pts[i][0], pts[i][1]); hvCtx.lineTo(pi.x, pi.y); }\n"
        + "    hvCtx.stroke();\n"
        + "  });\n"
        + "  const hh = String(hrInt).padStart(2,'0'), nh = String((hrInt+1)%24).padStart(2,'0');\n"
        + "  const sunTxt = hour < 6.85 ? '🌙 Night' : hour < 7.35 ? '🌅 Sunrise' : hour < 17.5 ? '☀️ Day' : hour < 18.5 ? '🌇 Sunset' : '🌙 Night';\n"
        + "  document.getElementById('hv-time-label').textContent = `${hh}:00 – ${nh}:00`;\n"
        + "  document.getElementById('m-hv-hour').textContent  = `${hh}:00`; document.getElementById('m-hv-roads').textContent = hvRoadsActive;\n"
        + "  document.getElementById('m-hv-peak').textContent  = hvPeakVol; document.getElementById('m-hv-total').textContent = hvTotalPeds;\n"
        + "  document.getElementById('m-hv-sky').textContent   = sunTxt;\n"
        + "  hvCtx.font = 'bold 13px Inter, sans-serif'; hvCtx.fillStyle = isDay ? 'rgba(20,20,40,0.85)' : 'rgba(200,210,240,0.9)';\n"
        + "  hvCtx.fillText(`March 12   ${hh}:00 – ${nh}:00   ${sunTxt}`, 20, hvCanvas.height - 54);\n"
        + "}\n"
        + "let currentHvHour = 0; const hvSlider = document.getElementById('hv-hour-slider');\n"
        + "hvSlider.addEventListener('input', () => { currentHvHour = parseInt(hvSlider.value); hvFrac = 0; updateHvHourButtons(); hvDraw(currentHvHour); });\n"
        + "let hvPlaying = false, hvAnimId = null, hvLastTs = 0, hvFrac = 0; const hvPlayBtn = document.getElementById('hv-play-btn'), hvSpeedSel = document.getElementById('hv-speed-select');\n"
        + "hvPlayBtn.addEventListener('click', () => { hvPlaying = !hvPlaying; hvPlayBtn.textContent = hvPlaying ? '⏸' : '▶'; if (hvPlaying) { hvPlayBtn.classList.add('playing'); hvLastTs = performance.now(); hvFrac = 0; hvAnimId = requestAnimationFrame(hvAnimate); } else { hvPlayBtn.classList.remove('playing'); cancelAnimationFrame(hvAnimId); } });\n"
        + "function hvAnimate(ts) {\n"
        + "  if (!hvPlaying) return;\n"
        + "  const dt = ts - hvLastTs; hvLastTs = ts; hvFrac += dt * (parseFloat(hvSpeedSel.value) || 1) / 1500;\n"
        + "  if (hvFrac >= 1) { hvFrac -= 1; currentHvHour = (currentHvHour + 1) % 24; hvSlider.value = currentHvHour; updateHvHourButtons(); }\n"
        + "  const floatHour = (currentHvHour + hvFrac) % 24;\n"
        + "  hvDraw(floatHour);\n"
        + "  hvAnimId = requestAnimationFrame(hvAnimate);\n"
        + "}\n"
        + "let hvZoomRafId = null; hvCanvas.addEventListener('mousedown', e => { hvDragging = true; hvDX = e.clientX; hvDY = e.clientY; });\n"
        + "window.addEventListener('mousemove', e => { if(!hvDragging) return; hvPanX += e.clientX-hvDX; hvPanY -= e.clientY-hvDY; hvDX=e.clientX; hvDY=e.clientY; if (hvZoomRafId) cancelAnimationFrame(hvZoomRafId); hvZoomRafId = requestAnimationFrame(() => { hvZoomRafId=null; hvDraw(currentHvHour + hvFrac); }); });\n"
        + "window.addEventListener('mouseup', () => { hvDragging = false; });\n"
        + "hvCanvas.addEventListener('wheel', e => {\n"
        + "  e.preventDefault(); const rect = hvCanvas.getBoundingClientRect(), mx = e.clientX-rect.left, my = e.clientY-rect.top; const wx = (mx-hvPanX)/hvScale+minX, wy = (hvCanvas.height-my-hvPanY)/hvScale+minY;\n"
        + "  const factor = e.deltaY < 0 ? 1.15 : 1/1.15; hvScale *= factor; hvPanX = mx-(wx-minX)*hvScale; hvPanY = (hvCanvas.height-my)-(wy-minY)*hvScale;\n"
        + "  if (hvZoomRafId) cancelAnimationFrame(hvZoomRafId); hvZoomRafId = requestAnimationFrame(() => { hvZoomRafId=null; hvDraw(currentHvHour + hvFrac); });\n"
        + "}, { passive: false });\n"
        + "document.getElementById('hv-reset-btn').addEventListener('click', resetHvView);\n"
        + "const grid = document.getElementById('hv-hour-grid');\n"
        + "for (let h = 0; h < 24; h++) {\n"
        + "  const btn = document.createElement('button');\n"
        + "  btn.className = 'hour-btn';\n"
        + "  btn.textContent = String(h).padStart(2, '0');\n"
        + "  btn.addEventListener('click', () => {\n"
        + "    currentHvHour = h; hvSlider.value = h; hvFrac = 0; updateHvHourButtons(); hvDraw(h);\n"
        + "  });\n"
        + "  grid.appendChild(btn);\n"
        + "}\n"
        + "function updateHvHourButtons() {\n"
        + "  const btns = document.querySelectorAll('.hour-btn');\n"
        + "  btns.forEach((btn, idx) => {\n"
        + "    if (idx === currentHvHour) btn.classList.add('active');\n"
        + "    else btn.classList.remove('active');\n"
        + "  });\n"
        + "}\n"
        + "updateHvHourButtons();\n"
        + "const hvThemeSelect = document.getElementById('hv-theme-select');\n"
        + "function updateHvLegend() {\n"
        + "  const theme = hvThemeSelect ? hvThemeSelect.value : 'consistency';\n"
        + "  const bar = document.querySelector('.hv-legend-bar');\n"
        + "  const legLight = document.getElementById('hv-leg-light');\n"
        + "  const legHeavy = document.getElementById('hv-leg-heavy');\n"
        + "  if (theme === 'consistency') {\n"
        + "    if (bar) bar.style.background = 'linear-gradient(to right, #161c2e, #38bdf8, #6366f1)';\n"
        + "    if (legLight) legLight.style.background = '#38bdf8';\n"
        + "    if (legHeavy) legHeavy.style.background = '#6366f1';\n"
        + "  } else {\n"
        + "    if (bar) bar.style.background = 'linear-gradient(to right, #161c2e, #f59e0b, #ef4444)';\n"
        + "    if (legLight) legLight.style.background = '#f59e0b';\n"
        + "    if (legHeavy) legHeavy.style.background = '#ef4444';\n"
        + "  }\n"
        + "}\n"
        + "if (hvThemeSelect) {\n"
        + "  hvThemeSelect.addEventListener('change', () => {\n"
        + "    updateHvLegend();\n"
        + "    hvDraw(currentHvHour + hvFrac);\n"
        + "  });\n"
        + "}\n"
        + "updateHvLegend();\n"
        + "const abCanvas = document.getElementById('ab-canvas'); const abCtx = abCanvas.getContext('2d');\n"
        + "let abScale = 1.0, abPanX = 0, abPanY = 0, abCurrentFloatStep = 0;\n"
        + "function abToScreen(wx, wy) { return { x: (wx - minX) * abScale + abPanX, y: abCanvas.height - ((wy - minY) * abScale + abPanY) }; }\n"
        + "function abResetView() {\n"
        + "  const dx = maxX - minX, dy = maxY - minY, pad = 40;\n"
        + "  abScale = Math.min((abCanvas.width - pad*2) / (dx || 1), (abCanvas.height - pad*2) / (dy || 1));\n"
        + "  abPanX = abCanvas.width / 2 - (minX + dx / 2 - minX) * abScale; abPanY = abCanvas.height / 2 - (minY + dy / 2 - minY) * abScale;\n"
        + "  abRoadsDirty = true; abDraw();\n"
        + "}\n"
        + "let abMaxStep = 0;\n"
        + "const abTripLengthById = {};\n"
        + "AB_TRIPS.forEach(t => {\n"
        + "  if (t[2] > abMaxStep) abMaxStep = t[2]; const coords = t[3]; const segs = []; let tot = 0;\n"
        + "  for (let i = 0; i < coords.length - 1; i++) { const dx = coords[i+1][0]-coords[i][0], dy = coords[i+1][1]-coords[i][1]; const l = Math.sqrt(dx*dx+dy*dy); segs.push(l); tot += l; }\n"
        + "  t[6] = segs; t[7] = tot;\n"
        + "  abTripLengthById[t[0]] = tot;\n"
        + "}); AB_TRIPS.sort((a, b) => a[1] - b[1]); let abActiveList = [], abNextTripIdx = 0;\n"
        + "function abUpdateActiveTrips(floatStep) {\n"
        + "  while(abNextTripIdx < AB_TRIPS.length && AB_TRIPS[abNextTripIdx][1] <= floatStep) { abActiveList.push(AB_TRIPS[abNextTripIdx]); abNextTripIdx++; }\n"
        + "  for (let i = abActiveList.length - 1; i >= 0; i--) { if (abActiveList[i][2] < floatStep) abActiveList.splice(i, 1); }\n"
        + "}\n"
        + "function abGetLiveAgents(floatStep) {\n"
        + "  const res = []; for (let i = 0; i < abActiveList.length; i++) { const t = abActiveList[i]; const duration = t[2] - t[1]; const progress = duration > 0 ? (floatStep - t[1]) / duration : 1.0; const pt = getPointAlongPath(t[3], t[6], t[7], progress); if (pt) res.push({ id: t[0], vuln: t[4], x: pt.x, y: pt.y, progress }); } return res;\n"
        + "}\n"
        + "const abOffscreenVol = document.createElement('canvas'); const abOffCtxVol = abOffscreenVol.getContext('2d'); const abOffscreenLight = document.createElement('canvas'); const abOffscreenLightCtx = abOffscreenLight.getContext('2d'); let abRoadsDirty = true;\n"
        + "const abTgLight = document.getElementById('ab-tg-light'); const abTgTethers = document.getElementById('ab-tg-tethers');\n"
        + "if (abTgLight) abTgLight.addEventListener('change', () => { abDraw(); }); if (abTgTethers) abTgTethers.addEventListener('change', () => { abDraw(); });\n"
        + "function abBuildRoadLayers() {\n"
        + "  abOffscreenVol.width = abCanvas.width; abOffscreenVol.height = abCanvas.height; abOffscreenLight.width = abCanvas.width; abOffscreenLight.height = abCanvas.height;\n"
        + "  abOffCtxVol.clearRect(0, 0, abCanvas.width, abCanvas.height); abOffscreenLightCtx.clearRect(0, 0, abCanvas.width, abCanvas.height);\n"
        + "  ROADS_GEOJSON.features.forEach(f => {\n"
        + "    if (!f.geometry) return; const pts = getPoints(f.geometry); if (pts.length < 2) return;\n"
        + "    abOffCtxVol.strokeStyle = getVolColor(f, false); abOffCtxVol.lineWidth = getVolWeight(f); abOffCtxVol.lineCap = 'round'; abOffCtxVol.lineJoin = 'round'; abOffCtxVol.beginPath();\n"
        + "    let p0 = abToScreen(pts[0][0], pts[0][1]); abOffCtxVol.moveTo(p0.x, p0.y); for (let i = 1; i < pts.length; i++) { const pi = abToScreen(pts[i][0], pts[i][1]); abOffCtxVol.lineTo(pi.x, pi.y); } abOffCtxVol.stroke();\n"
        + "    abOffscreenLightCtx.strokeStyle = getVolColor(f, true); abOffscreenLightCtx.lineWidth = getVolWeight(f); abOffscreenLightCtx.lineCap = 'round'; abOffscreenLightCtx.lineJoin = 'round'; abOffscreenLightCtx.beginPath();\n"
        + "    p0 = abToScreen(pts[0][0], pts[0][1]); abOffscreenLightCtx.moveTo(p0.x, p0.y); for (let i = 1; i < pts.length; i++) { const pi = abToScreen(pts[i][0], pts[i][1]); abOffscreenLightCtx.lineTo(pi.x, pi.y); } abOffscreenLightCtx.stroke();\n"
        + "  }); abRoadsDirty = false;\n"
        + "}\n"
        + "function abDraw(agents) {\n"
        + "  if (!agents) agents = abGetLiveAgents(abCurrentFloatStep);\n"
        + "  if (abRoadsDirty) abBuildRoadLayers(); const simHour = (abCurrentFloatStep * 20 / 60) % 24; abCtx.fillStyle = getSkyColor(simHour); abCtx.fillRect(0, 0, abCanvas.width, abCanvas.height);\n"
        + "  abCtx.drawImage(abTgLight && abTgLight.checked ? abOffscreenLight : abOffscreenVol, 0, 0); const posById = {};\n"
        + "  agents.forEach(a => { posById[a.id] = abToScreen(a.x, a.y); }); if (abTgTethers && abTgTethers.checked) {\n"
        + "    abCtx.strokeStyle = 'rgba(255, 255, 255, 0.4)'; abCtx.lineWidth = 1.5; abCtx.setLineDash([4, 4]); abCtx.beginPath();\n"
        + "    agents.forEach(a => { if (a.vuln && posById[a.id + 1]) { const p1 = posById[a.id], p2 = posById[a.id + 1]; abCtx.moveTo(p1.x, p1.y); abCtx.lineTo(p2.x, p2.y); } });\n"
        + "    abCtx.stroke(); abCtx.setLineDash([]);\n"
        + "  }\n"
        + "  agents.forEach(a => { const p = posById[a.id]; abCtx.beginPath(); abCtx.arc(p.x, p.y, a.vuln ? 7 : 5, 0, Math.PI * 2); abCtx.fillStyle = a.vuln ? '#ef4444' : '#38bdf8'; abCtx.fill(); });\n"
        + "}\n"
        + "let abIsDragging = false, abStartX = 0, abStartY = 0; abCanvas.addEventListener('mousedown', e => { abIsDragging = true; abStartX = e.clientX; abStartY = e.clientY; });\n"
        + "window.addEventListener('mousemove', e => { if (!abIsDragging) return; abPanX += e.clientX - abStartX; abPanY -= e.clientY - abStartY; abStartX = e.clientX; abStartY = e.clientY; abRoadsDirty = true; abDraw(); });\n"
        + "window.addEventListener('mouseup', () => { abIsDragging = false; }); let abZoomRafId = null;\n"
        + "abCanvas.addEventListener('wheel', e => {\n"
        + "  e.preventDefault(); const rect = abCanvas.getBoundingClientRect(), mouseX = e.clientX - rect.left, mouseY = e.clientY - rect.top; const worldX = (mouseX - abPanX) / abScale + minX, worldY = (abCanvas.height - mouseY - abPanY) / abScale + minY; const factor = e.deltaY < 0 ? 1.12 : 1/1.12;\n"
        + "  abScale *= factor; abPanX = mouseX - (worldX - minX) * abScale; abPanY = (abCanvas.height - mouseY) - (worldY - minY) * abScale; abRoadsDirty = true; if (abZoomRafId) cancelAnimationFrame(abZoomRafId); abZoomRafId = requestAnimationFrame(() => { abZoomRafId = null; abDraw(); });\n"
        + "}, { passive: false }); document.getElementById('ab-reset-btn').addEventListener('click', abResetView);\n"
        + "function abUpdateMetrics(step, agents) {\n"
        + "  if (!agents) agents = abGetLiveAgents(step);\n"
        + "  const total = agents.length, vuln = agents.filter(a=>a.vuln).length; document.getElementById('m-active').textContent = total; document.getElementById('m-vuln').textContent = total > 0 ? (vuln/total*100).toFixed(1)+'%' : '0%';\n"
        + "  const simHour = (step * 20 / 60) % 24; const hh = String(Math.floor(simHour)).padStart(2,'0'); const mm = String(Math.floor(step*20)%60).padStart(2,'0');\n"
        + "  document.getElementById('m-time').textContent = `March 12 ${hh}:${mm}`; document.getElementById('ab-time-label').textContent = document.getElementById('m-time').textContent;\n"
        + "  if (isNight && abTgLight) { const shouldLight = simHour >= 18.5 || simHour < 6.85; if (abTgLight.checked !== shouldLight) { abTgLight.checked = shouldLight; abDraw(agents); } }\n"
        + "  const vulnActive = agents.filter(a => a.vuln); const normActive = agents.filter(a => !a.vuln);\n"
        + "  const avgVulnActiveDist = vulnActive.length > 0 ? Math.round(vulnActive.reduce((s, a) => s + (abTripLengthById[a.id] || 0), 0) / vulnActive.length) : 0;\n"
        + "  const avgNormActiveDist = normActive.length > 0 ? Math.round(normActive.reduce((s, a) => s + (abTripLengthById[a.id] || 0), 0) / normActive.length) : 0;\n"
        + "  document.getElementById('m-vuln-d').textContent = avgVulnActiveDist > 0 ? avgVulnActiveDist + ' m' : '-'; document.getElementById('m-norm-d').textContent = avgNormActiveDist > 0 ? avgNormActiveDist + ' m' : '-';\n"
        + "  if (abTgLight && abTgLight.checked && total > 0) {\n"
        + "    let sumLux = 0; agents.forEach(a => {\n"
        + "      const gx = Math.floor(a.x/GRID_SZ), gy = Math.floor(a.y/GRID_SZ); sumLux += luxGrid[gx+','+gy] || 0; });\n"
        + "    document.getElementById('m-lux').textContent = (sumLux / total).toFixed(2); } else { document.getElementById('m-lux').textContent = '-'; }\n"
        + "}\n"
        + "const abSlider = document.getElementById('ab-step-slider'); if (abSlider) { abSlider.min = 0; abSlider.max = (Math.max(1, abMaxStep)) * 10; abSlider.value = 0;\n"
        + "  abSlider.addEventListener('input', () => { abSliderFloatVal = parseInt(abSlider.value); abCurrentFloatStep = abSliderFloatVal / 10; abNextTripIdx = 0; abActiveList = []; abUpdateActiveTrips(abCurrentFloatStep); const agents = abGetLiveAgents(abCurrentFloatStep); abDraw(agents); abUpdateMetrics(abCurrentFloatStep, agents); }); }\n"
        + "let abPlaying = false, abAnimId = null, abLastTs = 0, abSliderFloatVal = 0; const abPlayBtn = document.getElementById('ab-play-btn'), abSpeedSelect = document.getElementById('ab-speed-select');\n"
        + "if (abPlayBtn) { abPlayBtn.addEventListener('click', () => { abPlaying = !abPlaying; abPlayBtn.textContent = abPlaying ? '⏸' : '▶'; if (abPlaying) { abPlayBtn.classList.add('playing'); abAnimate(); } else { abPlayBtn.classList.remove('playing'); cancelAnimationFrame(abAnimId); } }); }\n"
        + "function abAnimate(ts) {\n"
        + "  if (!abPlaying) return; if (ts - abLastTs > 30) { abLastTs = ts; abSliderFloatVal = abSliderFloatVal + (parseFloat(abSpeedSelect.value) || 2); if (abSliderFloatVal > abSlider.max) { abSliderFloatVal = 0; abNextTripIdx = 0; abActiveList = []; }\n"
        + "    abSlider.value = Math.floor(abSliderFloatVal); abCurrentFloatStep = abSliderFloatVal / 10; abUpdateActiveTrips(abCurrentFloatStep); const agents = abGetLiveAgents(abCurrentFloatStep); abDraw(agents); abUpdateMetrics(abCurrentFloatStep, agents); } abAnimId = requestAnimationFrame(abAnimate);\n"
        + "}\n"
        + "window.addEventListener('resize', () => {\n"
        + "  canvas.width = canvas.clientWidth; canvas.height = canvas.clientHeight; roadsDirty = true; draw();\n"
        + "  if (document.getElementById('panel-hourly').classList.contains('active')) { hvCanvas.width = hvCanvas.clientWidth; hvCanvas.height = hvCanvas.clientHeight; hvDraw(currentHvHour + hvFrac); }\n"
        + "  if (document.getElementById('panel-volumes').classList.contains('active')) { volCanvas.width = volCanvas.clientWidth; volCanvas.height = volCanvas.clientHeight; volDraw(); }\n"
        + "  if (document.getElementById('panel-ab').classList.contains('active')) { abCanvas.width = abCanvas.clientWidth; abCanvas.height = abCanvas.clientHeight; abRoadsDirty = true; abDraw(); }\n"
        + "});\n"
        + "const volCanvas = document.getElementById('vol-canvas'); const volCtx = volCanvas.getContext('2d');\n"
        + "let volScale = 1.0, volPanX = 0, volPanY = 0, volDragging = false, volDX = 0, volDY = 0;\n"
        + "function toVolScreen(wx, wy) { return { x: (wx - minX) * volScale + volPanX, y: volCanvas.height - ((wy - minY) * volScale + volPanY) }; }\n"
        + "function resetVolView() {\n"
        + "  const dx = maxX - minX, dy = maxY - minY, pad = 40;\n"
        + "  volScale = Math.min((volCanvas.width - pad*2) / (dx || 1), (volCanvas.height - pad*2) / (dy || 1));\n"
        + "  volPanX = volCanvas.width / 2 - (dx / 2) * volScale; volPanY = volCanvas.height / 2 - (dy / 2) * volScale;\n"
        + "  volDraw();\n"
        + "}\n"
        + "const edgeAverages = {}; let maxDayAvg = 0, maxNightAvg = 0;\n"
        + "Object.keys(HOURLY_VOL).forEach(eid => {\n"
        + "  const vols = HOURLY_VOL[eid]; let daySum = 0, nightSum = 0;\n"
        + "  for (let h = 0; h < 24; h++) {\n"
        + "    if (h >= 7 && h <= 18) daySum += vols[h];\n"
        + "    else nightSum += vols[h];\n"
        + "  }\n"
        + "  const dayAvg = daySum / 12, nightAvg = nightSum / 12;\n"
        + "  edgeAverages[eid] = { day: dayAvg, night: nightAvg };\n"
        + "  if (dayAvg > maxDayAvg) maxDayAvg = dayAvg;\n"
        + "  if (nightAvg > maxNightAvg) maxNightAvg = nightAvg;\n"
        + "});\n"
        + "if (maxDayAvg === 0) maxDayAvg = 1; if (maxNightAvg === 0) maxNightAvg = 1;\n"
        + "function getVolTabRoadColor(vol, maxV) {\n"
        + "  if (vol === 0) return '#161c2e';\n"
        + "  const t = Math.min(Math.sqrt(vol) / Math.sqrt(maxV), 1);\n"
        + "  const theme = document.getElementById('vol-theme-select') ? document.getElementById('vol-theme-select').value : 'consistency';\n"
        + "  if (theme === 'consistency') {\n"
        + "    const r = Math.round(56 + t * (99 - 56));\n"
        + "    const g = Math.round(189 + t * (102 - 189));\n"
        + "    const b = Math.round(248 + t * (241 - 248));\n"
        + "    return `rgb(${r},${g},${b})`;\n"
        + "  } else {\n"
        + "    let r, g, b;\n"
        + "    if (t < 0.5) { const u = t * 2; r = Math.round(245 + u * (239 - 245)); g = Math.round(158 + u * (68 - 158)); b = Math.round(11 + u * (68 - 11)); }\n"
        + "    else { const u = (t - 0.5) * 2; r = Math.round(239 + u * (162 - 239)); g = Math.round(68 + u * (28 - 68)); b = Math.round(68 + u * (175 - 68)); }\n"
        + "    return `rgb(${r},${g},${b})`;\n"
        + "  }\n"
        + "}\n"
        + "function volDraw() {\n"
        + "  const period = document.getElementById('vol-period-select').value;\n"
        + "  const maxV = period === 'day' ? maxDayAvg : maxNightAvg;\n"
        + "  const isDay = period === 'day';\n"
        + "  volCtx.fillStyle = isDay ? getSkyColor(12) : getSkyColor(0);\n"
        + "  volCtx.fillRect(0, 0, volCanvas.width, volCanvas.height);\n"
        + "  let activeRoads = 0, peakVol = 0, totalPeds = 0;\n"
        + "  ROADS_GEOJSON.features.forEach(f => {\n"
        + "    if (!f.geometry) return; const pts = getPoints(f.geometry); if (pts.length < 2) return;\n"
        + "    const eid = f.properties && f.properties.edgeID;\n"
        + "    const vol = (eid != null && edgeAverages[eid]) ? edgeAverages[eid][period] : 0;\n"
        + "    if (vol > 0) { activeRoads++; totalPeds += vol; if (vol > peakVol) peakVol = vol; }\n"
        + "    volCtx.strokeStyle = getVolTabRoadColor(vol, maxV);\n"
        + "    volCtx.lineWidth = vol > 0 ? (2.5 + Math.min(Math.sqrt(vol) / Math.sqrt(maxV), 1) * 6) : 1.5;\n"
        + "    volCtx.lineCap = 'round'; volCtx.lineJoin = 'round'; volCtx.beginPath();\n"
        + "    let p0 = toVolScreen(pts[0][0], pts[0][1]); volCtx.moveTo(p0.x, p0.y);\n"
        + "    for (let i = 1; i < pts.length; i++) { let pi = toVolScreen(pts[i][0], pts[i][1]); volCtx.lineTo(pi.x, pi.y); }\n"
        + "    volCtx.stroke();\n"
        + "  });\n"
        + "  const titleTxt = period === 'day' ? '☀️ Day Flow Averages (07:00 – 19:00)' : '🌙 Night Flow Averages (19:00 – 07:00)';\n"
        + "  document.getElementById('m-vol-type').textContent = period === 'day' ? 'Day Flow' : 'Night Flow';\n"
        + "  document.getElementById('m-vol-peak').textContent = Math.round(peakVol) + ' / h';\n"
        + "  document.getElementById('m-vol-total').textContent = Math.round(totalPeds);\n"
        + "  volCtx.font = 'bold 13px Inter, sans-serif';\n"
        + "  volCtx.fillStyle = isDay ? 'rgba(20,20,40,0.85)' : 'rgba(200,210,240,0.9)';\n"
        + "  volCtx.fillText(`Torino   ${titleTxt}`, 20, volCanvas.height - 54);\n"
        + "}\n"
        + "let volZoomRafId = null; volCanvas.addEventListener('mousedown', e => { volDragging = true; volDX = e.clientX; volDY = e.clientY; });\n"
        + "window.addEventListener('mousemove', e => { if(!volDragging) return; volPanX += e.clientX-volDX; volPanY -= e.clientY-volDY; volDX=e.clientX; volDY=e.clientY; if (volZoomRafId) cancelAnimationFrame(volZoomRafId); volZoomRafId = requestAnimationFrame(() => { volZoomRafId=null; volDraw(); }); });\n"
        + "window.addEventListener('mouseup', () => { volDragging = false; });\n"
        + "volCanvas.addEventListener('wheel', e => {\n"
        + "  e.preventDefault(); const rect = volCanvas.getBoundingClientRect(), mx = e.clientX-rect.left, my = e.clientY-rect.top; const wx = (mx-volPanX)/volScale+minX, wy = (volCanvas.height-my-volPanY)/volScale+minY;\n"
        + "  const factor = e.deltaY < 0 ? 1.15 : 1/1.15; volScale *= factor; volPanX = mx-(wx-minX)*volScale; volPanY = (volCanvas.height-my)-(wy-minY)*volScale;\n"
        + "  if (volZoomRafId) cancelAnimationFrame(volZoomRafId); volZoomRafId = requestAnimationFrame(() => { volZoomRafId=null; volDraw(); });\n"
        + "}, { passive: false });\n"
        + "document.getElementById('vol-reset-btn').addEventListener('click', resetVolView);\n"
        + "const volPeriodSelect = document.getElementById('vol-period-select');\n"
        + "const volThemeSelect = document.getElementById('vol-theme-select');\n"
        + "function updateVolLegend() {\n"
        + "  const theme = volThemeSelect ? volThemeSelect.value : 'consistency';\n"
        + "  const bar = document.getElementById('vol-legend-bar');\n"
        + "  const legLight = document.getElementById('vol-leg-light');\n"
        + "  const legHeavy = document.getElementById('vol-leg-heavy');\n"
        + "  if (theme === 'consistency') {\n"
        + "    if (bar) bar.style.background = 'linear-gradient(to right, #161c2e, #38bdf8, #6366f1)';\n"
        + "    if (legLight) legLight.style.background = '#38bdf8';\n"
        + "    if (legHeavy) legHeavy.style.background = '#6366f1';\n"
        + "  } else {\n"
        + "    if (bar) bar.style.background = 'linear-gradient(to right, #161c2e, #f59e0b, #ef4444)';\n"
        + "    if (legLight) legLight.style.background = '#f59e0b';\n"
        + "    if (legHeavy) legHeavy.style.background = '#ef4444';\n"
        + "  }\n"
        + "}\n"
        + "if (volPeriodSelect) volPeriodSelect.addEventListener('change', volDraw);\n"
        + "if (volThemeSelect) {\n"
        + "  volThemeSelect.addEventListener('change', () => {\n"
        + "    updateVolLegend();\n"
        + "    volDraw();\n"
        + "  });\n"
        + "}\n"
        + "updateVolLegend();\n"
        + "canvas.width = canvas.clientWidth; canvas.height = canvas.clientHeight; resetView(); updateMetrics(0);\n"
        + "document.querySelector('[onclick=\"switchTab(\\'hourly\\')\"]').addEventListener('click', () => { setTimeout(() => { hvCanvas.width = hvCanvas.clientWidth; hvCanvas.height = hvCanvas.clientHeight; resetHvView(); }, 50); });\n"
        + "document.querySelector('[onclick=\"switchTab(\\'volumes\\')\"]').addEventListener('click', () => { setTimeout(() => { volCanvas.width = volCanvas.clientWidth; volCanvas.height = volCanvas.clientHeight; resetVolView(); }, 50); });\n"
        + "document.querySelector('[onclick=\"switchTab(\\'ab\\')\"]').addEventListener('click', () => { setTimeout(() => { abCanvas.width = abCanvas.clientWidth; abCanvas.height = abCanvas.clientHeight; abResetView(); if (AB_TRIPS && AB_TRIPS.length > 0) abUpdateMetrics(abCurrentFloatStep); }, 50); });\n"
        + "</script>\n"
        + "</body>\n"
        + "</html>\n";
  }

  private static boolean isABTestingEnabled() {
    try {
      Class<?> cls = Class.forName("pedsim.night.parameters.NightPars");
      java.lang.reflect.Field f = cls.getField("enableLightABTesting");
      return f.getBoolean(null);
    } catch (Exception e) {
      return false;
    }
  }
}
