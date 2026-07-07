package pedsim.core.website;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.logging.Logger;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.TripRouteRecorder;
import pedsim.core.parameters.Pars;
import pedsim.core.parameters.TimePars;
import pedsim.core.utilities.LoggerUtil;

/**
 * Generates a self-contained, single-file HTML dashboard that embeds:
 *
 * <ul>
 *   <li>The road network as a GeoJSON FeatureCollection with edgeID and cumulative volume.</li>
 *   <li>All agent trips with exact street coordinates traversed.</li>
 *   <li>Hourly pedestrian volumes per street.</li>
 *   <li>Simulation volume summaries.</li>
 *   <li>A/B testing playback when available.</li>
 * </ul>
 *
 * This version includes the visual/dashboard tweaks previously applied manually:
 * thinner edges, smaller animated agents, high-volume edges drawn on top,
 * heatmap defaults, no Sim Consistency selector, smoother animation, and fixed A/B lux metrics.
 */
public class HtmlExporter {

  private static final Logger logger = LoggerUtil.getLogger();
  // Results live with the project, under outputs/results/ — portable and gitignored
  // (was a hardcoded C:\Users\<user>\PedSimCity\Output\results path).
  private static final String OUTPUT_ROOT = "outputs" + File.separator + "results";

  /**
   * Exports a complete, self-contained HTML dashboard.
   */
  @SuppressWarnings({"rawtypes", "unchecked"})
  public static String export(int day, int job, List trips, Map volumesMap) {
    try {
      Files.createDirectories(Paths.get(OUTPUT_ROOT));

      String city = Pars.cityName;
      String timestamp =
          java.time.LocalDateTime.now()
              .format(java.time.format.DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss"));
      String filename = "results_" + city + "_day" + day + "_job" + job + "_" + timestamp + ".html";
      String outputPath = OUTPUT_ROOT + File.separator + filename;

      String roadsGeoJson = GeoJsonExporter.exportRoadsWithVolumes(PedSimCity.roads, volumesMap);
      String tripsJs = buildTripsJs(trips);

      if (isABTestingEnabled()) {
        saveAbTestTrips(tripsJs);
      }

      String abTripsJs = loadAbTestTrips();
      String hourlyVolJs = buildHourlyVolumesJs(trips);
      double[] centre = computeCentre();

      String html =
          renderHtml(city, day, job, roadsGeoJson, tripsJs, abTripsJs, hourlyVolJs, centre);

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

  /**
   * Converts trip records into a compact JavaScript array literal.
   * Trip shape: [agentId, startStep, endStep, [[x,y],...], vulnerable, [[x,y],...spooks]]
   */
  @SuppressWarnings("rawtypes")
  private static String buildTripsJs(List trips) {
    StringBuilder sb = new StringBuilder("[");
    boolean first = true;

    for (Object obj : trips) {
      TripRouteRecorder.TripRecord trip = (TripRouteRecorder.TripRecord) obj;
      if (!first) {
        sb.append(',');
      }
      first = false;

      sb.append('[')
          .append(trip.agentId)
          .append(',')
          .append(String.format(Locale.US, "%.2f", trip.startStep))
          .append(',')
          .append(String.format(Locale.US, "%.2f", trip.endStep))
          .append(',')
          .append('[');

      boolean firstCoord = true;
      for (org.locationtech.jts.geom.Coordinate c : trip.pathCoords) {
        if (!firstCoord) {
          sb.append(',');
        }
        firstCoord = false;
        sb.append(String.format(Locale.US, "[%.6f,%.6f]", c.x, c.y));
      }

      sb.append("],").append(trip.vulnerable ? "true" : "false").append(",[");

      boolean firstSpook = true;
      for (org.locationtech.jts.geom.Coordinate s : trip.spookLocations) {
        if (!firstSpook) {
          sb.append(',');
        }
        firstSpook = false;
        sb.append(String.format(Locale.US, "[%.6f,%.6f]", s.x, s.y));
      }

      sb.append("]]");
    }

    sb.append(']');
    return sb.toString();
  }

  /**
   * Builds a JS object: { edgeId: [vol_h0, vol_h1, ..., vol_h23], ... }.
   * Each trip is attributed to the hour-of-day bucket of its startStep.
   */
  @SuppressWarnings("rawtypes")
  private static String buildHourlyVolumesJs(List trips) {
    Map<Integer, int[]> hourlyMap = new HashMap<>();

    for (Object obj : trips) {
      TripRouteRecorder.TripRecord trip = (TripRouteRecorder.TripRecord) obj;

      long totalMinutes = (long) (trip.startStep * (TimePars.STEP_DURATION / 60));
      int hourOfDay = (int) ((totalMinutes / 60) % 24);

      for (int edgeId : trip.edgeIds) {
        hourlyMap.computeIfAbsent(edgeId, k -> new int[24])[hourOfDay]++;
      }
    }

    StringBuilder sb = new StringBuilder("{");
    boolean first = true;

    for (Map.Entry<Integer, int[]> entry : hourlyMap.entrySet()) {
      if (!first) {
        sb.append(',');
      }
      first = false;
      sb.append(entry.getKey()).append(": [");

      int[] counts = entry.getValue();
      for (int h = 0; h < 24; h++) {
        if (h > 0) {
          sb.append(',');
        }
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
      return new double[] {45.07, 7.68};
    }
    double cx = (PedSimCity.MBR.getMinX() + PedSimCity.MBR.getMaxX()) / 2.0;
    double cy = (PedSimCity.MBR.getMinY() + PedSimCity.MBR.getMaxY()) / 2.0;
    return new double[] {cy, cx};
  }

  private static String renderHtml(
      String city,
      int day,
      int job,
      String roadsGeoJson,
      String tripsJs,
      String abTripsJs,
      String hourlyVolJs,
      double[] centre) {

    String isNight = String.valueOf(Pars.isNight);
    String enableAB = String.valueOf(isABTestingEnabled());
    String runLabel = "Day " + day;
    String dashboardTitle =
        "Night Pedestrian Simulation: " + city + " — " + runLabel + " · Job " + job;
    String dashboardTitleHtml = escapeHtml(dashboardTitle).replace(" — ", " &nbsp;—&nbsp; ");

    return new StringBuilder(HTML_TEMPLATE_1).append(HTML_TEMPLATE_2).toString()
        .replace("__DASHBOARD_TITLE__", escapeHtml(dashboardTitle))
        .replace("__DASHBOARD_TITLE_HTML__", dashboardTitleHtml)
        .replace("__RUN_LABEL__", escapeJsTemplateLiteral(runLabel))
        .replace("__ROADS_GEOJSON__", roadsGeoJson)
        .replace("__TRIPS_JS__", tripsJs)
        .replace("__AB_TRIPS_JS__", abTripsJs)
        .replace("__HOURLY_VOL_JS__", hourlyVolJs)
        .replace("__IS_NIGHT__", isNight)
        .replace("__ENABLE_AB__", enableAB)
        .replace("__DAY_START__", String.valueOf(TimePars.DAY_START_HOUR))
        .replace("__NIGHT_START__", String.valueOf(TimePars.NIGHT_START_HOUR));
  }

  private static String escapeHtml(String value) {
    if (value == null) {
      return "";
    }
    return value
        .replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace("\"", "&quot;")
        .replace("'", "&#39;");
  }

  private static String escapeJsTemplateLiteral(String value) {
    if (value == null) {
      return "";
    }
    return value.replace("\\", "\\\\").replace("`", "\\`").replace("$", "\\$");
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

  private static final String HTML_TEMPLATE_1 =
"""
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>__DASHBOARD_TITLE__</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link href="https://fonts.googleapis.com/css2?family=Inter:wght@300;400;600;700&display=swap" rel="stylesheet">
<style>
  :root {
    --bg-light: #0f172a;
    --panel-bg: rgba(30, 41, 59, 0.85);
    --panel-border: rgba(255, 255, 255, 0.1);
    --text-main: #f8fafc;
    --text-muted: #94a3b8;
    --accent: #eab308;
    --accent-hover: #ca8a04;
    --danger: #ef4444;
    --success: #10b981;
  }
  *{box-sizing:border-box;margin:0;padding:0}
  body{font-family:'Inter',sans-serif;background:var(--bg-light);color:var(--text-main);display:flex;flex-direction:column;height:100vh;overflow:hidden}
  
  .global-nav-bar { width: 100%; background: rgba(15, 23, 42, 0.95); backdrop-filter: blur(10px); border-bottom: 1px solid var(--panel-border); padding: 1rem 2rem; display: flex; justify-content: center; gap: 2rem; flex-shrink: 0; box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.5); z-index: 20; position: relative; }
  .global-nav-item { color: var(--text-muted); font-weight: 600; font-size: 0.95rem; text-decoration: none; cursor: pointer; padding: 0.5rem 1rem; border-radius: 6px; transition: all 0.2s ease; }
  .global-nav-item:hover { color: var(--text-main); background: rgba(255, 255, 255, 0.1); }
  .global-nav-item.active { color: var(--accent); background: rgba(234, 179, 8, 0.15); }

  .global-tab-content { display: none; width: 100%; height: 100%; overflow-y: auto; padding-bottom: 2rem; }
  .global-tab-content.active { display: block; }
  #tab-dashboard.active { display: flex; flex-direction: column; overflow: hidden; } /* results need flex layout */

  .static-content-container { max-width: 900px; margin: 3rem auto; }
  .glass-panel { background: var(--panel-bg); backdrop-filter: blur(16px); border: 1px solid var(--panel-border); border-radius: 12px; padding: 2rem; box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.3); }
  .glass-panel h1 { font-size: 1.5rem; margin-bottom: 0.5rem; font-weight: 600; }
  .glass-panel .subtitle { color: var(--text-muted); font-size: 0.9rem; margin-bottom: 1.5rem; padding-bottom: 1rem; border-bottom: 1px solid var(--panel-border); }

  #header{padding:16px 32px;background:rgba(15,23,42,0.95);backdrop-filter:blur(10px);border-bottom:1px solid var(--panel-border);display:flex;align-items:center;justify-content:space-between;gap:16px;flex-shrink:0;box-shadow:0 4px 6px -1px rgba(0,0,0,0.5);z-index:10}
  #header h1{font-size:1.1rem;font-weight:700;color:var(--text-main);letter-spacing:0.02em}
  .metrics{display:flex;gap:12px}
  .card{background:var(--panel-bg);backdrop-filter:blur(16px);-webkit-backdrop-filter:blur(16px);border:1px solid var(--panel-border);border-radius:8px;padding:12px 18px;min-width:120px;box-shadow:0 4px 6px -1px rgba(0,0,0,0.3)}
  .card .label{font-size:.7rem;color:var(--text-muted);font-weight:600;text-transform:uppercase;letter-spacing:.05em}
  .card .value{font-size:1.4rem;font-weight:700;color:var(--text-main);margin-top:4px}
  .card .value.red{color:var(--danger)}
  .card .value.orange{color:#fb923c}
  .card .value.yellow{color:var(--accent)}
  
  #tab-bar{flex-shrink:0;display:flex;justify-content:center;gap:32px;background:rgba(15,23,42,0.95);backdrop-filter:blur(10px);border-bottom:1px solid var(--panel-border);padding:12px 32px}
  .tab-btn{padding:8px 16px;font-size:0.95rem;font-weight:600;color:var(--text-muted);background:none;border:none;border-radius:6px;cursor:pointer;transition:all .2s;}
  .tab-btn.active{color:var(--accent);background:rgba(234,179,8,0.15);}
  .tab-btn:hover:not(.active){color:var(--text-main);background:rgba(255,255,255,0.1);}
  
  .tab-panel{display:none;flex:1;position:relative;overflow:hidden}
  .tab-panel.active{display:flex;flex-direction:column}
  #container, #ab-container, #hv-container, #vol-container {position:relative;flex:1;width:100%;overflow:hidden;background:var(--bg-light)}
  #canvas, #ab-canvas, #hv-canvas, #vol-canvas {width:100%;height:100%;display:block;cursor:grab}
  #canvas:active, #ab-canvas:active, #hv-canvas:active, #vol-canvas:active {cursor:grabbing}
  #floating-controls{position:absolute;top:20px;right:20px;display:flex;flex-direction:column;gap:8px}
  .btn-float{background:rgba(30,41,59,0.85);border:1px solid #334155;border-radius:8px;color:#f1f5f9;padding:8px 14px;font-size:.75rem;font-weight:600;cursor:pointer;backdrop-filter:blur(4px);transition:all .2s;display:flex;align-items:center;gap:6px}
  .btn-float:hover{background:#6366f1;color:#f1f5f9;border-color:#6366f1}
  .toggle-label{display:flex;align-items:center;gap:10px;font-size:0.8rem;background:rgba(30,41,59,0.85);padding:8px 14px;border-radius:8px;border:1px solid #334155;backdrop-filter:blur(4px);cursor:pointer;color:#f1f5f9;font-weight:600;user-select:none;}
  .toggle-switch { position: relative; width: 34px; height: 18px; background: #0f172a; border-radius: 18px; border: 1px solid #334155; transition: 0.3s; flex-shrink: 0; }
  .toggle-switch::after { content: ''; position: absolute; top: 1px; left: 1px; width: 14px; height: 14px; background: #94a3b8; border-radius: 50%; transition: 0.3s; }
  input[type="checkbox"]:checked + .toggle-switch { background: #6366f1; border-color: #6366f1; }
  input[type="checkbox"]:checked + .toggle-switch::after { transform: translateX(16px); background: #fff; }
  input[type="checkbox"] { display: none; }
  #timeline-panel{flex-shrink:0;padding:14px 20px 16px;background:rgba(0,0,0,0.98);border-top:1px solid #1e293b}
  #timeline-panel .row{display:flex;align-items:center;gap:14px}
  #hv-timeline-panel{flex-shrink:0;padding:14px 20px 16px;background:rgba(0,0,0,0.98);border-top:1px solid #1e293b}
  #hv-timeline-panel .row{display:flex;align-items:center;gap:14px}
  #hv-time-label{font-size:1rem;font-weight:700;color:#f8c56d;min-width:90px;text-align:center}
  #hv-hour-slider{flex:1;-webkit-appearance:none;height:5px;border-radius:3px;background:#1e293b;outline:none;cursor:pointer}
  #hv-hour-slider::-webkit-slider-thumb{-webkit-appearance:none;width:16px;height:16px;border-radius:50%;background:#f8c56d;cursor:pointer;box-shadow:0 0 6px rgba(248,197,109,0.6)}
  .hv-btn{width:42px;height:42px;border-radius:50%;border:2px solid #f8c56d;background:transparent;color:#f8c56d;cursor:pointer;font-size:1.2rem;display:flex;align-items:center;justify-content:center;flex-shrink:0;transition:all .2s;padding-left:3px;}
  .hv-btn.playing{padding-left:0;}
  .hv-btn:hover{background:#f8c56d;color:#0a0c14}
  .hv-speed{background:rgba(30,41,59,0.85);color:#f1f5f9;border:1px solid #334155;border-radius:8px;padding:6px 12px;font-size:.75rem;font-weight:600;cursor:pointer;outline:none;transition:all .2s;}
  .hv-speed:hover{background:#f8c56d;color:#0a0c14;border-color:#f8c56d}
  #speed-select{background:rgba(30,41,59,0.85);color:#f1f5f9;border:1px solid #334155;border-radius:8px;padding:6px 12px;font-size:.75rem;font-weight:600;cursor:pointer;outline:none;transition:all .2s;display:inline-flex;align-items:center}
  #speed-select:hover{background:#6366f1;color:#f1f5f9;border-color:#6366f1}
  #play-btn{width:42px;height:42px;border-radius:50%;border:2px solid #6366f1;background:transparent;color:#6366f1;cursor:pointer;font-size:1.2rem;display:flex;align-items:center;justify-content:center;flex-shrink:0;transition:all .2s;padding-left:3px;}
  #play-btn.playing{padding-left:0;}
  #play-btn:hover{background:#6366f1;color:#f1f5f9}
  #step-slider{flex:1;-webkit-appearance:none;height:5px;border-radius:3px;background:#1e293b;outline:none;cursor:pointer}
  #step-slider::-webkit-slider-thumb{-webkit-appearance:none;width:16px;height:16px;border-radius:50%;background:#6366f1;cursor:pointer;box-shadow:0 0 6px rgba(99,102,241,0.5)}
  #time-label{font-size:.8rem;color:#a5b4fc;min-width:48px;text-align:right;font-weight:600}
  .legend{display:flex;gap:16px;margin-top:10px;font-size:.7rem;color:#94a3b8}
  .leg-item{display:flex;align-items:center;gap:6px}
  .leg-dot{width:8px;height:8px;border-radius:50%}
  .leg-line{width:20px;height:3px;border-radius:2px}
  .hv-legend{position:absolute;bottom:16px;left:20px;display:flex;gap:6px;align-items:center;font-size:0.72rem;color:#94a3b8;background:rgba(0,0,0,0.7);padding:6px 12px;border-radius:8px;border:1px solid #1e293b;backdrop-filter:blur(4px);}
  .hv-legend-bar{width:80px;height:8px;border-radius:4px;background:linear-gradient(to right,#161c2e,#38bdf8,#00ffff,#ffffff)}
  .hv-floating{position:absolute;top:20px;right:20px;}
  #hv-hour-grid{display:flex;gap:4px;overflow-x:auto;padding-bottom:4px;scrollbar-width:thin}
  .hour-btn{background:rgba(30,41,59,0.7);border:1px solid #1e293b;border-radius:4px;color:#94a3b8;padding:4px 8px;font-size:0.7rem;font-weight:600;cursor:pointer;transition:all 0.2s;flex-shrink:0}
  .hour-btn:hover{background:#334155;color:#f1f5f9}
  .hour-btn.active{background:#6366f1 !important;color:#ffffff !important;border-color:#6366f1 !important}
</style>
<script>
  function switchGlobalTab(tabId, element) {
    document.querySelectorAll('.global-tab-content').forEach(el => el.classList.remove('active'));
    document.getElementById(tabId).classList.add('active');
    document.querySelectorAll('.global-nav-item').forEach(el => el.classList.remove('active'));
    element.classList.add('active');
  }
</script>
</head>
<body>

  <!-- Global Navigation -->
  <nav class="global-nav-bar">
    <a class="global-nav-item active" onclick="switchGlobalTab('tab-dashboard', this)">Dashboard</a>
    <a class="global-nav-item" onclick="switchGlobalTab('tab-summary', this)">Summary</a>
    <a class="global-nav-item" onclick="switchGlobalTab('tab-pubs', this)">Publications & Uses</a>
    <a class="global-nav-item" href="https://github.com/g-filomena/PedSimCity" target="_blank">GitHub Repo</a>
  </nav>

  <!-- Summary Tab -->
  <div id="tab-summary" class="global-tab-content">
    <div class="static-content-container">
      <div class="glass-panel">
        <h1>Model Summary</h1>
        <p class="subtitle">How the PedSimCity Engine Works</p>
        <div style="line-height: 1.6; color: var(--text-muted); font-size: 0.95rem;">
          <p style="margin-bottom: 1rem;">PedSimCity is an advanced pedestrian simulation engine designed specifically to model how individuals navigate urban environments at night.</p>
          <p style="margin-bottom: 1rem;">By fusing detailed street lighting data, active commercial frontages, and localized demographics, the model predicts realistic night-time routing choices. It helps researchers understand pedestrian flows after dark and identify areas of high vulnerability based on illumination levels and street activity.</p>
        </div>
      </div>
    </div>
  </div>

  <!-- Publications Tab -->
  <div id="tab-pubs" class="global-tab-content">
    <div class="static-content-container">
      <div class="glass-panel">
        <h1>Publications & Uses</h1>
        <p class="subtitle">Research, Case Studies, and Papers</p>
        <div style="line-height: 1.6; color: var(--text-muted); font-size: 0.95rem;">
          <ul style="margin-left: 1.5rem; display: flex; flex-direction: column; gap: 1rem;">
            <li></li>
            <li></li>
            <li></li>
          </ul>
        </div>
      </div>
    </div>
  </div>

  <!-- Results Dashboard Tab -->
  <div id="tab-dashboard" class="global-tab-content active">
<div id="header">
  <h1>__DASHBOARD_TITLE_HTML__</h1>
  <div class="metrics" id="metrics-trips">
    <div class="card"><div class="label">Active Agents</div><div class="value" id="m-active">-</div></div>
    <div class="card"><div class="label">% Vulnerable</div><div class="value red" id="m-vuln">-</div></div>
    <div class="card"><div class="label">Avg Vuln Trip</div><div class="value orange" id="m-vuln-d">-</div></div>
    <div class="card"><div class="label">Avg Normal Trip</div><div class="value" id="m-norm-d">-</div></div>
    <div class="card"><div class="label">Avg Vuln Lux</div><div class="value yellow" id="m-lux-vuln">-</div></div>
    <div class="card"><div class="label">Avg Normal Lux</div><div class="value yellow" id="m-lux-norm">-</div></div>
    <div class="card"><div class="label">Sim Time</div><div class="value" id="m-time">-</div></div>
  </div>
  <div class="metrics" id="metrics-hourly" style="display:none">
    <div class="card"><div class="label">Hour</div><div class="value yellow" id="m-hv-hour">00:00</div></div>
    <div class="card"><div class="label">Active Roads</div><div class="value" id="m-hv-roads">-</div></div>
    <div class="card"><div class="label">Peak Vol</div><div class="value red" id="m-hv-peak">-</div></div>
    <div class="card"><div class="label">Total Peds</div><div class="value" id="m-hv-total">-</div></div>
    <div class="card"><div class="label">Avg Vuln Lux</div><div class="value yellow" id="m-hv-lux-vuln">-</div></div>
    <div class="card"><div class="label">Avg Normal Lux</div><div class="value yellow" id="m-hv-lux-norm">-</div></div>
    <div class="card"><div class="label">Sky</div><div class="value" id="m-hv-sky">🌙 Night</div></div>
  </div>
  <div class="metrics" id="metrics-volumes" style="display:none">
    <div class="card"><div class="label">Flow Type</div><div class="value yellow" id="m-vol-type">Day Flow</div></div>
    <div class="card"><div class="label">Peak Period Vol</div><div class="value red" id="m-vol-peak">-</div></div>
    <div class="card"><div class="label">Total Traversed</div><div class="value" id="m-vol-total">-</div></div>
    <div class="card"><div class="label">Avg Vuln Lux</div><div class="value yellow" id="m-vol-lux-vuln">-</div></div>
    <div class="card"><div class="label">Avg Normal Lux</div><div class="value yellow" id="m-vol-lux-norm">-</div></div>
  </div>
</div>
<div id="tab-bar">
  <button class="tab-btn active" onclick="switchTab('trips')">Agent Trips</button>
  <button class="tab-btn" onclick="switchTab('hourly')">Hourly Volumes</button>
  <button class="tab-btn" onclick="switchTab('volumes')">Simulation Volumes</button>
  <button class="tab-btn" onclick="switchTab('ab')">A/B Testing</button>
</div>
<div class="tab-panel active" id="panel-trips">
  <div id="container">
    <canvas id="canvas"></canvas>
    <div id="floating-controls">
      <label class="toggle-label" id="lbl-light"><input type="checkbox" id="tg-light" /><div class="toggle-switch"></div> Light Level Map</label>
      <label class="toggle-label" id="lbl-tethers"><input type="checkbox" id="tg-tethers" checked /><div class="toggle-switch"></div> Show A/B Tethers</label>
      <button class="btn-float" id="reset-btn">Reset Zoom</button>
    </div>
  </div>
  <div id="timeline-panel">
    <div class="row">
      <button id="play-btn" title="Play/Pause">▶</button>
      <input type="range" id="step-slider" min="0" value="0"/>
      <span id="time-label">__RUN_LABEL__ 00:00</span>
      <select id="speed-select" title="Playback Speed">
        <option value="0.1">0.1x Speed</option>
        <option value="0.5">0.5x Speed</option>
        <option value="1">1x Speed</option>
        <option value="2" selected>2x Speed</option>
        <option value="5">5x Speed</option>
        <option value="10">10x Speed</option>
      </select>
    </div>
    <div class="legend">
      <div class="leg-item"><div class="leg-dot" style="background:#ef4444"></div> Vulnerable agent</div>
      <div class="leg-item"><div class="leg-dot" style="background:#38bdf8"></div> Normal agent</div>
    </div>
  </div>
</div>
<div class="tab-panel" id="panel-hourly">
  <div id="hv-container">
    <canvas id="hv-canvas"></canvas>
    <div class="hv-legend">
      <span>Low</span><div class="hv-legend-bar"></div><span>High</span>
      &nbsp;&nbsp; Pedestrian volume/hour
    </div>
    <div class="hv-floating">
      <button class="btn-float" id="hv-reset-btn">Reset Zoom</button>
    </div>
  </div>
  <div id="hv-timeline-panel">
    <div class="row">
      <button class="hv-btn" id="hv-play-btn" title="Play/Pause">▶</button>
      <div style="flex:1;display:flex;flex-direction:column;gap:8px">
        <div id="hv-hour-grid"></div>
        <input type="range" id="hv-hour-slider" min="0" max="23" value="0"/>
      </div>
      <span id="hv-time-label">00:00 – 01:00</span>
      <select class="hv-speed" id="hv-speed-select">
        <option value="0.5">0.5x</option>
        <option value="1" selected>1x</option>
        <option value="2">2x</option>
        <option value="4">4x</option>
      </select>
      <select class="hv-speed" id="hv-theme-select" title="Color Theme">
        <option value="heatmap" selected>Classic Heatmap</option>
      </select>
    </div>
    <div class="legend" style="margin-top:8px">
      <div class="leg-item"><div style="width:20px;height:3px;border-radius:2px;background:#161c2e"></div> No traffic</div>
      <div class="leg-item"><div id="hv-leg-light" style="width:20px;height:3px;border-radius:2px;background:#38bdf8"></div> Light traffic</div>
      <div class="leg-item"><div id="hv-leg-heavy" style="width:20px;height:3px;border-radius:2px;background:#6366f1"></div> Heavy traffic</div>
      <div class="leg-item" style="margin-left:auto;font-style:italic;color:#f8c56d">Sunrise 06:51 · Sunset 18:30</div>
    </div>
  </div>
</div>
<div class="tab-panel" id="panel-volumes">
  <div id="vol-container">
    <canvas id="vol-canvas"></canvas>
    <div class="hv-legend">
      <span>Low</span><div class="hv-legend-bar" id="vol-legend-bar"></div><span>High</span>
      &nbsp;&nbsp; Average flow/hour
    </div>
    <div class="hv-floating">
      <button class="btn-float" id="vol-reset-btn">Reset Zoom</button>
    </div>
  </div>
  <div id="vol-timeline-panel" style="flex-shrink:0;padding:14px 20px 16px;background:rgba(0,0,0,0.98);border-top:1px solid #1e293b">
    <div class="row" style="display:flex;align-items:center;gap:14px">
      <span style="font-size:0.85rem;font-weight:600;color:#818cf8">Flow Period:</span>
      <select class="hv-speed" id="vol-period-select" style="background:rgba(30,41,59,0.85);color:#f1f5f9;border:1px solid #334155;border-radius:8px;padding:6px 12px;font-size:.75rem;font-weight:600;cursor:pointer;outline:none;transition:all .2s;display:inline-flex;align-items:center">
        <option value="day" selected>☀️ Day Flow Average (__DAY_START__:00 – __NIGHT_START__:00)</option>
        <option value="night">🌙 Night Flow Average (__NIGHT_START__:00 – __DAY_START__:00)</option>
      </select>
      <select class="hv-speed" id="vol-theme-select" title="Color Theme" style="background:rgba(30,41,59,0.85);color:#f1f5f9;border:1px solid #334155;border-radius:8px;padding:6px 12px;font-size:.75rem;font-weight:600;cursor:pointer;outline:none;transition:all .2s;display:inline-flex;align-items:center">
        <option value="heatmap" selected>Magma Heatmap</option>
      </select>
      <label class="toggle-label" style="margin-left:auto"><input type="checkbox" id="vol-tg-top10" /><div class="toggle-switch"></div> Top-10 Edges</label>
    </div>
    <div class="legend" style="margin-top:8px">
      <div class="leg-item"><div style="width:20px;height:3px;border-radius:2px;background:#161c2e"></div> No traffic</div>
      <div class="leg-item"><div id="vol-leg-light" style="width:20px;height:3px;border-radius:2px;background:#d13c7b"></div> Light traffic</div>
      <div class="leg-item"><div id="vol-leg-heavy" style="width:20px;height:3px;border-radius:2px;background:#fcf340"></div> Heavy traffic</div>
      <div class="leg-item" style="margin-left:auto;font-style:italic;color:#f8c56d">Sunrise 06:51 · Sunset 18:30</div>
    </div>
  </div>
</div>
<div class="tab-panel" id="panel-ab">
  <div id="ab-container">
    <canvas id="ab-canvas"></canvas>
    <div id="ab-empty-overlay" style="position:absolute;top:0;left:0;width:100%;height:100%;background:rgba(15,23,42,0.95);display:flex;flex-direction:column;align-items:center;justify-content:center;color:#94a3b8;font-size:1rem;font-weight:600;padding:20px;text-align:center;z-index:10">
      <p style="margin-bottom:12px;color:#f1f5f9;font-size:1.2rem">No A/B Test Data Found</p>
      <p style="max-width:450px;font-size:0.85rem;line-height:1.6">Please run the simulation with A/B testing enabled (enableLightABTesting = true in parameters) once. The simulation will save the 72-pair hourly release test data, which will then sit here permanently.</p>
    </div>
    <div id="ab-floating-controls" style="position:absolute;top:20px;right:20px;display:none;flex-direction:column;gap:8px">
      <label class="toggle-label" id="ab-lbl-light"><input type="checkbox" id="ab-tg-light" /><div class="toggle-switch"></div> Light Level Map</label>
      <label class="toggle-label"><input type="checkbox" id="ab-tg-tethers" checked /><div class="toggle-switch"></div> Show A/B Tethers</label>
      <button class="btn-float" id="ab-reset-btn">Reset Zoom</button>
    </div>
  </div>
  <div id="ab-timeline-panel" style="display:none;flex-shrink:0;padding:14px 20px 16px;background:rgba(0,0,0,0.98);border-top:1px solid #1e293b">
    <div class="row" style="display:flex;align-items:center;gap:14px">
      <button id="ab-play-btn" style="width:42px;height:42px;border-radius:50%;border:2px solid #6366f1;background:transparent;color:#6366f1;cursor:pointer;font-size:1.2rem;display:flex;align-items:center;justify-content:center;flex-shrink:0;transition:all .2s;padding-left:3px;" title="Play/Pause">▶</button>
      <input type="range" id="ab-step-slider" style="flex:1;-webkit-appearance:none;height:5px;border-radius:3px;background:#1e293b;outline:none;cursor:pointer" min="0" value="0"/>
      <span id="ab-time-label" style="font-size:.8rem;color:#a5b4fc;min-width:48px;text-align:right;font-weight:600">__RUN_LABEL__ 00:00</span>
      <select id="ab-speed-select" style="background:rgba(30,41,59,0.85);color:#f1f5f9;border:1px solid #334155;border-radius:8px;padding:6px 12px;font-size:.75rem;font-weight:600;cursor:pointer;outline:none;transition:all .2s;display:inline-flex;align-items:center" title="Playback Speed">
        <option value="0.1">0.1x Speed</option>
        <option value="0.5">0.5x Speed</option>
        <option value="1">1x Speed</option>
        <option value="2" selected>2x Speed</option>
        <option value="5">5x Speed</option>
        <option value="10">10x Speed</option>
      </select>
    </div>
    <div class="legend" style="display:flex;gap:16px;margin-top:10px;font-size:.7rem;color:#94a3b8">
      <div class="leg-item"><div class="leg-dot" style="width:8px;height:8px;border-radius:50%;background:#ef4444"></div> Vulnerable agent</div>
      <div class="leg-item"><div class="leg-dot" style="width:8px;height:8px;border-radius:50%;background:#38bdf8"></div> Normal agent</div>
      <div class="leg-item"><div style="width:20px;height:3px;border-radius:2px;border-top:1.5px dashed rgba(255,255,255,0.6)"></div> Tether</div>
    </div>
  </div>
</div>
""";

  private static final String HTML_TEMPLATE_2 =
"""
<script>
const ROADS_GEOJSON = __ROADS_GEOJSON__;
const TRIPS = __TRIPS_JS__;
const AB_TRIPS = __AB_TRIPS_JS__;
const HOURLY_VOL = __HOURLY_VOL_JS__;
const isNight = __IS_NIGHT__;
const enableAB = __ENABLE_AB__;
if (!isNight) {
  const lbl = document.getElementById('lbl-light'); if (lbl) lbl.style.display = 'none';
  const abLbl = document.getElementById('ab-lbl-light'); if (abLbl) abLbl.style.display = 'none';
}
if (!enableAB) {
  const lbl = document.getElementById('lbl-tethers'); if (lbl) lbl.style.display = 'none';
  const tg = document.getElementById('tg-tethers'); if (tg) tg.checked = false;
}
if (AB_TRIPS && AB_TRIPS.length > 0) {
  document.getElementById('ab-empty-overlay').style.display = 'none';
  document.getElementById('ab-floating-controls').style.display = 'flex';
  document.getElementById('ab-timeline-panel').style.display = 'block';
}
function switchTab(name) {
  if (playing) { playing = false; playBtn.textContent = '▶'; playBtn.classList.remove('playing'); cancelAnimationFrame(animId); }
  if (hvPlaying) { hvPlaying = false; hvPlayBtn.textContent = '▶'; hvPlayBtn.classList.remove('playing'); cancelAnimationFrame(hvAnimId); }
  if (abPlaying) { abPlaying = false; abPlayBtn.textContent = '▶'; abPlayBtn.classList.remove('playing'); cancelAnimationFrame(abAnimId); }
  document.querySelectorAll('.tab-panel').forEach(p => p.classList.remove('active'));
  document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
  document.getElementById('panel-' + name).classList.add('active');
  const btns = document.querySelectorAll('.tab-btn');
  if (name === 'trips') btns[0].classList.add('active');
  else if (name === 'hourly') btns[1].classList.add('active');
  else if (name === 'volumes') btns[2].classList.add('active');
  else if (name === 'ab') btns[3].classList.add('active');
  document.getElementById('metrics-trips').style.display  = (name === 'trips' || name === 'ab') ? '' : 'none';
  document.getElementById('metrics-hourly').style.display = name === 'hourly' ? '' : 'none';
  document.getElementById('metrics-volumes').style.display = name === 'volumes' ? '' : 'none';
  if (name === 'trips') { canvas.width = canvas.clientWidth; canvas.height = canvas.clientHeight; roadsDirty = true; draw(); }
  if (name === 'hourly') {
    setTimeout(() => { hvCanvas.width = hvCanvas.clientWidth; hvCanvas.height = hvCanvas.clientHeight; resetHvView(); }, 50);
  }
  if (name === 'volumes') {
    setTimeout(() => { volCanvas.width = volCanvas.clientWidth; volCanvas.height = volCanvas.clientHeight; resetVolView(); }, 50);
  }
  if (name === 'ab') {
    setTimeout(() => { abCanvas.width = abCanvas.clientWidth; abCanvas.height = abCanvas.clientHeight; abResetView(); if (AB_TRIPS && AB_TRIPS.length > 0) abUpdateMetrics(abCurrentFloatStep); }, 50);
  }
}
let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
function getPoints(geom) {
  let pts = [];
  if (geom.type === 'LineString') { pts = geom.coordinates; }
  else if (geom.type === 'MultiLineString') { geom.coordinates.forEach(line => { pts = pts.concat(line); }); }
  return pts;
}
ROADS_GEOJSON.features.forEach(f => {
  if (f.geometry) {
    getPoints(f.geometry).forEach(pt => {
      if (pt[0] < minX) minX = pt[0]; if (pt[0] > maxX) maxX = pt[0];
      if (pt[1] < minY) minY = pt[1]; if (pt[1] > maxY) maxY = pt[1];
    });
  }
});
if (minX === Infinity) { minX = 0; maxX = 1000; minY = 0; maxY = 1000; }
const edgeCoords = {};
ROADS_GEOJSON.features.forEach(f => {
  if (!f.geometry || !f.properties || f.properties.edgeID == null) return;
  edgeCoords[f.properties.edgeID] = getPoints(f.geometry);
});
const maxHourlyVol = (()=>{ let mx=0; Object.values(HOURLY_VOL).forEach(arr=>{ arr.forEach(v=>{ if(v>mx) mx=v; }); }); return mx||1; })();
function getSkyColor(hour) {
  return '#000000';
}
const maxVol = (()=>{ let mx=0; ROADS_GEOJSON.features.forEach(f=>{ const v=f.properties.volume||0; if(v>mx) mx=v; }); return mx||1; })();
function getVolColor(f, useLight) {
  if (useLight) {
    const lux = parseFloat(f.properties.mean_lux) || 0;
    if (lux < 5) return '#1e1b4b';
    const t = Math.min(lux / 30, 1);
    const r = Math.round(30 + t * (253 - 30));
    const g = Math.round(27 + t * (224 - 27));
    const b = Math.round(75 + t * (71 - 75));
    return `rgb(${r},${g},${b})`;
  }
  const v = f.properties.volume || 0;
  if (v < 1) return '#1e2a3a';  // dim but visible base road colour
  const t = Math.min(Math.pow(v / maxVol, 0.5), 1.0);
  let r, g, b;
  if (t < 0.33) {
    const u = t / 0.33;
    r = Math.round(27 + u * (209 - 27));
    g = Math.round(12 + u * (60 - 12));
    b = Math.round(60 + u * (123 - 60));
  } else if (t < 0.67) {
    const u = (t - 0.33) / 0.34;
    r = Math.round(209 + u * (247 - 209));
    g = Math.round(60 + u * (122 - 60));
    b = Math.round(123 + u * (28 - 123));
  } else {
    const u = (t - 0.67) / 0.33;
    r = Math.round(247 + u * (252 - 247));
    g = Math.round(122 + u * (243 - 122));
    b = Math.round(28 + u * (64 - 28));
  }
  return `rgb(${r},${g},${b})`;
}
function getVolWeight(f) {
  const v = f.properties.volume || 0;
  return v < 1 ? 0.8 : Math.max(0.8, Math.min(3.0, 0.8 + (v / maxVol) * 2.2));
}
function getPointAlongPath(coords, segs, totalLength, progress) {
  if (!coords || coords.length === 0) return null;
  if (progress <= 0 || totalLength === 0) return { x: coords[0][0], y: coords[0][1] };
  if (progress >= 1) return { x: coords[coords.length-1][0], y: coords[coords.length-1][1] };
  const targetDist = progress * totalLength; let acc = 0;
  for (let i = 0; i < segs.length; i++) {
    const l = segs[i];
    if (acc + l >= targetDist) { const f = (targetDist - acc) / (l || 1); return { x: coords[i][0]+(coords[i+1][0]-coords[i][0])*f, y: coords[i][1]+(coords[i+1][1]-coords[i][1])*f }; }
    acc += l;
  }
  return { x: coords[coords.length-1][0], y: coords[coords.length-1][1] };
}
let luxGrid = {}, GRID_SZ = 15;
ROADS_GEOJSON.features.forEach(f => {
  if (!f.geometry) return; const lx = parseFloat(f.properties.mean_lux) || 0; if (lx === 0) return;
  getPoints(f.geometry).forEach(pt => {
    const gx = Math.floor(pt[0]/GRID_SZ), gy = Math.floor(pt[1]/GRID_SZ), k = gx+','+gy;
    if(!luxGrid[k] || lx > luxGrid[k]) luxGrid[k] = lx;
  });
});
let maxStep = 0;
const tripLengthById = {};
TRIPS.forEach(t => {
  if (t[2] > maxStep) maxStep = t[2];
  const coords = t[3]; const segs = []; let tot = 0;
  for (let i = 0; i < coords.length - 1; i++) {
    const dx = coords[i+1][0]-coords[i][0], dy = coords[i+1][1]-coords[i][1];
    const l = Math.sqrt(dx*dx+dy*dy); segs.push(l); tot += l;
  }
  t[6] = segs; t[7] = tot;
  tripLengthById[t[0]] = tot;
});
TRIPS.sort((a, b) => a[1] - b[1]);
const canvas = document.getElementById('canvas'); const ctx = canvas.getContext('2d'); ctx.imageSmoothingEnabled = true;
let currentFloatStep = 0, scale = 1.0, panX = 0, panY = 0;
function toScreen(wx, wy) { return { x: (wx - minX) * scale + panX, y: canvas.height - ((wy - minY) * scale + panY) }; }
function resetView() {
  const dx = maxX - minX, dy = maxY - minY, pad = 40;
  scale = Math.min((canvas.width - pad*2) / (dx || 1), (canvas.height - pad*2) / (dy || 1));
  panX = canvas.width / 2 - (minX + dx / 2 - minX) * scale; panY = canvas.height / 2 - (minY + dy / 2 - minY) * scale;
  roadsDirty = true; draw();
}
let activeList = [], nextTripIdx = 0;
function updateActiveTrips(floatStep) {
  while(nextTripIdx < TRIPS.length && TRIPS[nextTripIdx][1] <= floatStep) {
    activeList.push(TRIPS[nextTripIdx]); nextTripIdx++;
  }
  for (let i = activeList.length - 1; i >= 0; i--) {
    if (activeList[i][2] < floatStep) activeList.splice(i, 1);
  }
}
function getLiveAgents(floatStep) {
  const res = [];
  for (let i = 0; i < activeList.length; i++) {
    const t = activeList[i];
    const duration = t[2] - t[1]; const progress = duration > 0 ? (floatStep - t[1]) / duration : 1.0;
    const pt = getPointAlongPath(t[3], t[6], t[7], progress);
    if (pt) res.push({ id: t[0], vuln: t[4], x: pt.x, y: pt.y, progress });
  }
  return res;
}
const offscreenVol = document.createElement('canvas'); const offCtxVol = offscreenVol.getContext('2d');
const offscreenLight = document.createElement('canvas'); const offCtxLight = offscreenLight.getContext('2d');
let roadsDirty = true;
const tgLight = document.getElementById('tg-light');
const tgTethers = document.getElementById('tg-tethers');
tgLight.addEventListener('change', () => { draw(); });
tgTethers.addEventListener('change', () => { draw(); });
function buildRoadLayers() {
  if (offscreenVol.width !== canvas.width || offscreenVol.height !== canvas.height) {
    offscreenVol.width = canvas.width; offscreenVol.height = canvas.height;
    offscreenLight.width = canvas.width; offscreenLight.height = canvas.height;
  }
  offCtxVol.clearRect(0, 0, canvas.width, canvas.height); offCtxLight.clearRect(0, 0, canvas.width, canvas.height);
  const sortedFeatures = [...ROADS_GEOJSON.features].sort((a, b) => (a.properties.volume || 0) - (b.properties.volume || 0));
  sortedFeatures.forEach(f => {
    if (!f.geometry) return; const pts = getPoints(f.geometry); if (pts.length < 2) return;
    offCtxVol.strokeStyle = getVolColor(f, false); offCtxVol.lineWidth = getVolWeight(f);
    offCtxVol.lineCap = 'round'; offCtxVol.lineJoin = 'round'; offCtxVol.beginPath();
    let p0 = toScreen(pts[0][0], pts[0][1]); offCtxVol.moveTo(p0.x, p0.y);
    for (let i = 1; i < pts.length; i++) { const pi = toScreen(pts[i][0], pts[i][1]); offCtxVol.lineTo(pi.x, pi.y); }
    offCtxVol.stroke();
    offCtxLight.strokeStyle = getVolColor(f, true); offCtxLight.lineWidth = getVolWeight(f);
    offCtxLight.lineCap = 'round'; offCtxLight.lineJoin = 'round'; offCtxLight.beginPath();
    p0 = toScreen(pts[0][0], pts[0][1]); offCtxLight.moveTo(p0.x, p0.y);
    for (let i = 1; i < pts.length; i++) { const pi = toScreen(pts[i][0], pts[i][1]); offCtxLight.lineTo(pi.x, pi.y); }
    offCtxLight.stroke();
  });
  roadsDirty = false;
}
function draw(agents) {
  if (!agents) agents = getLiveAgents(currentFloatStep);
  if (roadsDirty) buildRoadLayers();
  const simHour = (currentFloatStep * 20 / 60) % 24;
  ctx.fillStyle = getSkyColor(simHour);
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  ctx.drawImage(tgLight.checked ? offscreenLight : offscreenVol, 0, 0);
  const posById = {};
  agents.forEach(a => { posById[a.id] = toScreen(a.x, a.y); });
  if (tgTethers.checked) {
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.35)'; ctx.lineWidth = 1.0; ctx.setLineDash([4, 4]);
    ctx.beginPath();
    agents.forEach(a => {
      if (a.vuln && posById[a.id + 1]) {
        const p1 = posById[a.id], p2 = posById[a.id + 1];
        ctx.moveTo(p1.x, p1.y); ctx.lineTo(p2.x, p2.y);
      }
    });
    ctx.stroke(); ctx.setLineDash([]);
  }
  agents.forEach(a => {
    const p = posById[a.id];
    ctx.beginPath();
    ctx.arc(p.x, p.y, a.vuln ? 4 : 3, 0, Math.PI * 2);
    ctx.fillStyle = a.vuln ? '#ef4444' : '#38bdf8';
    ctx.fill();
  });
}
let isDragging = false, startX = 0, startY = 0, zoomRafId = null;
canvas.addEventListener('mousedown', e => { isDragging = true; startX = e.clientX; startY = e.clientY; });
window.addEventListener('mousemove', e => { if (!isDragging) return; panX += e.clientX - startX; panY -= e.clientY - startY; startX = e.clientX; startY = e.clientY; roadsDirty = true; if (zoomRafId) cancelAnimationFrame(zoomRafId); zoomRafId = requestAnimationFrame(() => { zoomRafId = null; draw(); }); });
window.addEventListener('mouseup', () => { isDragging = false; });
canvas.addEventListener('wheel', e => {
  e.preventDefault(); const rect = canvas.getBoundingClientRect(), mouseX = e.clientX - rect.left, mouseY = e.clientY - rect.top;
  const worldX = (mouseX - panX) / scale + minX, worldY = (canvas.height - mouseY - panY) / scale + minY;
  const factor = e.deltaY < 0 ? 1.12 : 1/1.12;
  scale *= factor;
  panX = mouseX - (worldX - minX) * scale; panY = (canvas.height - mouseY) - (worldY - minY) * scale;
  roadsDirty = true; if (zoomRafId) cancelAnimationFrame(zoomRafId); zoomRafId = requestAnimationFrame(() => { zoomRafId = null; draw(); });
}, { passive: false });
document.getElementById('reset-btn').addEventListener('click', resetView);
function updateGlobalLuxMetrics(step, tripsList, prefix, periodFilter) {
  let vulnSum = 0, vulnQty = 0;
  let normSum = 0, normQty = 0;
  tripsList.forEach(t => {
    if (t[2] <= step) {
      if (periodFilter) {
        const simHour = (t[1] * 20 / 60) % 24;
        const isDayTrip = simHour >= __DAY_START__ && simHour < __NIGHT_START__;
        if (periodFilter === 'day' && !isDayTrip) return;
        if (periodFilter === 'night' && isDayTrip) return;
      }
      if (t[8] === undefined) {
        let tSum = 0; const coords = t[3];
        if (coords && coords.length > 0) {
          coords.forEach(c => { const gx = Math.floor(c[0]/GRID_SZ), gy = Math.floor(c[1]/GRID_SZ); tSum += parseFloat(luxGrid[gx+','+gy] || 0); });
          t[8] = tSum / coords.length;
        } else { t[8] = 0; }
      }
      if (t[4]) {
        vulnSum += t[8];
        vulnQty++;
      } else {
        normSum += t[8];
        normQty++;
      }
    }
  });
  const avgVuln = vulnQty > 0 ? (vulnSum / vulnQty).toFixed(2) : '-';
  const avgNorm = normQty > 0 ? (normSum / normQty).toFixed(2) : '-';
  const elVuln = document.getElementById(prefix + 'lux-vuln');
  const elNorm = document.getElementById(prefix + 'lux-norm');
  if (elVuln) elVuln.textContent = avgVuln;
  if (elNorm) elNorm.textContent = avgNorm;
}
function updateMetrics(step, agents) {
  if (!agents) agents = getLiveAgents(step);
  const total = agents.length, vuln = agents.filter(a=>a.vuln).length;
  document.getElementById('m-active').textContent = total;
  document.getElementById('m-vuln').textContent = total > 0 ? (vuln/total*100).toFixed(1)+'%' : '0%';
  const simHour = (step * 20 / 60) % 24;
  const hh = String(Math.floor(simHour)).padStart(2,'0');
  const mm = String(Math.floor(step*20)%60).padStart(2,'0');
  document.getElementById('m-time').textContent = `__RUN_LABEL__ ${hh}:${mm}`;
  document.getElementById('time-label').textContent = document.getElementById('m-time').textContent;
  const shouldLight = simHour >= 18.5 || simHour < 6.85;
  if (tgLight.checked !== shouldLight) {
    tgLight.checked = shouldLight;
    draw(agents);
  }
  const vulnActive = agents.filter(a => a.vuln);
  const normActive = agents.filter(a => !a.vuln);
  const avgVulnActiveDist = vulnActive.length > 0 ? Math.round(vulnActive.reduce((s, a) => s + (tripLengthById[a.id] || 0), 0) / vulnActive.length) : 0;
  const avgNormActiveDist = normActive.length > 0 ? Math.round(normActive.reduce((s, a) => s + (tripLengthById[a.id] || 0), 0) / normActive.length) : 0;
  document.getElementById('m-vuln-d').textContent = avgVulnActiveDist > 0 ? avgVulnActiveDist + ' m' : '-';
  document.getElementById('m-norm-d').textContent = avgNormActiveDist > 0 ? avgNormActiveDist + ' m' : '-';
  updateGlobalLuxMetrics(step, TRIPS, 'm-');
}
const slider = document.getElementById('step-slider'); slider.min = 0; slider.max = (Math.max(1, maxStep)) * 10; slider.value = 0;
slider.addEventListener('input', () => {
  let val = parseInt(slider.value); currentFloatStep = val / 10;
  nextTripIdx = 0; activeList = []; updateActiveTrips(currentFloatStep);
  const agents = getLiveAgents(currentFloatStep);
  draw(agents); updateMetrics(currentFloatStep, agents);
});
let playing = false, animId = null, lastTs = 0, sliderFloatVal = 0; const playBtn = document.getElementById('play-btn'), speedSelect = document.getElementById('speed-select');
playBtn.addEventListener('click', () => {
  playing = !playing;
  playBtn.textContent = playing ? '⏸' : '▶';
  if (playing) {
    playBtn.classList.add('playing');
    lastTs = performance.now();
    sliderFloatVal = parseFloat(slider.value) || 0;
    animId = requestAnimationFrame(animate);
  } else {
    playBtn.classList.remove('playing');
    cancelAnimationFrame(animId);
  }
});
function animate(ts) {
  if (!playing) return;
  const dt = Math.min(64, ts - lastTs);
  lastTs = ts;
  sliderFloatVal += (parseFloat(speedSelect.value) || 2) * dt / 30;
  if (sliderFloatVal > parseFloat(slider.max)) { sliderFloatVal = 0; nextTripIdx = 0; activeList = []; }
  slider.value = Math.floor(sliderFloatVal); currentFloatStep = sliderFloatVal / 10;
  updateActiveTrips(currentFloatStep);
  const agents = getLiveAgents(currentFloatStep);
  draw(agents); updateMetrics(currentFloatStep, agents);
  animId = requestAnimationFrame(animate);
}
const hvCanvas = document.getElementById('hv-canvas'); const hvCtx = hvCanvas.getContext('2d'); hvCtx.imageSmoothingEnabled = true;
let hvScale = 1.0, hvPanX = 0, hvPanY = 0, hvDragging = false, hvDX = 0, hvDY = 0;
function toHvScreen(wx, wy) { return { x: (wx - minX) * hvScale + hvPanX, y: hvCanvas.height - ((wy - minY) * hvScale + hvPanY) }; }
function resetHvView() {
  const dx = maxX - minX, dy = maxY - minY, pad = 40;
  hvScale = Math.min((hvCanvas.width - pad*2) / (dx || 1), (hvCanvas.height - pad*2) / (dy || 1));
  hvPanX = hvCanvas.width / 2 - (dx / 2) * hvScale; hvPanY = hvCanvas.height / 2 - (dy / 2) * hvScale;
  hvDraw(currentHvHour);
}
function getHvRoadColor(vol, maxV, isDay) {
  if (vol === 0) return '#161c2e';
  const t = Math.min(Math.pow(vol / maxV, 0.5), 1.0);
  const theme = document.getElementById('hv-theme-select') ? document.getElementById('hv-theme-select').value : 'heatmap';
  if (theme === 'consistency') {
    let r, g, b;
    if (t < 0.5) {
      const u = t * 2;
      r = Math.round(56 - u * 56);
      g = Math.round(189 + u * 66);
      b = Math.round(248 + u * 7);
    } else {
      const u = (t - 0.5) * 2;
      r = Math.round(u * 255);
      g = 255;
      b = 255;
    }
    return `rgb(${r},${g},${b})`;
  } else {
    let r, g, b;
    if (t < 0.33) {
      const u = t / 0.33;
      r = Math.round(27 + u * (209 - 27));
      g = Math.round(12 + u * (60 - 12));
      b = Math.round(60 + u * (123 - 60));
    } else if (t < 0.67) {
      const u = (t - 0.33) / 0.34;
      r = Math.round(209 + u * (247 - 209));
      g = Math.round(60 + u * (122 - 60));
      b = Math.round(123 + u * (28 - 123));
    } else {
      const u = (t - 0.67) / 0.33;
      r = Math.round(247 + u * (252 - 247));
      g = Math.round(122 + u * (243 - 122));
      b = Math.round(28 + u * (64 - 28));
    }
    return `rgb(${r},${g},${b})`;
  }
}
function hvDraw(hour) {
  const baseHour = ((hour % 24) + 24) % 24;
  const hrInt = Math.floor(baseHour) % 24;
  const nextHrInt = (hrInt + 1) % 24;
  const frac = baseHour - Math.floor(baseHour);
  const isDay = baseHour >= 6.85 && baseHour < 18.5;
  hvCtx.clearRect(0, 0, hvCanvas.width, hvCanvas.height);
  hvCtx.fillStyle = getSkyColor(baseHour);
  hvCtx.fillRect(0, 0, hvCanvas.width, hvCanvas.height);
  let hvRoadsActive = 0, hvPeakVol = 0, hvTotalPeds = 0;
  const hvEdges = [];
  ROADS_GEOJSON.features.forEach(f => {
    if (!f.geometry) return;
    const pts = getPoints(f.geometry);
    if (pts.length < 2) return;
    const eid = f.properties && f.properties.edgeID;
    const vols = (eid != null && HOURLY_VOL[eid]) ? HOURLY_VOL[eid] : null;
    const v0 = vols ? (vols[hrInt] || 0) : 0;
    const v1 = vols ? (vols[nextHrInt] || 0) : 0;
    const vol = v0 + (v1 - v0) * frac;
    if (vol > 0.01) { hvRoadsActive++; hvTotalPeds += vol; if (vol > hvPeakVol) hvPeakVol = vol; }
    hvEdges.push({ pts, volume: vol });
  });
  hvEdges.sort((a, b) => a.volume - b.volume);
  hvEdges.forEach(edge => {
    const vol = edge.volume;
    const pts = edge.pts;
    hvCtx.strokeStyle = getHvRoadColor(vol, maxHourlyVol, isDay);
    hvCtx.lineWidth = vol > 0.01 ? 1.4 : 0.7;
    hvCtx.lineCap = 'round'; hvCtx.lineJoin = 'round'; hvCtx.beginPath();
    let p0 = toHvScreen(pts[0][0], pts[0][1]); hvCtx.moveTo(p0.x, p0.y);
    for (let i = 1; i < pts.length; i++) { let pi = toHvScreen(pts[i][0], pts[i][1]); hvCtx.lineTo(pi.x, pi.y); }
    hvCtx.stroke();
  });
  const hh = String(hrInt).padStart(2,'0'), nh = String((hrInt+1)%24).padStart(2,'0');
  const sunTxt = baseHour < 6.85 ? '🌙 Night' : baseHour < 7.35 ? '🌅 Sunrise' : baseHour < 17.5 ? '☀️ Day' : baseHour < 18.5 ? '🌇 Sunset' : '🌙 Night';
  document.getElementById('hv-time-label').textContent = `${hh}:00 – ${nh}:00`;
  document.getElementById('m-hv-hour').textContent  = `${hh}:00`; document.getElementById('m-hv-roads').textContent = hvRoadsActive;
  document.getElementById('m-hv-peak').textContent  = Math.round(hvPeakVol); document.getElementById('m-hv-total').textContent = Math.round(hvTotalPeds);
  document.getElementById('m-hv-sky').textContent   = sunTxt;
  hvCtx.font = 'bold 13px Inter, sans-serif'; hvCtx.fillStyle = 'rgba(200,210,240,0.9)';
  hvCtx.fillText(`__RUN_LABEL__   ${hh}:00 – ${nh}:00   ${sunTxt}`, 20, hvCanvas.height - 54);
}
let currentHvHour = 0; const hvSlider = document.getElementById('hv-hour-slider');
hvSlider.addEventListener('input', () => { currentHvHour = parseInt(hvSlider.value); hvFrac = 0; updateHvHourButtons(); hvDraw(currentHvHour); });
let hvPlaying = false, hvAnimId = null, hvLastTs = 0, hvFrac = 0; const hvPlayBtn = document.getElementById('hv-play-btn'), hvSpeedSel = document.getElementById('hv-speed-select');
hvPlayBtn.addEventListener('click', () => { hvPlaying = !hvPlaying; hvPlayBtn.textContent = hvPlaying ? '⏸' : '▶'; if (hvPlaying) { hvPlayBtn.classList.add('playing'); hvLastTs = performance.now(); hvFrac = 0; hvAnimId = requestAnimationFrame(hvAnimate); } else { hvPlayBtn.classList.remove('playing'); cancelAnimationFrame(hvAnimId); } });
function hvAnimate(ts) {
  if (!hvPlaying) return;
  const dt = ts - hvLastTs; hvLastTs = ts; hvFrac += dt * (parseFloat(hvSpeedSel.value) || 1) / 1500;
  if (hvFrac >= 1) { hvFrac -= 1; currentHvHour = (currentHvHour + 1) % 24; hvSlider.value = currentHvHour; updateHvHourButtons(); }
  const floatHour = (currentHvHour + hvFrac) % 24;
  hvDraw(floatHour);
  hvAnimId = requestAnimationFrame(hvAnimate);
}
let hvZoomRafId = null; hvCanvas.addEventListener('mousedown', e => { hvDragging = true; hvDX = e.clientX; hvDY = e.clientY; });
window.addEventListener('mousemove', e => { if(!hvDragging) return; hvPanX += e.clientX-hvDX; hvPanY -= e.clientY-hvDY; hvDX=e.clientX; hvDY=e.clientY; if (hvZoomRafId) cancelAnimationFrame(hvZoomRafId); hvZoomRafId = requestAnimationFrame(() => { hvZoomRafId=null; hvDraw(currentHvHour + hvFrac); }); });
window.addEventListener('mouseup', () => { hvDragging = false; });
hvCanvas.addEventListener('wheel', e => {
  e.preventDefault(); const rect = hvCanvas.getBoundingClientRect(), mx = e.clientX-rect.left, my = e.clientY-rect.top; const wx = (mx-hvPanX)/hvScale+minX, wy = (hvCanvas.height-my-hvPanY)/hvScale+minY;
  const factor = e.deltaY < 0 ? 1.15 : 1/1.15; hvScale *= factor; hvPanX = mx-(wx-minX)*hvScale; hvPanY = (hvCanvas.height-my)-(wy-minY)*hvScale;
  if (hvZoomRafId) cancelAnimationFrame(hvZoomRafId); hvZoomRafId = requestAnimationFrame(() => { hvZoomRafId=null; hvDraw(currentHvHour + hvFrac); });
}, { passive: false });
document.getElementById('hv-reset-btn').addEventListener('click', resetHvView);
const grid = document.getElementById('hv-hour-grid');
for (let h = 0; h < 24; h++) {
  const btn = document.createElement('button');
  btn.className = 'hour-btn';
  btn.textContent = String(h).padStart(2, '0');
  btn.addEventListener('click', () => {
    currentHvHour = h; hvSlider.value = h; hvFrac = 0; updateHvHourButtons(); hvDraw(h);
  });
  grid.appendChild(btn);
}
function updateHvHourButtons() {
  const btns = document.querySelectorAll('.hour-btn');
  btns.forEach((btn, idx) => {
    if (idx === currentHvHour) btn.classList.add('active');
    else btn.classList.remove('active');
  });
}
updateHvHourButtons();
const hvThemeSelect = document.getElementById('hv-theme-select');
function updateHvLegend() {
  const theme = hvThemeSelect ? hvThemeSelect.value : 'heatmap';
  const bar = document.querySelector('.hv-legend-bar');
  const legLight = document.getElementById('hv-leg-light');
  const legHeavy = document.getElementById('hv-leg-heavy');
  if (theme === 'consistency') {
    if (bar) bar.style.background = 'linear-gradient(to right, #161c2e, #38bdf8, #00ffff, #ffffff)';
    if (legLight) legLight.style.background = '#38bdf8';
    if (legHeavy) legHeavy.style.background = '#ffffff';
  } else {
    if (bar) bar.style.background = 'linear-gradient(to right, #161c2e, #d13c7b, #f77a1c, #fcf340)';
    if (legLight) legLight.style.background = '#d13c7b';
    if (legHeavy) legHeavy.style.background = '#fcf340';
  }
}
if (hvThemeSelect) {
  hvThemeSelect.addEventListener('change', () => {
    updateHvLegend();
    hvDraw(currentHvHour + hvFrac);
  });
}
updateHvLegend();
const abCanvas = document.getElementById('ab-canvas'); const abCtx = abCanvas.getContext('2d'); abCtx.imageSmoothingEnabled = true;
let abScale = 1.0, abPanX = 0, abPanY = 0, abCurrentFloatStep = 0;
function abToScreen(wx, wy) { return { x: (wx - minX) * abScale + abPanX, y: abCanvas.height - ((wy - minY) * abScale + abPanY) }; }
function abResetView() {
  const dx = maxX - minX, dy = maxY - minY, pad = 40;
  abScale = Math.min((abCanvas.width - pad*2) / (dx || 1), (abCanvas.height - pad*2) / (dy || 1));
  abPanX = abCanvas.width / 2 - (minX + dx / 2 - minX) * abScale; abPanY = abCanvas.height / 2 - (minY + dy / 2 - minY) * abScale;
  abRoadsDirty = true; abDraw();
}
let abMaxStep = 0;
const abTripLengthById = {};
AB_TRIPS.forEach(t => {
  if (t[2] > abMaxStep) abMaxStep = t[2]; const coords = t[3]; const segs = []; let tot = 0;
  for (let i = 0; i < coords.length - 1; i++) { const dx = coords[i+1][0]-coords[i][0], dy = coords[i+1][1]-coords[i][1]; const l = Math.sqrt(dx*dx+dy*dy); segs.push(l); tot += l; }
  t[6] = segs; t[7] = tot;
  abTripLengthById[t[0]] = tot;
}); AB_TRIPS.sort((a, b) => a[1] - b[1]); let abActiveList = [], abNextTripIdx = 0;
function abUpdateActiveTrips(floatStep) {
  while(abNextTripIdx < AB_TRIPS.length && AB_TRIPS[abNextTripIdx][1] <= floatStep) { abActiveList.push(AB_TRIPS[abNextTripIdx]); abNextTripIdx++; }
  for (let i = abActiveList.length - 1; i >= 0; i--) { if (abActiveList[i][2] < floatStep) abActiveList.splice(i, 1); }
}
function abGetLiveAgents(floatStep) {
  const res = []; for (let i = 0; i < abActiveList.length; i++) { const t = abActiveList[i]; const duration = t[2] - t[1]; const progress = duration > 0 ? (floatStep - t[1]) / duration : 1.0; const pt = getPointAlongPath(t[3], t[6], t[7], progress); if (pt) res.push({ id: t[0], vuln: t[4], x: pt.x, y: pt.y, progress }); } return res;
}
const abOffscreenVol = document.createElement('canvas'); const abOffCtxVol = abOffscreenVol.getContext('2d'); const abOffscreenLight = document.createElement('canvas'); const abOffscreenLightCtx = abOffscreenLight.getContext('2d'); let abRoadsDirty = true;
const abTgLight = document.getElementById('ab-tg-light'); const abTgTethers = document.getElementById('ab-tg-tethers');
if (abTgLight) abTgLight.addEventListener('change', () => { abDraw(); }); if (abTgTethers) abTgTethers.addEventListener('change', () => { abDraw(); });
function abBuildRoadLayers() {
  if (abOffscreenVol.width !== abCanvas.width || abOffscreenVol.height !== abCanvas.height) {
    abOffscreenVol.width = abCanvas.width; abOffscreenVol.height = abCanvas.height;
    abOffscreenLight.width = abCanvas.width; abOffscreenLight.height = abCanvas.height;
  }
  abOffCtxVol.clearRect(0, 0, abCanvas.width, abCanvas.height); abOffscreenLightCtx.clearRect(0, 0, abCanvas.width, abCanvas.height);
  const sortedFeatures = [...ROADS_GEOJSON.features].sort((a, b) => (a.properties.volume || 0) - (b.properties.volume || 0));
  sortedFeatures.forEach(f => {
    if (!f.geometry) return; const pts = getPoints(f.geometry); if (pts.length < 2) return;
    abOffCtxVol.strokeStyle = getVolColor(f, false); abOffCtxVol.lineWidth = getVolWeight(f); abOffCtxVol.lineCap = 'round'; abOffCtxVol.lineJoin = 'round'; abOffCtxVol.beginPath();
    let p0 = abToScreen(pts[0][0], pts[0][1]); abOffCtxVol.moveTo(p0.x, p0.y); for (let i = 1; i < pts.length; i++) { const pi = abToScreen(pts[i][0], pts[i][1]); abOffCtxVol.lineTo(pi.x, pi.y); } abOffCtxVol.stroke();
    abOffscreenLightCtx.strokeStyle = getVolColor(f, true); abOffscreenLightCtx.lineWidth = getVolWeight(f); abOffscreenLightCtx.lineCap = 'round'; abOffscreenLightCtx.lineJoin = 'round'; abOffscreenLightCtx.beginPath();
    p0 = abToScreen(pts[0][0], pts[0][1]); abOffscreenLightCtx.moveTo(p0.x, p0.y); for (let i = 1; i < pts.length; i++) { const pi = abToScreen(pts[i][0], pts[i][1]); abOffscreenLightCtx.lineTo(pi.x, pi.y); } abOffscreenLightCtx.stroke();
  }); abRoadsDirty = false;
}
function abDraw(agents) {
  if (!agents) agents = abGetLiveAgents(abCurrentFloatStep);
  if (abRoadsDirty) abBuildRoadLayers();
  const simHour = (abTgLight && abTgLight.checked) ? ((abCurrentFloatStep * 20 / 60) % 24) : 12;
  abCtx.fillStyle = getSkyColor(simHour);
  abCtx.fillRect(0, 0, abCanvas.width, abCanvas.height);
  abCtx.drawImage(abTgLight && abTgLight.checked ? abOffscreenLight : abOffscreenVol, 0, 0); const posById = {};
  agents.forEach(a => { posById[a.id] = abToScreen(a.x, a.y); }); if (abTgTethers && abTgTethers.checked) {
    abCtx.strokeStyle = 'rgba(255, 255, 255, 0.35)'; abCtx.lineWidth = 1.0; abCtx.setLineDash([4, 4]); abCtx.beginPath();
    agents.forEach(a => { if (a.vuln && posById[a.id + 1]) { const p1 = posById[a.id], p2 = posById[a.id + 1]; abCtx.moveTo(p1.x, p1.y); abCtx.lineTo(p2.x, p2.y); } });
    abCtx.stroke(); abCtx.setLineDash([]);
  }
  AB_TRIPS.forEach(t => {
    if (t[1] > abCurrentFloatStep) return;
    const isCompleted = t[2] <= abCurrentFloatStep;
    if (isCompleted && (abCurrentFloatStep - t[2] > 120)) return; // disappear after ~2 seconds (120 frames at 60fps)
    const progress = isCompleted ? 1.0 : (abCurrentFloatStep - t[1]) / (t[2] - t[1] || 1.0);
    const coords = t[3];
    const segs = t[6];
    const totalLength = t[7];
    const vuln = t[4];
    if (coords && coords.length > 0) {
      let alpha = 0.85;
      if (isCompleted) {
         alpha = 0.85 * (1.0 - (abCurrentFloatStep - t[2]) / 120.0);
         if (alpha < 0) alpha = 0;
      }
      abCtx.strokeStyle = vuln ? `rgba(239, 68, 68, ${alpha})` : `rgba(56, 189, 248, ${alpha})`;
      abCtx.lineWidth = 2.5;
      abCtx.beginPath();
      let p0 = abToScreen(coords[0][0], coords[0][1]);
      abCtx.moveTo(p0.x, p0.y);
      const targetDist = progress * totalLength;
      let acc = 0;
      for (let i = 0; i < segs.length; i++) {
        const l = segs[i];
        if (acc + l >= targetDist) {
          const f = (targetDist - acc) / (l || 1);
          const lastX = coords[i][0] + (coords[i+1][0] - coords[i][0]) * f;
          const lastY = coords[i][1] + (coords[i+1][1] - coords[i][1]) * f;
          const pLast = abToScreen(lastX, lastY);
          abCtx.lineTo(pLast.x, pLast.y);
          break;
        } else {
          const pi = abToScreen(coords[i+1][0], coords[i+1][1]);
          abCtx.lineTo(pi.x, pi.y);
        }
        acc += l;
      }
      abCtx.stroke();
    }
  });
  agents.forEach(a => { const p = posById[a.id]; abCtx.beginPath(); abCtx.arc(p.x, p.y, a.vuln ? 7 : 5, 0, Math.PI * 2); abCtx.fillStyle = a.vuln ? '#ef4444' : '#38bdf8'; abCtx.fill(); });
}
let abIsDragging = false, abStartX = 0, abStartY = 0; abCanvas.addEventListener('mousedown', e => { abIsDragging = true; abStartX = e.clientX; abStartY = e.clientY; });
window.addEventListener('mousemove', e => { if (!abIsDragging) return; abPanX += e.clientX - abStartX; abPanY -= e.clientY - abStartY; abStartX = e.clientX; abStartY = e.clientY; abRoadsDirty = true; abDraw(); });
window.addEventListener('mouseup', () => { abIsDragging = false; }); let abZoomRafId = null;
abCanvas.addEventListener('wheel', e => {
  e.preventDefault(); const rect = abCanvas.getBoundingClientRect(), mouseX = e.clientX - rect.left, mouseY = e.clientY - rect.top; const worldX = (mouseX - abPanX) / abScale + minX, worldY = (abCanvas.height - mouseY - abPanY) / abScale + minY; const factor = e.deltaY < 0 ? 1.12 : 1/1.12;
  abScale *= factor; abPanX = mouseX - (worldX - minX) * abScale; abPanY = (abCanvas.height - mouseY) - (worldY - minY) * abScale; abRoadsDirty = true; if (abZoomRafId) cancelAnimationFrame(abZoomRafId); abZoomRafId = requestAnimationFrame(() => { abZoomRafId = null; abDraw(); });
}, { passive: false }); document.getElementById('ab-reset-btn').addEventListener('click', abResetView);
function abUpdateMetrics(step, agents) {
  if (!agents) agents = abGetLiveAgents(step);
  const total = agents.length, vuln = agents.filter(a=>a.vuln).length; document.getElementById('m-active').textContent = total; document.getElementById('m-vuln').textContent = total > 0 ? (vuln/total*100).toFixed(1)+'%' : '0%';
  const simHour = (step * 20 / 60) % 24; const hh = String(Math.floor(simHour)).padStart(2,'0'); const mm = String(Math.floor(step*20)%60).padStart(2,'0');
  document.getElementById('m-time').textContent = `__RUN_LABEL__ ${hh}:${mm}`; document.getElementById('ab-time-label').textContent = document.getElementById('m-time').textContent;
  if (abTgLight) { const shouldLight = simHour >= 18.5 || simHour < 6.85; if (abTgLight.checked !== shouldLight) { abTgLight.checked = shouldLight; abDraw(agents); } }
  const vulnActive = agents.filter(a => a.vuln); const normActive = agents.filter(a => !a.vuln);
  const avgVulnActiveDist = vulnActive.length > 0 ? Math.round(vulnActive.reduce((s, a) => s + (abTripLengthById[a.id] || 0), 0) / vulnActive.length) : 0;
  const avgNormActiveDist = normActive.length > 0 ? Math.round(normActive.reduce((s, a) => s + (abTripLengthById[a.id] || 0), 0) / normActive.length) : 0;
  document.getElementById('m-vuln-d').textContent = avgVulnActiveDist > 0 ? avgVulnActiveDist + ' m' : '-'; document.getElementById('m-norm-d').textContent = avgNormActiveDist > 0 ? avgNormActiveDist + ' m' : '-';
  if (abTgLight && abTgLight.checked && total > 0) {
    let sumLux = 0; agents.forEach(a => {
      const gx = Math.floor(a.x/GRID_SZ), gy = Math.floor(a.y/GRID_SZ); sumLux += parseFloat(luxGrid[gx+','+gy] || 0); });
    document.getElementById('ab-m-lux').textContent = (sumLux / total).toFixed(2); } else { document.getElementById('m-lux').textContent = '-'; }
}
const abSlider = document.getElementById('ab-step-slider'); if (abSlider) { abSlider.min = 0; abSlider.max = (Math.max(1, abMaxStep)) * 10; abSlider.value = 0;
  abSlider.addEventListener('input', () => { abSliderFloatVal = parseInt(abSlider.value); abCurrentFloatStep = abSliderFloatVal / 10; abNextTripIdx = 0; abActiveList = []; abUpdateActiveTrips(abCurrentFloatStep); const agents = abGetLiveAgents(abCurrentFloatStep); abDraw(agents); abUpdateMetrics(abCurrentFloatStep, agents); }); }
let abPlaying = false, abAnimId = null, abLastTs = 0, abSliderFloatVal = 0; const abPlayBtn = document.getElementById('ab-play-btn'), abSpeedSelect = document.getElementById('ab-speed-select');
if (abPlayBtn) { abPlayBtn.addEventListener('click', () => {
  abPlaying = !abPlaying;
  abPlayBtn.textContent = abPlaying ? '⏸' : '▶';
  if (abPlaying) {
    abPlayBtn.classList.add('playing');
    abLastTs = performance.now();
    abSliderFloatVal = parseFloat(abSlider.value) || 0;
    abAnimId = requestAnimationFrame(abAnimate);
  } else {
    abPlayBtn.classList.remove('playing');
    cancelAnimationFrame(abAnimId);
  }
}); }
function abAnimate(ts) {
  if (!abPlaying) return;
  const dt = Math.min(64, ts - abLastTs);
  abLastTs = ts;
  abSliderFloatVal += (parseFloat(abSpeedSelect.value) || 2) * dt / 30;
  if (abSliderFloatVal > parseFloat(abSlider.max)) { abSliderFloatVal = 0; abNextTripIdx = 0; abActiveList = []; }
  abSlider.value = Math.floor(abSliderFloatVal); abCurrentFloatStep = abSliderFloatVal / 10;
  abUpdateActiveTrips(abCurrentFloatStep);
  const agents = abGetLiveAgents(abCurrentFloatStep);
  abDraw(agents); abUpdateMetrics(abCurrentFloatStep, agents);
  abAnimId = requestAnimationFrame(abAnimate);
}
window.addEventListener('resize', () => {
  canvas.width = canvas.clientWidth; canvas.height = canvas.clientHeight; roadsDirty = true; draw();
  if (document.getElementById('panel-hourly').classList.contains('active')) { hvCanvas.width = hvCanvas.clientWidth; hvCanvas.height = hvCanvas.clientHeight; hvDraw(currentHvHour + hvFrac); }
  if (document.getElementById('panel-volumes').classList.contains('active')) { volCanvas.width = volCanvas.clientWidth; volCanvas.height = volCanvas.clientHeight; volDraw(); }
  if (document.getElementById('panel-ab').classList.contains('active')) { abCanvas.width = abCanvas.clientWidth; abCanvas.height = abCanvas.clientHeight; abRoadsDirty = true; abDraw(); }
});
const volCanvas = document.getElementById('vol-canvas'); const volCtx = volCanvas.getContext('2d'); volCtx.imageSmoothingEnabled = true;
let volScale = 1.0, volPanX = 0, volPanY = 0, volDragging = false, volDX = 0, volDY = 0;
function toVolScreen(wx, wy) { return { x: (wx - minX) * volScale + volPanX, y: volCanvas.height - ((wy - minY) * volScale + volPanY) }; }
function resetVolView() {
  const dx = maxX - minX, dy = maxY - minY, pad = 40;
  volScale = Math.min((volCanvas.width - pad*2) / (dx || 1), (volCanvas.height - pad*2) / (dy || 1));
  volPanX = volCanvas.width / 2 - (dx / 2) * volScale; volPanY = volCanvas.height / 2 - (dy / 2) * volScale;
  volDraw();
}
const edgeAverages = {}; let maxDayAvg = 0, maxNightAvg = 0;
Object.keys(HOURLY_VOL).forEach(eid => {
  const vols = HOURLY_VOL[eid]; let daySum = 0, nightSum = 0;
  for (let h = 0; h < 24; h++) {
    if (h >= 7 && h <= 18) daySum += vols[h];
    else nightSum += vols[h];
  }
  const dayAvg = daySum / 12, nightAvg = nightSum / 12;
  edgeAverages[eid] = { day: dayAvg, night: nightAvg };
  if (dayAvg > maxDayAvg) maxDayAvg = dayAvg;
  if (nightAvg > maxNightAvg) maxNightAvg = nightAvg;
});
if (maxDayAvg === 0) maxDayAvg = 1; if (maxNightAvg === 0) maxNightAvg = 1;
function getVolTabRoadColor(vol, maxV) {
  if (vol === 0) return '#161c2e';
  const t = Math.min(Math.pow(vol / maxV, 0.5), 1.0);
  const theme = document.getElementById('vol-theme-select') ? document.getElementById('vol-theme-select').value : 'heatmap';
  if (theme === 'consistency') {
    let r, g, b;
    if (t < 0.5) {
      const u = t * 2;
      r = Math.round(56 - u * 56);
      g = Math.round(189 + u * 66);
      b = Math.round(248 + u * 7);
    } else {
      const u = (t - 0.5) * 2;
      r = Math.round(u * 255);
      g = 255;
      b = 255;
    }
    return `rgb(${r},${g},${b})`;
  } else {
    let r, g, b;
    if (t < 0.33) {
      const u = t / 0.33;
      r = Math.round(27 + u * (209 - 27));
      g = Math.round(12 + u * (60 - 12));
      b = Math.round(60 + u * (123 - 60));
    } else if (t < 0.67) {
      const u = (t - 0.33) / 0.34;
      r = Math.round(209 + u * (247 - 209));
      g = Math.round(60 + u * (122 - 60));
      b = Math.round(123 + u * (28 - 123));
    } else {
      const u = (t - 0.67) / 0.33;
      r = Math.round(247 + u * (252 - 247));
      g = Math.round(122 + u * (243 - 122));
      b = Math.round(28 + u * (64 - 28));
    }
    return `rgb(${r},${g},${b})`;
  }
}
function getEdgeMidpoint(coords) {
  if (!coords || coords.length === 0) return null;
  const segs = []; let totalLength = 0;
  for (let i = 0; i < coords.length - 1; i++) {
    const dx = coords[i+1][0] - coords[i][0];
    const dy = coords[i+1][1] - coords[i][1];
    const l = Math.sqrt(dx * dx + dy * dy);
    segs.push(l); totalLength += l;
  }
  return getPointAlongPath(coords, segs, totalLength, 0.5);
}
function volDraw() {
  const period = document.getElementById('vol-period-select').value;
  const maxV = period === 'day' ? maxDayAvg : maxNightAvg;
  const isDay = period === 'day';
  const showTop10 = document.getElementById('vol-tg-top10') && document.getElementById('vol-tg-top10').checked;
  volCtx.fillStyle = isDay ? getSkyColor(12) : getSkyColor(0);
  volCtx.fillRect(0, 0, volCanvas.width, volCanvas.height);
  const periodEdges = [];
  ROADS_GEOJSON.features.forEach(f => {
    const eid = f.properties && f.properties.edgeID;
    if (eid == null) return;
    const vol = edgeAverages[eid] ? edgeAverages[eid][period] : 0;
    periodEdges.push({ feature: f, edgeID: eid, volume: vol });
  });
  periodEdges.sort((a, b) => b.volume - a.volume);
  const top10Ids = new Set();
  const top10List = [];
  for (let i = 0; i < Math.min(10, periodEdges.length); i++) {
    if (periodEdges[i].volume > 0) {
      top10Ids.add(periodEdges[i].edgeID);
      top10List.push(periodEdges[i]);
    }
  }
  let activeRoads = 0, peakVol = 0, totalPeds = 0;
  const drawEdges = [];
  periodEdges.forEach(item => {
    const f = item.feature;
    if (!f.geometry) return;
    const pts = getPoints(f.geometry);
    if (pts.length < 2) return;
    const vol = item.volume;
    if (vol > 0) { activeRoads++; totalPeds += vol; if (vol > peakVol) peakVol = vol; }
    const drawVolume = (showTop10 && !top10Ids.has(item.edgeID)) ? 0 : vol;
    drawEdges.push({ pts, volume: drawVolume });
  });
  drawEdges.sort((a, b) => a.volume - b.volume);
  drawEdges.forEach(edge => {
    const vol = edge.volume;
    const pts = edge.pts;
    volCtx.strokeStyle = getVolTabRoadColor(vol, maxV);
    volCtx.lineWidth = vol > 0 ? 1.4 : 0.7;
    volCtx.lineCap = 'round'; volCtx.lineJoin = 'round'; volCtx.beginPath();
    let p0 = toVolScreen(pts[0][0], pts[0][1]); volCtx.moveTo(p0.x, p0.y);
    for (let i = 1; i < pts.length; i++) { let pi = toVolScreen(pts[i][0], pts[i][1]); volCtx.lineTo(pi.x, pi.y); }
    volCtx.stroke();
  });
  if (showTop10) {
    top10List.forEach((item, index) => {
      const eid = item.edgeID;
      const coords = edgeCoords[eid];
      if (!coords) return;
      const mid = getEdgeMidpoint(coords);
      if (!mid) return;
      const p = toVolScreen(mid.x, mid.y);
      volCtx.save();
      volCtx.beginPath();
      volCtx.arc(p.x, p.y, 9, 0, Math.PI * 2);
      volCtx.fillStyle = '#ef4444';
      volCtx.fill();
      volCtx.strokeStyle = '#ffffff';
      volCtx.lineWidth = 1.5;
      volCtx.stroke();
      volCtx.font = 'bold 10px Inter, sans-serif';
      volCtx.fillStyle = '#ffffff';
      volCtx.textAlign = 'center';
      volCtx.textBaseline = 'middle';
      volCtx.fillText(String(index + 1), p.x, p.y);
      volCtx.restore();
    });
  }
  const titleTxt = period === 'day' ? '☀️ Day Flow Averages (__DAY_START__:00 – __NIGHT_START__:00)' : '🌙 Night Flow Averages (__NIGHT_START__:00 – __DAY_START__:00)';
  document.getElementById('m-vol-type').textContent = period === 'day' ? 'Day Flow' : 'Night Flow';
  document.getElementById('m-vol-peak').textContent = Math.round(peakVol) + ' / h';
  document.getElementById('m-vol-total').textContent = Math.round(totalPeds);
  volCtx.save();
  volCtx.font = 'bold 13px Inter, sans-serif';
  volCtx.fillStyle = 'rgba(200,210,240,0.9)';
  volCtx.fillText(`Torino   ${titleTxt}`, 20, volCanvas.height - 54);
  volCtx.restore();
}
let volZoomRafId = null; volCanvas.addEventListener('mousedown', e => { volDragging = true; volDX = e.clientX; volDY = e.clientY; });
window.addEventListener('mousemove', e => { if(!volDragging) return; volPanX += e.clientX-volDX; volPanY -= e.clientY-volDY; volDX=e.clientX; volDY=e.clientY; if (volZoomRafId) cancelAnimationFrame(volZoomRafId); volZoomRafId = requestAnimationFrame(() => { volZoomRafId=null; volDraw(); }); });
window.addEventListener('mouseup', () => { volDragging = false; });
volCanvas.addEventListener('wheel', e => {
  e.preventDefault(); const rect = volCanvas.getBoundingClientRect(), mx = e.clientX-rect.left, my = e.clientY-rect.top; const wx = (mx-volPanX)/volScale+minX, wy = (volCanvas.height-my-volPanY)/volScale+minY;
  const factor = e.deltaY < 0 ? 1.15 : 1/1.15; volScale *= factor; volPanX = mx-(wx-minX)*volScale; volPanY = (volCanvas.height-my)-(wy-minY)*volScale;
  if (volZoomRafId) cancelAnimationFrame(volZoomRafId); volZoomRafId = requestAnimationFrame(() => { volZoomRafId=null; volDraw(); });
}, { passive: false });
document.getElementById('vol-reset-btn').addEventListener('click', resetVolView);
const volPeriodSelect = document.getElementById('vol-period-select');
const volThemeSelect = document.getElementById('vol-theme-select');
function updateVolLegend() {
  const theme = volThemeSelect ? volThemeSelect.value : 'consistency';
  const bar = document.getElementById('vol-legend-bar');
  const legLight = document.getElementById('vol-leg-light');
  const legHeavy = document.getElementById('vol-leg-heavy');
  if (theme === 'consistency') {
    if (bar) bar.style.background = 'linear-gradient(to right, #161c2e, #38bdf8, #00ffff, #ffffff)';
    if (legLight) legLight.style.background = '#38bdf8';
    if (legHeavy) legHeavy.style.background = '#ffffff';
  } else {
    if (bar) bar.style.background = 'linear-gradient(to right, #161c2e, #d13c7b, #f77a1c, #fcf340)';
    if (legLight) legLight.style.background = '#d13c7b';
    if (legHeavy) legHeavy.style.background = '#fcf340';
  }
}
if (volPeriodSelect) volPeriodSelect.addEventListener('change', volDraw);
if (volThemeSelect) {
  volThemeSelect.addEventListener('change', () => {
    updateVolLegend();
    volDraw();
  });
}
const volTgTop10 = document.getElementById('vol-tg-top10');
if (volTgTop10) volTgTop10.addEventListener('change', volDraw);
updateVolLegend();
canvas.width = canvas.clientWidth; canvas.height = canvas.clientHeight; resetView(); updateMetrics(0);
document.querySelector(`[onclick="switchTab('hourly')"]`).addEventListener('click', () => { setTimeout(() => { hvCanvas.width = hvCanvas.clientWidth; hvCanvas.height = hvCanvas.clientHeight; resetHvView(); }, 50); });
document.querySelector(`[onclick="switchTab('volumes')"]`).addEventListener('click', () => { setTimeout(() => { volCanvas.width = volCanvas.clientWidth; volCanvas.height = volCanvas.clientHeight; resetVolView(); }, 50); });
document.querySelector(`[onclick="switchTab('ab')"]`).addEventListener('click', () => { setTimeout(() => { abCanvas.width = abCanvas.clientWidth; abCanvas.height = abCanvas.clientHeight; abResetView(); if (AB_TRIPS && AB_TRIPS.length > 0) abUpdateMetrics(abCurrentFloatStep); }, 50); });
  initTimeline();
  switchTab('trips');
  startAnim();
</script>
</div>
</body>
</html>
""";
}
