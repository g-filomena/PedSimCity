package pedsim.core.engine;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.List;
import java.util.Map;
import java.util.logging.Logger;
import pedsim.core.parameters.Pars;
import pedsim.core.utilities.LoggerUtil;
import pedsim.core.website.GeoJsonExporter;

/**
 * Generates a self-contained, single-file HTML dashboard that embeds:
 * <ul>
 *   <li>The road network as a GeoJSON FeatureCollection (with edgeID + cumulative volume per edge).</li>
 *   <li>All agent trips with exact street coordinates traversed.</li>
 *   <li>The full interactive visualizer engine inline.</li>
 * </ul>
 *
 * <p>Output path:
 * {@code C:\Users\<username>\PedSimCity\Output\results\results_<city>_day<day>_job<job>.html}
 */
public class HtmlExporter {

  private static final Logger logger = LoggerUtil.getLogger();
  private static final String USER = System.getProperty("user.name");

  /**
   * The root output directory for all HTML dashboard files.
   * Mirrors the pattern used by {@link Exporter}.
   */
  private static final String OUTPUT_ROOT =
      "C:" + File.separator + "Users" + File.separator + USER
      + File.separator + "PedSimCity" + File.separator + "Output"
      + File.separator + "results";

  /**
   * Exports a complete, self-contained HTML dashboard.
   *
   * @param day         The simulated day number (1-based).
   * @param job         The job index of this run.
   * @param trips       All trip records from {@link TripRouteRecorder}.
   * @param volumesMap  Map of edgeID → (scenario → cumulativeVolume) from {@link FlowHandler}.
   * @return The absolute path of the written HTML file, or {@code null} on failure.
   */
  public static String export(int day, int job,
      List<TripRouteRecorder.TripRecord> trips,
      Map<Integer, Map<String, Integer>> volumesMap) {

    try {
      // 1. Ensure the output directory exists
      Files.createDirectories(Paths.get(OUTPUT_ROOT));

      String city = Pars.cityName;
      String filename = "results_" + city + "_day" + day + "_job" + job + ".html";
      String outputPath = OUTPUT_ROOT + File.separator + filename;

      // 2. Build the embedded data as compact JS strings
      String roadsGeoJson = GeoJsonExporter.exportRoadsWithVolumes(
          PedSimCity.roads, volumesMap);

      String tripsJs = buildTripsJs(trips);

      // 3. Determine map centre from the road network bounding box
      double[] centre = computeCentre();

      // 4. Render the full HTML and write it
      String html = renderHtml(city, day, job, roadsGeoJson, tripsJs, centre);
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

  // -------------------------------------------------------------------------
  // Private helpers
  // -------------------------------------------------------------------------

  /**
   * Converts the trip records list into a compact JavaScript array literal.
   * Format: [[agentID, startStep, endStep, [[x1, y1], [x2, y2], ...], vulnerable], ...]
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
        .append(']');
    }
    sb.append(']');
    return sb.toString();
  }

  /** Returns [centreLatitude, centreLongitude] from the road layer MBR. */
  private static double[] computeCentre() {
    if (PedSimCity.MBR == null) {
      return new double[]{45.07, 7.68}; // Torino fallback
    }
    double cx = (PedSimCity.MBR.getMinX() + PedSimCity.MBR.getMaxX()) / 2.0;
    double cy = (PedSimCity.MBR.getMinY() + PedSimCity.MBR.getMaxY()) / 2.0;
    return new double[]{cy, cx};
  }

  // -------------------------------------------------------------------------
  // HTML template
  // -------------------------------------------------------------------------

  /**
   * Renders the full, self-contained HTML string.
   */
  private static String renderHtml(String city, int day, int job,
      String roadsGeoJson, String tripsJs, double[] centre) {

    return "<!DOCTYPE html>\n"
        + "<html lang=\"en\">\n"
        + "<head>\n"
        + "<meta charset=\"UTF-8\"/>\n"
        + "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\"/>\n"
        + "<title>PedSimCity – " + city + " Job " + job + "</title>\n"
        // Google Font
        + "<link rel=\"preconnect\" href=\"https://fonts.googleapis.com\">\n"
        + "<link href=\"https://fonts.googleapis.com/css2?family=Inter:wght@300;400;600;700&display=swap\" rel=\"stylesheet\">\n"
        + "<style>\n"
        + "  *{box-sizing:border-box;margin:0;padding:0}\n"
        + "  body{font-family:'Inter',sans-serif;background:#000000;color:#f1f5f9;display:flex;flex-direction:column;height:100vh;overflow:hidden}\n"
        + "  #header{padding:14px 20px;background:rgba(0,0,0,0.95);border-bottom:1px solid #1e293b;display:flex;align-items:center;justify-content:space-between;gap:16px;flex-shrink:0}\n"
        + "  #header h1{font-size:1.15rem;font-weight:700;color:#38bdf8;letter-spacing:0.02em}\n"
        + "  .metrics{display:flex;gap:12px}\n"
        + "  .card{background:rgba(30,41,59,0.45);border:1px solid #1e293b;border-radius:10px;padding:10px 18px;min-width:120px}\n"
        + "  .card .label{font-size:.65rem;color:#94a3b8;text-transform:uppercase;letter-spacing:.08em}\n"
        + "  .card .value{font-size:1.4rem;font-weight:700;color:#38bdf8;margin-top:2px}\n"
        + "  .card .value.red{color:#ef4444}\n"
        + "  .card .value.orange{color:#fb923c}\n"
        + "  #container{position:relative;flex:1;width:100%;overflow:hidden;background:#000000}\n"
        + "  #canvas{width:100%;height:100%;display:block;cursor:grab}\n"
        + "  #canvas:active{cursor:grabbing}\n"
        + "  #floating-controls{position:absolute;top:20px;right:20px;display:flex;flex-direction:column;gap:8px}\n"
        + "  .btn-float{background:rgba(30,41,59,0.85);border:1px solid #334155;border-radius:8px;color:#f1f5f9;padding:8px 14px;font-size:.75rem;font-weight:600;cursor:pointer;backdrop-filter:blur(4px);transition:all .2s;display:flex;align-items:center;gap:6px}\n"
        + "  .btn-float:hover{background:#38bdf8;color:#0b0f19;border-color:#38bdf8}\n"
        + "  #timeline-panel{flex-shrink:0;padding:14px 20px 16px;background:rgba(0,0,0,0.98);border-top:1px solid #1e293b}\n"
        + "  #timeline-panel .row{display:flex;align-items:center;gap:14px}\n"
        + "  #speed-select{background:rgba(30,41,59,0.85);color:#f1f5f9;border:1px solid #334155;border-radius:8px;padding:6px 12px;font-size:.75rem;font-weight:600;cursor:pointer;outline:none;transition:all .2s;display:inline-flex;align-items:center}\n"
        + "  #speed-select:hover{background:#38bdf8;color:#0b0f19;border-color:#38bdf8}\n"
        + "  #play-btn{width:36px;height:36px;border-radius:50%;border:2px solid #38bdf8;background:transparent;color:#38bdf8;cursor:pointer;font-size:1.1rem;display:flex;align-items:center;justify-content:center;flex-shrink:0;transition:all .2s}\n"
        + "  #play-btn:hover{background:#38bdf8;color:#0b0f19}\n"
        + "  #step-slider{flex:1;-webkit-appearance:none;height:5px;border-radius:3px;background:#1e293b;outline:none;cursor:pointer}\n"
        + "  #step-slider::-webkit-slider-thumb{-webkit-appearance:none;width:16px;height:16px;border-radius:50%;background:#38bdf8;cursor:pointer;box-shadow:0 0 6px #38bdf880}\n"
        + "  #time-label{font-size:.8rem;color:#7dd3fc;min-width:48px;text-align:right;font-weight:600}\n"
        + "  .legend{display:flex;gap:16px;margin-top:10px;font-size:.7rem;color:#94a3b8}\n"
        + "  .leg-item{display:flex;align-items:center;gap:6px}\n"
        + "  .leg-dot{width:8px;height:8px;border-radius:50%}\n"
        + "  .leg-line{width:20px;height:3px;border-radius:2px}\n"
        + "</style>\n"
        + "</head>\n"
        + "<body>\n"
        + "<div id=\"header\">\n"
        + "  <h1>PedSimCity &nbsp;·&nbsp; " + city + " &nbsp;|&nbsp; Job " + job + "</h1>\n"
        + "  <div class=\"metrics\">\n"
        + "    <div class=\"card\"><div class=\"label\">Active Agents</div><div class=\"value\" id=\"m-active\">—</div></div>\n"
        + "    <div class=\"card\"><div class=\"label\">% Vulnerable</div><div class=\"value red\" id=\"m-vuln\">—</div></div>\n"
        + "    <div class=\"card\"><div class=\"label\">Peak Edge Flow</div><div class=\"value orange\" id=\"m-flow\">—</div></div>\n"
        + "    <div class=\"card\"><div class=\"label\">Sim Time</div><div class=\"value\" id=\"m-time\">—</div></div>\n"
        + "  </div>\n"
        + "</div>\n"
        + "<div id=\"container\">\n"
        + "  <canvas id=\"canvas\"></canvas>\n"
        + "  <div id=\"floating-controls\">\n"
        + "    <button class=\"btn-float\" id=\"reset-btn\">Reset Zoom</button>\n"
        + "  </div>\n"
        + "</div>\n"
        + "<div id=\"timeline-panel\">\n"
        + "  <div class=\"row\">\n"
        + "    <button id=\"play-btn\" title=\"Play/Pause\">▶</button>\n"
        + "    <input type=\"range\" id=\"step-slider\" min=\"0\" value=\"0\"/>\n"
        + "    <span id=\"time-label\">Day 1 00:00</span>\n"
        + "    <select id=\"speed-select\" title=\"Playback Speed\">\n"
        + "      <option value=\"0.01\">0.01x Speed</option>\n"
        + "      <option value=\"0.05\">0.05x Speed</option>\n"
        + "      <option value=\"0.1\">0.1x Speed</option>\n"
        + "      <option value=\"0.25\">0.25x Speed</option>\n"
        + "      <option value=\"0.5\">0.5x Speed</option>\n"
        + "      <option value=\"1\">1x Speed</option>\n"
        + "      <option value=\"2\" selected>2x Speed</option>\n"
        + "      <option value=\"5\">5x Speed</option>\n"
        + "      <option value=\"10\">10x Speed</option>\n"
        + "      <option value=\"20\">20x Speed</option>\n"
        + "    </select>\n"
        + "  </div>\n"
        + "  <div class=\"legend\">\n"
        + "    <div class=\"leg-item\"><div class=\"leg-dot\" style=\"background:#3b82f6\"></div>Normal Agent</div>\n"
        + "    <div class=\"leg-item\"><div class=\"leg-dot\" style=\"background:#ef4444;box-shadow:0 0 4px #ef4444\"></div>Vulnerable Agent</div>\n"
        + "    <div class=\"leg-item\"><div class=\"leg-line\" style=\"background:#1e293b\"></div>Road Segment</div>\n"
        + "    <div class=\"leg-item\"><div class=\"leg-line\" style=\"background:#fb923c\"></div>Medium Volume</div>\n"
        + "    <div class=\"leg-item\"><div class=\"leg-line\" style=\"background:#ef4444\"></div>High Volume</div>\n"
        + "  </div>\n"
        + "</div>\n"
        + "<script>\n"
        + "const ROADS_GEOJSON = " + roadsGeoJson + ";\n"
        + "const TRIPS = " + tripsJs + ";\n"
        // Parse bounding box dynamically
        + "let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;\n"
        + "function getPoints(geom) {\n"
        + "  let pts = [];\n"
        + "  if (geom.type === 'LineString') { pts = geom.coordinates; }\n"
        + "  else if (geom.type === 'MultiLineString') {\n"
        + "    geom.coordinates.forEach(line => { pts = pts.concat(line); });\n"
        + "  }\n"
        + "  return pts;\n"
        + "}\n"
        + "ROADS_GEOJSON.features.forEach(f => {\n"
        + "  if (f.geometry) {\n"
        + "    getPoints(f.geometry).forEach(pt => {\n"
        + "      const x = pt[0], y = pt[1];\n"
        + "      if (x < minX) minX = x;\n"
        + "      if (x > maxX) maxX = x;\n"
        + "      if (y < minY) minY = y;\n"
        + "      if (y > maxY) maxY = y;\n"
        + "    });\n"
        + "  }\n"
        + "});\n"
        + "if (minX === Infinity) { minX = 0; maxX = 1000; minY = 0; maxY = 1000; }\n"
        // Compute minStep and maxStep; pre-compute segment lengths for each trip (stored as t[5])
        + "let minStep = 0;\n"
        + "let maxStep = 0;\n"
        + "TRIPS.forEach(t => {\n"
        + "  if (t[2] > maxStep) maxStep = t[2];\n"
        + "  const coords = t[3];\n"
        + "  const segs = [];\n"
        + "  let tot = 0;\n"
        + "  for (let i = 0; i < coords.length - 1; i++) {\n"
        + "    const dx = coords[i+1][0]-coords[i][0], dy = coords[i+1][1]-coords[i][1];\n"
        + "    const l = Math.sqrt(dx*dx+dy*dy);\n"
        + "    segs.push(l); tot += l;\n"
        + "  }\n"
        + "  t[5] = segs; t[6] = tot;\n"
        + "});\n"
        + "if (maxStep === 0) maxStep = 72;\n"
        // Step → time format
        + "function stepToTime(s) {\n"
        + "  const totalMins = s * 20;\n"
        + "  const day = Math.floor(totalMins / (24 * 60)) + 1;\n"
        + "  const remainingMins = totalMins % (24 * 60);\n"
        + "  const h = Math.floor(remainingMins / 60);\n"
        + "  const m = remainingMins % 60;\n"
        + "  return 'Day ' + day + ' ' + String(h).padStart(2,'0') + ':' + String(m).padStart(2,'0');\n"
        + "}\n"
        // Traffic volume mapping
        + "const maxVol = (()=>{ let mx=0; ROADS_GEOJSON.features.forEach(f=>{ const v=f.properties.volume||0; if(v>mx) mx=v; }); return mx||1; })();\n"
        + "function getVolColor(v) {\n"
        + "  if (v < 1) return '#161c2e';\n" // Visible but quiet dark slate blue for unused roads
        + "  const t = Math.min(Math.sqrt(v) / Math.sqrt(maxVol), 1);\n"
        + "  // v >= 1 starts at Sky Blue: rgb(56, 189, 248)\n"
        + "  // Max volume is Fire Red: rgb(239, 68, 68)\n"
        + "  const r = Math.round(56 + t * (239 - 56));\n"
        + "  const g = Math.round(189 + t * (68 - 189));\n"
        + "  const b = Math.round(248 + t * (68 - 248));\n"
        + "  return `rgb(${r},${g},${b})`;\n"
        + "}\n"
        + "function getVolWeight(v) {\n"
        + "  if (v < 1) return 2.0;\n" // Thicker unused roads so they are visible but quiet
        + "  return 3.5 + Math.min(Math.sqrt(v) / Math.sqrt(maxVol), 1) * 6.5;\n" // Thicker used routes (3.5px to 10px) to make the volume colors clearer
        + "}\n"
        // Init Canvas
        + "const canvas = document.getElementById('canvas');\n"
        + "const ctx = canvas.getContext('2d');\n"
        + "let currentFloatStep = minStep;\n"
        // Interactive state (Scale & Pan)
        + "let scale = 1.0;\n"
        + "let panX = 0;\n"
        + "let panY = 0;\n"
        // Resize listener
        + "function resize() {\n"
        + "  canvas.width = canvas.clientWidth;\n"
        + "  canvas.height = canvas.clientHeight;\n"
        + "  draw();\n"
        + "}\n"
        + "window.addEventListener('resize', resize);\n"
        // Transform helpers
        + "function toScreen(wx, wy) {\n"
        + "  const sx = (wx - minX) * scale + panX;\n"
        + "  const sy = canvas.height - ((wy - minY) * scale + panY);\n"
        + "  return { x: sx, y: sy };\n"
        + "}\n"
        + "function resetView() {\n"
        + "  const dx = maxX - minX;\n"
        + "  const dy = maxY - minY;\n"
        + "  const pad = 40;\n"
        + "  const scaleX = (canvas.width - pad*2) / (dx || 1);\n"
        + "  const scaleY = (canvas.height - pad*2) / (dy || 1);\n"
        + "  scale = Math.min(scaleX, scaleY);\n"
        + "  const cx = minX + dx / 2;\n"
        + "  const cy = minY + dy / 2;\n"
        + "  panX = canvas.width / 2 - (cx - minX) * scale;\n"
        + "  panY = canvas.height / 2 - (cy - minY) * scale;\n"
        + "  draw();\n"
        + "}\n"
        // Point along path solver — uses pre-computed lengths from t[5]/t[6]
        + "function getPointAlongPath(coords, segs, totalLength, progress) {\n"
        + "  if (!coords || coords.length === 0) return null;\n"
        + "  if (progress <= 0) return { x: coords[0][0], y: coords[0][1] };\n"
        + "  if (progress >= 1) return { x: coords[coords.length-1][0], y: coords[coords.length-1][1] };\n"
        + "  if (totalLength === 0) return { x: coords[0][0], y: coords[0][1] };\n"
        + "  const targetDist = progress * totalLength;\n"
        + "  let acc = 0;\n"
        + "  for (let i = 0; i < segs.length; i++) {\n"
        + "    const l = segs[i];\n"
        + "    if (acc + l >= targetDist) {\n"
        + "      const f = (targetDist - acc) / (l || 1);\n"
        + "      return { x: coords[i][0]+(coords[i+1][0]-coords[i][0])*f, y: coords[i][1]+(coords[i+1][1]-coords[i][1])*f };\n"
        + "    }\n"
        + "    acc += l;\n"
        + "  }\n"
        + "  return { x: coords[coords.length-1][0], y: coords[coords.length-1][1] };\n"
        + "}\n"
        // Draw trail using pre-computed segment data — single stroke only
        + "function drawAgentTrail(coords, segs, totalLength, progress, color) {\n"
        + "  if (!coords || coords.length < 2 || totalLength === 0) return;\n"
        + "  const targetDist = progress * totalLength;\n"
        + "  ctx.beginPath();\n"
        + "  ctx.moveTo(...Object.values(toScreen(coords[0][0], coords[0][1])));\n"
        + "  let acc = 0;\n"
        + "  for (let i = 0; i < segs.length; i++) {\n"
        + "    const l = segs[i];\n"
        + "    if (acc + l >= targetDist) {\n"
        + "      const f = (targetDist - acc) / (l || 1);\n"
        + "      const cx = coords[i][0]+(coords[i+1][0]-coords[i][0])*f;\n"
        + "      const cy = coords[i][1]+(coords[i+1][1]-coords[i][1])*f;\n"
        + "      const ps = toScreen(cx, cy); ctx.lineTo(ps.x, ps.y);\n"
        + "      break;\n"
        + "    }\n"
        + "    const ps = toScreen(coords[i+1][0], coords[i+1][1]); ctx.lineTo(ps.x, ps.y);\n"
        + "    acc += l;\n"
        + "  }\n"
        + "  ctx.strokeStyle = color === '#ef4444' ? 'rgba(239,68,68,0.7)' : 'rgba(56,189,248,0.7)';\n"
        + "  ctx.lineWidth = 3.5 * scale;\n"
        + "  ctx.lineCap = 'round'; ctx.lineJoin = 'round';\n"
        + "  ctx.stroke();\n"
        + "}\n"
        // Get active trips + interpolated position in one pass — reused for trails AND dots
        + "function getActiveTrips(floatStep) {\n"
        + "  const active = [];\n"
        + "  for (let i = 0; i < TRIPS.length; i++) {\n"
        + "    const t = TRIPS[i];\n"
        + "    if (floatStep < t[1] || floatStep > t[2]) continue;\n"
        + "    const duration = t[2] - t[1];\n"
        + "    const progress = duration > 0 ? (floatStep - t[1]) / duration : 1.0;\n"
        + "    const pt = getPointAlongPath(t[3], t[5], t[6], progress);\n"
        + "    if (pt) active.push({ coords: t[3], segs: t[5], totalLen: t[6], progress, vuln: t[4] ? 1 : 0, x: pt.x, y: pt.y });\n"
        + "  }\n"
        + "  return active;\n"
        + "}\n"
        // Offscreen canvas for the static road layer — only redrawn on pan/zoom/resize
        + "const offscreen = document.createElement('canvas');\n"
        + "const offCtx = offscreen.getContext('2d');\n"
        + "let roadsDirty = true;\n"
        + "function buildRoadLayer() {\n"
        + "  offscreen.width = canvas.width; offscreen.height = canvas.height;\n"
        + "  offCtx.clearRect(0, 0, offscreen.width, offscreen.height);\n"
        + "  ROADS_GEOJSON.features.forEach(f => {\n"
        + "    if (!f.geometry) return;\n"
        + "    const pts = getPoints(f.geometry);\n"
        + "    if (pts.length < 2) return;\n"
        + "    const vol = f.properties.volume || 0;\n"
        + "    offCtx.strokeStyle = getVolColor(vol);\n"
        + "    offCtx.lineWidth = getVolWeight(vol);\n"
        + "    offCtx.lineCap = 'round'; offCtx.lineJoin = 'round';\n"
        + "    offCtx.beginPath();\n"
        + "    const p0 = toScreen(pts[0][0], pts[0][1]);\n"
        + "    offCtx.moveTo(p0.x, p0.y);\n"
        + "    for (let i = 1; i < pts.length; i++) {\n"
        + "      const pi = toScreen(pts[i][0], pts[i][1]);\n"
        + "      offCtx.lineTo(pi.x, pi.y);\n"
        + "    }\n"
        + "    offCtx.stroke();\n"
        + "  });\n"
        + "  roadsDirty = false;\n"
        + "}\n"
        // Main render — composites cached roads then draws moving elements only
        + "function draw() {\n"
        + "  if (roadsDirty) buildRoadLayer();\n"
        + "  ctx.clearRect(0, 0, canvas.width, canvas.height);\n"
        + "  ctx.drawImage(offscreen, 0, 0);\n"
        // Draw active agent trails then dots — one pass
        + "  const active = getActiveTrips(currentFloatStep);\n"
        + "  for (let i = 0; i < active.length; i++) {\n"
        + "    const a = active[i];\n"
        + "    drawAgentTrail(a.coords, a.segs, a.totalLen, a.progress, a.vuln ? '#ef4444' : '#38bdf8');\n"
        + "  }\n"
        + "  for (let i = 0; i < active.length; i++) {\n"
        + "    const a = active[i];\n"
        + "    const p = toScreen(a.x, a.y);\n"
        + "    ctx.beginPath();\n"
        + "    ctx.arc(p.x, p.y, a.vuln ? 7 : 5, 0, Math.PI * 2);\n"
        + "    ctx.fillStyle = a.vuln ? '#ef4444' : '#38bdf8';\n"
        + "    ctx.fill();\n"
        + "  }\n"
        + "}\n"
        // Interaction Handlers (Pan & Zoom)
        + "let isDragging = false, startX = 0, startY = 0;\n"
        + "canvas.addEventListener('mousedown', e => {\n"
        + "  isDragging = true;\n"
        + "  startX = e.clientX;\n"
        + "  startY = e.clientY;\n"
        + "});\n"
        + "window.addEventListener('mousemove', e => {\n"
        + "  if (!isDragging) return;\n"
        + "  const dx = e.clientX - startX;\n"
        + "  const dy = e.clientY - startY;\n"
        + "  panX += dx; panY -= dy;\n"
        + "  startX = e.clientX; startY = e.clientY;\n"
        + "  roadsDirty = true;\n"
        + "  draw();\n"
        + "});\n"
        + "window.addEventListener('mouseup', () => { isDragging = false; });\n"
        // Zoom-to-mouse-pointer
        + "canvas.addEventListener('wheel', e => {\n"
        + "  e.preventDefault();\n"
        + "  const rect = canvas.getBoundingClientRect();\n"
        + "  const mouseX = e.clientX - rect.left;\n"
        + "  const mouseY = e.clientY - rect.top;\n"
        + "  const worldX = (mouseX - panX) / scale + minX;\n"
        + "  const worldY = (canvas.height - mouseY - panY) / scale + minY;\n"
        + "  const zoomFactor = 1.15;\n"
        + "  if (e.deltaY < 0) { scale *= zoomFactor; } else { scale /= zoomFactor; }\n"
        + "  panX = mouseX - (worldX - minX) * scale;\n"
        + "  panY = (canvas.height - mouseY) - (worldY - minY) * scale;\n"
        + "  roadsDirty = true;\n"
        + "  draw();\n"
        + "});\n"
        + "document.getElementById('reset-btn').addEventListener('click', resetView);\n"
        // Mark road layer dirty whenever pan/zoom changes so it gets rebuilt
        + "function invalidateRoads() { roadsDirty = true; }\n"
        // Metrics update — reuse getActiveTrips so no second full scan
        + "function updateMetrics(step) {\n"
        + "  const active = getActiveTrips(step);\n"
        + "  const total = active.length;\n"
        + "  const vuln = active.filter(r=>r.vuln===1).length;\n"
        + "  document.getElementById('m-active').textContent = total;\n"
        + "  document.getElementById('m-vuln').textContent = total > 0 ? (vuln/total*100).toFixed(1)+'%' : '0%';\n"
        + "  document.getElementById('m-flow').textContent = maxVol;\n"
        + "  document.getElementById('m-time').textContent = stepToTime(step);\n"
        + "  document.getElementById('time-label').textContent = stepToTime(step);\n"
        + "}\n"
        // Slider controls
        + "const slider = document.getElementById('step-slider');\n"
        + "slider.min = 0;\n"
        + "slider.max = maxStep * 10;\n"
        + "slider.value = 0;\n"
        + "let sliderFloatVal = 0;\n"
        + "slider.addEventListener('input', () => {\n"
        + "  const val = parseInt(slider.value);\n"
        + "  sliderFloatVal = val;\n"
        + "  currentFloatStep = val / 10;\n"
        + "  draw();\n"
        + "  updateMetrics(currentFloatStep);\n"
        + "});\n"
        // Animation loop
        + "let playing = false, animId = null, lastTs = 0;\n"
        + "const playBtn = document.getElementById('play-btn');\n"
        + "const speedSelect = document.getElementById('speed-select');\n"
        + "playBtn.addEventListener('click', () => {\n"
        + "  playing = !playing;\n"
        + "  playBtn.textContent = playing ? '⏸' : '▶';\n"
        + "  if (playing) animate();\n"
        + "  else cancelAnimationFrame(animId);\n"
        + "});\n"
        + "function animate(ts) {\n"
        + "  if (!playing) return;\n"
        + "  if (ts - lastTs > 30) {\n"
        + "    lastTs = ts;\n"
        + "    const mult = parseFloat(speedSelect.value) || 2;\n"
        + "    const maxVal = maxStep * 10;\n"
        + "    sliderFloatVal = (sliderFloatVal + mult) % (maxVal + 1);\n"
        + "    slider.value = Math.floor(sliderFloatVal);\n"
        + "    currentFloatStep = sliderFloatVal / 10;\n"
        + "    draw();\n"
        + "    updateMetrics(currentFloatStep);\n"
        + "  }\n"
        + "  animId = requestAnimationFrame(animate);\n"
        + "}\n"
        // Initial setup — mark roads dirty on resize too
        + "function resize() { canvas.width = canvas.clientWidth; canvas.height = canvas.clientHeight; roadsDirty = true; draw(); }\n"
        + "window.addEventListener('resize', resize);\n"
        + "canvas.width = canvas.clientWidth;\n"
        + "canvas.height = canvas.clientHeight;\n"
        + "resetView();\n"
        + "updateMetrics(currentFloatStep);\n"
        + "</script>\n"
        + "</body>\n"
        + "</html>\n";
  }
}
