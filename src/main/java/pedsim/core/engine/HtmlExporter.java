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
        + "  .card .value.yellow{color:#fde047}\n"
        + "  #container{position:relative;flex:1;width:100%;overflow:hidden;background:#000000}\n"
        + "  #canvas{width:100%;height:100%;display:block;cursor:grab}\n"
        + "  #canvas:active{cursor:grabbing}\n"
        + "  #floating-controls{position:absolute;top:20px;right:20px;display:flex;flex-direction:column;gap:8px}\n"
        + "  .btn-float{background:rgba(30,41,59,0.85);border:1px solid #334155;border-radius:8px;color:#f1f5f9;padding:8px 14px;font-size:.75rem;font-weight:600;cursor:pointer;backdrop-filter:blur(4px);transition:all .2s;display:flex;align-items:center;gap:6px}\n"
        + "  .btn-float:hover{background:#38bdf8;color:#0b0f19;border-color:#38bdf8}\n"
        + "  .toggle-label{display:flex;align-items:center;gap:8px;font-size:0.8rem;background:rgba(30,41,59,0.85);padding:8px 14px;border-radius:8px;border:1px solid #334155;backdrop-filter:blur(4px);cursor:pointer;color:#f1f5f9;font-weight:600;}\n"
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
        + "  <h1>PedSimCity &nbsp;-&nbsp; " + city + " &nbsp;|&nbsp; Job " + job + "</h1>\n"
        + "  <div class=\"metrics\">\n"
        + "    <div class=\"card\"><div class=\"label\">Active Agents</div><div class=\"value\" id=\"m-active\">-</div></div>\n"
        + "    <div class=\"card\"><div class=\"label\">% Vulnerable</div><div class=\"value red\" id=\"m-vuln\">-</div></div>\n"
        + "    <div class=\"card\"><div class=\"label\">Avg Live Lux</div><div class=\"value yellow\" id=\"m-lux\">-</div></div>\n"
        + "    <div class=\"card\"><div class=\"label\">Sim Time</div><div class=\"value\" id=\"m-time\">-</div></div>\n"
        + "  </div>\n"
        + "</div>\n"
        + "<div id=\"container\">\n"
        + "  <canvas id=\"canvas\"></canvas>\n"
        + "  <div id=\"floating-controls\">\n"
        + "    <label class=\"toggle-label\"><input type=\"checkbox\" id=\"tg-light\" /> Light Level Map</label>\n"
        + "    <label class=\"toggle-label\"><input type=\"checkbox\" id=\"tg-spooks\" checked /> Show Spooks</label>\n"
        + "    <label class=\"toggle-label\"><input type=\"checkbox\" id=\"tg-tethers\" checked /> Show A/B Tethers</label>\n"
        + "    <button class=\"btn-float\" id=\"reset-btn\">Reset Zoom</button>\n"
        + "  </div>\n"
        + "</div>\n"
        + "<div id=\"timeline-panel\">\n"
        + "  <div class=\"row\">\n"
        + "    <button id=\"play-btn\" title=\"Play/Pause\">Play</button>\n"
        + "    <input type=\"range\" id=\"step-slider\" min=\"0\" value=\"0\"/>\n"
        + "    <span id=\"time-label\">Day 1 00:00</span>\n"
        + "    <select id=\"speed-select\" title=\"Playback Speed\">\n"
        + "      <option value=\"0.1\">0.1x Speed</option>\n"
        + "      <option value=\"0.5\">0.5x Speed</option>\n"
        + "      <option value=\"1\">1x Speed</option>\n"
        + "      <option value=\"2\" selected>2x Speed</option>\n"
        + "      <option value=\"5\">5x Speed</option>\n"
        + "      <option value=\"10\">10x Speed</option>\n"
        + "    </select>\n"
        + "  </div>\n"
        + "</div>\n"
        + "<script>\n"
        + "const ROADS_GEOJSON = " + roadsGeoJson + ";\n"
        + "const TRIPS = " + tripsJs + ";\n"
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
        + "let maxStep = 0;\n"
        + "TRIPS.forEach(t => {\n"
        + "  if (t[2] > maxStep) maxStep = t[2];\n"
        + "  const coords = t[3]; const segs = []; let tot = 0;\n"
        + "  for (let i = 0; i < coords.length - 1; i++) {\n"
        + "    const dx = coords[i+1][0]-coords[i][0], dy = coords[i+1][1]-coords[i][1];\n"
        + "    const l = Math.sqrt(dx*dx+dy*dy); segs.push(l); tot += l;\n"
        + "  }\n"
        + "  t[6] = segs; t[7] = tot;\n"
        + "});\n"
        + "TRIPS.sort((a, b) => a[1] - b[1]);\n"
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
        + "  const r = Math.round(56 + t * (239 - 56));\n"
        + "  const g = Math.round(189 + t * (68 - 189));\n"
        + "  const b = Math.round(248 + t * (68 - 248));\n"
        + "  return `rgb(${r},${g},${b})`;\n"
        + "}\n"
        + "function getVolWeight(f) { const v = f.properties.volume || 0; if (v < 1) return 2.0; return 3.5 + Math.min(Math.sqrt(v) / Math.sqrt(maxVol), 1) * 6.5; }\n"
        + "const canvas = document.getElementById('canvas'); const ctx = canvas.getContext('2d');\n"
        + "let currentFloatStep = 0, scale = 1.0, panX = 0, panY = 0;\n"
        + "function toScreen(wx, wy) { return { x: (wx - minX) * scale + panX, y: canvas.height - ((wy - minY) * scale + panY) }; }\n"
        + "function resetView() {\n"
        + "  const dx = maxX - minX, dy = maxY - minY, pad = 40;\n"
        + "  scale = Math.min((canvas.width - pad*2) / (dx || 1), (canvas.height - pad*2) / (dy || 1));\n"
        + "  panX = canvas.width / 2 - (minX + dx / 2 - minX) * scale; panY = canvas.height / 2 - (minY + dy / 2 - minY) * scale;\n"
        + "  roadsDirty = true; draw();\n"
        + "}\n"
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
        + "    if (pt) res.push({ id: t[0], vuln: t[4], x: pt.x, y: pt.y, spooks: t[5], progress });\n"
        + "  }\n"
        + "  return res;\n"
        + "}\n"
        + "const offscreen = document.createElement('canvas'); const offCtx = offscreen.getContext('2d'); let roadsDirty = true;\n"
        + "const tgLight = document.getElementById('tg-light');\n"
        + "const tgSpooks = document.getElementById('tg-spooks');\n"
        + "const tgTethers = document.getElementById('tg-tethers');\n"
        + "tgLight.addEventListener('change', () => { roadsDirty = true; draw(); });\n"
        + "tgSpooks.addEventListener('change', () => { draw(); });\n"
        + "tgTethers.addEventListener('change', () => { draw(); });\n"
        + "function buildRoadLayer() {\n"
        + "  offscreen.width = canvas.width; offscreen.height = canvas.height; offCtx.clearRect(0, 0, offscreen.width, offscreen.height);\n"
        + "  const useLight = tgLight.checked;\n"
        + "  ROADS_GEOJSON.features.forEach(f => {\n"
        + "    if (!f.geometry) return; const pts = getPoints(f.geometry); if (pts.length < 2) return;\n"
        + "    offCtx.strokeStyle = getVolColor(f, useLight); offCtx.lineWidth = getVolWeight(f);\n"
        + "    offCtx.lineCap = 'round'; offCtx.lineJoin = 'round'; offCtx.beginPath();\n"
        + "    const p0 = toScreen(pts[0][0], pts[0][1]); offCtx.moveTo(p0.x, p0.y);\n"
        + "    for (let i = 1; i < pts.length; i++) { const pi = toScreen(pts[i][0], pts[i][1]); offCtx.lineTo(pi.x, pi.y); }\n"
        + "    offCtx.stroke();\n"
        + "  });\n"
        + "  roadsDirty = false;\n"
        + "}\n"
        + "const activeSpooks = [];\n"
        + "function draw() {\n"
        + "  if (roadsDirty) buildRoadLayer();\n"
        + "  ctx.clearRect(0, 0, canvas.width, canvas.height); ctx.drawImage(offscreen, 0, 0);\n"
        + "  const agents = getLiveAgents(currentFloatStep);\n"
        + "  const posById = {};\n"
        + "  agents.forEach(a => { posById[a.id] = toScreen(a.x, a.y); });\n"
        + "  if (tgTethers.checked) {\n"
        + "    ctx.strokeStyle = 'rgba(255, 255, 255, 0.4)'; ctx.lineWidth = 1.5; ctx.setLineDash([4, 4]);\n"
        + "    ctx.beginPath();\n"
        + "    agents.forEach(a => {\n"
        + "      if (a.vuln && posById[a.id - 1]) {\n"
        + "        const p1 = posById[a.id], p2 = posById[a.id - 1];\n"
        + "        ctx.moveTo(p1.x, p1.y); ctx.lineTo(p2.x, p2.y);\n"
        + "      }\n"
        + "    });\n"
        + "    ctx.stroke(); ctx.setLineDash([]);\n"
        + "  }\n"
        + "  if (tgSpooks.checked) {\n"
        + "    agents.forEach(a => {\n"
        + "      a.spooks.forEach(sp => {\n"
        + "        const spDist = Math.sqrt(Math.pow(a.x - sp[0], 2) + Math.pow(a.y - sp[1], 2));\n"
        + "        if (spDist < 12) {\n"
        + "           const sps = toScreen(sp[0], sp[1]);\n"
        + "           ctx.beginPath(); ctx.arc(sps.x, sps.y, 14, 0, Math.PI*2);\n"
        + "           ctx.fillStyle = 'rgba(239, 68, 68, 0.5)'; ctx.fill();\n"
        + "        }\n"
        + "      });\n"
        + "    });\n"
        + "  }\n"
        + "  ctx.beginPath();\n"
        + "  agents.forEach(a => {\n"
        + "    const p = posById[a.id];\n"
        + "    ctx.moveTo(p.x, p.y); ctx.arc(p.x, p.y, a.vuln ? 7 : 5, 0, Math.PI * 2);\n"
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
        + "  scale *= e.deltaY < 0 ? 1.15 : 1/1.15;\n"
        + "  panX = mouseX - (worldX - minX) * scale; panY = (canvas.height - mouseY) - (worldY - minY) * scale;\n"
        + "  roadsDirty = true; draw();\n"
        + "});\n"
        + "document.getElementById('reset-btn').addEventListener('click', resetView);\n"
        + "function updateMetrics(step) {\n"
        + "  const total = activeList.length, vuln = activeList.filter(t=>t[4]).length;\n"
        + "  document.getElementById('m-active').textContent = total;\n"
        + "  document.getElementById('m-vuln').textContent = total > 0 ? (vuln/total*100).toFixed(1)+'%' : '0%';\n"
        + "  document.getElementById('m-time').textContent = 'Day 1 ' + String(Math.floor(step*20/60)%24).padStart(2,'0') + ':' + String(Math.floor(step*20)%60).padStart(2,'0');\n"
        + "  document.getElementById('time-label').textContent = document.getElementById('m-time').textContent;\n"
        // Try to map agent to street lux
        + "  if (tgLight.checked && total > 0) {\n"
        + "    document.getElementById('m-lux').textContent = 'Simulating...';\n"
        + "  } else {\n"
        + "    document.getElementById('m-lux').textContent = '-';\n"
        + "  }\n"
        + "}\n"
        + "const slider = document.getElementById('step-slider'); slider.min = 0; slider.max = maxStep * 10; slider.value = 0; let sliderFloatVal = 0;\n"
        + "slider.addEventListener('input', () => {\n"
        + "  sliderFloatVal = parseInt(slider.value); currentFloatStep = sliderFloatVal / 10;\n"
        + "  nextTripIdx = 0; activeList = []; updateActiveTrips(currentFloatStep);\n"
        + "  draw(); updateMetrics(currentFloatStep);\n"
        + "});\n"
        + "let playing = false, animId = null, lastTs = 0; const playBtn = document.getElementById('play-btn'), speedSelect = document.getElementById('speed-select');\n"
        + "playBtn.addEventListener('click', () => { playing = !playing; playBtn.textContent = playing ? 'Pause' : 'Play'; if (playing) animate(); else cancelAnimationFrame(animId); });\n"
        + "function animate(ts) {\n"
        + "  if (!playing) return;\n"
        + "  if (ts - lastTs > 30) {\n"
        + "    lastTs = ts; sliderFloatVal = (sliderFloatVal + parseFloat(speedSelect.value) || 2) % (maxStep * 10 + 1);\n"
        + "    slider.value = Math.floor(sliderFloatVal); currentFloatStep = sliderFloatVal / 10;\n"
        + "    updateActiveTrips(currentFloatStep); draw(); updateMetrics(currentFloatStep);\n"
        + "  }\n"
        + "  animId = requestAnimationFrame(animate);\n"
        + "}\n"
        + "window.addEventListener('resize', () => { canvas.width = canvas.clientWidth; canvas.height = canvas.clientHeight; roadsDirty = true; draw(); });\n"
        + "canvas.width = canvas.clientWidth; canvas.height = canvas.clientHeight; resetView(); updateMetrics(0);\n"
        + "</script>\n"
        + "</body>\n"
        + "</html>\n";
  }
}
