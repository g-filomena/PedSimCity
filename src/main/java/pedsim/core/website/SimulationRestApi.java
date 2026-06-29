package pedsim.core.website;

import java.io.IOException;
import java.net.BindException;
import java.net.InetSocketAddress;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.logging.Logger;

import com.sun.net.httpserver.HttpServer;

import pedsim.core.engine.SimulationModule;
import pedsim.core.engine.SimulationStateStore;
import pedsim.core.parameters.Pars;
import pedsim.core.utilities.LoggerUtil;
import tools.jackson.core.type.TypeReference;
import tools.jackson.databind.ObjectMapper;

/**
 * Lightweight HTTP server exposing simulation state and control to a browser dashboard.
 *
 * <p>Endpoints:
 *
 * <ul>
 *   <li>{@code GET /api/state} — live simulation state as JSON
 *   <li>{@code GET /api/roads} — GeoJSON road network (served once after pre-load)
 *   <li>{@code GET /api/modules} — registered module descriptors with parameter schemas
 *   <li>{@code POST /api/start} — start a simulation run; body may include {@code "module"} to
 *       select which registered module to use
 * </ul>
 *
 * <p>The server is idempotent: calling {@link #start} when a server is already running stops the
 * old one first. Call {@link #stop} to tear down the server and clear all registrations.
 *
 * <h2>Module registration</h2>
 *
 * <p>Only concrete runnable modules (night, cityImage, empirical) should be registered. The {@code
 * core} infrastructure module must not be registered here. {@code SimulationLauncher} handles this
 * automatically via {@link SimulationModule#isConcreteRunnable()}.
 *
 * <pre>{@code
 * SimulationRestApi.registerModule(NightSimulationModule.INSTANCE);
 * SimulationRestApi.start(8081);
 * }</pre>
 *
 * The {@code POST /api/start} body selects a module by its {@link SimulationModule#moduleId()}:
 *
 * <pre>{@code
 * {
 *   "module":           "night",
 *   "cityName":         "Torino",
 *   "days":             7,
 *   "actualPopulation": 900000,
 *   "percentage":       0.001,
 *   "jobs":             1
 * }
 * }</pre>
 *
 * If {@code "module"} is absent the first registered concrete module is used. If no concrete
 * modules are registered, {@code POST /api/start} returns {@code 400}.
 */
public final class SimulationRestApi {

  private static final Logger logger = LoggerUtil.getLogger();
  private static final int MAX_PORT_RETRIES = 10;
  private static final ObjectMapper MAPPER = new ObjectMapper();

  /** Common parameters accepted by every module (merged into the /api/modules schema). */
  private static final Map<String, Object> COMMON_PARAMS =
      Map.of(
          "module", "string",
          "cityName", "string",
          "days", "integer",
          "actualPopulation", "integer",
          "percentage", "double",
          "jobs", "integer");

  // ----------------------------------------------------------------
  // Module registry
  // ----------------------------------------------------------------

  /** Registered modules, keyed by {@link SimulationModule#moduleId()}. Insertion-ordered. */
  private static final LinkedHashMap<String, SimulationModule> modules = new LinkedHashMap<>();

  /**
   * Registers a module so it can be selected by name in {@code POST /api/start}.
   * Replaces any previously registered module with the same ID.
   */
  public static synchronized void registerModule(SimulationModule module) {
    modules.put(module.moduleId(), module);
    logger.info("[REST API] Registered simulation module: " + module.moduleId());
  }

  // ----------------------------------------------------------------
  // Legacy start-listener (kept for backward compatibility)
  // ----------------------------------------------------------------

  /**
   * Registers a legacy start listener. Prefer {@link #registerModule(SimulationModule)} instead.
   * Used only when no modules are registered and a raw callback is needed (e.g. custom tooling).
   */
  public static void setOnStart(Consumer<Map<String, Object>> listener) {
    legacyStartListener = listener;
  }

  private static Consumer<Map<String, Object>> legacyStartListener;

  // ----------------------------------------------------------------
  // Server lifecycle
  // ----------------------------------------------------------------

  private static HttpServer activeServer;
  private static int boundPort = -1;

  /** Stops the running server (if any) and clears all module registrations and listeners. */
  public static synchronized void stop() {
    if (activeServer != null) {
      activeServer.stop(0);
      activeServer = null;
      boundPort = -1;
    }
    legacyStartListener = null;
    modules.clear();
  }

  /**
   * Starts the REST API server on {@code preferredPort}, automatically retrying on up to {@value
   * #MAX_PORT_RETRIES} subsequent ports if the preferred one is occupied. If a server is already
   * running it is stopped first. If all ports are occupied a warning is logged and the method
   * returns without throwing.
   */
  public static synchronized void start(int preferredPort) {
    if (activeServer != null) {
      activeServer.stop(0);
      activeServer = null;
      boundPort = -1;
    }

    for (int attempt = 0; attempt < MAX_PORT_RETRIES; attempt++) {
      int port = preferredPort + attempt;
      try {
        HttpServer server = HttpServer.create(new InetSocketAddress(port), 0);

        // --- GET /api/state ---
        server.createContext(
            "/api/state",
            exchange -> {
              try {
                if (!"GET".equalsIgnoreCase(exchange.getRequestMethod())) {
                  exchange.sendResponseHeaders(405, -1);
                  return;
                }
                byte[] body =
                    SimulationStateStore.getInstance().toJson().getBytes(StandardCharsets.UTF_8);
                exchange.getResponseHeaders().add("Content-Type", "application/json");
                exchange.getResponseHeaders().add("Access-Control-Allow-Origin", "*");
                exchange.sendResponseHeaders(200, body.length);
                try (var out = exchange.getResponseBody()) {
                  out.write(body);
                }
              } catch (Exception ex) {
                exchange.sendResponseHeaders(500, -1);
              }
            });

        // --- GET / (serve dashboard.html) ---
        server.createContext(
            "/",
            exchange -> {
              try {
                if (!"/".equals(exchange.getRequestURI().getPath())
                    && !"/dashboard.html".equals(exchange.getRequestURI().getPath())) {
                  exchange.sendResponseHeaders(404, -1);
                  return;
                }
                java.io.File htmlFile = new java.io.File("dashboard.html");
                if (!htmlFile.exists()) {
                  byte[] err = "dashboard.html not found".getBytes(StandardCharsets.UTF_8);
                  exchange.sendResponseHeaders(404, err.length);
                  try (var out = exchange.getResponseBody()) { out.write(err); }
                  return;
                }
                byte[] body = java.nio.file.Files.readAllBytes(htmlFile.toPath());
                exchange.getResponseHeaders().add("Content-Type", "text/html; charset=UTF-8");
                exchange.getResponseHeaders().add("Cache-Control", "no-cache");
                exchange.sendResponseHeaders(200, body.length);
                try (var out = exchange.getResponseBody()) { out.write(body); }
              } catch (Exception ex) {
                exchange.sendResponseHeaders(500, -1);
              }
            });

        // --- GET /api/roads ---
        server.createContext(
            "/api/roads",
            exchange -> {
              try {
                if (!"GET".equalsIgnoreCase(exchange.getRequestMethod())) {
                  exchange.sendResponseHeaders(405, -1);
                  return;
                }
                String roads = SimulationStateStore.getInstance().roadsGeoJson;
                if (roads == null) roads = "{\"type\":\"FeatureCollection\",\"features\":[]}";
                byte[] body = roads.getBytes(StandardCharsets.UTF_8);
                exchange.getResponseHeaders().add("Content-Type", "application/json");
                exchange.getResponseHeaders().add("Access-Control-Allow-Origin", "*");
                exchange.sendResponseHeaders(200, body.length);
                try (var out = exchange.getResponseBody()) {
                  out.write(body);
                }
              } catch (Exception ex) {
                exchange.sendResponseHeaders(500, -1);
              }
            });

        // --- GET /api/modules ---
        server.createContext(
            "/api/modules",
            exchange -> {
              try {
                if (!"GET".equalsIgnoreCase(exchange.getRequestMethod())) {
                  exchange.sendResponseHeaders(405, -1);
                  return;
                }
                byte[] body = buildModulesJson().getBytes(StandardCharsets.UTF_8);
                exchange.getResponseHeaders().add("Content-Type", "application/json");
                exchange.getResponseHeaders().add("Access-Control-Allow-Origin", "*");
                exchange.sendResponseHeaders(200, body.length);
                try (var out = exchange.getResponseBody()) {
                  out.write(body);
                }
              } catch (Exception ex) {
                exchange.sendResponseHeaders(500, -1);
              }
            });

        // --- POST /api/start ---
        server.createContext(
            "/api/start",
            exchange -> {
              try {
                exchange.getResponseHeaders().add("Access-Control-Allow-Origin", "*");
                exchange
                    .getResponseHeaders()
                    .add("Access-Control-Allow-Methods", "POST, GET, OPTIONS");
                exchange.getResponseHeaders().add("Access-Control-Allow-Headers", "Content-Type");

                if ("OPTIONS".equalsIgnoreCase(exchange.getRequestMethod())) {
                  exchange.sendResponseHeaders(204, -1);
                  return;
                }
                if (!"POST".equalsIgnoreCase(exchange.getRequestMethod())) {
                  exchange.sendResponseHeaders(405, -1);
                  return;
                }

                Map<String, Object> params = new HashMap<>();
                byte[] rawBody = exchange.getRequestBody().readAllBytes();
                if (rawBody.length > 0) {
                  try {
                    params = MAPPER.readValue(rawBody, new TypeReference<Map<String, Object>>() {});
                  } catch (Exception e) {
                    logger.warning("POST /api/start: malformed JSON body: " + e.getMessage());
                    byte[] err = "Invalid JSON body".getBytes(StandardCharsets.UTF_8);
                    exchange.getResponseHeaders().add("Content-Type", "text/plain");
                    exchange.sendResponseHeaders(400, err.length);
                    try (var out = exchange.getResponseBody()) {
                      out.write(err);
                    }
                    return;
                  }
                }

                // Resolve module under the class lock so the LinkedHashMap is not raced.
                SimulationModule selectedModule = null;
                String moduleError = null;
                synchronized (SimulationRestApi.class) {
                  if (params.containsKey("module")) {
                    String requestedId = params.get("module").toString();
                    selectedModule = modules.get(requestedId);
                    if (selectedModule == null) {
                      // Specified but unknown → 400, not a fallback
                      moduleError =
                          "Unknown module: \""
                              + requestedId
                              + "\". Registered modules: "
                              + modules.keySet();
                    }
                  } else if (!modules.isEmpty()) {
                    // No module key → default to first registered
                    selectedModule = modules.values().iterator().next();
                  }
                }

                if (moduleError != null) {
                  byte[] err = moduleError.getBytes(StandardCharsets.UTF_8);
                  exchange.getResponseHeaders().add("Content-Type", "text/plain");
                  exchange.sendResponseHeaders(400, err.length);
                  try (var out = exchange.getResponseBody()) {
                    out.write(err);
                  }
                  return;
                }

                if (selectedModule != null) {
                  final SimulationModule mod = selectedModule;
                  final Map<String, Object> finalParams = params;
                  new Thread(() -> runModuleSimulation(mod, finalParams), "pedsim-rest-simulation")
                      .start();
                  logger.info(
                      "[REST API] Simulation start triggered — module="
                          + mod.moduleId()
                          + " params="
                          + params);
                  exchange.sendResponseHeaders(200, 0);

                } else if (legacyStartListener != null) {
                  // Legacy path: raw Consumer<Map> registered without a module
                  final Map<String, Object> finalParams = params;
                  new Thread(() -> legacyStartListener.accept(finalParams)).start();
                  logger.info("[REST API] Simulation start triggered (legacy listener).");
                  exchange.sendResponseHeaders(200, 0);

                } else {
                  byte[] err =
                      "No runnable simulation modules registered. Start a module-specific server (e.g. exec:java@night-website)."
                          .getBytes(StandardCharsets.UTF_8);
                  exchange.getResponseHeaders().add("Content-Type", "text/plain");
                  exchange.sendResponseHeaders(400, err.length);
                  try (var out = exchange.getResponseBody()) {
                    out.write(err);
                  }
                  return;
                }

                exchange.getResponseBody().close();
              } catch (Exception ex) {
                exchange.sendResponseHeaders(500, -1);
              }
            });

        server.setExecutor(null);
        server.start();
        activeServer = server;
        boundPort = port;
        logger.info("[REST API] Server started on port " + port);
        return;

      } catch (BindException be) {
        logger.warning("[REST API] Port " + port + " in use, trying " + (port + 1) + "…");
      } catch (IOException e) {
        logger.warning(
            "[REST API] Could not start on port "
                + port
                + ": "
                + e.getMessage()
                + " — dashboard unavailable.");
        return;
      }
    }

    logger.warning(
        "[REST API] Could not bind to any port in range "
            + preferredPort
            + "–"
            + (preferredPort + MAX_PORT_RETRIES - 1)
            + ". Dashboard unavailable.");
  }

  /**
   * Attempts to open the dashboard.html file in the system's default browser.
   */
  public static void openDashboardInBrowser() {
    try {
      if (java.awt.Desktop.isDesktopSupported() && java.awt.Desktop.getDesktop().isSupported(java.awt.Desktop.Action.BROWSE)) {
        java.awt.Desktop.getDesktop().browse(new java.net.URI("http://localhost:" + boundPort));
      }
    } catch (Exception e) {
      Logger logger = LoggerUtil.getLogger();
      logger.warning("[REST API] Could not open browser automatically: " + e.getMessage());
    }
  }

  // ----------------------------------------------------------------
  // Internal helpers
  // ----------------------------------------------------------------

  /**
   * Runs a simulation for the given module on the calling thread (intended to be a dedicated
   * background thread so the HTTP handler returns immediately).
   *
   * <p>Ordering contract — must not be changed:
   *
   * <ol>
   *   <li>{@code applyMode()} — sets {@code Pars.isNight} and any other mode flags
   *   <li>{@code applyCommonParams()} / {@code module.applyParameters()} — writes Pars fields
   *   <li>{@code Engine.runJobs()} — internally calls {@code clearStaticData()},
   *       {@code Pars.setSimulationParameters()}, {@code Import.importFiles()}, then jobs
   * </ol>
   *
   * <p>Steps 1–2 must complete before step 3, because {@code setSimulationParameters()} and
   * {@code importFiles()} inside {@code runJobs()} read {@code Pars.isNight} and other fields set
   * in steps 1–2.
   */
  private static void runModuleSimulation(SimulationModule module, Map<String, Object> params) {
    try {
      // 1. Mode flags first (sets Pars.isNight etc.)
      module.applyMode();
      SimulationStateStore.getInstance().setActiveModule(module);

      // 2. Parameters (common Pars fields, then module-specific)
      applyCommonParams(params);
      module.applyParameters(params);

      // 3. Engine lifecycle (clear → setSimulationParameters → importFiles → jobs)
      module.createEngine().runJobs(module.scenarioConfig(), Pars.parallel);
    } catch (Exception e) {
      logger.severe("[REST API] Module simulation failed: " + e.getMessage());
    }
  }

  /** Applies the standard Pars fields shared by all modules. */
  private static void applyCommonParams(Map<String, Object> params) {
    if (params.containsKey("cityName")) Pars.cityName = (String) params.get("cityName");
    if (params.containsKey("days"))
      Pars.durationDays = Integer.parseInt(params.get("days").toString());
    if (params.containsKey("actualPopulation"))
      Pars.population = Integer.parseInt(params.get("actualPopulation").toString());
    if (params.containsKey("percentage"))
      Pars.percentagePopulationAgent = Double.parseDouble(params.get("percentage").toString());
    if (params.containsKey("jobs")) {
      Pars.jobs = Integer.parseInt(params.get("jobs").toString());
      Pars.parallel = (Pars.jobs > 1);
    }
  }

  /** Builds the JSON body for {@code GET /api/modules}. */
  private static synchronized String buildModulesJson() throws Exception {
    List<Map<String, Object>> list = new ArrayList<>();
    for (SimulationModule m : modules.values()) {
      Map<String, Object> schema = new LinkedHashMap<>(COMMON_PARAMS);
      schema.putAll(m.parameterSchema());
      Map<String, Object> entry = new LinkedHashMap<>();
      entry.put("id", m.moduleId());
      entry.put("parameters", schema);
      list.add(entry);
    }
    Map<String, Object> root = Map.of("modules", list);
    return MAPPER.writeValueAsString(root);
  }
}
