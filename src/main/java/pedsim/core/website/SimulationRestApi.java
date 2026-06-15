package pedsim.core.website;

import java.io.IOException;
import java.net.BindException;
import java.net.InetSocketAddress;
import java.nio.charset.StandardCharsets;
import java.util.logging.Logger;

import com.sun.net.httpserver.HttpServer;

import pedsim.core.engine.SimulationStateStore;
import pedsim.core.utilities.LoggerUtil;

/**
 * Lightweight HTTP server that exposes the live simulation state to the browser
 * dashboard as a JSON REST endpoint at {@code GET /api/state}.
 */
public final class SimulationRestApi {

  private static final Logger logger = LoggerUtil.getLogger();

  private static java.util.function.Consumer<java.util.Map<String, Object>> startListener;

  /**
   * Registers a listener to be called when the /api/start endpoint is hit.
   */
  public static void setOnStart(java.util.function.Consumer<java.util.Map<String, Object>> listener) {
    startListener = listener;
  }

  /**
   * Maximum number of consecutive ports to try if the preferred port is occupied.
   */
  private static final int MAX_PORT_RETRIES = 10;

  /**
   * Starts the REST API server on {@code preferredPort}, automatically retrying
   * on up to {@value #MAX_PORT_RETRIES} subsequent ports if the preferred one is
   * already in use.  If all ports are occupied the method logs a warning and
   * returns without throwing, so the calling GUI thread is never crashed.
   *
   * @param preferredPort The first port to try (e.g. 8081).
   */
  public static void start(int preferredPort) {
    for (int attempt = 0; attempt < MAX_PORT_RETRIES; attempt++) {
      int port = preferredPort + attempt;
      try {
        HttpServer server = HttpServer.create(new InetSocketAddress(port), 0);

        // --- GET /api/state ---
        server.createContext("/api/state", exchange -> {
          try {
            if (!"GET".equalsIgnoreCase(exchange.getRequestMethod())) {
              exchange.sendResponseHeaders(405, -1);
              return;
            }
            byte[] responseBytes = SimulationStateStore.getInstance()
                .toJson()
                .getBytes(StandardCharsets.UTF_8);

            exchange.getResponseHeaders().add("Content-Type", "application/json");
            exchange.getResponseHeaders().add("Access-Control-Allow-Origin", "*");
            exchange.sendResponseHeaders(200, responseBytes.length);
            try (var body = exchange.getResponseBody()) {
              body.write(responseBytes);
            }
          } catch (Exception ex) {
            exchange.sendResponseHeaders(500, -1);
          }
        });

        // --- GET /api/roads --- (served once; heavy GeoJSON not bundled in /api/state)
        server.createContext("/api/roads", exchange -> {
          try {
            if (!"GET".equalsIgnoreCase(exchange.getRequestMethod())) {
              exchange.sendResponseHeaders(405, -1);
              return;
            }
            String roads = SimulationStateStore.getInstance().roadsGeoJson;
            if (roads == null) roads = "{\"type\":\"FeatureCollection\",\"features\":[]}";
            byte[] responseBytes = roads.getBytes(StandardCharsets.UTF_8);
            exchange.getResponseHeaders().add("Content-Type", "application/json");
            exchange.getResponseHeaders().add("Access-Control-Allow-Origin", "*");
            exchange.sendResponseHeaders(200, responseBytes.length);
            try (var body = exchange.getResponseBody()) {
              body.write(responseBytes);
            }
          } catch (Exception ex) {
            exchange.sendResponseHeaders(500, -1);
          }
        });

        // --- POST /api/start ---
        server.createContext("/api/start", exchange -> {
          try {
            // Allow CORS preflight
            exchange.getResponseHeaders().add("Access-Control-Allow-Origin", "*");
            exchange.getResponseHeaders().add("Access-Control-Allow-Methods", "POST, GET, OPTIONS");
            exchange.getResponseHeaders().add("Access-Control-Allow-Headers", "Content-Type");

            if ("OPTIONS".equalsIgnoreCase(exchange.getRequestMethod())) {
              exchange.sendResponseHeaders(204, -1);
              return;
            }

            if (!"POST".equalsIgnoreCase(exchange.getRequestMethod())) {
              exchange.sendResponseHeaders(405, -1);
              return;
            }

            if (startListener != null) {
            	java.util.Map<String, Object> params = new java.util.HashMap<>();

            	try {
            	  if (exchange.getRequestBody().available() > 0) {
            	    params =
            	        new tools.jackson.databind.ObjectMapper()
            	            .readValue(
            	                exchange.getRequestBody(),
            	                new tools.jackson.core.type.TypeReference<java.util.Map<String, Object>>() {});
            	  }
            	} catch (Exception e) {
            	  logger.warning("Failed to parse start request body: " + e.getMessage());
            	}

              final java.util.Map<String, Object> finalParams = params;
              new Thread(() -> startListener.accept(finalParams)).start();

              logger.info("[REST API] Simulation start triggered with params: " + params);
              exchange.sendResponseHeaders(200, 0);
            } else {
              exchange.sendResponseHeaders(503, 0); // Service Unavailable
            }
            exchange.getResponseBody().close();
          } catch (Exception ex) {
            exchange.sendResponseHeaders(500, -1);
          }
        });

        server.setExecutor(null);
        server.start();
        logger.info("[REST API] Simulation state endpoint started on port " + port);
        return; // Successfully bound — exit the retry loop

      } catch (BindException be) {
        // Port is taken — try the next one
        logger.warning("[REST API] Port " + port + " already in use, trying " + (port + 1) + "…");
      } catch (IOException e) {
        // Any other IO problem — log and give up gracefully (do NOT crash the GUI)
        logger.warning("[REST API] Could not start on port " + port + ": " + e.getMessage()
            + " — live dashboard will be unavailable.");
        return;
      }
    }

    // All ports in the range were occupied
    logger.warning("[REST API] Could not bind to any port in range "
        + preferredPort + "–" + (preferredPort + MAX_PORT_RETRIES - 1)
        + ". Live dashboard will be unavailable, but the GUI will continue normally.");
  }
}
