package pedsim.core.website;

import com.sun.net.httpserver.HttpServer;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.charset.StandardCharsets;
import java.util.logging.Logger;
import pedsim.core.engine.SimulationStateStore;
import pedsim.core.utilities.LoggerUtil;

/**
 * Lightweight HTTP server that exposes the live simulation state to the browser
 * dashboard as a JSON REST endpoint at {@code GET /api/state}.
 */
public final class SimulationRestApi {

  private static Runnable startListener;

  /**
   * Registers a listener to be called when the /api/start endpoint is hit.
   */
  public static void setOnStart(Runnable listener) {
    startListener = listener;
  }

  /**
   * Starts the REST API server on the specified port.
   */
  public static void start(int port) {
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
            new Thread(startListener).start();
            logger.info("[REST API] Simulation start triggered.");
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

    } catch (IOException e) {
      throw new RuntimeException("Failed to start SimulationRestApi on port " + port, e);
    }
  }
}
