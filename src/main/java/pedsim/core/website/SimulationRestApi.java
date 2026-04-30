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

  private static final Logger logger = LoggerUtil.getLogger();

  private SimulationRestApi() {}

  /**
   * Starts the REST API server on the specified port and registers the
   * {@code /api/state} handler.
   */
  public static void start(int port) {
    try {
      HttpServer server = HttpServer.create(new InetSocketAddress(port), 0);

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
          logger.warning("Error serving /api/state: " + ex.getMessage());
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
