package pedsim.core.website;

import io.javelit.core.JtRunnable;
import io.javelit.core.Server;
import java.util.logging.Logger;
import pedsim.core.utilities.LoggerUtil;

/**
 * Entry point for the Javelit browser dashboard and REST API.
 */
public final class SimulationWebServer {

  private static final Logger logger = LoggerUtil.getLogger();

  /** Javelit UI port. */
  public static final int UI_PORT = 8080;

  /** REST API port for live agent data. */
  public static final int API_PORT = 8081;

  private SimulationWebServer() {}

  /**
   * Starts the Javelit dashboard and the REST API.
   */
  public static void start(JtRunnable appPage) {
    // Start the Javelit browser UI (non-blocking)
    Server.builder(appPage, UI_PORT).build().start();
    logger.info("[Dashboard] Javelit UI started at http://localhost:" + UI_PORT);

    // Start the live-state REST API
    SimulationRestApi.start(API_PORT);
  }
}
