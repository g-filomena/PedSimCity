package pedsim.core.launcher;

import pedsim.core.engine.SimulationLauncher;
import pedsim.core.engine.SimulationModule;
import pedsim.core.utilities.LoggerUtil;
import pedsim.core.website.SimulationRestApi;

/**
 * How a run starts: parse the flags, hand the module to {@link SimulationLauncher}.
 *
 * <p>Every module's {@code main} is one line of delegation to this, so a run starts the same way
 * whichever module it is. Nothing here touches AWT: a run is configured entirely by the running
 * module's per-city configuration and by the command line, both of which are logged, so a run can be
 * reproduced from what is written down.
 */
public final class ModuleLauncher {

  private ModuleLauncher() {}

  /**
   * Runs {@code module} according to the flags in {@code args}.
   *
   * <p>{@code --website} starts the REST dashboard and returns; anything else runs the simulation
   * directly and returns when it finishes. There is no interactive mode.
   *
   * @param module the module to run
   * @param args the command line, passed through unchanged
   */
  public static void run(SimulationModule module, String[] args) throws Exception {
    boolean website = false;
    for (String arg : args) {
      if ("--website".equals(arg)) {
        website = true;
      }
    }

    SimulationLauncher launcher = new SimulationLauncher(module);

    if (website) {
      LoggerUtil.getLogger().info("[STARTUP] Starting REST API for the browser dashboard...");
      launcher.wireAndStartRestServer(8081);
      SimulationRestApi.openDashboardInBrowser();
      return;
    }

    LoggerUtil.getLogger().info("[RUN] " + module.moduleId() + " simulation, headless.");
    launcher.headlessRun(args);
  }
}
