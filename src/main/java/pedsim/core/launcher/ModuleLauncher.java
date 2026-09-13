package pedsim.core.launcher;

import pedsim.core.engine.SimulationLauncher;
import pedsim.core.engine.SimulationModule;
import pedsim.core.utilities.LoggerUtil;
import pedsim.core.website.SimulationRestApi;

/**
 * How a run starts: parse the flags, hand the module to {@link SimulationLauncher}.
 *
 * <p>Every module's {@code main} is one line of delegation to this. The logic used to be copied into
 * six {@code PedSimCityXxxApplet.main} methods, each a {@code java.awt.Frame} subclass that also
 * carried {@code buildEngine()}, {@code buildStateFactory()}, {@code module()} and a
 * {@code getCityName()} that read a {@code Choice} widget. Launching and drawing a window were the
 * same class, which is how a {@code --headless} run came to construct the entire GUI in order to
 * call a static method through an instance — and why, on a JVM started with
 * {@code -Djava.awt.headless=true}, the documented headless invocation would have thrown in the
 * {@code Frame} constructor before reaching the simulation.
 *
 * <p>The AWT panels are gone. What they configured is now set by
 * the running module's per-city configuration and by the command line for
 * everything else, both of which are logged and both of which a script can reproduce — where a
 * value typed into a text field left no record of what a run was.
 */
public final class ModuleLauncher {

  private ModuleLauncher() {}

  /**
   * Runs {@code module} according to the flags in {@code args}.
   *
   * <p>{@code --website} starts the REST dashboard instead of running; anything else runs the
   * simulation directly. There is no interactive mode: the prompt that used to ask "1) Standard
   * Applet GUI, 2) Web Dashboard" had only one surviving answer.
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
