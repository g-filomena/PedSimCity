package pedsim.activity.launcher;

import pedsim.core.launcher.ModuleLauncher;

/**
 * Entry point for the activity module.
 *
 * <p>Parses no arguments of its own: {@link ModuleLauncher} handles the command line and starts the
 * run. Pass {@code --website} to serve the browser dashboard instead of running directly.
 */
public final class ActivityLauncher {

  private ActivityLauncher() {}

  public static void main(String[] args) throws Exception {
    ModuleLauncher.run(pedsim.activity.engine.ActivitySimulationModule.INSTANCE, args);
  }
}
