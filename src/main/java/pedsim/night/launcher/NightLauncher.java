package pedsim.night.launcher;

import pedsim.core.launcher.ModuleLauncher;

/**
 * Entry point for the night module.
 *
 * <p>Parses no arguments of its own: {@link ModuleLauncher} handles the command line and starts the
 * run. Pass {@code --website} to serve the browser dashboard instead of running directly.
 */
public final class NightLauncher {

  private NightLauncher() {}

  public static void main(String[] args) throws Exception {
    ModuleLauncher.run(pedsim.night.engine.NightSimulationModule.INSTANCE, args);
  }
}
