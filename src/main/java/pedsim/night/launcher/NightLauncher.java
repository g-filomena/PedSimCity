package pedsim.night.launcher;

import pedsim.core.launcher.ModuleLauncher;

/**
 * Entry point for the night module.
 *
 * <p>Delegates to {@link ModuleLauncher}; see that class for why launching no longer lives on a
 * {@code java.awt.Frame}.
 */
public final class NightLauncher {

  private NightLauncher() {}

  public static void main(String[] args) throws Exception {
    ModuleLauncher.run(pedsim.night.engine.NightSimulationModule.INSTANCE, args);
  }
}
