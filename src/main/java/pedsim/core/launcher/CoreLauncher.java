package pedsim.core.launcher;

import pedsim.core.launcher.ModuleLauncher;

/**
 * Entry point for the core module.
 *
 * <p>Delegates to {@link ModuleLauncher}; see that class for why launching no longer lives on a
 * {@code java.awt.Frame}.
 */
public final class CoreLauncher {

  private CoreLauncher() {}

  public static void main(String[] args) throws Exception {
    ModuleLauncher.run(pedsim.core.engine.CoreSimulationModule.INSTANCE, args);
  }
}
