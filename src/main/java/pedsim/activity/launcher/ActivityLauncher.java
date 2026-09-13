package pedsim.activity.launcher;

import pedsim.core.launcher.ModuleLauncher;

/**
 * Entry point for the activity module.
 *
 * <p>Delegates to {@link ModuleLauncher}; see that class for why launching no longer lives on a
 * {@code java.awt.Frame}.
 */
public final class ActivityLauncher {

  private ActivityLauncher() {}

  public static void main(String[] args) throws Exception {
    ModuleLauncher.run(pedsim.activity.engine.ActivitySimulationModule.INSTANCE, args);
  }
}
