package pedsim.empirical.launcher;

import pedsim.core.launcher.ModuleLauncher;
import pedsim.empirical.engine.EmpiricalSimulationModule;

/**
 * Entry point for the empirical ABM module.
 *
 * <p>This module had no headless entry at all until 13 Sep 2026: its applet opened a window and
 * nothing else, and the only display-less route was a {@code main} on the state class taking the
 * city as a bare positional argument with no parameter handling. It now takes {@code --key=value}
 * like every other module.
 */
public final class EmpiricalLauncher {

  private EmpiricalLauncher() {}

  public static void main(String[] args) throws Exception {
    ModuleLauncher.run(EmpiricalSimulationModule.INSTANCE, args);
  }
}
