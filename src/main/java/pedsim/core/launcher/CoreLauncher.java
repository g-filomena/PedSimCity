package pedsim.core.launcher;

/**
 * Entry point for the core module.
 *
 * <p>Parses no arguments of its own: {@link ModuleLauncher} handles the command line and starts the
 * run. Pass {@code --website} to serve the browser dashboard instead of running directly.
 */
public final class CoreLauncher {

  private CoreLauncher() {}

  public static void main(String[] args) throws Exception {
    ModuleLauncher.run(pedsim.core.engine.CoreSimulationModule.INSTANCE, args);
  }
}
