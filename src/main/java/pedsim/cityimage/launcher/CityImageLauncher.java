package pedsim.cityimage.launcher;

import pedsim.cityimage.engine.CityImageSimulationModule;
import pedsim.core.launcher.ModuleLauncher;

/**
 * Entry point for the city-image testing module.
 *
 * <p>Goes through {@link ModuleLauncher} like every other module now that cityImage has a
 * {@code SimulationModule}. Before that it had to do its own argument parsing and engine
 * construction, and reached the deprecated single-argument {@code initFromArgs}, so every
 * {@code TestPars} key given on the command line was accepted and ignored.
 */
public final class CityImageLauncher {

  private CityImageLauncher() {}

  public static void main(String[] args) throws Exception {
    ModuleLauncher.run(CityImageSimulationModule.INSTANCE, args);
  }
}
