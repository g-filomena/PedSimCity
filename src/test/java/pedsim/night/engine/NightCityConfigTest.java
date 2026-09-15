package pedsim.night.engine;

import pedsim.activity.engine.CityConfigContractTest;
import pedsim.core.engine.SimulationModule;

class NightCityConfigTest extends CityConfigContractTest {
  @Override
  protected SimulationModule module() {
    return NightSimulationModule.INSTANCE;
  }
}
