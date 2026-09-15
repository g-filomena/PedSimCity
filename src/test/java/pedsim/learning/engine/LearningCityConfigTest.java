package pedsim.learning.engine;

import pedsim.activity.engine.CityConfigContractTest;
import pedsim.core.engine.SimulationModule;

class LearningCityConfigTest extends CityConfigContractTest {
  @Override
  protected SimulationModule module() {
    return LearningSimulationModule.INSTANCE;
  }
}
