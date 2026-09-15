package pedsim.social.engine;

import pedsim.activity.engine.CityConfigContractTest;
import pedsim.core.engine.SimulationModule;

class SocialCityConfigTest extends CityConfigContractTest {
  @Override
  protected SimulationModule module() {
    return SocialSimulationModule.INSTANCE;
  }
}
