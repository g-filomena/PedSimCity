package pedsim.cityimage.engine;

import pedsim.cityimage.utilities.StringEnum.RouteChoice;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.ScenarioConfig;

/** Simulation state for the city-image testing module. */
public class PedSimCityImage extends PedSimCity {

  private static final long serialVersionUID = 1L;

  public PedSimCityImage(long seed, int job, ScenarioConfig scenarioConfig) {
    super(seed, job, defaultScenarioConfig(scenarioConfig));
  }

  private static ScenarioConfig defaultScenarioConfig(ScenarioConfig scenarioConfig) {
    if (scenarioConfig != null) {
      return scenarioConfig;
    }

    return new ScenarioConfig(RouteChoice.values(), null);
  }

  @Override
  protected void populateEnvironment() {
    CityImagePopulate populate = new CityImagePopulate();
    populate.populateTests(this);
  }
}
