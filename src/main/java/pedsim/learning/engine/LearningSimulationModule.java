package pedsim.learning.engine;

import java.util.Map;
import pedsim.activity.engine.CensusPopulation;
import pedsim.activity.engine.PedSimCityActivity;
import pedsim.core.engine.Engine;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.engine.SimulationModule;
import pedsim.core.utilities.StringEnum;
import pedsim.learning.parameters.LearningPars;
import pedsim.learning.routeMemorability.RouteProperties;

/**
 * {@link SimulationModule} for the incremental-learning pedestrian model, where agents navigate
 * within an evolving partial cognitive map built from the routes they walk.
 *
 * <p>Registered with {@code SimulationRestApi} like {@code NightSimulationModule}; selectable via
 * {@code POST /api/start {"module":"learning"}}.
 */
public final class LearningSimulationModule implements SimulationModule {

  public static final LearningSimulationModule INSTANCE = new LearningSimulationModule();

  private LearningSimulationModule() {}

  @Override
  public long populationForCity(String city) {
    return CensusPopulation.residentTotal(city);
  }

  @Override
  public String moduleId() {
    return "learning";
  }

  /** Learning is an activity-tier module, so the activity parameters are in scope as well. */
  @Override
  public Class<?>[] parameterClasses() {
    return new Class<?>[] {
      pedsim.core.parameters.Pars.class,
      pedsim.core.parameters.TimePars.class,
      pedsim.core.parameters.RouteChoicePars.class,
      pedsim.activity.parameters.ActivityPars.class,
      pedsim.learning.parameters.LearningPars.class
    };
  }

  @Override
  public void loadCityConfig(String cityName) {
    pedsim.activity.parameters.CityConfig.load(cityName, parameterClasses());
  }

  @Override
  public void applyDefaults(java.util.Map<String, String> selectors) {}

  @Override
  public void clearStaticData() {
    // VividnessGrid and IncrementalLearning are per-agent instance state, but
    // RouteProperties keeps a process-wide static cache of visibility cones that must be cleared
    // between jobs (the cones depend on the run's obstruction layer). Core + activity handle the
    // rest of the shared static data.
    RouteProperties.clearVisibilityCache();
    PedSimCity.clearStaticData();
    PedSimCityActivity.clearStaticData();
  }

  @Override
  public Engine createEngine() {
    return new LearningEngine(PedSimCityLearning::new);
  }

  @Override
  public ScenarioConfig scenarioConfig() {
    return new ScenarioConfig(StringEnum.Learner.values(), null);
  }

  @Override
  public void applyParameters(Map<String, Object> params) {
    if (params.containsKey("memoryThreshold"))
      LearningPars.memoryThreshold = Double.parseDouble(params.get("memoryThreshold").toString());
  }

  @Override
  public Map<String, Object> extraState() {
    return Map.of(
        "halfLife", LearningPars.halfLife, "memoryThreshold", LearningPars.memoryThreshold);
  }

  @Override
  public Map<String, Object> parameterSchema() {
    return Map.of("halfLife", "double", "memoryThreshold", "double");
  }
}
