package pedsim.cityimage.engine;

import java.util.Map;
import pedsim.cityimage.parameters.TestPars;
import pedsim.cityimage.utilities.StringEnum.RouteChoice;
import pedsim.core.engine.Engine;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.engine.SimulationModule;
import pedsim.core.parameters.ParameterManager;

/**
 * The city-image testing module.
 *
 * <p>Generates one shared origin-destination matrix, puts one agent per route-choice model on it,
 * and exports the per-edge volumes each model produces, so the models differ in nothing but how they
 * choose a route. Which models run, how many trips each walks, and which of the three test designs
 * is used are all set through {@link #applyParameters}; see {@link TestPars}.
 *
 * <p>Reads no per-city configuration file — {@link #loadCityConfig} stays core's no-op — because
 * those files configure the activity model's behaviour and this module has none.
 */
public final class CityImageSimulationModule implements SimulationModule {

  public static final CityImageSimulationModule INSTANCE = new CityImageSimulationModule();

  private CityImageSimulationModule() {}

  @Override
  public String moduleId() {
    return "cityImage";
  }

  @Override
  public Class<?>[] parameterClasses() {
    return new Class<?>[] {
      pedsim.core.parameters.Pars.class,
      pedsim.core.parameters.TimePars.class,
      pedsim.core.parameters.RouteChoicePars.class,
      TestPars.class
    };
  }

  /** Command-line overrides, kept so they can be re-applied after {@code defineMode()}. */
  private Map<String, Object> overrides = Map.of();

  /**
   * Resolves the test design, then re-applies the command line over it.
   *
   * <p>The order matters: {@code TestPars.defineMode()} sets {@code numberTripsPerAgent} and
   * {@code jobs} to the chosen design's own defaults - 255 trips and 50 jobs for landmarks, 2,000 and
   * 10 for subdivisions - so anything given on the command line has to be written again afterwards
   * to take effect.
   */
  @Override
  public void applyMode() {
    pedsim.core.parameters.Pars.isNight = false;
    TestPars.defineMode();
    writeOverrides(overrides);
    pedsim.core.utilities.LoggerUtil.getLogger()
        .info(
            String.format(
                "cityImage mode '%s': %d route-choice models x %d trips each, %d job(s)",
                TestPars.stringMode,
                TestPars.routeChoiceModels.length,
                TestPars.numberTripsPerAgent,
                pedsim.core.parameters.Pars.jobs));
  }

  @Override
  public Engine createEngine() {
    return new CityImageEngine(PedSimCityImage::new);
  }

  /**
   * One scenario per route-choice model. The agent count follows from it: this module runs one agent
   * per model over a shared OD matrix, so the models differ in nothing but their route choice.
   */
  @Override
  public ScenarioConfig scenarioConfig() {
    return new ScenarioConfig(RouteChoice.values(), null);
  }

  @Override
  public void clearStaticData() {
    PedSimCity.clearStaticData();
    TestPars.distances.clear();
  }

  /**
   * {@code stringMode} selects the test design: testing landmarks, testing urban subdivisions, or
   * testing a user-chosen set of route-choice models.
   */
  @Override
  public void applyParameters(Map<String, Object> params) {
    overrides = Map.copyOf(params);
    writeOverrides(params);
  }

  /** Writes this module's own keys out of an argument map. Called twice; see {@link #applyMode()}. */
  private static void writeOverrides(Map<String, Object> params) {
    for (String key :
        new String[] {
          "stringMode", "numberTripsPerAgent", "testingSpecificOD", "verboseMode",
          "originsTmp", "destinationsTmp"
        }) {
      if (params.containsKey(key)) {
        ParameterManager.setFieldValue(TestPars.class, key, params.get(key).toString());
      }
    }
    if (params.containsKey("jobs")) {
      ParameterManager.setFieldValue(
          pedsim.core.parameters.Pars.class, "jobs", params.get("jobs").toString());
    }
  }

  @Override
  public Map<String, Object> extraState() {
    return Map.of(
        "stringMode", TestPars.stringMode,
        "numberTripsPerAgent", TestPars.numberTripsPerAgent,
        "testingSpecificOD", TestPars.testingSpecificOD);
  }

  @Override
  public Map<String, Object> parameterSchema() {
    return Map.of(
        "stringMode", "string",
        "numberTripsPerAgent", "integer",
        "testingSpecificOD", "boolean",
        "verboseMode", "boolean");
  }
}
