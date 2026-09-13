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
 * <p>Written 13 Sep 2026. Until then cityImage implemented no {@link SimulationModule} at all — the
 * interface's own javadoc listed a {@code CityImageSimulationModule} as an extension point, and no
 * such class had ever existed. Three things followed from its absence, and all three were silent:
 *
 * <ul>
 *   <li>the module was invisible to the REST layer, so the browser dashboard could not run it;
 *   <li>its parameters were unreachable from the command line. {@code initFromArgs} wrote into
 *       {@code Pars}, {@code TimePars} and {@code RouteChoicePars} only, so {@code --stringMode} or
 *       {@code --numberTripsPerAgent} was accepted without complaint and ignored — the same defect
 *       that swallowed {@code --useDestinationChoice}, and the reason
 *       {@link #parameterClasses()} exists;
 *   <li>with the AWT {@code TestPanel} deleted, that left no way at all to choose a test mode, so
 *       the module could only run whichever mode {@code TestPars} declared.
 * </ul>
 *
 * <p>It still does not read a per-city configuration file: {@link #loadCityConfig} is left as core's
 * no-op, because the city files configure the activity model's behaviour and this module has none —
 * it generates synthetic OD pairs and compares route-choice strategies over them.
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
   * Resolves the test mode, then puts the command line back on top of it.
   *
   * <p>The order is the point. {@code TestPars.defineMode()} does not only select a mode: it also
   * sets {@code numberTripsPerAgent} and {@code jobs} to that mode's own defaults - 255 trips and 50
   * jobs for landmarks, 2,000 and 10 for subdivisions. So it overwrites anything the command line
   * asked for, and a {@code --numberTripsPerAgent=8} silently became 2,000. Re-applying the
   * overrides afterwards is what makes the flag mean something.
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
   * {@code stringMode} is the one that matters: it selects between testing landmarks, testing urban
   * subdivisions, and testing a user-chosen set of route-choice models, and until this module existed
   * it could only be set from a GUI panel that no longer exists.
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
