package pedsim.empirical.engine;

import java.util.Map;
import pedsim.core.engine.Engine;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.engine.SimulationModule;
import pedsim.core.parameters.ParameterManager;
import pedsim.core.parameters.Pars;
import pedsim.empirical.agent.EmpiricalGroup;
import pedsim.empirical.parameters.EmpiricalPars;

/**
 * The empirical ABM module: agents drawn from the survey-derived route-choice clusters.
 *
 * <p>Walks a shared origin-destination matrix with one agent group per survey-derived cluster, so
 * the groups differ in the route-choice preferences the clusters describe.
 *
 * <p>Defaults to Muenster, 301 agents and 10 jobs, the study the cluster data comes from
 * ({@code Muenster_clusters.csv}). Those defaults are applied in {@link #applyParameters} and only
 * where the command line is silent, because the command line is applied after the defaults.
 * override them.
 *
 * <p>Reads no per-city configuration file — {@link #loadCityConfig} stays core's no-op — because
 * those files configure the activity model's behaviour and this module has none. The cluster CSV
 * plays the equivalent role here, carrying a survey-derived route-choice profile per group:
 * {@code usingElements_mean}, {@code onlyDistance_mean}, {@code regions_mean} and the rest.
 */
public final class EmpiricalSimulationModule implements SimulationModule {

  public static final EmpiricalSimulationModule INSTANCE = new EmpiricalSimulationModule();

  private EmpiricalSimulationModule() {}

  @Override
  public String moduleId() {
    return "empirical";
  }

  @Override
  public Class<?>[] parameterClasses() {
    return new Class<?>[] {
      Pars.class,
      pedsim.core.parameters.TimePars.class,
      pedsim.core.parameters.RouteChoicePars.class,
      EmpiricalPars.class
    };
  }

  @Override
  public void applyDefaults(java.util.Map<String, String> selectors) {
    // The study these clusters come from is Muenster, so that is the city, the cohort and the job
    // count this module expects. Stated unconditionally: the command line runs after this.
    Pars.cityName = EmpiricalPars.defaultCityName;
    Pars.jobs = EmpiricalPars.defaultJobs;
    Pars.numAgents = EmpiricalPars.numAgents;
  }

  @Override
  public Engine createEngine() {
    return new EmpiricalEngine(PedSimCityEmpirical::new);
  }

  /** One scenario per empirical cluster. */
  @Override
  public ScenarioConfig scenarioConfig() {
    return new ScenarioConfig(EmpiricalGroup.values(), null);
  }

  @Override
  public void clearStaticData() {
    PedSimCity.clearStaticData();
    EmpiricalPars.empiricalGroups.clear();
  }

  @Override
  public void applyParameters(Map<String, Object> params) {
    for (String key :
        new String[] {
          "numberTripsPerAgent",
          "usingDMA",
          "includePopulationBenchmark",
          "includeNullBenchmark",
          "numAgents"
        }) {
      if (params.containsKey(key)) {
        ParameterManager.setFieldValue(EmpiricalPars.class, key, params.get(key).toString());
      }
    }
  }

  @Override
  public Map<String, Object> extraState() {
    return Map.of(
        "numberTripsPerAgent", EmpiricalPars.numberTripsPerAgent,
        "usingDMA", EmpiricalPars.usingDMA,
        "numAgents", EmpiricalPars.numAgents);
  }

  @Override
  public Map<String, Object> parameterSchema() {
    return Map.of(
        "numberTripsPerAgent", "integer",
        "usingDMA", "boolean",
        "includePopulationBenchmark", "boolean",
        "includeNullBenchmark", "boolean",
        "numAgents", "integer");
  }
}
