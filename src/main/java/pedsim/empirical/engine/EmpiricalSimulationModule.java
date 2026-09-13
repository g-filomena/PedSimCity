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
 * <p>Written 13 Sep 2026, for the same reasons as {@code CityImageSimulationModule} — see that class
 * for what implementing no {@link SimulationModule} cost. Empirical's version of the problem was the
 * sharper one: it had no headless entry point at all. Its applet opened a window and nothing else,
 * and the only way to run it without a display was a {@code main} on the state class that took the
 * city as a bare positional argument and reached no parameter handling whatsoever.
 *
 * <p>Its defaults are Muenster, 301 agents, 10 jobs, because that is the study the cluster data
 * comes from ({@code Muenster_clusters.csv}). They are applied only where the command line has not
 * spoken, which is why they live in {@link #applyParameters} rather than in
 * {@link #applyMode}: {@code applyMode} runs after the arguments and would overwrite them.
 * {@code EmpiricalPars.applyDefaults()} did exactly that — it set {@code Pars.cityName} to Muenster
 * unconditionally, so a {@code --cityName} on the command line was ignored.
 *
 * <p>It reads no per-city configuration file: {@link #loadCityConfig} stays core's no-op, because
 * those files configure the activity model's behaviour and this module has none. What stands in for
 * them here is the cluster CSV, which carries the same kind of survey-derived quantity per group —
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
  public void applyMode() {
    Pars.isNight = false;
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
          "numberTripsPerAgent", "usingDMA", "includePopulationBenchmark", "includeNullBenchmark",
          "numAgents"
        }) {
      if (params.containsKey(key)) {
        ParameterManager.setFieldValue(EmpiricalPars.class, key, params.get(key).toString());
      }
    }

    // Defaults only where the command line was silent. The study these clusters come from is
    // Muenster, so that is the city, the agent count and the job count this module expects unless
    // told otherwise.
    if (!params.containsKey("cityName")) {
      Pars.cityName = EmpiricalPars.defaultCityName;
    }
    if (!params.containsKey("jobs")) {
      Pars.jobs = EmpiricalPars.defaultJobs;
    }
    // EmpiricalEngine.afterSetParameters() puts this back after recomputeAgentCount() overwrites it.
    Pars.numAgents = EmpiricalPars.numAgents;
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
