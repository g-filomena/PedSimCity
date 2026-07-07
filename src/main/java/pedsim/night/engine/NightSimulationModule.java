package pedsim.night.engine;

import java.util.Map;
import pedsim.activity.engine.CensusPopulation;
import pedsim.activity.engine.PedSimCityActivity;
import pedsim.core.engine.Engine;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.engine.SimulationModule;
import pedsim.core.parameters.Pars;
import pedsim.core.utilities.StringEnum;
import pedsim.night.parameters.NightPars;

/** {@link SimulationModule} for the night-time illumination-aware pedestrian simulation. */
public final class NightSimulationModule implements SimulationModule {

  public static final NightSimulationModule INSTANCE = new NightSimulationModule();

  private NightSimulationModule() {}

  @Override
  public long populationForCity(String city) {
    return CensusPopulation.residentTotal(city);
  }

  @Override
  public String moduleId() {
    return "night";
  }

  @Override
  public void applyMode() {
    Pars.isNight = true;
  }

  /**
   * Clears core and night-specific static data.
   *
   * <p>Cleanup coverage by path:
   *
   * <ul>
   *   <li><b>Dashboard preload</b>: {@link pedsim.core.engine.SimulationLauncher#clearAll()}
   *       delegates entirely to this method, which clears both core and night caches.
   *   <li><b>Actual run</b>: {@link NightEngine#clearStaticData()} (called inside
   *       {@code Engine.runJobs()}) calls {@code super.clearStaticData()} (= core) then
   *       {@code PedSimCityNight.clearNightStaticData()} (= night). These are separate invocation
   *       paths; no double-clearing occurs.
   * </ul>
   */
  @Override
  public void clearStaticData() {
    PedSimCity.clearStaticData();
    PedSimCityActivity.clearStaticData();
    PedSimCityNight.clearNightStaticData();
  }

  @Override
  public Engine createEngine() {
    return new NightEngine(PedSimCityNight::new);
  }

  @Override
  public ScenarioConfig scenarioConfig() {
    return new ScenarioConfig(StringEnum.Vulnerable.values(), StringEnum.Hour.values());
  }

  @Override
  public void applyParameters(Map<String, Object> params) {
    if (params.containsKey("enableAB")) {
      NightPars.enableLightABTesting = Boolean.parseBoolean(params.get("enableAB").toString());
    }
    if (params.containsKey("abTestPairs")) {
      NightPars.abTestPairs = Integer.parseInt(params.get("abTestPairs").toString().trim());
    }
    if (params.containsKey("crowdednessPercentile")) {
      NightPars.crowdednessPercentile =
          Double.parseDouble(params.get("crowdednessPercentile").toString());
    }
    if (params.containsKey("directionalLuxStatistic")) {
      NightPars.directionalLuxStatistic =
          NightPars.DirectionalLuxStatistic.valueOf(
              params.get("directionalLuxStatistic").toString().trim().toUpperCase());
    }
    if (params.containsKey("minVulnerableLightSensitivity")) {
      NightPars.minVulnerableLightSensitivity =
          Double.parseDouble(params.get("minVulnerableLightSensitivity").toString());
    }
    if (params.containsKey("maxVulnerableLightSensitivity")) {
      NightPars.maxVulnerableLightSensitivity =
          Double.parseDouble(params.get("maxVulnerableLightSensitivity").toString());
    }
    if (params.containsKey("nonVulnerableLightSensitivity")) {
      NightPars.nonVulnerableLightSensitivity =
          Double.parseDouble(params.get("nonVulnerableLightSensitivity").toString());
    }
    if (params.containsKey("useGravityModel")) {
      pedsim.core.parameters.RouteChoicePars.useGravityModel =
          Boolean.parseBoolean(params.get("useGravityModel").toString());
    }
  }

  @Override
  public Map<String, Object> extraState() {
        return Map.of(
            "enableAB", NightPars.enableLightABTesting,
            "abTestPairs", NightPars.abTestPairs,
            "crowdednessPercentile", NightPars.crowdednessPercentile,
            "directionalLuxStatistic", NightPars.directionalLuxStatistic.toString(),
            "nonVulnerableLightSensitivity", NightPars.nonVulnerableLightSensitivity,
            "useGravityModel", pedsim.core.parameters.RouteChoicePars.useGravityModel);
    }

  @Override
  public Map<String, Object> parameterSchema() {
        return Map.of(
            "enableAB", "boolean",
            "abTestPairs", "int",
            "crowdednessPercentile", "double",
            "directionalLuxStatistic", "enum:MIN|MEAN",
            "nonVulnerableLightSensitivity", "double",
            "useGravityModel", "boolean");
    }
}
