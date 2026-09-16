package pedsim.night.engine;

import java.util.Map;
import pedsim.activity.engine.CensusPopulation;
import pedsim.activity.engine.PedSimCityActivity;
import pedsim.core.engine.Engine;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.engine.SimulationModule;
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

  /** Night is an activity-tier module, so the activity parameters are in scope as well. */
  @Override
  public Class<?>[] parameterClasses() {
    return new Class<?>[] {
      pedsim.core.parameters.Pars.class,
      pedsim.core.parameters.TimePars.class,
      pedsim.core.parameters.RouteChoicePars.class,
      pedsim.activity.parameters.ActivityPars.class,
      pedsim.night.parameters.NightPars.class
    };
  }

  @Override
  public void loadCityConfig(String cityName) {
    pedsim.activity.parameters.CityConfig.load(cityName, parameterClasses());
  }

  @Override
  public void applyDefaults(java.util.Map<String, String> selectors) {}

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
  public Map<String, Object> extraState() {
    return Map.of(
        "enableLightABTesting",
        NightPars.enableLightABTesting,
        "abTestPairs",
        NightPars.abTestPairs,
        "crowdednessPercentile",
        NightPars.crowdednessPercentile,
        "directionalLuxStatistic",
        NightPars.directionalLuxStatistic.toString(),
        "nonVulnerableLightSensitivity",
        NightPars.nonVulnerableLightSensitivity,
        "useGravityModel",
        pedsim.core.parameters.RouteChoicePars.useGravityModel);
  }

  @Override
  public Map<String, Object> parameterSchema() {
    return Map.of(
        "enableLightABTesting", "boolean",
        "abTestPairs", "int",
        "crowdednessPercentile", "double",
        "directionalLuxStatistic", "enum:MIN|MEAN",
        "nonVulnerableLightSensitivity", "double",
        "useGravityModel", "boolean");
  }
}
