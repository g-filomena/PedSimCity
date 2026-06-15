package pedsim.night.engine;

import java.util.Map;
import pedsim.core.engine.Engine;
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
  public String moduleId() {
    return "night";
  }

  @Override
  public void applyMode() {
    Pars.isNight = true;
  }

  /**
   * Clears night-specific static data.
   *
   * <p>Cleanup coverage by path:
   *
   * <ul>
   *   <li><b>Dashboard preload</b>: {@link pedsim.core.engine.SimulationLauncher#clearAll()}
   *       calls {@code PedSimCity.clearStaticData()} then this method, covering both core and night
   *       caches.
   *   <li><b>Actual run</b>: {@link NightEngine#clearStaticData()} (called inside
   *       {@code Engine.runJobs()}) calls {@code super.clearStaticData()} (= core) then
   *       {@code PedSimCityNight.clearNightStaticData()} (= night). Both covered, no double-call.
   * </ul>
   */
  @Override
  public void clearStaticData() {
    PedSimCityNight.clearNightStaticData();
  }

  @Override
  public Engine createEngine() {
    return new NightEngine(PedSimCityNight::new);
  }

  @Override
  public ScenarioConfig scenarioConfig() {
    return new ScenarioConfig(StringEnum.Vulnerable.values(), StringEnum.TimeOfDay.values());
  }

  @Override
  public void applyParameters(Map<String, Object> params) {
    if (params.containsKey("enableAB"))
      NightPars.enableLightABTesting = Boolean.parseBoolean(params.get("enableAB").toString());
    if (params.containsKey("crowdednessPercentile"))
      NightPars.crowdednessPercentile =
          Double.parseDouble(params.get("crowdednessPercentile").toString());
  }

  @Override
  public Map<String, Object> extraState() {
    return Map.of(
        "enableAB", NightPars.enableLightABTesting,
        "crowdednessPercentile", NightPars.crowdednessPercentile);
  }

  @Override
  public Map<String, Object> parameterSchema() {
    return Map.of("enableAB", "boolean", "crowdednessPercentile", "double");
  }
}
