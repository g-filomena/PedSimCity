package pedsim.core.engine;

import java.util.Map;
import pedsim.core.parameters.Pars;
import pedsim.core.utilities.StringEnum;

/** {@link SimulationModule} for the standard (non-night) core pedestrian simulation. */
public final class CoreSimulationModule implements SimulationModule {

  public static final CoreSimulationModule INSTANCE = new CoreSimulationModule();

  private CoreSimulationModule() {}

  @Override
  public String moduleId() {
    return "core";
  }

  @Override
  public void applyMode() {
    Pars.isNight = false;
  }

  @Override
  public void clearStaticData() {
    // Core static data is cleared by Engine.clearStaticData(); nothing extra needed here.
  }

  @Override
  public Engine createEngine() {
    return new Engine(PedSimCity::new);
  }

  @Override
  public ScenarioConfig scenarioConfig() {
    return new ScenarioConfig(StringEnum.Learner.values(), null);
  }

  @Override
  public void applyParameters(Map<String, Object> params) {
    // No module-specific parameters beyond the common Pars fields.
  }
}
