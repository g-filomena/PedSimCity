package pedsim.core.engine;

import java.util.Map;
import pedsim.core.parameters.Pars;
import pedsim.core.utilities.StringEnum;

/**
 * Infrastructure-only {@link SimulationModule} used by {@link SimulationLauncher} for pre-loading
 * GIS data and setting mode flags in core (non-night) context. This is <em>not</em> a runnable
 * simulation module and must not appear in {@code GET /api/modules} or be selectable via
 * {@code POST /api/start}. See {@link #isConcreteRunnable()}.
 */
public final class CoreSimulationModule implements SimulationModule {

  public static final CoreSimulationModule INSTANCE = new CoreSimulationModule();

  private CoreSimulationModule() {}

  @Override
  public String moduleId() {
    return "core";
  }

  /** Returns {@code false}: core is infrastructure, not a selectable runnable simulation. */
  @Override
  public boolean isConcreteRunnable() {
    return false;
  }

  @Override
  public void applyMode() {
    Pars.isNight = false;
  }

  @Override
  public void clearStaticData() {
    PedSimCity.clearStaticData();
  }

  @Override
  public Engine createEngine() {
    return new Engine(PedSimCity::new);
  }

  @Override
  public ScenarioConfig scenarioConfig() {
    return new ScenarioConfig(StringEnum.Default.values(), null);
  }

  @Override
  public void applyParameters(Map<String, Object> params) {
    // No module-specific parameters beyond the common Pars fields.
  }
}
