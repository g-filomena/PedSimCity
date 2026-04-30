package pedsim.night.website;

import io.javelit.core.JtContainer;
import pedsim.core.engine.Engine;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.utilities.StringEnum;
import pedsim.core.website.SimulationApp;
import pedsim.night.engine.PedSimCityNight;

/**
 * Javelit dashboard page for the PedSimCity Night simulation.
 *
 * <p>Wires up the {@link PedSimCityNight} engine factory. Night-specific
 * parameter controls have been removed per user request for a simpler UI.
 */
public class NightSimulationApp extends SimulationApp {

  // ----------------------------------------------------------------
  // Page entry point
  // ----------------------------------------------------------------

  /**
   * Entry point passed to {@code Server.builder()} when the night applet starts.
   */
  public static void render() {
    SimulationApp.render(new NightSimulationApp());
  }

  // ----------------------------------------------------------------
  // Overrides
  // ----------------------------------------------------------------

  /**
   * Night-specific controls removed per request.
   *
   * @param form the Javelit form container to add widgets to.
   */
  @Override
  protected void renderExtraControls(JtContainer form) {
    // night-specific parameter controls removed per request
  }

  /**
   * Returns the scenario configuration for the night simulation.
   */
  @Override
  protected ScenarioConfig buildScenarioConfig() {
    return new ScenarioConfig(
        StringEnum.Vulnerable.values(),
        StringEnum.TimeOfDay.values()
    );
  }

  /**
   * Returns the {@link Engine.StateFactory} for the night simulation engine.
   */
  @Override
  protected Engine.StateFactory buildStateFactory() {
    return PedSimCityNight::new;
  }
}
