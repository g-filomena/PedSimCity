package pedsim.learning.engine;

import pedsim.activity.engine.PedSimCityActivity;
import pedsim.core.engine.ScenarioConfig;

/**
 * Simulation state for the learning module. Reuses the activity-based {@link PedSimCityActivity}
 * infrastructure (census + workplace) and only overrides {@link #populateEnvironment()} to generate
 * {@link pedsim.learning.agents.LearningAgent} instances via {@link LearningPopulate}.
 *
 * <p>Mirrors the {@code PedSimCityNight} pattern: all GIS data, scheduling, flow handling and the
 * day-based engine loop are inherited.
 */
public class PedSimCityLearning extends PedSimCityActivity {

  private static final long serialVersionUID = 1L;

  public PedSimCityLearning(long seed, int job, ScenarioConfig scenarioConfig) {
    super(seed, job, scenarioConfig);
  }

  @Override
  protected void populateEnvironment() {
    new LearningPopulate().populate(this);
  }
}
