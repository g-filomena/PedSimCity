package pedsim.activity.agents;

import java.util.Map;
import pedsim.activity.engine.PedSimCityActivity;
import pedsim.core.agents.Agent;
import pedsim.core.engine.PedSimCity;
import sim.graph.NodeGraph;

/**
 * Pedestrian agent for the activity-based model. Behaves like a core {@link Agent} but
 * follows a 24h activity pattern: it weights daytime destinations by workplace POI density and
 * evening destinations (when {@code isDark}) by night POI density. No vulnerability or lighting
 * behaviour — those belong to the night module.
 */
public class ActivityAgent extends Agent {

  private static final long serialVersionUID = 1L;

  public ActivityAgent(PedSimCity state, boolean registerSpatial) {
    super(state, registerSpatial);
  }

  /** Reads the activity 24h clock so destination selection and work-targeting follow time of day. */
  @Override
  protected boolean isDark() {
    return state instanceof PedSimCityActivity activityState && activityState.isDark;
  }

  /**
   * Weights candidate destinations by activity POI density: night POI when dark, workplace POI
   * during the day. Returns 0.0 (uniform) when the relevant dataset was not loaded.
   */
  @Override
  protected double getPOIWeight(NodeGraph node, boolean isDark) {
    Map<NodeGraph, Double> weightMap =
        isDark ? PedSimCityActivity.nodesNightWeight : PedSimCityActivity.nodesWorkplaceWeight;
    if (weightMap.isEmpty()) {
      return 0.0;
    }
    return weightMap.getOrDefault(node, 0.0);
  }
}
