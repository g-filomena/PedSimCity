package pedsim.activity.agents;

import ec.util.MersenneTwisterFast;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import pedsim.activity.parameters.ActivityPars;
import pedsim.core.parameters.RouteChoicePars;
import sim.graph.GraphUtils;
import sim.graph.NodeGraph;

/**
 * Where a person works, drawn from where the jobs are against how far away they are.
 *
 * <p>Extracted from {@code ActivityPopulate} so the same draw can be run outside a simulation.
 * Calibrating it means asking what distribution of commutes a given decay produces, which is a
 * question about this function and the city's WORK tags - not about agents, days or routes. Running
 * a simulated day to answer it costs minutes and adds nothing.
 *
 * <p>The two parameters are {@link ActivityPars#workplaceDistanceDecay} and
 * {@link ActivityPars#workplaceMinDistanceMetres}, and both are uncalibrated. What they have to
 * reproduce is in {@code COMMUTE_DISTANCE.md}.
 */
public final class WorkplaceChoice {

  private WorkplaceChoice() {}

  /**
   * Draws a workplace for a home location.
   *
   * @param homeNode where the person lives
   * @param attraction per-node attraction for the purpose (WORK or EDUCATION)
   * @param beta distance decay exponent; ignored when {@code RouteChoicePars.useGravityModel} is off
   * @param floorMetres closest a workplace may be assigned
   * @param random the generator to draw with
   * @return the chosen node, or null when nothing qualifies
   */
  public static NodeGraph draw(
      NodeGraph homeNode,
      Map<NodeGraph, Double> attraction,
      double beta,
      double floorMetres,
      MersenneTwisterFast random) {

    if (attraction == null || attraction.isEmpty() || homeNode == null) {
      return null;
    }

    List<NodeGraph> candidates = new ArrayList<>();
    List<Double> weights = new ArrayList<>();
    double totalWeight = 0.0;

    for (Map.Entry<NodeGraph, Double> entry : attraction.entrySet()) {
      double distance = GraphUtils.nodesDistance(homeNode, entry.getKey());
      // Lower bound only, and unsourced: with a 1/d^beta decay and no floor the nearest tagged node
      // takes almost all the mass and everyone works next door. No upper bound - a workplace is
      // where it is, and what decides whether the commute is walked is decideCommuteMode.
      if (distance < floorMetres) {
        continue;
      }
      double weight = entry.getValue();
      if (RouteChoicePars.useGravityModel) {
        weight /= Math.pow(Math.max(10.0, distance), beta);
      }
      candidates.add(entry.getKey());
      weights.add(weight);
      totalWeight += weight;
    }
    if (candidates.isEmpty() || totalWeight <= 0.0) {
      return null;
    }

    double r = random.nextDouble() * totalWeight;
    double cumulative = 0.0;
    for (int i = 0; i < candidates.size(); i++) {
      cumulative += weights.get(i);
      if (r <= cumulative) {
        return candidates.get(i);
      }
    }
    return candidates.get(candidates.size() - 1);
  }
}
