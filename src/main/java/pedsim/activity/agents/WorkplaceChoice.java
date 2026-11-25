package pedsim.activity.agents;

import ec.util.MersenneTwisterFast;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.Map;
import pedsim.activity.parameters.ActivityPars;
import pedsim.core.parameters.RouteChoicePars;
import sim.graph.NodeGraph;

/**
 * Where a person works, drawn from where the jobs are against how far away they are.
 *
 * <p>The draw needs no agents, days or routes, so the decay can be calibrated outside a simulation.
 * Its two parameters are {@link ActivityPars#workplaceDistanceDecay} and {@link
 * ActivityPars#workplaceMinDistanceMetres}; what they have to reproduce is in {@code
 * COMMUTE_DISTANCE.md}.
 */
public final class WorkplaceChoice {

  /** The gravity model treats everything closer than this as being this far. */
  private static final double NEAREST_METRES = 10.0;

  private static final Map<Map<NodeGraph, Double>, Layout> layouts =
      Collections.synchronizedMap(new IdentityHashMap<>());

  private WorkplaceChoice() {}

  /**
   * Draws a workplace for a home location.
   *
   * <p>An attraction map is read once and laid out as arrays, so it must not change after its first
   * draw.
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
    return layouts
        .computeIfAbsent(attraction, Layout::new)
        .draw(homeNode, beta, floorMetres, random);
  }

  /** Forgets every layout; called when the city's static data is cleared. */
  public static void clear() {
    layouts.clear();
  }

  /** The nodes of an attraction map, in its order, with their coordinates and weights. */
  private static final class Layout {
    private final NodeGraph[] nodes;
    private final double[] x;
    private final double[] y;
    private final double[] attraction;

    Layout(Map<NodeGraph, Double> source) {
      nodes = new NodeGraph[source.size()];
      x = new double[nodes.length];
      y = new double[nodes.length];
      attraction = new double[nodes.length];
      int i = 0;
      for (Map.Entry<NodeGraph, Double> entry : source.entrySet()) {
        nodes[i] = entry.getKey();
        x[i] = nodes[i].getCoordinate().x;
        y[i] = nodes[i].getCoordinate().y;
        attraction[i] = entry.getValue();
        i++;
      }
    }

    NodeGraph draw(
        NodeGraph homeNode, double beta, double floorMetres, MersenneTwisterFast random) {
      double homeX = homeNode.getCoordinate().x;
      double homeY = homeNode.getCoordinate().y;
      boolean gravity = RouteChoicePars.useGravityModel;
      boolean inverseDistance = beta == 1.0;

      // Running totals; NaN marks a node closer than the floor, which no draw can fall under.
      double[] cumulative = new double[nodes.length];
      double total = 0.0;
      int last = -1;
      for (int i = 0; i < nodes.length; i++) {
        double dx = homeX - x[i];
        double dy = homeY - y[i];
        double distance = Math.sqrt(dx * dx + dy * dy);
        // Lower bound only, and unsourced: a decay with no floor can put a workplace next door. No
        // upper bound - a workplace is where it is, and what decides whether the commute is walked
        // is decideCommuteMode.
        if (distance < floorMetres) {
          cumulative[i] = Double.NaN;
          continue;
        }
        double weight = attraction[i];
        if (gravity) {
          double metres = Math.max(NEAREST_METRES, distance);
          weight /= inverseDistance ? metres : Math.pow(metres, beta);
        }
        total += weight;
        cumulative[i] = total;
        last = i;
      }
      if (last < 0 || total <= 0.0) {
        return null;
      }

      double r = random.nextDouble() * total;
      for (int i = 0; i < nodes.length; i++) {
        if (r <= cumulative[i]) {
          return nodes[i];
        }
      }
      return nodes[last];
    }
  }
}
