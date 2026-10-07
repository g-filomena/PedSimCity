package pedsim.night.routing;

import java.util.List;
import org.locationtech.jts.planargraph.DirectedEdge;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.night.agents.NightAgent;
import pedsim.night.engine.NightLighting;
import pedsim.night.parameters.NightPars;
import sim.graph.EdgeGraph;

/**
 * What a street costs a night agent to walk after dark: its length, raised by how dark the agent
 * takes it to be and by lying in a park or along water.
 *
 * <p>{@code cost = length * (1 + darknessWeight * darkness(lux)) * (1 + parkWaterWeight)}, with the
 * park term only on park or waterside streets and both weights by vulnerability ({@link
 * NightPars}). Darkness is {@link NightLighting#darkness}. The one function serves the plan made
 * before setting off and the re-plan made on the way, so the two cannot disagree about what
 * darkness is worth.
 *
 * <p><b>How dark the agent takes a street to be.</b> For a street it knows, or has already seen on
 * this trip, the darkness of its measured illuminance; for any other, the expected darkness of its
 * class ({@link NightLighting#expectedDarkness}). A busy street costs no darkness: the presence of
 * others reassures (Ferraro 1995), and only an agent standing at it can see that it is busy.
 */
public final class NightRouteCost {

  private NightRouteCost() {}

  /** How dark the agent believes an edge to be, in [0, 1]. */
  public static double believedDarkness(NightAgent agent, EdgeGraph edge) {
    return agent.knowsLightingOf(edge)
        ? NightLighting.darkness(NightLighting.measuredLux(edge))
        : NightLighting.expectedDarkness(edge);
  }

  /**
   * The cost multiplier, at least 1.0, of an edge at a given darkness.
   *
   * @param darkness how dark the edge is taken to be, in [0, 1]
   * @param busy whether the edge is busy, which removes the darkness term
   */
  public static double factor(NightAgent agent, EdgeGraph edge, double darkness, boolean busy) {
    boolean vulnerable = agent.isVulnerable();
    if (busy) {
      darkness = 0.0;
    }
    double darknessWeight =
        vulnerable ? NightPars.darknessWeightVulnerable : NightPars.darknessWeightNonVulnerable;
    double parkWater =
        SharedCognitiveMap.getEdgesWithinParksOrAlongWater().contains(edge)
            ? (vulnerable
                ? NightPars.parkWaterWeightVulnerable
                : NightPars.parkWaterWeightNonVulnerable)
            : 0.0;
    return (1.0 + darknessWeight * darkness) * (1.0 + parkWater);
  }

  /** The multiplier the agent plans with: believed darkness, busyness unseen. */
  public static double plannedFactor(NightAgent agent, EdgeGraph edge) {
    return factor(agent, edge, believedDarkness(agent, edge), false);
  }

  /** The planned cost of walking a sequence of edges, without perception error. */
  public static double pathCost(NightAgent agent, List<DirectedEdge> path) {
    double cost = 0.0;
    for (DirectedEdge directedEdge : path) {
      EdgeGraph edge = (EdgeGraph) directedEdge.getEdge();
      cost += edge.getLength() * plannedFactor(agent, edge);
    }
    return cost;
  }
}
