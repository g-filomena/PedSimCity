package pedsim.night.routing;

import java.util.List;
import org.locationtech.jts.planargraph.DirectedEdge;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.parameters.TimePars;
import pedsim.night.agents.NightAgent;
import pedsim.night.engine.Enclosure;
import pedsim.night.engine.EyesOnStreet;
import pedsim.night.engine.NightLighting;
import pedsim.night.parameters.NightPars;
import sim.graph.EdgeGraph;

/**
 * What a street costs a night agent to walk after dark: its length, raised by how dark the agent
 * takes it to be, by lying in a park or along water, by how enclosed it is, and by how deserted.
 *
 * <p>{@code cost = length * (1 + darknessWeight * darkness(lux)) * (1 + parkWaterWeight) * (1 +
 * enclosureWeight * enclosure) * (1 + emptyStreetWeight * (1 - eyes))}, with the park term only on
 * park or waterside streets, enclosure from {@link Enclosure}, eyes from {@link EyesOnStreet} at the
 * hour, and every weight by vulnerability ({@link NightPars}). Darkness is {@link
 * NightLighting#darkness} at the agent's own reassurance level. The one function serves the plan
 * made before setting off, the re-plan made on the way and the decision whether to walk at all, so
 * the three cannot disagree about what darkness is worth.
 *
 * <p><b>How dark the agent takes a street to be.</b> For a street it knows, or has already seen on
 * this trip, the darkness of its measured illuminance; for any other, the expected darkness of its
 * class ({@link NightLighting#expectedDarkness}).
 *
 * <p><b>Others on the street.</b> The venues open along a street are known to every agent, so they
 * count at planning. A busy street - one the agent stands at and sees people on (Ferraro 1995) -
 * costs neither darkness nor emptiness.
 */
public final class NightRouteCost {

  private NightRouteCost() {}

  /** How dark the agent believes an edge to be, in [0, 1]. */
  public static double believedDarkness(NightAgent agent, EdgeGraph edge) {
    return agent.knowsLightingOf(edge)
        ? seenDarkness(agent, edge)
        : NightLighting.expectedDarkness(edge, agent.reassuranceLux);
  }

  /** How dark an edge is to the agent once it has seen it. */
  public static double seenDarkness(NightAgent agent, EdgeGraph edge) {
    return NightLighting.darkness(NightLighting.measuredLux(edge), agent.reassuranceLux);
  }

  /**
   * The cost of a walked path as a multiple of its length: what a leg feels like against what it
   * measures. 1.0 for an empty path.
   */
  public static double perceivedLengthRatio(NightAgent agent, List<DirectedEdge> path) {
    double metres = 0.0;
    for (DirectedEdge directedEdge : path) {
      metres += ((EdgeGraph) directedEdge.getEdge()).getLength();
    }
    return metres > 0.0 ? pathCost(agent, path) / metres : 1.0;
  }

  /**
   * The cost multiplier, at least 1.0, of an edge at a given darkness.
   *
   * @param darkness how dark the edge is taken to be, in [0, 1]
   * @param busy whether the edge is busy, which removes the darkness and emptiness terms
   */
  public static double factor(NightAgent agent, EdgeGraph edge, double darkness, boolean busy) {
    boolean vulnerable = agent.isVulnerable();
    double eyes = busy ? 1.0 : EyesOnStreet.at(edge, hourOfDay(agent));
    if (busy) {
      darkness = 0.0;
    }
    double emptyWeight =
        vulnerable
            ? NightPars.emptyStreetWeightVulnerable
            : NightPars.emptyStreetWeightNonVulnerable;
    double darknessWeight =
        vulnerable ? NightPars.darknessWeightVulnerable : NightPars.darknessWeightNonVulnerable;
    double parkWater =
        SharedCognitiveMap.getEdgesWithinParksOrAlongWater().contains(edge)
            ? (vulnerable
                ? NightPars.parkWaterWeightVulnerable
                : NightPars.parkWaterWeightNonVulnerable)
            : 0.0;
    double enclosureWeight =
        vulnerable ? NightPars.enclosureWeightVulnerable : NightPars.enclosureWeightNonVulnerable;
    return (1.0 + darknessWeight * darkness)
        * (1.0 + parkWater)
        * (1.0 + enclosureWeight * Enclosure.of(edge))
        * (1.0 + emptyWeight * (1.0 - eyes));
  }

  private static int hourOfDay(NightAgent agent) {
    return TimePars.getTime(agent.getState().schedule.getSteps()).getHour();
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
