package pedsim.night.agents;

import ec.util.MersenneTwisterFast;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.engine.Crowdness;
import pedsim.night.parameters.NightPars;
import sim.graph.EdgeGraph;
import sim.graph.NodeGraph;

public class NightBehaviour {

  NightAgent agent;
  NightAgentMovement nightMovement;

  protected boolean increaseSpeedAtNight = false;
  protected boolean avoidParksWater;

  protected MersenneTwisterFast random;

  /**
   * P(reroute) at or above the agent's sensitivity threshold, and the value used wherever the
   * darkness of the current edge cannot be graded (no continuous lux). Equal to the fixed split
   * this whole method used before darkness entered it, so an edge with no measurement behaves
   * exactly as it did.
   */
  private static final double BASELINE_REROUTE_PROBABILITY = 0.5;

  NightBehaviour(NightAgent agent, NightAgentMovement nightMovement) {
    this.agent = agent;
    this.nightMovement = nightMovement;
    this.random = agent.getRandom();
  }

  /**
   * Handles the case when the current edge is lit.
   *
   * <p>Important: non-vulnerable agents should not be processed by the vulnerable-agent rule-set
   * simply because the edge is lit. They only trigger night behaviour here when the edge is close
   * to parks/water and runtime rerouting is allowed.
   *
   * @param edge the edge to be approached
   */
  protected void whenLit(EdgeGraph edge) {
    if (isParkWaterNonVulnerable(edge)) {
      whenParkWater(edge);
      return;
    }

    if (agent.isVulnerable()) {
      whenLitVulnerable(edge);
    }
  }

  /**
   * Checks if the edge is next to a park/water and the agent is non-vulnerable.
   *
   * @param edge the edge to check
   * @return true if the edge is next to a park/water and the agent is non-vulnerable
   */
  protected boolean isParkWaterNonVulnerable(EdgeGraph edge) {
    return SharedCognitiveMap.isEdgeNextToParkOrWater(edge) && !agent.isVulnerable();
  }

  /**
   * Handles the case when an agent approaches an edge in proximity to parks or water.
   *
   * @param edge the edge to be approached
   */
  protected void whenParkWater(EdgeGraph edge) {
    if (nightMovement.canReroute()) {
      avoidParksWater = true;
      nightMovement.computeAlternativeRoute();
    } else {
      increaseSpeedAtNight = true;
    }
  }

  /**
   * Handles the case when a vulnerable agent is approaching a lit edge.
   *
   * @param edge the lit edge to be approached
   */
  protected void whenLitVulnerable(EdgeGraph edge) {
    // Unknown, not main road, not busy -> recompute.
    if (!agent.getCognitiveMap().isEdgeKnown(edge)
        && !SharedCognitiveMap.isEdgeMainRoad(edge)
        && !Crowdness.isEdgeCrowded(edge)) {
      if (nightMovement.canReroute()) {
        nightMovement.computeAlternativeRoute();
      } else {
        increaseSpeedAtNight = true;
      }
      return;
    }

    // Not main road and not crowded -> reroute or increase speed.
    if (!SharedCognitiveMap.isEdgeMainRoad(edge) && !Crowdness.isEdgeCrowded(edge)) {
      rerouteOrIncreaseSpeed();
    }
  }

  /**
   * Determines what to do when an agent approaches a non-lit edge.
   *
   * @param edge the non-lit edge to be approached
   */
  protected void whenNonLit(EdgeGraph edge) {
    if (agent.isVulnerable()) {
      nonLitVulnerable(edge);
    } else {
      nonLit(edge);
    }
  }

  /**
   * Handles the case when a non-vulnerable agent is approaching a non-lit edge.
   *
   * @param edge the non-lit edge to be approached
   */
  private void nonLit(EdgeGraph edge) {
    if (isParkWaterNonVulnerable(edge)) {
      whenParkWater(edge);
      return;
    }

    // Crowded -> ok.
    if (Crowdness.isEdgeCrowded(edge)) {
      return;
    }

    // Unknown and not main road -> reroute or increase speed. Otherwise (main road or known)
    // proceed.
    if (!agent.getCognitiveMap().isEdgeKnown(edge) && !SharedCognitiveMap.isEdgeMainRoad(edge)) {
      rerouteOrIncreaseSpeed();
    }
  }

  /**
   * Handles the case when a vulnerable agent is approaching a non-lit edge.
   *
   * @param edge the non-lit edge to be approached
   */
  protected void nonLitVulnerable(EdgeGraph edge) {
    // Unknown, not main road, not crowded -> reroute.
    if (!agent.getCognitiveMap().isEdgeKnown(edge)
        && !SharedCognitiveMap.isEdgeMainRoad(edge)
        && !Crowdness.isEdgeCrowded(edge)) {
      if (nightMovement.canReroute()) {
        nightMovement.computeAlternativeRoute();
      } else {
        increaseSpeedAtNight = true;
      }
      return;
    }

    // Unknown but crowded -> ok.
    if (!agent.getCognitiveMap().isEdgeKnown(edge) && Crowdness.isEdgeCrowded(edge)) {
      return;
    }

    // Main road but not crowded -> increase speed.
    if (SharedCognitiveMap.isEdgeMainRoad(edge) && !Crowdness.isEdgeCrowded(edge)) {
      increaseSpeedAtNight = true;
      return;
    }

    // Known, not main road, not crowded -> reroute or increase speed.
    if (agent.getCognitiveMap().isEdgeKnown(edge) && !Crowdness.isEdgeCrowded(edge)) {
      rerouteOrIncreaseSpeed();
    }
  }

  /**
   * Checks the current edge using explicit lighting pass/fail semantics.
   *
   * <p>Mean-light passes if measured mean_lux exists, is above the agent's threshold, and the edge
   * carries no unlit gap (see {@link #meanLightPasses}); or, with no measurement, if the binary lit
   * fallback says the edge is lit. Entrance-light passes if directional lux exists and is above
   * threshold, or if the binary lit fallback says the edge is lit. Missing data without a
   * binary-lit fallback fails closed.
   */
  protected void checkLightLevel() {
    final double threshold = agent.lightSensitivityThreshold;

    boolean meanLightPasses = meanLightPasses(threshold);
    boolean entranceLightPasses = entranceLightPasses(threshold);

    if (meanLightPasses && entranceLightPasses) {
      whenLit(nightMovement.currentEdge);
    } else {
      whenNonLit(nightMovement.currentEdge);
    }
  }

  /**
   * Whether the edge as a whole reads as lit to an agent with this threshold: bright enough on
   * average <i>and</i> with no unlit gap along it.
   *
   * <p>The second test is what an average cannot see. {@code min_lux} is the darkest 2 m sample
   * point on the edge, and a street that is bright at both ends and black in the middle passes on
   * {@code mean_lux} alone. It is compared against {@link NightPars#darkSpotLuxThreshold}, the
   * pipeline's own service level, not against the agent's personal threshold: the minimum over a
   * whole edge is an extreme value, and asking it to clear a 15-lux threshold would fail nearly
   * every street in the city.
   */
  private boolean meanLightPasses(double threshold) {
    var meanLuxAttr = nightMovement.currentEdge.attributes.get("mean_lux");
    if (meanLuxAttr == null) {
      // No measured lux: pass only if the binary lit flag says the edge is lit.
      return SharedCognitiveMap.getLitEdges().contains(nightMovement.currentEdge);
    }
    if (meanLuxAttr.getDouble() < threshold) {
      return false;
    }
    var minLuxAttr = nightMovement.currentEdge.attributes.get("min_lux");
    return minLuxAttr == null || minLuxAttr.getDouble() >= NightPars.darkSpotLuxThreshold;
  }

  private boolean entranceLightPasses(double threshold) {
    Double entranceLux = directionalEntranceLuxOrNull();
    if (entranceLux != null) {
      return entranceLux >= threshold;
    }
    // No directional value: pass only if the binary lit flag says the edge is lit.
    return SharedCognitiveMap.getLitEdges().contains(nightMovement.currentEdge);
  }

  /**
   * Directional entrance illuminance for the current directed edge, or null when no directional
   * value exists for this edge/direction.
   */
  private Double directionalEntranceLuxOrNull() {
    if (nightMovement.currentDirectedEdge == null) {
      return null;
    }

    NodeGraph fromNode = (NodeGraph) nightMovement.currentDirectedEdge.getFromNode();
    NodeGraph toNode = (NodeGraph) nightMovement.currentDirectedEdge.getToNode();
    return pedsim.night.engine.PedSimCityNight.directionalLuxMap.get(
        pedsim.night.engine.PedSimCityNight.luxKey(fromNode.getID(), toNode.getID()));
  }

  /**
   * Determines whether to reroute the agent or increase its speed.
   *
   * <p>The split is graded by how dark the current edge is rather than fixed: it starts at
   * {@link #BASELINE_REROUTE_PROBABILITY} at the agent's own sensitivity threshold and rises
   * toward {@link NightPars#maxRerouteProbabilityInDarkness} as the edge approaches full darkness.
   * Turning off a street and walking it faster are the two answers to the same fright, and which
   * one is taken should depend on how dark the street is; the fixed 0.5 said it never did.
   */
  protected void rerouteOrIncreaseSpeed() {
    double rerouteProbability =
        BASELINE_REROUTE_PROBABILITY
            + (NightPars.maxRerouteProbabilityInDarkness - BASELINE_REROUTE_PROBABILITY)
                * darknessDepth();
    if (random.nextDouble() < rerouteProbability && nightMovement.canReroute()) {
      nightMovement.computeAlternativeRoute();
    } else {
      increaseSpeedAtNight = true;
    }
  }

  /**
   * How far the current edge falls below the agent's sensitivity threshold, as a fraction: 0.0 at
   * or above the threshold (and wherever no continuous lux exists, so behaviour there is the one
   * the fixed split gave), 1.0 at 0 lux.
   */
  private double darknessDepth() {
    double threshold = agent.lightSensitivityThreshold;
    if (threshold <= 0) {
      return 0.0;
    }
    var meanLuxAttr = nightMovement.currentEdge.attributes.get("mean_lux");
    if (meanLuxAttr == null) {
      return 0.0;
    }
    double lux = meanLuxAttr.getDouble();
    if (lux >= threshold) {
      return 0.0;
    }
    return Math.min(1.0, (threshold - lux) / threshold);
  }
}
