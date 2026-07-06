package pedsim.night.agents;

import java.util.Random;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.engine.Crowdness;
import sim.graph.EdgeGraph;
import sim.graph.NodeGraph;

public class NightBehaviour {

  NightAgent agent;
  NightAgentMovement nightMovement;

  protected boolean increaseSpeedAtNight = false;
  protected boolean avoidParksWater;

  protected Random random = new Random();

  NightBehaviour(NightAgent agent, NightAgentMovement nightMovement) {
    this.agent = agent;
    this.nightMovement = nightMovement;
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

    // Unknown and not main road -> reroute or increase speed. Otherwise (main road or known) proceed.
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
     * mean-light passes if measured mean_lux exists and is above threshold, or if the binary
     * lit fallback says the edge is lit. Entrance-light passes if directional lux exists and is
     * above threshold, or if the binary lit fallback says the edge is lit. Missing data without
     * a binary-lit fallback fails closed.
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

    private boolean meanLightPasses(double threshold) {
        var meanLuxAttr = nightMovement.currentEdge.attributes.get("mean_lux");
        if (meanLuxAttr != null) {
            return meanLuxAttr.getDouble() >= threshold;
        }
        // No measured lux: pass only if the binary lit flag says the edge is lit.
        return SharedCognitiveMap.getLitEdges().contains(nightMovement.currentEdge);
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

  /** Determines whether to reroute the agent or increase its speed. */
  protected void rerouteOrIncreaseSpeed() {
    if (random.nextDouble() < 0.5 && nightMovement.canReroute()) {
      nightMovement.computeAlternativeRoute();
    } else {
      increaseSpeedAtNight = true;
    }
  }
}
