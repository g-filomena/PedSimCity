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

    // Unknown and not main road -> reroute or increase speed.
    if (!agent.getCognitiveMap().isEdgeKnown(edge) && !SharedCognitiveMap.isEdgeMainRoad(edge)) {
      rerouteOrIncreaseSpeed();
      return;
    }

    // Main road or known -> ok.
    if (SharedCognitiveMap.isEdgeMainRoad(edge) || agent.getCognitiveMap().isEdgeKnown(edge)) {
      return;
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
   * Checks the light level of the current edge and dispatches to the lit / non-lit behaviour.
   *
   * <p>Each lighting signal (whole-edge average, directional entrance) is applied only when its
   * dataset was loaded for this city. Within a loaded dataset a missing per-edge value means "no
   * light there" (dark), so incomplete data can no longer silently pass the check; a dataset that
   * was never loaded simply does not contribute its term. When no lighting data exists at all, the
   * edge is treated as lit — there is no basis to judge it dark.
   */
  protected void checkLightLevel() {
    final double threshold = agent.lightSensitivityThreshold;
    boolean tooDark = false;

    // --- Whole-edge average illuminance, with the binary "lit" flag as a fallback. ---
    var meanLuxAttr = nightMovement.currentEdge.attributes.get("mean_lux");
    if (meanLuxAttr != null) {
      if (meanLuxAttr.getDouble() < threshold) {
        tooDark = true;
      }
    } else {
      // No continuous value on this edge. Judge by the binary lit flag only if some edge-level
      // lighting dataset exists; if none exists at all, this term is not applied.
      boolean edgeLightingDataExists =
          !pedsim.night.engine.PedSimCityNight.illuminatedEdges.getGeometries().isEmpty()
              || !SharedCognitiveMap.getLitEdges().isEmpty();
      if (edgeLightingDataExists
          && !SharedCognitiveMap.getLitEdges().contains(nightMovement.currentEdge)) {
        tooDark = true;
      }
    }

    // --- Directional entrance illuminance, applied only where the directional dataset was loaded.
    // A missing entry within a loaded dataset counts as dark (0), consistent with the mean term. ---
    if (!pedsim.night.engine.PedSimCityNight.directionalLuxMap.isEmpty()
        && directionalEntranceLux() < threshold) {
      tooDark = true;
    }

    if (tooDark) {
      whenNonLit(nightMovement.currentEdge);
    } else {
      whenLit(nightMovement.currentEdge);
    }
  }

  /**
   * Directional entrance illuminance for the current directed edge, from the directional lighting
   * lookup. Returns 0.0 (dark) when the dataset is loaded but has no entry for this edge/direction,
   * so incomplete directional data is treated conservatively rather than silently passing.
   */
  private double directionalEntranceLux() {
    if (nightMovement.currentDirectedEdge == null) {
      return 0.0;
    }
    NodeGraph fromNode = (NodeGraph) nightMovement.currentDirectedEdge.getFromNode();
    NodeGraph toNode = (NodeGraph) nightMovement.currentDirectedEdge.getToNode();
    if (fromNode == null || toNode == null) {
      return 0.0;
    }
    Double luxValue =
        pedsim.night.engine.PedSimCityNight.directionalLuxMap.get(
            pedsim.night.engine.PedSimCityNight.luxKey(fromNode.getID(), toNode.getID()));
    return luxValue != null ? luxValue : 0.0;
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
