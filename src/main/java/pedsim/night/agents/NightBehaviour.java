package pedsim.night.agents;

import java.util.Random;

import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.engine.Crowdness;
import sim.graph.EdgeGraph;
import sim.graph.NodeGraph;

public class NightBehaviour {

  Agent agent;
  AgentMovement nightMovement;

  protected boolean increaseSpeedAtNight = false;
  protected boolean avoidParksWater;

  protected Random random = new Random();

  NightBehaviour(Agent agent, AgentMovement nightMovement) {
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

  /** Checks the light level of the current edge. */
  protected void checkLightLevel() {
    double meanLux = 0.0;
    
    if (nightMovement.currentEdge.attributes.get("mean_lux") != null) {
      meanLux = nightMovement.currentEdge.attributes.get("mean_lux").getDouble();
    } else if (SharedCognitiveMap.getLitEdges().contains(nightMovement.currentEdge)) {
      meanLux = Double.MAX_VALUE; // Fallback to fully lit if old binary data is used
    }
    
    double entranceLux = Double.MAX_VALUE;
    if (nightMovement.currentDirectedEdge != null) {
      NodeGraph fromNode = (NodeGraph) nightMovement.currentDirectedEdge.getFromNode();
      NodeGraph toNode = (NodeGraph) nightMovement.currentDirectedEdge.getToNode();
      if (fromNode != null && toNode != null) {
        long edgeKey = pedsim.night.engine.PedSimCityNight.luxKey(fromNode.getID(), toNode.getID());
        Double luxValue = pedsim.night.engine.PedSimCityNight.directionalLuxMap.get(edgeKey);
        if (luxValue != null) {
          entranceLux = luxValue;
        }
      }
    }
    
    // If either the average street light or the immediate entrance is too dark, reroute
    if (meanLux >= agent.lightSensitivityThreshold && entranceLux >= agent.lightSensitivityThreshold) {
      whenLit(nightMovement.currentEdge);
    } else {
      whenNonLit(nightMovement.currentEdge);
    }
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
