package pedsim.night.agents;

import java.util.Random;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.engine.Crowdness;
import sim.graph.EdgeGraph;

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
   * Handles the case when the edge is lit.
   *
   * @param edge The edge to be approached.
   */
  protected void whenLit(EdgeGraph edge) {

    // vulnerable agents avoid parks at night at planning phase.
    if (isParkWaterNonVulnerable(edge))
      whenParkWater(edge);
    else
      whenLitVulernable(edge);
  }

  /**
   * Checks if the edge is next to a park or water and the agent is non-vulnerable.
   *
   * @param edge The edge to check.
   * @return true if the edge is next to a park or water and non-vulnerable, false otherwise.
   */
  protected boolean isParkWaterNonVulnerable(EdgeGraph edge) {
    return SharedCognitiveMap.isEdgeNextToParkOrWater(edge) && !agent.isVulnerable();
  }

  /**
   * Handles the case when the agent is approaching an edge in proximity or parks or water.
   *
   * @param edge The edge to be approached.
   */
  protected void whenParkWater(EdgeGraph edge) {
    // vulnerable avoid parks at night at planning phase.
    if (nightMovement.canReroute()) {
      avoidParksWater = true;
      nightMovement.computeAlternativeRoute();
    } else
      increaseSpeedAtNight = true;
  }

  /**
   * Handles the case when a vulnerable agent is approaching a lit edge.
   *
   * @param edge The lit edge to be approached.
   */
  protected void whenLitVulernable(EdgeGraph edge) {

    // not known, not main road, not busy -> recompute
    if (!agent.getCognitiveMap().isEdgeKnown(edge) && !SharedCognitiveMap.isEdgeMainRoad(edge)
        && !Crowdness.isEdgeCrowded(edge)) {
      if (nightMovement.canReroute())
        nightMovement.computeAlternativeRoute();
      else
        increaseSpeedAtNight = true;
    }
    // not main road and not crowded -> reroute or increase speed
    else if (!SharedCognitiveMap.isEdgeMainRoad(edge) && !Crowdness.isEdgeCrowded(edge)) {
      if (random.nextDouble() < 0.5 && nightMovement.canReroute())
        nightMovement.computeAlternativeRoute();
      else
        increaseSpeedAtNight = true;
    }
  }

  /**
   * Determines what to do, based on the agent vulnerability, when an agent is approaching a non-lit
   * edge.
   *
   * @param edge The lit edge to be approached.
   */
  protected void whenNonLit(EdgeGraph edge) {

    if (agent.isVulnerable())
      nonLitVulnerable(edge);
    else
      nonLit(edge);
  }

  /**
   * Handles the case when a non-vulnerable agent is approaching a non-lit edge.
   *
   * @param edge The lit edge to be approached.
   */
  private void nonLit(EdgeGraph edge) {

    if (isParkWaterNonVulnerable(edge)) {
      whenParkWater(edge);
      return;
    }
    // crowded -> ok
    else if (Crowdness.isEdgeCrowded(edge))
      return;
    // not known, not main road, not crowded -> reroute or increase speed
    else if (!agent.getCognitiveMap().isEdgeKnown(edge) && !SharedCognitiveMap.isEdgeMainRoad(edge))
      rerouteOrIncreaseSpeed();
    // main road or known, not crowded -> ok
    else if (SharedCognitiveMap.isEdgeMainRoad(edge) || agent.getCognitiveMap().isEdgeKnown(edge))
      return;
  }

  /**
   * Handles the case when a vulnerable agent is approaching a non-lit edge.
   *
   * @param edge The lit edge to be approached.
   */
  protected void nonLitVulnerable(EdgeGraph edge) {

    // not known, not main road, not crowded -> reroute
    if (!agent.getCognitiveMap().isEdgeKnown(edge) && !SharedCognitiveMap.isEdgeMainRoad(edge)
        && !Crowdness.isEdgeCrowded(edge)) {
      if (nightMovement.canReroute())
        nightMovement.computeAlternativeRoute();
      else
        increaseSpeedAtNight = true;
    }
    // not known but crowded -> OK
    else if (!agent.getCognitiveMap().isEdgeKnown(edge) && Crowdness.isEdgeCrowded(edge))
      return;
    // main road but not crowded -> increase speed
    else if (SharedCognitiveMap.isEdgeMainRoad(edge) && !Crowdness.isEdgeCrowded(edge))
      increaseSpeedAtNight = true;
    // known, not main road, not crowded -> > reroute or increase speed
    else if (agent.getCognitiveMap().isEdgeKnown(edge) && !Crowdness.isEdgeCrowded(edge))
      rerouteOrIncreaseSpeed();
  }

  /**
   * Checks the light level of the current edge.
   */
  protected void checkLightLevel() {

    // edge is lit but night
    if (SharedCognitiveMap.getLitEdges().contains(nightMovement.currentEdge))
      whenLit(nightMovement.currentEdge);
    else
      whenNonLit(nightMovement.currentEdge);
  }

  /**
   * Determines whether to reroute the agent or increase its speed.
   */
  protected void rerouteOrIncreaseSpeed() {
    if (random.nextDouble() < 0.5 && nightMovement.canReroute())
      nightMovement.computeAlternativeRoute();
    else
      increaseSpeedAtNight = true;
  }

}
