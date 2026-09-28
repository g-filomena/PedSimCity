package pedsim.night.agents;

import java.util.List;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.planargraph.DirectedEdge;
import pedsim.core.engine.Crowdness;
import pedsim.core.engine.PedSimCity;
import pedsim.night.engine.NightLighting;
import pedsim.night.engine.PedSimCityNight;
import pedsim.night.routing.NightRouteCost;
import pedsim.night.routing.search.DijkstraRoadDistanceNight;
import sim.graph.EdgeGraph;
import sim.graph.NodeGraph;
import sim.routing.Route;

/**
 * Walks a night agent along its route, and changes the route when what it sees makes the rest of
 * the trip dearer than it planned.
 *
 * <p><b>Plan with beliefs, re-plan on surprise.</b> The route is planned before setting off, at the
 * costs {@link NightRouteCost} gives the streets the agent believes it knows: measured lighting
 * for streets it knows, the typical lighting of their class for the rest. Arriving at a street it
 * did not know, it sees how the street is actually lit. If that makes the street dearer than
 * assumed, it plans again from where it stands to its destination, and takes the new route only if
 * it is cheaper than finishing the old one at the costs it now believes. A street seen once is
 * known for the rest of the trip, so it cannot surprise twice, and every re-plan strictly lowers
 * the cost of what is left; there is no cap to set.
 *
 * <p>Pedestrians do update route decisions on the way as they perceive new information (Tong and
 * Bode 2022); what bounds a detour here is its cost, as in revealed-preference route choice (e.g.
 * Broach and Dill 2015), not a forbidden set of streets.
 *
 * <p><b>Walking faster.</b> On a street that reads as unlit at the agent's own sensitivity and is
 * not busy, the agent speeds up - pedestrians hurry through dim and unfamiliar places (Fotios et
 * al. 2019; Basu et al. 2022). This is the reaction to a street it is walking, not a choice of
 * where to go.
 */
public class NightAgentMovement extends pedsim.core.agents.AgentMovement {

  private final PedSimCityNight state;
  private final NightAgent nightAgent;

  /** Whether the agent is hurrying on the current edge; reset on every edge. */
  private boolean increaseSpeedAtNight = false;

  public NightAgentMovement(NightAgent agent) {
    super(agent);
    this.nightAgent = agent;
    this.state = (PedSimCityNight) agent.getState();
  }

  /** Starts a trip: clears what was seen on the last one and sets up the first edge. */
  @Override
  public void initialisePath(Route route) {
    if (route == null
        || route.directedEdgesSequence == null
        || route.directedEdgesSequence.isEmpty()) {
      agent.setReachedDestination(true);
      return;
    }
    nightAgent.accumulatedLuxMetres = 0.0;
    nightAgent.metresWalkedForLux = 0.0;
    nightAgent.edgesWalked = 0;
    nightAgent.lightingSeenThisTrip.clear();

    indexOnSequence = 0;
    this.directedEdgesSequence = route.directedEdgesSequence;
    firstDirectedEdge = directedEdgesSequence.get(indexOnSequence);
    currentNode = (NodeGraph) firstDirectedEdge.getFromNode();
    agent.updateAgentPosition(currentNode.getCoordinate());
    setupEdge(firstDirectedEdge);
  }

  /**
   * Sets the agent up on the next edge. After dark it first looks at the edge, which may replace
   * the route and with it the edge; everything after that applies to the edge actually walked.
   */
  @Override
  protected void setupEdge(DirectedEdge directedEdge) {
    increaseSpeedAtNight = false;
    currentDirectedEdge = directedEdge;
    currentEdge = (EdgeGraph) currentDirectedEdge.getEdge();

    if (state.isDark) {
      // A re-plan puts a different edge ahead, which the agent then looks at in turn. Each pass
      // adds an edge to what it has seen, so the loop ends.
      while (lookAhead()) {
        // the route changed; look at its first edge
      }
      increaseSpeedAtNight = !Crowdness.isEdgeCrowded(currentEdge) && !readsAsLit(currentEdge);
    }

    recordLightingMetric(currentEdge);
    updateCounts();

    if (PedSimCity.indexedEdgeCache.containsKey(currentDirectedEdge)) {
      indexedSegment = PedSimCity.indexedEdgeCache.get(currentDirectedEdge);
    } else {
      addIndexedSegment(currentEdge);
      indexedSegment = PedSimCity.indexedEdgeCache.get(currentDirectedEdge);
    }
    currentIndex = indexedSegment.getStartIndex();
    endIndex = indexedSegment.getEndIndex();
  }

  /**
   * Looks at the edge ahead and re-plans if what it sees is worse than what was assumed.
   *
   * @return true if the route was replaced
   */
  private boolean lookAhead() {
    EdgeGraph edge = currentEdge;
    if (nightAgent.knowsLightingOf(edge)) {
      return false;
    }
    double assumed = NightRouteCost.factor(nightAgent, edge, NightLighting.typicalLux(edge), false);
    nightAgent.lightingSeenThisTrip.add(edge);
    double seen =
        NightRouteCost.factor(
            nightAgent, edge, NightLighting.measuredLux(edge), Crowdness.isEdgeCrowded(edge));
    if (seen <= assumed) {
      return false;
    }
    return replan();
  }

  /**
   * Plans again from the start of the edge ahead to the destination, and takes the new route if it
   * is cheaper than the remainder of the current one at the costs the agent now believes.
   */
  private boolean replan() {
    NodeGraph from = (NodeGraph) currentDirectedEdge.getFromNode();
    List<DirectedEdge> remainder =
        directedEdgesSequence.subList(indexOnSequence, directedEdgesSequence.size());
    List<DirectedEdge> alternative =
        new DijkstraRoadDistanceNight().dijkstraAlgorithm(from, agent.destinationNode, agent);
    if (alternative.isEmpty()
        || NightRouteCost.pathCost(nightAgent, alternative)
            >= NightRouteCost.pathCost(nightAgent, remainder)) {
      return false;
    }
    agent.spookLocations.add(from.getCoordinate());
    resetPath(alternative);
    return true;
  }

  /**
   * Whether the edge reads as lit to this agent: bright enough on average with no unlit gap ({@link
   * NightLighting#isLit}), and bright enough where the agent enters it. With no directional value
   * for the entrance, the OSM tag decides.
   */
  private boolean readsAsLit(EdgeGraph edge) {
    double threshold = nightAgent.lightSensitivityThreshold;
    if (!NightLighting.isLit(edge, threshold)) {
      return false;
    }
    NodeGraph fromNode = (NodeGraph) currentDirectedEdge.getFromNode();
    NodeGraph toNode = (NodeGraph) currentDirectedEdge.getToNode();
    Double entranceLux =
        PedSimCityNight.directionalLuxMap.get(
            PedSimCityNight.luxKey(fromNode.getID(), toNode.getID()));
    return entranceLux != null ? entranceLux >= threshold : NightLighting.isTaggedLit(edge);
  }

  /**
   * Records the lighting of the edge walked, over its length: {@link NightLighting#measuredLux},
   * so dark metres count at the illuminance they have.
   */
  private void recordLightingMetric(EdgeGraph edge) {
    nightAgent.edgesWalked++;
    double metres = edge.getLength();
    nightAgent.accumulatedLuxMetres += NightLighting.measuredLux(edge) * metres;
    nightAgent.metresWalkedForLux += metres;
  }

  /** Moves the agent along the current path, faster on an edge it is hurrying through. */
  @Override
  public void keepWalking() {
    resetReach();
    if (increaseSpeedAtNight) {
      increaseReach();
    }
    currentIndex += reach;
    if (currentIndex > endIndex) {
      final Coordinate currentPos = indexedSegment.extractPoint(endIndex);
      agent.updateAgentPosition(currentPos);
      transitionToNextEdge(currentIndex - endIndex);
    } else {
      agent.updateAgentPosition(indexedSegment.extractPoint(currentIndex));
    }
  }

  /**
   * Puts the agent on a new edge sequence from where it stands. The base implementation anchors the
   * position to the trip's first edge, which mid-trip would move the agent back to its origin.
   *
   * @param newSequence the new sequence of directed edges to follow
   */
  @Override
  protected void resetPath(List<DirectedEdge> newSequence) {
    indexOnSequence = 0;
    this.directedEdgesSequence = newSequence;
    currentDirectedEdge = newSequence.get(indexOnSequence);
    currentEdge = (EdgeGraph) currentDirectedEdge.getEdge();
    currentNode = (NodeGraph) currentDirectedEdge.getFromNode();
    agent.updateAgentPosition(currentNode.getCoordinate());
  }
}
