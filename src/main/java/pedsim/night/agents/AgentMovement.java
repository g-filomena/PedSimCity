package pedsim.night.agents;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.javatuples.Pair;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.planargraph.DirectedEdge;

import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.SimulationStateStore;
import pedsim.night.engine.PedSimCityNight;
import sim.graph.EdgeGraph;
import sim.graph.Graph;
import sim.graph.GraphUtils;
import sim.graph.NodeGraph;
import sim.routing.Astar;
import sim.routing.Route;

/** Handles movement along paths, including night-time rerouting and speed adjustments. */
public class AgentMovement extends pedsim.core.agents.AgentMovement {

  private static final int MAX_ALTERNATIVE_ROUTE_RELAXATIONS = 5;

  private final NightBehaviour nightBehaviour;
  private final PedSimCityNight state;
  private final Graph network;

  boolean originalRoute = true;

  public AgentMovement(Agent agent) {
    super(agent);
    this.network = SharedCognitiveMap.getCommunityPrimalNetwork();
    this.nightBehaviour = new NightBehaviour(agent, this);
    this.state = (PedSimCityNight) agent.getState();
    this.edgesToAvoid = new HashSet<>();
  }

  @Override
  public void initialisePath(Route route) {
    if (route == null || route.directedEdgesSequence == null || route.directedEdgesSequence.isEmpty()) {
      agent.setReachedDestination(true);
      return;
    }

    indexOnSequence = 0;
    this.directedEdgesSequence = new ArrayList<>(route.directedEdgesSequence);
    firstDirectedEdge = this.directedEdgesSequence.get(indexOnSequence);
    currentNode = (NodeGraph) firstDirectedEdge.getFromNode();
    agent.updateAgentPosition(currentNode.getCoordinate());
    setupEdge(firstDirectedEdge);
  }

  @Override
  protected void setupEdge(DirectedEdge directedEdge) {
    nightBehaviour.avoidParksWater = false;
    nightBehaviour.increaseSpeedAtNight = false;

    currentDirectedEdge = directedEdge;
    currentEdge = (EdgeGraph) currentDirectedEdge.getEdge();

    if (state.isDark && currentDirectedEdge != firstDirectedEdge) {
      nightBehaviour.checkLightLevel();
    }

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

  @Override
  public void keepWalking() {
    resetReach();

    if (nightBehaviour.increaseSpeedAtNight) {
      increaseReach();
    }

    currentIndex += reach;

    if (currentIndex > endIndex) {
      final Coordinate currentPos = indexedSegment.extractPoint(endIndex);
      agent.updateAgentPosition(currentPos);
      double residualMove = currentIndex - endIndex;
      transitionToNextEdge(residualMove);
    } else {
      final Coordinate currentPos = indexedSegment.extractPoint(currentIndex);
      agent.updateAgentPosition(currentPos);
    }

    SimulationStateStore.getInstance().updateAgent(agent);
  }

  /** Computes an alternative route from the current node to the destination. */
  void computeAlternativeRoute() {
    NodeGraph routeOrigin = (NodeGraph) currentDirectedEdge.getFromNode();
    Pair<NodeGraph, NodeGraph> routeKey = Pair.with(routeOrigin, agent.destinationNode);

    Map<Pair<NodeGraph, NodeGraph>, List<DirectedEdge>> cache =
        (agent.isVulnerableBoolean() || nightBehaviour.avoidParksWater)
            ? PedSimCityNight.altRoutesVulnerable
            : PedSimCityNight.altRoutesNonVulnerable;

    if (cache.containsKey(routeKey)) {
      resetPath(new ArrayList<>(cache.get(routeKey)));
      originalRoute = false;
      return;
    }

    defineEdgesToAvoid();

    Astar aStar = new Astar();
    Route alternativeRoute = computeAstarRoute(aStar, routeOrigin);
    int iteration = 0;

    while (alternativeRoute == null && iteration < MAX_ALTERNATIVE_ROUTE_RELAXATIONS) {
      relaxAvoidanceRules(iteration);
      alternativeRoute = computeAstarRoute(aStar, routeOrigin);
      iteration++;
    }

    if (alternativeRoute == null
        || alternativeRoute.directedEdgesSequence == null
        || alternativeRoute.directedEdgesSequence.isEmpty()) {
      nightBehaviour.avoidParksWater = false;
      nightBehaviour.increaseSpeedAtNight = true;
      return;
    }

    cache.put(routeKey, new ArrayList<>(alternativeRoute.directedEdgesSequence));
    resetPath(alternativeRoute.directedEdgesSequence);
    nightBehaviour.avoidParksWater = false;
    originalRoute = false;
  }

  private Route computeAstarRoute(Astar aStar, NodeGraph routeOrigin) {
    Set<Integer> edgeIDsToAvoid = new HashSet<>(GraphUtils.getEdgeIDs(edgesToAvoid));
    return aStar.astarRoute(routeOrigin, agent.destinationNode, network, edgeIDsToAvoid);
  }

  private void relaxAvoidanceRules(int iteration) {
    switch (iteration) {
      case 0 -> {
        edgesToAvoid.removeAll(SharedCognitiveMap.getNeighbourhoodEdges());
        edgesToAvoid.addAll(SharedCognitiveMap.getEdgesNonLitNonCommunityKnown());
        if (agent.isVulnerableBoolean() || nightBehaviour.avoidParksWater) {
          edgesToAvoid.addAll(SharedCognitiveMap.getEdgesWithinParksOrAlongWater());
        }
      }
      case 1 -> edgesToAvoid.removeAll(SharedCognitiveMap.getEdgesWithinParks());
      case 2 -> edgesToAvoid.removeAll(SharedCognitiveMap.getEdgesAlongWater());
      case 3 -> edgesToAvoid.removeAll(SharedCognitiveMap.getEdgesNonLitNonCommunityKnown());
      default -> edgesToAvoid.clear();
    }
  }

  private void defineEdgesToAvoid() {
    edgesToAvoid.clear();
    edgesToAvoid.add(currentEdge);
    edgesToAvoid.addAll(SharedCognitiveMap.getEdgesNonLitNonCommunityKnown());

    if (agent.isVulnerableBoolean()) {
      edgesToAvoid.addAll(SharedCognitiveMap.getCommunityPrimalNetwork().getEdges());
      edgesToAvoid.removeAll(SharedCognitiveMap.getCommunityKnownEdges());

      Set<EdgeGraph> knownEdges =
          new HashSet<>(
              GraphUtils.getEdgesFromEdgeIDs(
                  agent.getCognitiveMap().getAgentKnownEdges(), PedSimCity.edgesMap));
      edgesToAvoid.removeAll(knownEdges);
    }

    if (agent.isVulnerableBoolean() || nightBehaviour.avoidParksWater) {
      edgesToAvoid.addAll(SharedCognitiveMap.getEdgesWithinParksOrAlongWater());
    }

    edgesToAvoid.removeAll(agent.destinationNode.getEdges());
  }

  @Override
  protected void resetPath(List<DirectedEdge> directedEdgesSequence) {
    if (directedEdgesSequence == null || directedEdgesSequence.isEmpty()) {
      agent.setReachedDestination(true);
      return;
    }

    indexOnSequence = 0;
    this.directedEdgesSequence = new ArrayList<>(directedEdgesSequence);
    firstDirectedEdge = this.directedEdgesSequence.get(indexOnSequence);
    currentNode = (NodeGraph) firstDirectedEdge.getFromNode();
    agent.updateAgentPosition(currentNode.getCoordinate());
    edgesToAvoid.clear();
    setupEdge(firstDirectedEdge);
  }

  protected boolean canReroute() {
    return !currentEdge.getNodes().contains(agent.destinationNode) && indexOnSequence != 0 && originalRoute;
  }

  protected boolean isEdgeKnown(EdgeGraph edge) {
    return agent.getCognitiveMap().isEdgeKnown(edge);
  }
}
