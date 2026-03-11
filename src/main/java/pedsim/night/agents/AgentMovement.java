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
import pedsim.night.engine.PedSimCityNight;
import sim.graph.EdgeGraph;
import sim.graph.Graph;
import sim.graph.GraphUtils;
import sim.graph.NodeGraph;
import sim.routing.Astar;
import sim.routing.Route;

/**
 * The AgentMovement class is responsible for handling the movement of agents within the simulation.
 * It manages the agent's movement along the path, including transitions between edges, speed
 * adjustments, and the handling of various conditions such as night-time speed increase and
 * vulnerable agent behaviour.
 */
public class AgentMovement extends pedsim.core.agents.AgentMovement {

  private Agent agent;
  private NightBehaviour nightBehaviour;
  private List<DirectedEdge> edgesWalkedSoFar = new ArrayList<>();
  boolean originalRoute = true;

  PedSimCityNight state;
  private Graph network;

  public AgentMovement(Agent agent) {
    super();
    this.network = SharedCognitiveMap.getCommunityPrimalNetwork();
    this.nightBehaviour = new NightBehaviour(agent, this);
  }

  /**
   * Initialises the directedEdgesSequence (the path) for the agent.
   */
  public void initialisePath(Route route) {

    indexOnSequence = 0;
    this.directedEdgesSequence = route.directedEdgesSequence;

    // set up how to traverse this first link
    firstDirectedEdge = directedEdgesSequence.get(indexOnSequence);
    currentNode = (NodeGraph) firstDirectedEdge.getFromNode();
    agent.updateAgentPosition(currentNode.getCoordinate());
    // Sets the Agent up to proceed along an Edge
    setupEdge(firstDirectedEdge);
  }

  @Override
  /**
   * Sets the agent up to proceed along a specified edge.
   * 
   * @param directedEdge The DirectedEdge to traverse next.
   */
  protected void setupEdge(DirectedEdge directedEdge) {

    nightBehaviour.avoidParksWater = false;
    nightBehaviour.increaseSpeedAtNight = false; // removing potential increases
    currentDirectedEdge = directedEdge;
    currentEdge = (EdgeGraph) currentDirectedEdge.getEdge();

    if (state.isDark && currentDirectedEdge != firstDirectedEdge)
      nightBehaviour.checkLightLevel();

    updateCounts();

    if (PedSimCity.indexedEdgeCache.containsKey(currentDirectedEdge))
      indexedSegment = PedSimCity.indexedEdgeCache.get(currentDirectedEdge);
    else {
      addIndexedSegment(currentEdge);
      indexedSegment = PedSimCity.indexedEdgeCache.get(currentDirectedEdge);
    }

    currentIndex = indexedSegment.getStartIndex();
    endIndex = indexedSegment.getEndIndex();
    return;
  }

  /**
   * Moves the agent along the current path.
   */
  @Override
  public void keepWalking() {

    resetReach(); // as the segment might have changed level of crowdness
    if (nightBehaviour.increaseSpeedAtNight)
      increaseReach();
    // move along the current segment
    currentIndex += reach;

    // check to see if the progress has taken the current index beyond its goal
    // If so, proceed to the next edge
    if (currentIndex > endIndex) {
      final Coordinate currentPos = indexedSegment.extractPoint(endIndex);
      agent.updateAgentPosition(currentPos);
      double residualMove = currentIndex - endIndex;
      transitionToNextEdge(residualMove);
    } else {
      // just update the position!
      final Coordinate currentPos = indexedSegment.extractPoint(currentIndex);
      agent.updateAgentPosition(currentPos);
    }
  }

  /**
   * Computes an alternative route for the agent to avoid dangerous or unsuitable edges, reusing a
   * cached route if available.
   */
  void computeAlternativeRoute() {
    NodeGraph currentNode =
        (NodeGraph) edgesWalkedSoFar.get(edgesWalkedSoFar.size() - 1).getToNode();
    Pair<NodeGraph, NodeGraph> routeKey = Pair.with(currentNode, agent.destinationNode);
    Map<Pair<NodeGraph, NodeGraph>, List<DirectedEdge>> cache =
        (agent.isVulnerable() || nightBehaviour.avoidParksWater)
            ? PedSimCityNight.altRoutesVulnerable
            : PedSimCityNight.altRoutesNonVulnerable;

    // Check if a cached route already exists
    if (cache.containsKey(routeKey)) {
      resetPath(new ArrayList<>(cache.get(routeKey)));
      originalRoute = false;
      return;
    }

    defineEdgesToAvoid();
    Astar aStar = new Astar();

    Set<Integer> edgeIDsToAvoid = new HashSet<>(GraphUtils.getEdgeIDs(edgesToAvoid));
    Route alternativeRoute =
        aStar.astarRoute(currentNode, agent.destinationNode, network, edgeIDsToAvoid);

    int iteration = 0;
    while (alternativeRoute == null) {
      switch (iteration) {
        case 0 -> {
          // Add secondary roads, still try avoiding non-lit and parks/water
          edgesToAvoid.removeAll(SharedCognitiveMap.getNeighbourhoodEdges());
          edgesToAvoid.addAll(SharedCognitiveMap.getEdgesNonLitNonCommunityKnown());
          if (agent.isVulnerable() || nightBehaviour.avoidParksWater)
            edgesToAvoid.addAll(SharedCognitiveMap.getEdgesWithinParksOrAlongWater());
        }
        // give up park avoidance
        case 1 -> edgesToAvoid.removeAll(SharedCognitiveMap.getEdgesWithinParks());
        // give up water avoidance
        case 2 -> edgesToAvoid.removeAll(SharedCognitiveMap.getEdgesAlongWater());
        // give up non-lit avoidance
        case 3 -> edgesToAvoid.removeAll(SharedCognitiveMap.getEdgesNonLitNonCommunityKnown());
        default -> {
          edgesToAvoid.clear();
        }
      }

      edgeIDsToAvoid.clear();
      edgeIDsToAvoid = new HashSet<>(GraphUtils.getEdgeIDs(edgesToAvoid));
      alternativeRoute = aStar.astarRoute(currentNode, agent.destinationNode,
          SharedCognitiveMap.getCommunityPrimalNetwork(), edgeIDsToAvoid);
      iteration++;
    }

    // Cache and apply the new route
    cache.put(routeKey, new ArrayList<>(alternativeRoute.directedEdgesSequence));
    resetPath(alternativeRoute.directedEdgesSequence);
    nightBehaviour.avoidParksWater = false;
    originalRoute = false;
  }

  /**
   * Gets the set of edges that the agent should avoid during movement.
   * 
   * @return The set of edges to avoid.
   */
  private void defineEdgesToAvoid() {

    // the disregarded one
    edgesToAvoid.add(currentEdge);

    // non-lit roads
    edgesToAvoid.addAll(SharedCognitiveMap.getEdgesNonLitNonCommunityKnown());

    // for vulnerable:
    if (agent.isVulnerable()) {
      // 1) add everything,
      edgesToAvoid.addAll(SharedCognitiveMap.getCommunityPrimalNetwork().getEdges());
      // 2) remove primary,
      edgesToAvoid.removeAll(SharedCognitiveMap.getCommunityKnownEdges());
      // 3) remove known
      Set<EdgeGraph> knownEdges = new HashSet<>(GraphUtils
          .getEdgesFromEdgeIDs(agent.getCognitiveMap().getAgentKnownEdges(), PedSimCity.edgesMap));
      edgesToAvoid.removeAll(knownEdges);
    }

    // for vulnerable and non-vulnerable agents who do not feel like water and parks
    if (agent.isVulnerable() || nightBehaviour.avoidParksWater)
      edgesToAvoid.addAll(SharedCognitiveMap.getEdgesWithinParksOrAlongWater());

    edgesToAvoid.removeAll(agent.destinationNode.getEdges());
  }

  /**
   * Checks if the agent can reroute based on the current edge and destination.
   *
   * @return true if the agent can reroute, false otherwise.
   */
  protected boolean canReroute() {

    if (currentEdge.getNodes().contains(agent.destinationNode) || indexOnSequence == 0
        || !originalRoute)
      return false;
    return true;
  }

  /**
   * Checks if the edge is known by the agent.
   *
   * @param edge The edge to check.
   * @return true if the edge is known by the agent, false otherwise.
   */
  protected boolean isEdgeKnown(EdgeGraph edge) {
    return agent.getCognitiveMap().isEdgeKnown(edge);
  }


}
