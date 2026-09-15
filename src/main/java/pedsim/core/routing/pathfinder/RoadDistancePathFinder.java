package pedsim.core.routing.pathfinder;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import org.locationtech.jts.planargraph.DirectedEdge;
import pedsim.core.agents.Agent;
import pedsim.core.routing.pathfinding.DijkstraRoadDistance;
import sim.graph.NodeGraph;
import sim.routing.Route;

/**
 * A pathfinder for road-distance based route calculations. This class extends
 * the functionality of the base class PathFinder.
 */
public class RoadDistancePathFinder extends PathFinder {

  /**
   * Formulates a route based on road distance between the given origin and
   * destination nodes using the provided agent properties.
   *
   * @param originNode      the origin node;
   * @param destinationNode the destination node;
   * @param agent           The agent for which the route is computed.
   * @return a {@code Route} object representing the road-distance shortest path.
   */
  public Route roadDistance(NodeGraph originNode, NodeGraph destinationNode, Agent agent) {

    this.originNode = originNode;
    this.destinationNode = destinationNode;
    this.agent = agent;
    final DijkstraRoadDistance pathfinder = new DijkstraRoadDistance();

    partialSequence =
        pathfinder.dijkstraAlgorithm(
            originNode, destinationNode, destinationNode, directedEdgesToAvoid, this.agent);

    if (partialSequence.isEmpty()) {
      partialSequence = searchFullNetwork(originNode, destinationNode);
    }

    partialSequence = sequenceOnCommunityNetwork(partialSequence);
    fillRoute();
    return route;
  }

  /**
   * The same search again, over the whole network rather than the streets the agent knows.
   *
   * <p>When an individualised agent's known network cannot connect its origin to its destination,
   * the search widens rather than the trip being lost: walking unknown streets is what a person does
   * when the ones they know do not get them there. Without it the empty sequence becomes a two-node
   * route with no edges and a length of zero - a pedestrian who reaches its destination without
   * walking. Each widening is counted on the day trace.
   *
   * <p>The route it finds is one the agent could not have planned from its own knowledge, so the
   * length it plans against should carry a far larger error than a known route's. The model does
   * not represent that yet; sizing it needs a source.
   *
   * @param originNode the origin node.
   * @param destinationNode the destination node.
   * @return the widened search's edge sequence, empty if the whole network has no path either.
   */
  private List<DirectedEdge> searchFullNetwork(NodeGraph originNode, NodeGraph destinationNode) {
    if (agent == null
        || agent.getCognitiveMap() == null
        || !agent.getCognitiveMap().individualised) {
      return partialSequence; // not confined in the first place; the retry would be identical
    }
    DijkstraRoadDistance widened = new DijkstraRoadDistance();
    widened.ignoreKnownNetwork();
    List<DirectedEdge> sequence =
        widened.dijkstraAlgorithm(
            originNode, destinationNode, destinationNode, directedEdgesToAvoid, agent);
    if (!sequence.isEmpty() && agent.getState() != null) {
      agent.getState().trace().recordFullNetworkEscalation(false);
    }
    return sequence;
  }

  /**
   * Formulates a route based on road distance minimisation through a sequence of
   * intermediate nodes [originNode, ..., destinationNode] using the provided
   * agent properties. It allows combining the road-distance local minimisation
   * heuristic with navigational strategies based on the usage of urban elements.
   *
   * @param sequenceNodes sequence of intermediate nodes (e.g. on-route marks,
   *                      gateways) including the origin and the destination
   *                      nodes;
   * @param agent         The agent for which the route is computed.
   * @return a `Route' object representing the road-distance shortest path for the
   *         given sequence of landmarks and gateways.
   */
  public Route roadDistanceSequence(List<NodeGraph> sequenceNodes, Agent agent) {

    NodeGraph initialNode = sequenceNodes.get(0);

    routeSequence(
        sequenceNodes,
        agent,
        () -> {
          directedEdgesToAvoid = new HashSet<>(completeSequence);
          return new DijkstraRoadDistance()
              .dijkstraAlgorithm(
                  tmpOrigin, tmpDestination, destinationNode, directedEdgesToAvoid, agent);
        });

    if (completeSequence.isEmpty()) {
      agent.getProperties().setRegionBasedNavigation(false);
      return roadDistance(initialNode, destinationNode, this.agent);
    }
    // The whole sequence, not the last leg: fillRoute reads partialSequence, which at this point
    // holds only the final leg the loop computed.
    partialSequence = completeSequence;
    fillRoute();
    return route;
  }

  protected void fillRoute() {
    if (partialSequence == null || partialSequence.isEmpty()) {
      route.directedEdgesSequence = new ArrayList<>();
      route.nodesSequence = new ArrayList<>();
      if (originNode != null) {
        route.nodesSequence.add(originNode);
      }
      if (destinationNode != null && !destinationNode.equals(originNode)) {
        route.nodesSequence.add(destinationNode);
      }
      return;
    }
    route.directedEdgesSequence = partialSequence;
    route.computeRouteSequences();
  }
}
