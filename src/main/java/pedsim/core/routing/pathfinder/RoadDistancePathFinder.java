package pedsim.core.routing.pathfinder;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import pedsim.core.agents.Agent;
import pedsim.core.routing.pathfinding.DijkstraRoadDistance;
import sim.graph.NodeGraph;
import sim.routing.Route;

/**
 * A pathfinder for road-distance based route calculations. This class extends the functionality of
 * the base class PathFinder.
 */
public class RoadDistancePathFinder extends PathFinder {

  /**
   * Formulates a route based on road distance between the given origin and destination nodes using
   * the provided agent properties.
   *
   * @param originNode the origin node;
   * @param destinationNode the destination node;
   * @param agent The agent for which the route is computed.
   * @return a {@code Route} object representing the road-distance shortest path.
   */
  public Route roadDistance(NodeGraph originNode, NodeGraph destinationNode, Agent agent) {

    this.agent = agent;
    final DijkstraRoadDistance pathfinder = new DijkstraRoadDistance();
    if (!agent.getCognitiveMap().getNodesInKnownNetwork().contains(destinationNode)) {
      System.out.println("Destination node is not in the known network: " + destinationNode);
    }

    partialSequence = pathfinder.dijkstraAlgorithm(originNode, destinationNode, destinationNode,
        directedEdgesToAvoid, this.agent);

    partialSequence = sequenceOnCommunityNetwork(partialSequence);
    fillRoute();
    return route;
  }

  /**
   * Formulates a route based on road distance minimisation through a sequence of intermediate nodes
   * [originNode, ..., destinationNode] using the provided agent properties. It allows combining the
   * road-distance local minimisation heuristic with navigational strategies based on the usage of
   * urban elements.
   *
   * @param sequenceNodes sequence of intermediate nodes (e.g. on-route marks, gateways) including
   *        the origin and the destination nodes;
   * @param agent The agent for which the route is computed.
   * @return a `Route' object representing the road-distance shortest path for the sequence of
   *         nodes.
   */
  public Route roadDistanceSequence(List<NodeGraph> sequenceNodes, Agent agent) {

    this.agent = agent;
    this.sequenceNodes = new ArrayList<>(sequenceNodes);

    // originNode
    originNode = this.sequenceNodes.get(0);
    NodeGraph initialNode = originNode;
    tmpOrigin = originNode;
    destinationNode = sequenceNodes.get(sequenceNodes.size() - 1);
    this.sequenceNodes.remove(0);

    for (NodeGraph currentNode : this.sequenceNodes) {
      moveOn = false;
      tmpDestination = currentNode;

      // check if this tmpDestination has been traversed already
      if (nodesFromEdgesSequence(completeSequence).contains(tmpDestination)) {
        controlPath(tmpDestination);
        tmpOrigin = tmpDestination;
        continue;
      }

      if (haveEdgesBetween()) {
        continue;
      }

      directedEdgesToAvoid = new HashSet<>(completeSequence);
      DijkstraRoadDistance pathfinder = new DijkstraRoadDistance();
      partialSequence = pathfinder.dijkstraAlgorithm(tmpOrigin, tmpDestination, destinationNode,
          directedEdgesToAvoid, agent);
      while (partialSequence.isEmpty() && !moveOn) {
        backtracking(tmpDestination);
      }

      if (moveOn) {
        if (tmpOrigin == originNode) {
          continue;
        }
        tmpOrigin = tmpDestination;
        continue;
      }
      checkEdgesSequence(tmpOrigin);
      completeSequence.addAll(partialSequence);
      tmpOrigin = tmpDestination;
    }
    completeSequence = sequenceOnCommunityNetwork(completeSequence);
    if (completeSequence.isEmpty()) {
      agent.getProperties().regionBasedNavigation = false;
      return roadDistance(initialNode, destinationNode, this.agent);
    }
    fillRoute();
    return route;
  }

  protected void fillRoute() {
    route.directedEdgesSequence = partialSequence;
    route.computeRouteSequences();
  }
}
