package pedsim.core.routing.pathfinder;

import java.util.HashSet;
import java.util.List;
import pedsim.core.agents.Agent;
import pedsim.core.routing.pathfinding.DijkstraGlobalLandmarks;
import sim.graph.NodeGraph;
import sim.routing.Route;

public class GlobalLandmarksPathFinder extends PathFinder {

  /**
   * Formulates a route based on global landmarkness maximisation between the origin and destination
   * nodes only.
   *
   * @return The computed route.
   */
  public Route globalLandmarksPath(NodeGraph originNode, NodeGraph destinationNode, Agent agent) {

    this.agent = agent;
    this.originNode = originNode;
    this.destinationNode = destinationNode;
    DijkstraGlobalLandmarks pathfinder = new DijkstraGlobalLandmarks();
    partialSequence =
        pathfinder.dijkstraAlgorithm(
            originNode, destinationNode, destinationNode, directedEdgesToAvoid, agent);
    partialSequence = sequenceOnCommunityNetwork(partialSequence);
    if (partialSequence.isEmpty()) {
      return distanceFallback(originNode, destinationNode, agent);
    }
    route.directedEdgesSequence = partialSequence;
    route.computeRouteSequences();
    return route;
  }

  /**
   * Serves the shortest path when the landmark search finds none, and counts it.
   *
   * <p>The counterpart of {@code AngularChangePathFinder}'s own fallback, and for the same reason:
   * an unroutable leg that throws stops the run, and one that silently returns a shortest path is
   * reported as a landmark route. Counting it is what keeps the substitution visible - a handful of
   * awkward pairs at a low rate, and at a high one a set of results that are partly shortest paths.
   *
   * @param originNode the origin
   * @param destinationNode the destination
   * @param agent the agent the route is for
   * @return the shortest-path route between the two nodes
   */
  private Route distanceFallback(NodeGraph originNode, NodeGraph destinationNode, Agent agent) {
    if (agent.getState() != null) {
      agent.getState().trace().recordLandmarkFallback();
    }
    return new RoadDistancePathFinder().roadDistance(originNode, destinationNode, agent);
  }

  /**
   * Formulates a route based on global landmarkness maximisation through a sequence of intermediate
   * nodes [originNode, ..., destinationNode]. It allows combining global landmarkness maximisation
   * with a sequence of nodes resulting for example from the region-based navigation.
   *
   * @param sequenceNodes A list of nodes representing the sequence to follow.
   * @return The computed route.
   */
  public Route globalLandmarksPathSequence(List<NodeGraph> sequenceNodes, Agent agent) {

    routeSequence(
        sequenceNodes,
        agent,
        () -> {
          // The edges already in the route are off limits, as they are for the distance router: a
          // leg that re-uses one doubles back over ground the agent has walked.
          directedEdgesToAvoid = new HashSet<>(completeSequence);
          return new DijkstraGlobalLandmarks()
              .dijkstraAlgorithm(
                  tmpOrigin, tmpDestination, destinationNode, directedEdgesToAvoid, agent);
        });

    if (completeSequence.isEmpty()) {
      return distanceFallback(originNode, destinationNode, agent);
    }
    route.directedEdgesSequence = completeSequence;
    route.computeRouteSequences();
    return route;
  }
}
