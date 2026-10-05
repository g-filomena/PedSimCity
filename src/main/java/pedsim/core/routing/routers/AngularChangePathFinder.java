package pedsim.core.routing.routers;

import java.util.HashSet;
import java.util.List;
import org.locationtech.jts.planargraph.DirectedEdge;
import pedsim.core.agents.Agent;
import pedsim.core.engine.PedSimCity;
import pedsim.core.routing.search.DijkstraAngularChange;
import sim.graph.NodeGraph;
import sim.routing.Route;

/** Router for least cumulative angular change routes. */
public class AngularChangePathFinder extends PathFinder {

  /**
   * The least cumulative angular change route between an origin and a destination.
   *
   * <p>An individualised agent whose known network does not connect the two searches the whole
   * network instead, still minimising angular change, and the widening is counted. Shortest path is
   * the last resort, for when the network itself has no path.
   *
   * @param originNode The origin node.
   * @param destinationNode The destination node.
   * @param agent The agent for which the route is computed.
   * @return The route.
   */
  public Route angularChangeBased(NodeGraph originNode, NodeGraph destinationNode, Agent agent) {
    recordAttempt(agent);
    this.agent = agent;
    this.originNode = originNode;
    this.destinationNode = destinationNode;

    List<DirectedEdge> sequence =
        new DijkstraAngularChange()
            .dijkstraAlgorithm(originNode, destinationNode, destinationNode, null, null, agent);

    if (sequence.isEmpty()
        && agent != null
        && agent.getCognitiveMap() != null
        && agent.getCognitiveMap().individualised) {
      DijkstraAngularChange widened = new DijkstraAngularChange();
      widened.ignoreKnownNetwork();
      sequence =
          widened.dijkstraAlgorithm(
              originNode, destinationNode, destinationNode, null, null, agent);
      if (!sequence.isEmpty() && agent.getState() != null) {
        agent.getState().trace().recordFullNetworkEscalation(true);
      }
    }
    if (sequence.isEmpty()) {
      return distanceFallback(originNode, destinationNode, agent);
    }
    route.directedEdgesSequence = sequence;
    route.computeRouteSequences();
    return route;
  }

  /**
   * The least cumulative angular change route through a sequence of sub-goals [origin, ...,
   * destination]. Each leg pays the turn out of the street the walk arrived by, and backtracking
   * routes angularly too.
   *
   * @param sequenceNodes The sub-goals, origin first and destination last.
   * @param agent The agent for which the route is computed.
   * @return The route.
   */
  public Route angularChangeBasedSequence(List<NodeGraph> sequenceNodes, Agent agent) {
    recordAttempt(agent);
    this.regionBased = agent.getProperties().isRegionBasedNavigation();

    LegRouter leg =
        () -> {
          directedEdgesToAvoid = new HashSet<>(completeSequence);
          DirectedEdge arrival =
              completeSequence.isEmpty() ? null : completeSequence.get(completeSequence.size() - 1);
          if (arrival != null && !arrival.getToNode().equals(tmpOrigin)) {
            arrival = null;
          }
          return new DijkstraAngularChange()
              .dijkstraAlgorithm(
                  tmpOrigin, tmpDestination, destinationNode, arrival, directedEdgesToAvoid, agent);
        };
    routeSequence(sequenceNodes, agent, leg, leg);

    if (completeSequence.isEmpty()) {
      return distanceFallback(sequenceNodes.get(0), destinationNode, agent);
    }
    route.directedEdgesSequence = completeSequence;
    route.computeRouteSequences();
    return route;
  }

  private static void recordAttempt(Agent agent) {
    if (agent != null && agent.getState() != null) {
      agent.getState().trace().recordAngularAttempt();
    }
  }

  /**
   * The shortest path instead, when no angular route exists, counted on the day trace so the
   * substitution is visible.
   */
  private Route distanceFallback(NodeGraph originNode, NodeGraph destinationNode, Agent agent) {
    PedSimCity state = agent.getState();
    if (state != null) {
      state.trace().recordAngularFallback();
    }
    return new RoadDistancePathFinder().roadDistance(originNode, destinationNode, agent);
  }
}
