package pedsim.night.routing.routers;

import pedsim.night.agents.NightAgent;
import pedsim.night.routing.search.DijkstraRoadDistanceNight;
import sim.graph.NodeGraph;
import sim.routing.Route;

/**
 * Path finder for night-time road-distance based route calculations.
 *
 * <p>Deliberately reaches the three-argument {@code dijkstraAlgorithm}, which does not call
 * {@code initialisePrimal}: night navigation is not region-based, so no region subgraph is set up
 * and {@code directedEdgesToAvoid} is not consulted. That is a decision, not an omission - night
 * agents route on the whole community network, which is also why {@code buildSimpleActivityBone}
 * leaves their cognitive map un-individualised.
 */
public class RoadDistancePathFinder extends pedsim.core.routing.routers.RoadDistancePathFinder {

  /**
   * Formulates a night-time route based on road distance between origin and destination.
   *
   * @param originNode the origin node
   * @param destinationNode the destination node
   * @param agent the agent for which the route is computed
   * @return a route representing the calculated night-time path
   */
  public Route roadDistanceNight(
      NodeGraph originNode, NodeGraph destinationNode, NightAgent agent) {
    this.agent = agent;
    // Set on the finder, as the parent does: fillRoute reads them to build the degenerate two-node
    // route when no path is found, and without them a failed night route came back with no nodes
    // at all - not even an origin and a destination.
    this.originNode = originNode;
    this.destinationNode = destinationNode;
    partialSequence =
        new DijkstraRoadDistanceNight().dijkstraAlgorithm(originNode, destinationNode, this.agent);
    partialSequence = sequenceOnCommunityNetwork(partialSequence);
    fillRoute();
    return route;
  }
}
