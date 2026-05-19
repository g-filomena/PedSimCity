package pedsim.night.routing.pathfinder;

import pedsim.night.agents.Agent;
import pedsim.night.routing.pathfinding.DijkstraRoadDistanceNight;
import sim.graph.NodeGraph;
import sim.routing.Route;

/**
 * Path finder for night-time road-distance based route calculations.
 */
public class RoadDistancePathFinder extends pedsim.core.routing.pathfinder.RoadDistancePathFinder {

  /**
   * Formulates a night-time route based on road distance between origin and destination.
   *
   * @param originNode the origin node
   * @param destinationNode the destination node
   * @param agent the agent for which the route is computed
   * @return a route representing the calculated night-time path
   */
  public Route roadDistanceNight(NodeGraph originNode, NodeGraph destinationNode, Agent agent) {
    this.agent = agent;
    partialSequence =
        new DijkstraRoadDistanceNight().dijkstraAlgorithm(originNode, destinationNode, this.agent);
    fillRoute();
    return route;
  }
}
