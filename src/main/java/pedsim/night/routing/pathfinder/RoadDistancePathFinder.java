package pedsim.night.routing.pathfinder;

import pedsim.core.routing.pathfinding.DijkstraRoadDistance;
import pedsim.night.agents.Agent;
import sim.graph.NodeGraph;
import sim.routing.Route;

/**
 * A pathfinder for road-distance based route calculations. This class extends the functionality of
 * the base class PathFinder.
 */
public class RoadDistancePathFinder extends pedsim.core.routing.pathfinder.RoadDistancePathFinder {

  Agent agent;
  Route route = new Route();

  /**
   * Formulates a route based on road distance between the given origin and destination nodes using
   * the provided agent properties.
   * 
   * @param originNode the origin node;
   * @param destinationNode the destination node;
   * @param agent The agent for which the route is computed.
   * @return a {@code Route} object representing the road-distance shortest path.
   */
  public Route roadDistanceNight(NodeGraph originNode, NodeGraph destinationNode, Agent agent) {
    this.agent = agent;
    partialSequence =
        new DijkstraRoadDistance().dijkstraAlgorithm(originNode, destinationNode, this.agent);

    fillRoute();
    return route;
  }
}
