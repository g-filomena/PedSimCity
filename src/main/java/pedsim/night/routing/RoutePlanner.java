package pedsim.night.routing;

import pedsim.night.agents.Agent;
import pedsim.night.routing.pathfinder.RoadDistancePathFinder;
import sim.graph.NodeGraph;
import sim.routing.Route;

/**
 * The `RoutePlanner` class is responsible for calculating a route for an agent
 * within a pedestrian simulation. It considers the agent's route choice
 * properties and strategies to determine the optimal path from an origin node
 * to a destination node.
 */
public class RoutePlanner extends pedsim.core.routing.RoutePlanner {

	private Agent agent;

	public RoutePlanner(NodeGraph originNode, NodeGraph destinationNode, Agent agent) {
		super(originNode, destinationNode, agent);
		this.agent = agent;
	}

	/**
	 * Defines the path for the agent based on route choice properties and
	 * strategies.
	 *
	 * @return A `Route` object representing the calculated route.
	 */
	public Route definePath() {
		route = new RoadDistancePathFinder().roadDistanceNight(originNode, destinationNode, agent);
		return route;
	}
}
