package pedSim.routeChoice;

import java.util.ArrayList;

import pedSim.agents.Agent;
import pedSim.agents.AgentProperties;
import pedSim.engine.Parameters;
import sim.graph.GraphUtils;
import sim.graph.NodeGraph;

/**
 * The `RoutePlanner` class is responsible for calculating a route for an agent
 * within a pedestrian simulation. It considers the agent's route choice
 * properties and strategies to determine the optimal path from an origin node
 * to a destination node.
 */
public class RoutePlanner {

	private NodeGraph originNode;
	private NodeGraph destinationNode;
	private AgentProperties agentProperties;
	private ArrayList<NodeGraph> sequenceNodes;
	private Agent agent;

	/**
	 * Constructs a `RoutePlanner` instance for calculating a route.
	 *
	 * @param originNode      The starting node of the route.
	 * @param destinationNode The destination node of the route.
	 * @param agent           The agent for which the route is being planned.
	 */
	public RoutePlanner(NodeGraph originNode, NodeGraph destinationNode, Agent agent) {
		this.originNode = originNode;
		this.destinationNode = destinationNode;
		this.agent = agent;
		this.agentProperties = agent.getProperties();
		this.sequenceNodes = new ArrayList<>();
	}

	/**
	 * Defines the path for the agent based on route choice properties and
	 * strategies.
	 *
	 * @return A `Route` object representing the calculated route.
	 * @throws Exception
	 */
	public Route definePath() throws Exception {

		if (shouldUseMinimization()) {
			if (agentProperties.minimisingDistance) {
				RoadDistancePathFinder finder = new RoadDistancePathFinder();
				return finder.roadDistance(originNode, destinationNode, agent);
			} else {
				AngularChangePathFinder finder = new AngularChangePathFinder();
				return finder.angularChangeBased(originNode, destinationNode, agent);
			}
		}

		/**
		 * Through regions with barrier-subgoals or not.
		 */
		verifyRegionBasedNavigation();
		if (isRegionBased()) {
			RegionBasedNavigation regionsPath = new RegionBasedNavigation(originNode, destinationNode, agent);
			sequenceNodes = regionsPath.sequenceRegions();
		}

		/**
		 * Sub-goals: only barriers, no regions
		 */
		if (agentProperties.barrierBasedNavigation && !isRegionBased()) {
			BarrierBasedNavigation barriersPath = new BarrierBasedNavigation(originNode, destinationNode, agent);
			sequenceNodes = barriersPath.sequenceBarriers();
		}

		/**
		 * Sub-goals: localLandmarks, possibly through regions
		 */
		else if (agentProperties.usingLocalLandmarks) {
			LandmarkNavigation landmarkNavigation = new LandmarkNavigation(originNode, destinationNode, agent);
			if (isRegionBased() && !sequenceNodes.isEmpty())
				sequenceNodes = landmarkNavigation.regionOnRouteMarks(sequenceNodes);
			else
				sequenceNodes = landmarkNavigation.onRouteMarks();
		}

		/**
		 * Not active in the empirical-based simulation! Distant-landmarks - only when
		 * the agent doesn't minimise road costs (in other cases, distant landmarks are
		 * already considered when filling the sequence); a) possibly via sub-goals or
		 * through regions; b) just based on distant landmarks. else: just
		 * global-landmarks maximisation path
		 */
		else if (agentProperties.usingDistantLandmarks && !shouldUseLocalHeuristic()) {
			GlobalLandmarksPathFinder finder = new GlobalLandmarksPathFinder(originNode, destinationNode, agent);
			if (!sequenceNodes.isEmpty())
				return finder.globalLandmarksPathSequence(sequenceNodes);
			else {
				return finder.globalLandmarksPath();
			}
		}

		if (sequenceNodes.isEmpty()) {
			if (agentProperties.localHeuristicDistance) {
				RoadDistancePathFinder finder = new RoadDistancePathFinder();
				return finder.roadDistance(originNode, destinationNode, agent);
			} else {
				AngularChangePathFinder finder = new AngularChangePathFinder();
				return finder.angularChangeBased(originNode, destinationNode, agent);
			}
		}

		if (agentProperties.localHeuristicDistance) {
			RoadDistancePathFinder finder = new RoadDistancePathFinder();
			return finder.roadDistanceSequence(sequenceNodes, agent);
		} else {
			AngularChangePathFinder finder = new AngularChangePathFinder();
			return finder.angularChangeBasedSequence(sequenceNodes, agent);
		}
	}

	/**
	 * Checks if the agent should use minimization for route planning.
	 *
	 * @return True if the agent should use minimization, otherwise false.
	 */
	private boolean shouldUseMinimization() {
		return agentProperties.onlyMinimising;
	}

	/**
	 * Checks if the agent should use local heuristics for route planning.
	 *
	 * @return True if the agent should use local heuristics, otherwise false.
	 */
	private boolean shouldUseLocalHeuristic() {
		return (agentProperties.localHeuristicDistance || agentProperties.localHeuristicAngular);
	}

	/**
	 * Verifies if region-based navigation should be enabled for route planning
	 * based on distance thresholds. If not, it disables region-based navigation in
	 * agent properties.
	 */
	private void verifyRegionBasedNavigation() {
		if (GraphUtils.nodesDistance(originNode, destinationNode) < Parameters.regionBasedNavigationThreshold
				|| originNode.regionID == destinationNode.regionID) {
			agentProperties.regionBasedNavigation = false;
		}
	}

	/**
	 * Checks if region-based navigation is enabled in the agent properties.
	 *
	 * @return True if region-based navigation is enabled, otherwise false.
	 */
	private boolean isRegionBased() {
		return agentProperties.regionBasedNavigation;
	}
}