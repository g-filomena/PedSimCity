package pedSim.dijkstra;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;

import org.locationtech.jts.planargraph.DirectedEdge;

import pedSim.agents.Agent;
import pedSim.agents.AgentProperties;
import pedSim.cognitiveMap.CommunityCognitiveMap;
import pedSim.engine.Parameters;
import pedSim.engine.PedSimCity;
import pedSim.routeChoice.LandmarkNavigation;
import sim.graph.EdgeGraph;
import sim.graph.Graph;
import sim.graph.GraphUtils;
import sim.graph.NodeGraph;
import sim.graph.SubGraph;
import sim.routing.NodeWrapper;
import sim.routing.Route;
import sim.routing.RoutingUtils;
import sim.util.geo.Utilities;

/**
 * The Dijkstra class provides functionality for performing Dijkstra's algorithm
 * and related calculations for route planning in the pedestrian simulation.
 */
public class Dijkstra {

	NodeGraph originNode, destinationNode, previousJunction, finalDestinationNode;
	protected Set<NodeGraph> visitedNodes;
	protected PriorityQueue<NodeGraph> unvisitedNodes;

	Set<NodeGraph> centroidsToAvoid = new HashSet<>();
	Set<DirectedEdge> directedEdgesToAvoid = new HashSet<>();
	Set<EdgeGraph> edgesToAvoid = new HashSet<>();
	Map<NodeGraph, NodeWrapper> nodeWrappersMap = new HashMap<>();
	AgentProperties properties;
	double tentativeCost;

	protected Graph agentNetwork;
	protected Graph agentDualNetwork;
	SubGraph subGraph = null;

	Agent agent;
	Route route = new Route();

	protected static final double MAX_DEFLECTION_ANGLE = 180.00;
	protected static final double MIN_DEFLECTION_ANGLE = 0;

	protected void initialise(NodeGraph originNode, NodeGraph destinationNode, NodeGraph finalDestinationNode,
			Agent agent) {

		nodeWrappersMap.clear();
		this.agentNetwork = CommunityCognitiveMap.getNetwork();
		this.agent = agent;
		this.properties = agent.getProperties();
		this.originNode = originNode;
		this.destinationNode = destinationNode;
		this.finalDestinationNode = finalDestinationNode;
	}

	/**
	 * Initialises the Dijkstra algorithm for route calculation in a primal graph.
	 *
	 * @param segmentsToAvoid A set of directed edges to avoid during route
	 */
	protected void initialisePrimal(Set<DirectedEdge> segmentsToAvoid) {

		if (!segmentsToAvoid.isEmpty())
			getEdgesToAvoid(segmentsToAvoid);
		subGraphInitialisation();
	}

	/**
	 * Initialises the Dijkstra algorithm for route calculation in a dual graph.
	 *
	 * @param centroidsToAvoid A set of centroids to avoid during route calculation.
	 * @param previousJunction The previous junction node in the dual graph.
	 */
	protected void initialiseDual(Set<NodeGraph> centroidsToAvoid, NodeGraph previousJunction) {

		if (centroidsToAvoid != null)
			this.centroidsToAvoid = new HashSet<>(centroidsToAvoid);
		this.previousJunction = previousJunction;
		this.agentDualNetwork = CommunityCognitiveMap.getDualNetwork();
		subGraphInitialisationDual();
	}

	/**
	 * Extracts edges to avoid from a set of directed edges.
	 *
	 * @param directEdgesToAvoid A set of directed edges to avoid.
	 */
	protected void getEdgesToAvoid(Set<DirectedEdge> directEdgesToAvoid) {
		this.directedEdgesToAvoid = new HashSet<>(directEdgesToAvoid);
		for (DirectedEdge edge : this.directedEdgesToAvoid)
			edgesToAvoid.add((EdgeGraph) edge.getEdge());
	}

	/**
	 * Initialises the subgraph for primal graph route calculation either between
	 * the origin and the destination nodes or at the region level. Adjusts the set
	 * of centroids to avoid if necessary.
	 */
	protected void subGraphInitialisation() {
		if (regionCondition()) {
			subGraph = PedSimCity.regionsMap.get(originNode.getRegionID()).primalGraph;
			edgesToAvoid = (directedEdgesToAvoid.isEmpty())
					? new HashSet<>(subGraph.getChildEdges(new ArrayList<>(edgesToAvoid)))
					: new HashSet<>();
			originNode = subGraph.findNode(originNode.getCoordinate());
			destinationNode = subGraph.findNode(destinationNode.getCoordinate());
			agentNetwork = subGraph;
		}
	}

	/**
	 * Initialises the subgraph for dual graph route calculation either between the
	 * origin and the destination nodes or at the region level. Adjusts the set of
	 * centroids to avoid if necessary.
	 */
	protected void subGraphInitialisationDual() {
		if (regionCondition()) {
			subGraph = PedSimCity.regionsMap.get(originNode.getRegionID()).dualGraph;
			centroidsToAvoid = (!centroidsToAvoid.isEmpty())
					? new HashSet<>(subGraph.getChildNodes(new ArrayList<>(centroidsToAvoid)))
					: new HashSet<>();
			originNode = subGraph.findNode(originNode.getCoordinate());
			destinationNode = subGraph.findNode(destinationNode.getCoordinate());
			agentDualNetwork = subGraph;
		}
	}

	/**
	 * Computes the cost perception error based on the role of barriers.
	 *
	 * @param targetNode The target node for cost calculation.
	 * @param commonEdge The common edge used in cost calculation.
	 * @param dual       Indicates whether it is a dual graph.
	 * @return The computed cost perception error.
	 */
	protected double costPerceptionError(NodeGraph targetNode, EdgeGraph commonEdge, boolean dual) {

		double error = Utilities.fromDistribution(1.0, 0.10, null);

		if (positiveBarrierEffect()) {
			List<Integer> pBarriers = dual ? targetNode.getPrimalEdge().attributes.get("positiveBarriers").getArray()
					: commonEdge.attributes.get("positiveBarriers").getArray();
			if (!pBarriers.isEmpty())
				error = Utilities.fromDistribution(properties.naturalBarriers, properties.naturalBarriersSD, "left");
		}
		if (negativeBarrierEffect()) {
			List<Integer> nBarriers = dual ? targetNode.getPrimalEdge().attributes.get("negativeBarriers").getArray()
					: commonEdge.attributes.get("negativeBarriers").getArray();
			if (!nBarriers.isEmpty())
				error = Utilities.fromDistribution(properties.severingBarriers, properties.severingBarriersSD, "right");
		}

		return error;
	}

	/**
	 * Computes the tentative cost for a given currentNode and targetNode with the
	 * specified edgeCost.
	 *
	 * @param currentNode The current node.
	 * @param targetNode  The target node.
	 * @param edgeCost    The cost of the edge between the current and target nodes.
	 */
	protected void computeTentativeCost(NodeGraph currentNode, NodeGraph targetNode, double edgeCost) {
		tentativeCost = 0.0;
		if (landmarkCondition(targetNode)) {
			double globalLandmarkness = LandmarkNavigation.globalLandmarknessNode(targetNode, finalDestinationNode);
			double nodeLandmarkness = 1.0 - globalLandmarkness * Parameters.globalLandmarknessWeightDistance;
			double nodeCost = edgeCost * nodeLandmarkness;
			tentativeCost = getBest(currentNode) + nodeCost;
		} else
			tentativeCost = getBest(currentNode) + edgeCost;

	}

	/**
	 * Computes the tentative cost for a dual currentNode and targetNode with the
	 * specified turnCost.
	 *
	 * @param currentNode The current node.
	 * @param targetNode  The target node.
	 * @param turnCost    The cost associated with turning from the current to the
	 *                    target node.
	 */
	protected void computeTentativeCostDual(NodeGraph currentNode, NodeGraph targetNode, double turnCost) {
		tentativeCost = 0.0;
		if (turnCost > MAX_DEFLECTION_ANGLE) {
			turnCost = MAX_DEFLECTION_ANGLE;
		}
		if (turnCost < MIN_DEFLECTION_ANGLE) {
			turnCost = MIN_DEFLECTION_ANGLE;
		}
		if (landmarkCondition(targetNode)) {
			double globalLandmarkness = LandmarkNavigation.globalLandmarknessDualNode(currentNode, targetNode,
					finalDestinationNode);
			double nodeLandmarkness = 1.0 - globalLandmarkness * Parameters.globalLandmarknessWeightAngular;
			double nodeCost = nodeLandmarkness * turnCost;
			tentativeCost = getBest(currentNode) + nodeCost;
		} else
			tentativeCost = getBest(currentNode) + turnCost;

	}

	/**
	 * Checks if the tentative cost is the best for the currentNode and targetNode
	 * with the specified outEdge.
	 *
	 * @param currentNode The current node.
	 * @param targetNode  The target node.
	 * @param outEdge     The directed edge from the current node to the target
	 *                    node.
	 */
	protected void isBest(NodeGraph currentNode, NodeGraph targetNode, DirectedEdge outEdge) {
		if (getBest(targetNode) > tentativeCost) {
			NodeWrapper nodeWrapper = nodeWrappersMap.computeIfAbsent(targetNode, NodeWrapper::new);
			nodeWrapper.nodeFrom = currentNode;
			nodeWrapper.directedEdgeFrom = outEdge;
			nodeWrapper.gx = tentativeCost;
			nodeWrappersMap.put(targetNode, nodeWrapper);
			unvisitedNodes.add(targetNode);
		}
	}

	/**
	 * Checks if the tentative cost is the best for the currentNode and targetNode
	 * with the specified outEdge in a dual context.
	 *
	 * @param currentNode The current node.
	 * @param targetNode  The target node.
	 * @param outEdge     The directed edge from the current node to the target
	 *                    node.
	 */
	protected void isBestDual(NodeGraph currentNode, NodeGraph targetNode, DirectedEdge outEdge) {
		if (getBest(targetNode) > tentativeCost) {
			NodeWrapper nodeWrapper = nodeWrappersMap.computeIfAbsent(targetNode, NodeWrapper::new);
			nodeWrapper.nodeFrom = currentNode;
			nodeWrapper.directedEdgeFrom = outEdge;
			nodeWrapper.commonPrimalJunction = RoutingUtils.getPrimalJunction(currentNode, targetNode);
			nodeWrapper.gx = tentativeCost;
			nodeWrappersMap.put(targetNode, nodeWrapper);
			unvisitedNodes.add(targetNode);
		}
	}

	/**
	 * Retrieves the best value for the specified targetNode from the
	 * nodeWrappersMap.
	 *
	 * @param targetNode The target node.
	 * @return The best value for the target node.
	 */
	protected double getBest(NodeGraph targetNode) {
		NodeWrapper nodeWrapper = nodeWrappersMap.get(targetNode);
		return nodeWrapper != null ? nodeWrapper.gx : Double.MAX_VALUE;
	}

	/**
	 * Determines whether there is a positive barrier effect based on the provided
	 * list of barriers.
	 *
	 * @param pBarriers The list of positive barriers to check.
	 * @return True if there is a positive barrier effect; otherwise, false.
	 */
	private boolean positiveBarrierEffect() {
		return (!properties.shouldOnlyUseMinimization() && properties.preferenceNaturalBarriers);
	}

	/**
	 * Determines whether there is a negative barrier effect based on the provided
	 * list of barriers and the agent properties.
	 *
	 * @param nBarriers The list of negative barriers to check.
	 * @return True if there is a negative barrier effect; otherwise, false.
	 */
	private boolean negativeBarrierEffect() {
		return (!properties.shouldOnlyUseMinimization() && properties.aversionSeveringBarriers);
	}

	/**
	 * Checks if the landmark condition is met for the target node and the agent
	 * properties.
	 *
	 * @param targetNode The node to check for the landmark condition.
	 * @return True if the landmark condition is met; otherwise, false.
	 */
	private boolean landmarkCondition(NodeGraph targetNode) {
		return (!properties.shouldOnlyUseMinimization() && properties.usingDistantLandmarks
				&& GraphUtils.nodesDistance(targetNode, finalDestinationNode) > Parameters.threshold3dVisibility);
	}

	/**
	 * Checks if the region-based navigation condition is met.
	 *
	 * @return True if the region-based navigation condition is met; otherwise,
	 *         false.
	 */
	protected boolean regionCondition() {
		return properties.regionBasedNavigation && originNode.getRegionID() == destinationNode.getRegionID();
	}

	/**
	 * Retrieves the directed edge from the parent graph for the given node.
	 *
	 * @param step the node from which to retrieve the directed edge
	 * @return the directed edge from the parent graph
	 */
	protected DirectedEdge retrieveFromParentGraph(NodeGraph step) {
		NodeGraph nodeTo = subGraph.getParentNode(step);
		NodeGraph nodeFrom = subGraph.getParentNode(nodeWrappersMap.get(step).nodeFrom);
		// retrieving from Primal Network (no SubGraph)
		return CommunityCognitiveMap.getNetwork().getDirectedEdgeBetween(nodeFrom, nodeTo);
	}

}
