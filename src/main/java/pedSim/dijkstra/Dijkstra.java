package pedSim.dijkstra;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;

import org.locationtech.jts.planargraph.DirectedEdge;

import pedSim.agents.Agent;
import pedSim.agents.AgentProperties;
import pedSim.engine.Parameters;
import pedSim.engine.PedSimCity;
import pedSim.routeChoice.LandmarkNavigation;
import pedSim.routeChoice.Route;
import sim.graph.EdgeGraph;
import sim.graph.Graph;
import sim.graph.GraphUtils;
import sim.graph.NodeGraph;
import sim.graph.SubGraph;
import sim.util.geo.Utilities;

/**
 * The Dijkstra class provides functionality for performing Dijkstra's algorithm
 * and related calculations for route planning in the pedestrian simulation.
 */
public class Dijkstra {

	NodeGraph originNode, destinationNode, previousJunction, finalDestinationNode;
	protected HashSet<NodeGraph> visitedNodes;
	protected HashSet<NodeGraph> unvisitedNodes;

	HashSet<NodeGraph> centroidsToAvoid = new HashSet<>();
	HashSet<DirectedEdge> segmentsToAvoid = new HashSet<>();
	HashSet<EdgeGraph> edgesToAvoid = new HashSet<>();
	HashMap<NodeGraph, NodeWrapper> nodeWrappersMap = new HashMap<>();
	SubGraph subGraph = null;
	AgentProperties agentProperties;
	boolean usingSubGraph;
	double tentativeCost;
	Graph graph;
	Agent agent;

	Route route = new Route();

	protected static final double MAX_DEFLECTION_ANGLE = 180.00;
	protected static final double MIN_DEFLECTION_ANGLE = 0;

	/**
	 * Initialises the Dijkstra algorithm for route calculation in a primal graph.
	 *
	 * @param originNode           The starting node for the route.
	 * @param destinationNode      The destination node for the route.
	 * @param finalDestinationNode The final destination node.
	 * @param segmentsToAvoid      A set of directed edges to avoid during route
	 *                             calculation.
	 * @param agentProperties      The properties of the agent using this algorithm.
	 * @param usingSubGraph        Indicates whether a subgraph is used for
	 *                             calculation.
	 */
	protected void initialise(NodeGraph originNode, NodeGraph destinationNode, NodeGraph finalDestinationNode,
			HashSet<DirectedEdge> segmentsToAvoid, Agent agent, boolean usingSubGraph) {

		nodeWrappersMap.clear();
		this.agent = agent;
		this.agentProperties = agent.getProperties();
		this.originNode = originNode;
		this.destinationNode = destinationNode;
		this.finalDestinationNode = finalDestinationNode;
		this.usingSubGraph = usingSubGraph;
		getEdgesToAvoid(segmentsToAvoid);
		subGraphInitialisation();
	}

	/**
	 * Initialises the Dijkstra algorithm for route calculation in a dual graph.
	 *
	 * @param originNode           The starting node for the route.
	 * @param destinationNode      The destination node for the route.
	 * @param finalDestinationNode The final destination node.
	 * @param centroidsToAvoid     A set of centroids to avoid during route
	 *                             calculation.
	 * @param previousJunction     The previous junction node in the dual graph.
	 * @param agentProperties      The properties of the agent using this algorithm.
	 * @param usingSubGraph        Indicates whether a subgraph is used for
	 *                             calculation.
	 */
	protected void initialiseDual(NodeGraph originNode, NodeGraph destinationNode, NodeGraph finalDestinationNode,
			HashSet<NodeGraph> centroidsToAvoid, NodeGraph previousJunction, Agent agent, boolean usingSubGraph) {

		nodeWrappersMap.clear();
		this.agent = agent;
		this.agentProperties = agent.getProperties();
		this.originNode = originNode;
		this.destinationNode = destinationNode;
		this.finalDestinationNode = finalDestinationNode;
		if (centroidsToAvoid != null)
			this.centroidsToAvoid = new HashSet<>(centroidsToAvoid);
		this.previousJunction = previousJunction;
		this.usingSubGraph = usingSubGraph;
		subGraphInitialisationDual();
	}

	/**
	 * Extracts edges to avoid from a set of directed edges.
	 *
	 * @param segmentsToAvoid A set of directed edges to avoid.
	 */
	protected void getEdgesToAvoid(HashSet<DirectedEdge> segmentsToAvoid) {
		if (segmentsToAvoid != null) {
			this.segmentsToAvoid = new HashSet<>(segmentsToAvoid);
			for (DirectedEdge edge : this.segmentsToAvoid) {
				edgesToAvoid.add((EdgeGraph) edge.getEdge());
			}
		}
	}

	/**
	 * Initialises the subgraph for primal graph route calculation either between
	 * the origin and the destination nodes or at the region level. Adjusts the set
	 * of centroids to avoid if necessary.
	 */
	protected void subGraphInitialisation() {
		if (regionCondition()) {
			subGraph = PedSimCity.regionsMap.get(originNode.regionID).primalGraph;
			edgesToAvoid = (segmentsToAvoid.isEmpty())
					? new HashSet<>(subGraph.getChildEdges(new ArrayList<>(edgesToAvoid)))
					: null;
		} else if (usingSubGraph) {
			ArrayList<EdgeGraph> containedEdges = PedSimCity.network.edgesInNodesSpace(originNode, destinationNode);
			subGraph = new SubGraph(PedSimCity.network, containedEdges);
			edgesToAvoid = (segmentsToAvoid.isEmpty())
					? new HashSet<>(subGraph.getChildEdges(new ArrayList<>(edgesToAvoid)))
					: null;
		}
		if (subGraph != null) {
			originNode = subGraph.findNode(originNode.getCoordinate());
			destinationNode = subGraph.findNode(destinationNode.getCoordinate());
			graph = subGraph;
		} else
			graph = PedSimCity.network;
	}

	/**
	 * Initialises the subgraph for dual graph route calculation either between the
	 * origin and the destination nodes or at the region level. Adjusts the set of
	 * centroids to avoid if necessary.
	 */
	protected void subGraphInitialisationDual() {
		if (regionCondition()) {
			subGraph = PedSimCity.regionsMap.get(originNode.regionID).dualGraph;
			centroidsToAvoid = (centroidsToAvoid != null)
					? new HashSet<>(subGraph.getChildNodes(new ArrayList<>(centroidsToAvoid)))
					: null;
		} else if (usingSubGraph) {
			ArrayList<EdgeGraph> containedEdges = PedSimCity.dualNetwork.edgesInNodesSpace(originNode, destinationNode);
			subGraph = new SubGraph(PedSimCity.dualNetwork, containedEdges);
			centroidsToAvoid = (centroidsToAvoid != null)
					? new HashSet<>(subGraph.getChildNodes(new ArrayList<>(centroidsToAvoid)))
					: null;
		}
		if (subGraph != null) {
			originNode = subGraph.findNode(originNode.getCoordinate());
			destinationNode = subGraph.findNode(destinationNode.getCoordinate());
			graph = subGraph;
		} else
			graph = PedSimCity.dualNetwork;
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

		double error = 1.0;
		if (this.agent.cognitiveMap.containsBarriers()) {
			List<Integer> pBarriers = dual ? targetNode.primalEdge.attributes.get("positiveBarriers").getArray()
					: commonEdge.attributes.get("positiveBarriers").getArray();
			List<Integer> nBarriers = dual ? targetNode.primalEdge.attributes.get("negativeBarriers").getArray()
					: commonEdge.attributes.get("negativeBarriers").getArray();
			if (positiveBarrierEffect(pBarriers))
				error = Utilities.fromDistribution(agentProperties.naturalBarriers, agentProperties.naturalBarriersSD,
						"left");
			if (negativeBarrierEffect(nBarriers))
				error = Utilities.fromDistribution(agentProperties.severingBarriers, agentProperties.severingBarriersSD,
						"right");
		} else
			error = Utilities.fromDistribution(1.0, 0.10, null);
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
		} else {
			tentativeCost = getBest(currentNode) + edgeCost;
		}
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
		} else {
			tentativeCost = getBest(currentNode) + turnCost;
		}
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
			nodeWrapper.commonPrimalJunction = route.commonPrimalJunction(currentNode, targetNode);
			nodeWrapper.gx = tentativeCost;
			nodeWrappersMap.put(targetNode, nodeWrapper);
			unvisitedNodes.add(targetNode);
		}
	}

	/**
	 * Retrieves the closest node from the given set of nodes.
	 *
	 * @param nodes The set of nodes.
	 * @return The closest node.
	 */
	protected NodeGraph getClosest(HashSet<NodeGraph> nodes) {
		PriorityQueue<NodeGraph> queue = new PriorityQueue<>(
				(node1, node2) -> Double.compare(getBest(node1), getBest(node2)));
		queue.addAll(nodes);
		return queue.peek();
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
	 * Clears the data structures.
	 */
	protected void clear() {
		usingSubGraph = false;
		visitedNodes.clear();
		unvisitedNodes.clear();
		nodeWrappersMap.clear();
		originNode = subGraph.getParentNode(originNode);
		destinationNode = subGraph.getParentNode(destinationNode);
		if (!centroidsToAvoid.isEmpty())
			centroidsToAvoid = new HashSet<>(subGraph.getParentNodes(new ArrayList<>(centroidsToAvoid)));
	}

	/**
	 * Determines whether there is a positive barrier effect based on the provided
	 * list of barriers.
	 *
	 * @param pBarriers The list of positive barriers to check.
	 * @return True if there is a positive barrier effect; otherwise, false.
	 */
	private boolean positiveBarrierEffect(List<Integer> pBarriers) {
		return (!agentProperties.onlyMinimising && agentProperties.preferenceNaturalBarriers && !pBarriers.isEmpty());
	}

	/**
	 * Determines whether there is a negative barrier effect based on the provided
	 * list of barriers and the agent properties.
	 *
	 * @param nBarriers The list of negative barriers to check.
	 * @return True if there is a negative barrier effect; otherwise, false.
	 */
	private boolean negativeBarrierEffect(List<Integer> nBarriers) {
		return (!agentProperties.onlyMinimising && agentProperties.aversionSeveringBarriers && !nBarriers.isEmpty());
	}

	/**
	 * Checks if the landmark condition is met for the target node and the agent
	 * properties.
	 *
	 * @param targetNode The node to check for the landmark condition.
	 * @return True if the landmark condition is met; otherwise, false.
	 */
	private boolean landmarkCondition(NodeGraph targetNode) {
		return (!agentProperties.onlyMinimising && agentProperties.usingDistantLandmarks && GraphUtils
				.getCachedNodesDistance(targetNode, finalDestinationNode) > Parameters.threshold3dVisibility);
	}

	/**
	 * Checks if the region-based navigation condition is met and the agent
	 * properties.
	 *
	 * @return True if the region-based navigation condition is met; otherwise,
	 *         false.
	 */
	protected boolean regionCondition() {
		return agentProperties.regionBasedNavigation && originNode.regionID == destinationNode.regionID;
	}

}
