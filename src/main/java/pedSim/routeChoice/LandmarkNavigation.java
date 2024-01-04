package pedSim.routeChoice;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import org.locationtech.jts.planargraph.DirectedEdge;

import pedSim.agents.Agent;
import pedSim.cognitiveMap.LandmarkIntegration;
import pedSim.cognitiveMap.Region;
import pedSim.engine.Parameters;
import pedSim.engine.PedSimCity;
import sim.field.geo.VectorLayer;
import sim.graph.Building;
import sim.graph.Graph;
import sim.graph.GraphUtils;
import sim.graph.NodeGraph;

/**
 * Series of functions that support landmark-based navigation, landmarkness
 * computation, identification of on-route marks and wayfinding easiness of a
 * certain space.
 */
public class LandmarkNavigation {

	NodeGraph originNode;
	NodeGraph destinationNode;
	private List<NodeGraph> sequence = new ArrayList<>();
	private List<NodeGraph> inRegionSequence = new ArrayList<>();
	private Complexity complexity = new Complexity();
	private Map<NodeGraph, Double> salientNodes = new HashMap<NodeGraph, Double>();
	private Agent agent;
	private NodeGraph currentNode;
	private Graph agentNetwork;

	/**
	 * Initialises a new instance of the LandmarkNavigation class with the specified
	 * origin node, destination node, and agent.
	 *
	 * @param originNode      The starting node for navigation.
	 * @param destinationNode The target destination node.
	 * @param agent           The agent associated with the navigation.
	 */
	public LandmarkNavigation(NodeGraph originNode, NodeGraph destinationNode, Agent agent) {
		this.originNode = originNode;
		this.destinationNode = destinationNode;
		this.agent = agent;
		this.agentNetwork = agent.getCognitiveMap().getKnownNetwork();
	}

	/**
	 * Generates a sequence of intermediate nodes (on-route marks) between the
	 * origin and destination nodes on the basis of local landmarkness.
	 *
	 * @return An ArrayList of on-route marks, including the origin and destination
	 *         nodes.
	 */
	public List<NodeGraph> onRouteMarks() {

		sequence = new ArrayList<>();
		findSalientJunctions(originNode);

		if (salientNodes.isEmpty())
			return sequence;

		// compute wayfinding easiness and the resulting research space
		double wayfindingEasiness = complexity.wayfindingEasiness(originNode, destinationNode, agent);
		double searchDistance = GraphUtils.nodesDistance(originNode, destinationNode) * wayfindingEasiness;
		currentNode = originNode;

		// while the wayfindingEasiness is lower than the threshold the agent looks for
		// intermediate-points.
		while (wayfindingEasiness < Parameters.wayfindingEasinessThreshold) {
			NodeGraph bestNode = findOnRouteMark(salientNodes, searchDistance);
			if (bestNode == null || bestNode.equals(currentNode))
				break;
			sequence.add(bestNode);
			findSalientJunctions(bestNode);
			if (salientNodes.isEmpty())
				return sequence;
			wayfindingEasiness = complexity.wayfindingEasiness(bestNode, destinationNode, agent);
			searchDistance = GraphUtils.nodesDistance(bestNode, destinationNode) * wayfindingEasiness;
			currentNode = bestNode;
			bestNode = null;
		}
		sequence.add(0, originNode);
		sequence.add(destinationNode);
		return sequence;
	}

	/**
	 * Identifies salient junctions within the network space between a given node
	 * and the destination node. This method finds salient nodes based on their
	 * salience within the specified network space.
	 *
	 * @param node The node from which to identify salient junctions, in the space
	 *             with the destination node.
	 */
	public void findSalientJunctions(NodeGraph node) {

		double percentile = Parameters.salientNodesPercentile;
		salientNodes = new HashMap<NodeGraph, Double>(
				agentNetwork.getSalientNodesWithinSpace(node, destinationNode, percentile));

		// If no salient junctions are found, the tolerance increases till the 0.50
		// percentile;
		// if still no salient junctions are found, the agent continues without
		// landmarks
		while (salientNodes.isEmpty()) {
			percentile -= 0.05;
			if (percentile < 0.50) {
				sequence.add(0, originNode);
				sequence.add(destinationNode);
				break;
			}
			salientNodes = new HashMap<NodeGraph, Double>(
					agentNetwork.getSalientNodesWithinSpace(node, destinationNode, percentile));
		}
	}

	/**
	 * Finds the most salient on-route mark between the current node and the
	 * destination node, amongst the salient nodes (junctions).
	 *
	 * @param currentNode    The current node in the navigation.
	 * @param salientNodes   A map of salient junctions and their centrality scores
	 *                       along the route.
	 * @param searchDistance The search distance limit from the currentNode for
	 *                       evaluating potential nodes.
	 * @return The selected node that serves as an on-route mark, or null if none is
	 *         found.
	 */
	private NodeGraph findOnRouteMark(Map<NodeGraph, Double> salientNodes, Double searchDistance) {

		List<NodeGraph> junctions = new ArrayList<>(salientNodes.keySet());
		List<Double> centralities = new ArrayList<>(salientNodes.values());
		double maxCentrality = Collections.max(centralities);
		double minCentrality = Collections.min(centralities);

		List<NodeGraph> sortedJunctions = junctions.stream()
				.filter(candidateNode -> checkCriteria(candidateNode, searchDistance))
				.sorted(Comparator
						.comparingDouble(candidateNode -> calculateScore(candidateNode, minCentrality, maxCentrality)))
				.collect(Collectors.toList());

		return sortedJunctions.isEmpty() ? null : sortedJunctions.get(sortedJunctions.size() - 1);
	}

	/**
	 * Checks the criteria for selecting a candidate node based on various
	 * conditions.
	 *
	 * @param candidateNode  The node being evaluated.
	 * @param searchDistance The search distance limit for evaluating potential
	 *                       nodes.
	 * @return {@code true} if the candidate node meets all criteria, {@code false}
	 *         otherwise.
	 */
	private boolean checkCriteria(NodeGraph candidateNode, double searchDistance) {

		return !sequence.contains(candidateNode) && !candidateNode.equals(originNode)
				&& agentNetwork.getEdgeBetween(candidateNode, currentNode) == null
				&& agentNetwork.getEdgeBetween(candidateNode, originNode) == null
				&& GraphUtils.getCachedNodesDistance(currentNode, candidateNode) <= searchDistance;
	}

	/**
	 * Calculates the score for a candidate node based on centrality and distance
	 * gain metrics.
	 *
	 * @param candidateNode The node being evaluated.
	 * @param minCentrality The minimum centrality value in the network.
	 * @param maxCentrality The maximum centrality value in the network.
	 * @return The calculated score for the candidate node, considering centrality
	 *         and distance gain.
	 */
	private double calculateScore(NodeGraph candidateNode, double minCentrality, double maxCentrality) {

		double score = agent.getProperties().usingLocalLandmarks ? localLandmarkness(candidateNode)
				: (candidateNode.getCentrality() - minCentrality) / (maxCentrality - minCentrality);
		double currentDistance = GraphUtils.getCachedNodesDistance(currentNode, destinationNode);
		double distanceGain = (currentDistance - GraphUtils.getCachedNodesDistance(candidateNode, destinationNode))
				/ currentDistance;
		return score * 0.60 + distanceGain * 0.40;

	}

	/**
	 * Generates a sequence of intermediate nodes (on-route marks) between the
	 * origin and destination nodes on the basis of local landmarkness while passing
	 * through region gateways (sequenceGateways).
	 *
	 * @param sequenceNodes An ArrayList of gateways (nodes at the boundary between
	 *                      regions) that need to be traversed on the route.
	 * @return An ArrayList of region-based on-route marks, including the origin and
	 *         destination nodes.
	 */
	public List<NodeGraph> regionOnRouteMarks(List<NodeGraph> sequenceNodes) {

		sequence = new ArrayList<>();
		currentNode = originNode;

		for (NodeGraph exitGateway : sequenceNodes) {
			if (exitGateway.equals(originNode) || currentNode.equals(destinationNode))
				continue;
			sequence.add(currentNode);
			if (currentNode.regionID != exitGateway.regionID) {
				currentNode = exitGateway;
				continue;
			}
			inRegionSequence = new ArrayList<>();
			// works also for nodeBasedNavigation only:
			inRegionSequence = onRouteMarksInRegion(exitGateway);
			sequence.addAll(inRegionSequence);
			currentNode = exitGateway;
		}
		sequence.add(destinationNode);
		return sequence;
	}

	/**
	 * Finds within-region on-route marks from the current node to the passed exit
	 * gateway node, within the current node's region.
	 *
	 * @param exitGateway The exit gateway node that marks the boundary of the
	 *                    region.
	 * @return An ArrayList of in-region on-route marks within the same region,
	 *         including the current node and exit gateway.
	 */
	public List<NodeGraph> onRouteMarksInRegion(NodeGraph exitGateway) {

		Region region = PedSimCity.regionsMap.get(currentNode.regionID);
		findRegionSalientJunctions(region);
		if (salientNodes.isEmpty())
			return inRegionSequence;
		// compute wayfinding complexity and the resulting easinesss
		double wayfindingEasiness = complexity.wayfindingEasinessRegion(currentNode, exitGateway, originNode,
				destinationNode, agent);
		double searchDistance = GraphUtils.getCachedNodesDistance(currentNode, exitGateway) * wayfindingEasiness;

		// while the wayfindingEasiness is lower than the threshold the agent looks for
		// intermediate-points.
		while (wayfindingEasiness < Parameters.wayfindingEasinessThresholdRegions) {
			NodeGraph bestNode = findOnRouteMarkRegion(exitGateway, salientNodes, searchDistance);

			if (bestNode == null || bestNode.equals(exitGateway) || bestNode.equals(destinationNode))
				break;
			inRegionSequence.add(bestNode);
			findRegionSalientJunctions(region);
			if (salientNodes.isEmpty())
				return inRegionSequence;

			wayfindingEasiness = complexity.wayfindingEasinessRegion(bestNode, originNode, destinationNode, exitGateway,
					agent);
			searchDistance = GraphUtils.getCachedNodesDistance(bestNode, exitGateway) * wayfindingEasiness;
			currentNode = bestNode;
			bestNode = null;
		}
		return inRegionSequence;
	}

	/**
	 * Identifies salient junctions within a specific region's graph based on their
	 * centrality in the graph.
	 *
	 * @param region The region for which to identify salient junctions.
	 */
	private void findRegionSalientJunctions(Region region) {

		double percentile = Parameters.salientNodesPercentile;
		salientNodes = new HashMap<NodeGraph, Double>(region.primalGraph.getSubGraphSalientNodes(percentile));

		// If no salient junctions are found, the tolerance increases till the 0.50
		// percentile;
		// still no salient junctions are found, the agent continues without landmarks
		while (salientNodes.isEmpty()) {
			percentile -= 0.05;
			if (percentile < 0.50)
				break;
			salientNodes = new HashMap<NodeGraph, Double>(region.primalGraph.getSubGraphSalientNodes(percentile));
		}
	}

	/**
	 * Finds the most salient on-route mark between the current node and the exit
	 * gateway, amongst the salientNodes within a specific region.
	 *
	 * @param currentNode    The current node in the region.
	 * @param exitGateway    The exit gateway node from the region.
	 * @param salientNodes   A map of salient junctions and their centrality scores
	 *                       within the region.
	 * @param searchDistance The search distance limit for evaluating potential
	 *                       nodes.
	 * @return The selected node that serves as an on-route mark, or null if none is
	 *         found.
	 */
	private NodeGraph findOnRouteMarkRegion(NodeGraph exitGateway, Map<NodeGraph, Double> salientNodes,
			Double searchDistance) {

		List<NodeGraph> junctions = new ArrayList<>(salientNodes.keySet());
		List<Double> centralities = new ArrayList<>(salientNodes.values());
		double currentDistance = GraphUtils.getCachedNodesDistance(currentNode, exitGateway);
		double maxCentrality = Collections.max(centralities);
		double minCentrality = Collections.min(centralities);

		// Optimised sorting using Comparator and Collections.sort
		List<NodeGraph> sortedJunctions = junctions.stream()
				.filter(candidateNode -> checkCriteria(candidateNode, exitGateway, searchDistance, currentDistance))
				.sorted(Comparator.comparingDouble(candidateNode -> calculateScore(candidateNode, exitGateway,
						currentDistance, minCentrality, maxCentrality)))
				.collect(Collectors.toList());

		return sortedJunctions.isEmpty() ? null : sortedJunctions.get(sortedJunctions.size() - 1);
	}

	/**
	 * Checks the criteria for selecting a candidate node based on various
	 * conditions.
	 *
	 * @param candidateNode   The node being evaluated.
	 * @param exitGateway     The exit gateway node within the region.
	 * @param searchDistance  The search distance limit for evaluating potential
	 *                        nodes.
	 * @param currentDistance The current distance to the exit gateway.
	 * @return {@code true} if the candidate node meets all criteria, {@code false}
	 *         otherwise.
	 */
	private boolean checkCriteria(NodeGraph candidateNode, NodeGraph exitGateway, double searchDistance,
			double currentDistance) {

		return !inRegionSequence.contains(candidateNode) && !candidateNode.equals(currentNode)
				&& agentNetwork.getEdgeBetween(candidateNode, currentNode) == null
				&& GraphUtils.getCachedNodesDistance(currentNode, candidateNode) <= searchDistance
				&& GraphUtils.getCachedNodesDistance(candidateNode, exitGateway) <= currentDistance
				&& !sequence.contains(candidateNode);
	}

	/**
	 * Calculates the score for a candidate node based on centrality and gain
	 * metrics within a region.
	 *
	 * @param candidateNode   The candidate node for which the score is calculated.
	 * @param exitGateway     The exit gateway node within the region.
	 * @param currentDistance The current distance to the exit gateway.
	 * @param minCentrality   The minimum centrality value in the network.
	 * @param maxCentrality   The maximum centrality value in the network.
	 * @return The calculated score for the candidate node, considering centrality
	 *         and gain metrics.
	 */
	private double calculateScore(NodeGraph candidateNode, NodeGraph exitGateway, double currentDistance,
			double minCentrality, double maxCentrality) {

		double score;
		if (agent.getProperties().usingLocalLandmarks)
			score = localLandmarkness(candidateNode);
		else
			score = (candidateNode.getCentrality() - minCentrality) / (maxCentrality - minCentrality);
		double gain = (currentDistance - GraphUtils.nodesDistance(candidateNode, exitGateway)) / currentDistance;
		return score * 0.50 + gain * 0.50;
	}

	/**
	 * Computes the local landmarkness score for a given node based on local
	 * landmarks in its proximity and the landmarks known by the agent.
	 *
	 * @param node The node for which to calculate local landmarkness.
	 * @return The computed local landmarkness score for the node.
	 */
	private double localLandmarkness(NodeGraph candidateNode) {
		List<Building> nodeLocalLandmarks = new ArrayList<>(candidateNode.adjacentBuildings);
		VectorLayer agentLocalLandmarks = agent.getCognitiveMap().getLocalLandmarks();
		List<Integer> agentLandmarksIDs = agentLocalLandmarks.getIDs();

		for (Building landmark : candidateNode.adjacentBuildings) {
			if (!agentLandmarksIDs.contains(landmark.buildingID))
				nodeLocalLandmarks.remove(landmark);
		}

		if (nodeLocalLandmarks.isEmpty())
			return 0.0;
		List<Double> localScores = new ArrayList<>();
		for (Building landmark : nodeLocalLandmarks)
			localScores.add(landmark.attributes.get("localLandmarkness").getDouble());
		return Collections.max(localScores);
	}

	/**
	 * Computes the global landmarkness score for a target node based on the global
	 * landmarks in its proximity and their relationship with the destination node.
	 *
	 * @param targetNode      The target node being examined.
	 * @param destinationNode The final destination node.
	 * @return The computed global landmarkness score for the target node.
	 */
	public static double globalLandmarknessNode(NodeGraph targetNode, NodeGraph destinationNode) {

		// get the distant landmarks
		List<Building> distantLandmarks = new ArrayList<>(targetNode.visibleBuildings3d);

		if (distantLandmarks.isEmpty())
			return 0.0;

		// get the anchors of the destination
		List<Building> anchors = new ArrayList<>(LandmarkIntegration.getAnchors(destinationNode).getArray());
		double nodeGlobalScore = 0.0;
		double targetDistance = GraphUtils.getCachedNodesDistance(targetNode, destinationNode);
		for (Building landmark : distantLandmarks) {
			if (!anchors.isEmpty() && !anchors.contains(landmark))
				continue;

			double score = landmark.attributes.get("globalLandmarkness").getDouble();
			ArrayList<Double> distances = new ArrayList<>(LandmarkIntegration.getDistances(destinationNode).getArray());
			double distanceLandmark = distances.get(anchors.indexOf(landmark));
			double distanceWeight = Math.min(targetDistance / distanceLandmark, 1.0);
			score *= distanceWeight;

			if (anchors.isEmpty())
				score *= 0.90;
			nodeGlobalScore = Math.max(nodeGlobalScore, score);
		}
		return nodeGlobalScore;
	}

	/**
	 * Computes the global landmarkness for a target dual node based on the global
	 * landmarks in its proximity and their relationship with the destination node.
	 *
	 * @param centroid        The current centroid node.
	 * @param targetCentroid  The target centroid node.
	 * @param destinationNode The destination node.
	 * @return The computed global landmarkness score for the dual node.
	 */
	public static double globalLandmarknessDualNode(NodeGraph centroid, NodeGraph targetCentroid,
			NodeGraph destinationNode) {

		// current real segment: identifying the node
		DirectedEdge streetSegment = targetCentroid.getPrimalEdge().getDirEdge(0);
		NodeGraph targetNode = (NodeGraph) streetSegment.getToNode(); // targetNode
		if (GraphUtils.getPrimalJunction(centroid, targetCentroid).equals(targetNode))
			targetNode = (NodeGraph) streetSegment.getFromNode();

		return globalLandmarknessNode(targetNode, destinationNode);
	}
}
