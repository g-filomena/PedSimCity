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
import pedSim.cognitiveMap.AgentCognitiveMap;
import pedSim.cognitiveMap.LandmarkIntegration;
import pedSim.cognitiveMap.Region;
import pedSim.engine.Parameters;
import pedSim.engine.PedSimCity;
import sim.field.geo.VectorLayer;
import sim.graph.Building;
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
	private ArrayList<NodeGraph> sequence = new ArrayList<>();
	private ArrayList<NodeGraph> inRegionSequence = new ArrayList<>();
	private Complexity complexity = new Complexity();
	private Map<NodeGraph, Double> knownJunctions = new HashMap<NodeGraph, Double>();
	private AgentCognitiveMap cognitiveMap;
	private Agent agent;
	private NodeGraph currentNode;

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
		this.cognitiveMap = agent.cognitiveMap;
	}

	/**
	 * Generates a sequence of intermediate nodes (on-route marks) between the
	 * origin and destination nodes on the basis of local landmarkness.
	 *
	 * @return An ArrayList of on-route marks, including the origin and destination
	 *         nodes.
	 */
	public ArrayList<NodeGraph> onRouteMarks() {

		sequence = new ArrayList<>();
		knownJunctions(originNode);

		if (knownJunctions.isEmpty())
			return sequence;

		// compute wayfinding easiness and the resulting research space
		double wayfindingEasiness = complexity.wayfindingEasiness(originNode, destinationNode, agent);
		double searchDistance = GraphUtils.nodesDistance(originNode, destinationNode) * wayfindingEasiness;
		currentNode = originNode;

		// while the wayfindingEasiness is lower than the threshold the agent looks for
		// intermediate-points.
		while (wayfindingEasiness < Parameters.wayfindingEasinessThreshold) {
			NodeGraph bestNode = findOnRouteMark(knownJunctions, searchDistance);
			if (bestNode == null || bestNode.equals(currentNode))
				break;
			sequence.add(bestNode);
			knownJunctions(bestNode);
			if (knownJunctions.isEmpty())
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
	 * Identifies known junctions within the network space between a given node and
	 * the destination node. This method finds known junctions based on their
	 * salience within the specified network space.
	 *
	 * @param node The node from which to identify known junctions, in the space
	 *             with the destination node.
	 */
	public void knownJunctions(NodeGraph node) {

		double percentile = Parameters.salientNodesPercentile;
		knownJunctions = new HashMap<NodeGraph, Double>(
				PedSimCity.network.salientNodesWithinSpace(node, destinationNode, percentile));

		// If no salient junctions are found, the tolerance increases till the 0.50
		// percentile;
		// if still no salient junctions are found, the agent continues without
		// landmarks
		while (knownJunctions.isEmpty()) {
			percentile -= 0.05;
			if (percentile < 0.50) {
				sequence.add(0, originNode);
				sequence.add(destinationNode);
				break;
			}
			knownJunctions = new HashMap<NodeGraph, Double>(
					PedSimCity.network.salientNodesWithinSpace(node, destinationNode, percentile));
		}
	}

	/**
	 * Finds the most salient on-route mark between the current node and the
	 * destination node, amongst the knownJunctions.
	 *
	 * @param currentNode    The current node in the navigation.
	 * @param knownJunctions A map of known junctions and their centrality scores
	 *                       along the route.
	 * @param searchDistance The search distance limit from the currentNode for
	 *                       evaluating potential nodes.
	 * @return The selected node that serves as an on-route mark, or null if none is
	 *         found.
	 */
	private NodeGraph findOnRouteMark(Map<NodeGraph, Double> knownJunctions, Double searchDistance) {

		ArrayList<NodeGraph> junctions = new ArrayList<>(knownJunctions.keySet());
		ArrayList<Double> centralities = new ArrayList<>(knownJunctions.values());
		double maxCentrality = Collections.max(centralities);
		double minCentrality = Collections.min(centralities);

		List<NodeGraph> sortedJunctions = junctions.stream()
				.filter(candidateNode -> checkCriteria(candidateNode, searchDistance))
				.sorted(Comparator
						.comparingDouble(candidateNode -> calculateScore(candidateNode, minCentrality, maxCentrality)))
				.collect(Collectors.toList());

		return sortedJunctions.isEmpty() ? null : sortedJunctions.get(sortedJunctions.size() - 1);
	}

	private boolean checkCriteria(NodeGraph candidateNode, double searchDistance) {

		return !sequence.contains(candidateNode) && !candidateNode.equals(originNode)
				&& PedSimCity.network.getEdgeBetween(candidateNode, currentNode) == null
				&& PedSimCity.network.getEdgeBetween(candidateNode, originNode) == null
				&& GraphUtils.getCachedNodesDistance(currentNode, candidateNode) <= searchDistance;
	}

	private double calculateScore(NodeGraph candidateNode, double minCentrality, double maxCentrality) {

		double score = agent.getProperties().usingLocalLandmarks ? localLandmarkness(candidateNode)
				: (candidateNode.centrality - minCentrality) / (maxCentrality - minCentrality);
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
	 * @param sequenceGateways An ArrayList of gateways (nodes at the boundary
	 *                         between regions) that need to be traversed on the
	 *                         route.
	 * @return An ArrayList of region-based on-route marks, including the origin and
	 *         destination nodes.
	 */
	public ArrayList<NodeGraph> regionOnRouteMarks(ArrayList<NodeGraph> sequenceGateways) {

		sequence = new ArrayList<>();
		currentNode = originNode;

		for (NodeGraph exitGateway : sequenceGateways) {
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
	 * @param currentNode The current node within the region.
	 * @param exitGateway The exit gateway node that marks the boundary of the
	 *                    region.
	 * @return An ArrayList of in-region on-route marks within the same region,
	 *         including the current node and exit gateway.
	 */
	public ArrayList<NodeGraph> onRouteMarksInRegion(NodeGraph exitGateway) {

		Region region = PedSimCity.regionsMap.get(currentNode.regionID);
		regionKnownJunctions(region);
		if (knownJunctions.isEmpty())
			return inRegionSequence;
		// compute wayfinding complexity and the resulting easinesss
		double wayfindingEasiness = complexity.wayfindingEasinessRegion(currentNode, exitGateway, originNode,
				destinationNode, agent);
		double searchDistance = GraphUtils.getCachedNodesDistance(currentNode, exitGateway) * wayfindingEasiness;

		// while the wayfindingEasiness is lower than the threshold the agent looks for
		// intermediate-points.
		while (wayfindingEasiness < Parameters.wayfindingEasinessThresholdRegions) {
			NodeGraph bestNode = findOnRouteMarkRegion(exitGateway, knownJunctions, searchDistance);

			if (bestNode == null || bestNode.equals(exitGateway) || bestNode.equals(destinationNode))
				break;
			inRegionSequence.add(bestNode);
			regionKnownJunctions(region);
			if (knownJunctions.isEmpty())
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
	 * Identifies known junctions within a specific region's graph based on their
	 * centrality in the graph.
	 *
	 * @param region The region for which to identify known junctions.
	 */
	private void regionKnownJunctions(Region region) {

		double percentile = Parameters.salientNodesPercentile;
		knownJunctions = new HashMap<NodeGraph, Double>(region.primalGraph.salientNodes);

		// If no salient junctions are found, the tolerance increases till the 0.50
		// percentile;
		// still no salient junctions are found, the agent continues without landmarks
		while (knownJunctions.isEmpty()) {
			percentile -= 0.05;
			if (percentile < 0.50)
				break;
			knownJunctions = new HashMap<NodeGraph, Double>(region.primalGraph.subGraphSalientNodes(percentile));
		}
	}

	/**
	 * Finds the most salient on-route mark between the current node and the exit
	 * gateway, amongst the knownJunctions within a specific region.
	 *
	 * @param currentNode    The current node in the region.
	 * @param exitGateway    The exit gateway node from the region.
	 * @param knownJunctions A map of known junctions and their centrality scores
	 *                       within the region.
	 * @param searchDistance The search distance limit for evaluating potential
	 *                       nodes.
	 * @return The selected node that serves as an on-route mark, or null if none is
	 *         found.
	 */
	private NodeGraph findOnRouteMarkRegion(NodeGraph exitGateway, Map<NodeGraph, Double> knownJunctions,
			Double searchDistance) {

		ArrayList<NodeGraph> junctions = new ArrayList<>(knownJunctions.keySet());
		ArrayList<Double> centralities = new ArrayList<>(knownJunctions.values());
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

	private boolean checkCriteria(NodeGraph candidateNode, NodeGraph exitGateway, double searchDistance,
			double currentDistance) {

		return !inRegionSequence.contains(candidateNode) && !candidateNode.equals(currentNode)
				&& PedSimCity.network.getEdgeBetween(candidateNode, currentNode) == null
				&& GraphUtils.getCachedNodesDistance(currentNode, candidateNode) <= searchDistance
				&& GraphUtils.getCachedNodesDistance(candidateNode, exitGateway) <= currentDistance
				&& !sequence.contains(candidateNode);
	}

	// Helper method for calculating the score
	private double calculateScore(NodeGraph candidateNode, NodeGraph exitGateway, double currentDistance,
			double minCentrality, double maxCentrality) {

		double score;
		if (agent.getProperties().usingLocalLandmarks)
			score = localLandmarkness(candidateNode);
		else
			score = (candidateNode.centrality - minCentrality) / (maxCentrality - minCentrality);
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
		ArrayList<Building> nodeLocalLandmarks = new ArrayList<>(candidateNode.adjacentBuildings);
		VectorLayer agentLocalLandmarks = cognitiveMap.getLocalLandmarks();
		ArrayList<Integer> agentLandmarksIDs = agentLocalLandmarks.getIDs();

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
		ArrayList<Building> distantLandmarks = new ArrayList<>(targetNode.visibleBuildings3d);

		if (distantLandmarks.isEmpty())
			return 0.0;

		// get the anchors of the destination
		ArrayList<Building> anchors = new ArrayList<>(LandmarkIntegration.getAnchors(destinationNode).getArray());
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
		DirectedEdge streetSegment = targetCentroid.primalEdge.getDirEdge(0);
		NodeGraph targetNode = (NodeGraph) streetSegment.getToNode(); // targetNode
		Route route = new Route();
		if (route.commonPrimalJunction(centroid, targetCentroid).equals(targetNode))
			targetNode = (NodeGraph) streetSegment.getFromNode();

		return globalLandmarknessNode(targetNode, destinationNode);
	}
}
