package pedSim.routeChoice;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

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
	ArrayList<NodeGraph> sequence = new ArrayList<>();
	ArrayList<NodeGraph> inRegionSequence = new ArrayList<>();
	private Map<NodeGraph, Double> knownJunctions = new HashMap<NodeGraph, Double>();
	private AgentCognitiveMap cognitiveMap;
	private Agent agent;

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
		// compute wayfinding easinesss and the resulting research space
		double wayfindingEasiness = Complexity.wayfindingEasiness(originNode, destinationNode, agent);
		double searchDistance = GraphUtils.nodesDistance(originNode, destinationNode) * wayfindingEasiness;
		NodeGraph currentNode = originNode;

		// while the wayfindingEasiness is lower than the threshold the agent looks for
		// intermediate-points.
		while (wayfindingEasiness < Parameters.wayfindingEasinessThreshold) {
			NodeGraph bestNode = findOnRouteMark(currentNode, knownJunctions, searchDistance);
			if (bestNode == null || bestNode.equals(currentNode))
				break;
			sequence.add(bestNode);
			knownJunctions(bestNode);
			if (knownJunctions == null)
				return sequence;
			wayfindingEasiness = Complexity.wayfindingEasiness(bestNode, destinationNode, agent);
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
		knownJunctions = PedSimCity.network.salientNodesWithinSpace(node, this.destinationNode, percentile);

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
			knownJunctions = PedSimCity.network.salientNodesWithinSpace(node, this.destinationNode, percentile);
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
	private NodeGraph findOnRouteMark(NodeGraph currentNode, Map<NodeGraph, Double> knownJunctions,
			Double searchDistance) {

		NodeGraph bestNode = null;
		double attractiveness = 0.0;
		ArrayList<NodeGraph> junctions = new ArrayList<>(knownJunctions.keySet());
		double maxCentrality = Collections.max(knownJunctions.values());
		double minCentrality = Collections.min(knownJunctions.values());

		for (final NodeGraph tmpNode : junctions) {
			if (sequence.contains(tmpNode) || tmpNode.equals(originNode)
					|| PedSimCity.network.getEdgeBetween(tmpNode, currentNode) != null
					|| PedSimCity.network.getEdgeBetween(tmpNode, originNode) != null
					|| GraphUtils.nodesDistance(currentNode, tmpNode) > searchDistance) {
				continue;
			}

			double score = agent.getProperties().usingLocalLandmarks ? localLandmarkness(tmpNode)
					: (tmpNode.centrality - minCentrality) / (maxCentrality - minCentrality);
			double currentDistance = GraphUtils.nodesDistance(currentNode, destinationNode);
			double distanceGain = (currentDistance - GraphUtils.nodesDistance(tmpNode, destinationNode))
					/ currentDistance;
			double tmpAttractiveness = score * 0.60 + distanceGain * 0.40;

			if (tmpAttractiveness > attractiveness) {
				attractiveness = tmpAttractiveness;
				bestNode = tmpNode;
			}
		}
		return bestNode;
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
		NodeGraph currentNode = originNode;

		for (final NodeGraph exitGateway : sequenceGateways) {
			if (exitGateway.equals(this.originNode) || currentNode.equals(this.destinationNode))
				continue;
			sequence.add(currentNode);
			if (currentNode.regionID != exitGateway.regionID) {
				currentNode = exitGateway;
				continue;
			}
			inRegionSequence = new ArrayList<>();
			// works also for nodeBasedNavigation only:
			inRegionSequence = onRouteMarksInRegion(currentNode, exitGateway);
			sequence.addAll(inRegionSequence);
			currentNode = exitGateway;
		}
		sequence.add(this.destinationNode);
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
	public ArrayList<NodeGraph> onRouteMarksInRegion(NodeGraph currentNode, NodeGraph exitGateway) {

		Region region = PedSimCity.regionsMap.get(currentNode.regionID);
		regionKnownJunctions(region);
		if (knownJunctions.isEmpty())
			return inRegionSequence;
		// compute wayfinding complexity and the resulting easinesss
		double wayfindingEasiness = Complexity.wayfindingEasinessRegion(currentNode, exitGateway, originNode,
				destinationNode, agent);
		double searchDistance = GraphUtils.nodesDistance(currentNode, exitGateway) * wayfindingEasiness;

		// while the wayfindingEasiness is lower than the threshold the agent looks for
		// intermediate-points.
		while (wayfindingEasiness < Parameters.wayfindingEasinessThresholdRegions) {

			NodeGraph bestNode = findOnRouteMarkRegion(currentNode, exitGateway, knownJunctions, searchDistance);

			if (bestNode == null || bestNode == exitGateway || bestNode == destinationNode)
				break;
			inRegionSequence.add(bestNode);
			regionKnownJunctions(region);
			if (knownJunctions.isEmpty())
				return inRegionSequence;

			wayfindingEasiness = Complexity.wayfindingEasinessRegion(bestNode, originNode, destinationNode, exitGateway,
					agent);
			searchDistance = GraphUtils.nodesDistance(bestNode, exitGateway) * wayfindingEasiness;
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
		knownJunctions = region.primalGraph.salientNodes;

		// If no salient junctions are found, the tolerance increases till the 0.50
		// percentile;
		// still no salient junctions are found, the agent continues without landmarks
		while (knownJunctions.isEmpty()) {
			percentile -= 0.05;
			if (percentile < 0.50)
				break;
			knownJunctions = region.primalGraph.subGraphSalientNodes(percentile);
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
	private NodeGraph findOnRouteMarkRegion(NodeGraph currentNode, NodeGraph exitGateway,
			Map<NodeGraph, Double> knownJunctions, Double searchDistance) {

		NodeGraph bestNode = null;
		double attractiveness = 0.0;
		ArrayList<NodeGraph> junctions = new ArrayList<>(knownJunctions.keySet());
		double maxCentrality = Collections.max(knownJunctions.values());
		double minCentrality = Collections.min(knownJunctions.values());

		for (final NodeGraph tmpNode : junctions) {
			if (inRegionSequence.contains(tmpNode) || tmpNode.equals(currentNode)
					|| PedSimCity.network.getEdgeBetween(tmpNode, currentNode) != null)
				continue;
			if (GraphUtils.nodesDistance(currentNode, tmpNode) > searchDistance)
				continue;
			if (GraphUtils.nodesDistance(tmpNode, exitGateway) > GraphUtils.nodesDistance(currentNode, exitGateway))
				continue;
			if (sequence.contains(tmpNode))
				continue;

			double score = 0.0;
			if (agent.getProperties().usingLocalLandmarks)
				score = localLandmarkness(tmpNode);
			else
				score = (tmpNode.centrality - minCentrality) / (maxCentrality - minCentrality);

			double currentDistance = GraphUtils.nodesDistance(currentNode, exitGateway);
			double gain = (currentDistance - GraphUtils.nodesDistance(tmpNode, exitGateway)) / currentDistance;

			double tmp = score * 0.50 + gain * 0.50;
			if (tmp > attractiveness) {
				attractiveness = tmp;
				bestNode = tmpNode;
			}
		}
		return bestNode;
	}

	/**
	 * Computes the local landmarkness score for a given node based on local
	 * landmarks in its proximity and the landmarks known by the agent.
	 *
	 * @param node The node for which to calculate local landmarkness.
	 * @return The computed local landmarkness score for the node.
	 */
	private double localLandmarkness(NodeGraph node) {
		ArrayList<Building> nodeLocalLandmarks = new ArrayList<>(node.adjacentBuildings);
		VectorLayer agentLocalLandmarks = cognitiveMap.getLocalLandmarks();
		ArrayList<Integer> agentLandmarksIDs = agentLocalLandmarks.getIDs();

		for (Building landmark : node.adjacentBuildings) {
			if (!agentLandmarksIDs.contains(landmark.buildingID))
				nodeLocalLandmarks.remove(landmark);
		}

		if (nodeLocalLandmarks.isEmpty())
			return 0.0;
		List<Double> localScores = new ArrayList<>();
		for (final Building landmark : nodeLocalLandmarks)
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
		ArrayList<Building> anchors = new ArrayList<>();
		anchors = LandmarkIntegration.getAnchors(destinationNode).getArray();
		double nodeGlobalScore = 0.0;

		for (final Building landmark : distantLandmarks) {
			if (!anchors.isEmpty() && !anchors.contains(landmark))
				continue;

			double score = landmark.attributes.get("globalLandmarkness").getDouble();
			ArrayList<Double> distances = LandmarkIntegration.getDistances(destinationNode).getArray();
			double distanceLandmark = distances.get(anchors.indexOf(landmark));
			double distanceWeight = Math.min(GraphUtils.nodesDistance(targetNode, destinationNode) / distanceLandmark,
					1.0);
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
