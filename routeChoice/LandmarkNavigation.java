/**
 * Series of functions that support landmark-based navigation, landmarkness computation, identification of on-route marks and wayfinding easiness
 * of a certain space.
 *
 * @author Gabriele Filomena
 */
package pedsimcity.routeChoice;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;

import com.vividsolutions.jts.geom.Geometry;

import pedsimcity.agents.AgentProperties;
import pedsimcity.elements.Building;
import pedsimcity.graph.NodeGraph;
import pedsimcity.main.PedSimCity;
import pedsimcity.main.UserParameters;
import pedsimcity.utilities.Path;
import pedsimcity.utilities.VectorLayer;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.MasonGeometry;

public class LandmarkNavigation {

	/**
	 * It generates a sequence of intermediate between two nodes (origin,
	 * destination) on the basis of local landmarkness (identification of "on-route
	 * marks"). The nodes are considered are salient junctions within a certain
	 * space, namely junctions likely to be cognitively represented. These are
	 * identified on the basis of betweenness centrality values.
	 *
	 * @param originNode      the origin node;
	 * @param destinationNode the destination node;
	 * @param ap              the AgentProperties;
	 */
	public static ArrayList<NodeGraph> onRouteMarks(NodeGraph originNode, NodeGraph destinationNode,
			AgentProperties ap) {

		double percentile = UserParameters.salientNodesPercentile;
		final ArrayList<NodeGraph> sequence = new ArrayList<>();
		Map<NodeGraph, Double> knownJunctions = PedSimCity.network.salientNodesWithinSpace(originNode, destinationNode,
				percentile);
		// If no salient junctions are found, the tolerance increases till the 0.50
		// percentile;
		// if still no salient junctions are found, the agent continues without
		// landmarks
		while (knownJunctions == null) {
			percentile -= 0.05;
			if (percentile < 0.50) {
				sequence.add(originNode);
				sequence.add(destinationNode);
				return sequence;
			}
			knownJunctions = PedSimCity.network.salientNodesWithinSpace(originNode, destinationNode, percentile);
		}
		// compute wayfinding easinesss and the resulting research space
		double wayfindingEasiness = wayfindingEasiness(originNode, destinationNode, ap.typeLandmarks);
		double searchDistance = NodeGraph.nodesDistance(originNode, destinationNode) * wayfindingEasiness;
		NodeGraph currentNode = originNode;

		// while the wayfindingEasiness is lower than the threshold the agent looks for
		// intermediate-points.
		while (wayfindingEasiness < UserParameters.wayfindingEasinessThreshold) {
			NodeGraph bestNode = null;
			double attractivness = 0.0;
			final ArrayList<NodeGraph> junctions = new ArrayList<>(knownJunctions.keySet());
			final double maxCentrality = Collections.max(knownJunctions.values());
			final double minCentrality = Collections.min(knownJunctions.values());

			for (final NodeGraph tmpNode : junctions) {
				if (sequence.contains(tmpNode) || tmpNode == originNode || tmpNode.getEdgeWith(currentNode) != null
						|| tmpNode.getEdgeWith(originNode) != null)
					continue;

				if (NodeGraph.nodesDistance(currentNode, tmpNode) > searchDistance)
					continue; // only nodes in range
				double score;
				if (ap.landmarkBasedNavigation)
					score = localLandmarkness(tmpNode);
				else
					score = (tmpNode.centrality - minCentrality) / (maxCentrality - minCentrality);
				final double currentDistance = NodeGraph.nodesDistance(currentNode, destinationNode);
				final double distanceGain = (currentDistance - NodeGraph.nodesDistance(tmpNode, destinationNode))
						/ currentDistance;
				final double tmpAttractivness = score * 0.60 + distanceGain * 0.40;
				if (tmpAttractivness > attractivness) {
					attractivness = tmpAttractivness;
					bestNode = tmpNode;
				}
			}

			if (bestNode == null || bestNode == destinationNode)
				break;
			sequence.add(bestNode);
			percentile = UserParameters.salientNodesPercentile;
			knownJunctions = PedSimCity.network.salientNodesWithinSpace(bestNode, destinationNode, percentile);
			while (knownJunctions == null) {
				percentile -= 0.05;
				if (percentile < 0.50) {
					sequence.add(0, originNode);
					sequence.add(destinationNode);
					return sequence;
				}
				knownJunctions = PedSimCity.network.salientNodesWithinSpace(bestNode, destinationNode, percentile);
			}
			wayfindingEasiness = wayfindingEasiness(bestNode, destinationNode, ap.typeLandmarks);
			searchDistance = NodeGraph.nodesDistance(bestNode, destinationNode) * wayfindingEasiness;
			currentNode = bestNode;
			bestNode = null;
		}
		sequence.add(0, originNode);
		sequence.add(destinationNode);
		return sequence;
	}

	/**
	 * It computes the local salience of a node (primal and dual);
	 *
	 * @param node, the candidate node
	 */
	static double localLandmarkness(NodeGraph node) {
		ArrayList<Building> localLandmarks = new ArrayList<>();
		localLandmarks = node.localLandmarks;
		if (localLandmarks.size() == 0)
			return 0.0;
		final List<Double> localScores = new ArrayList<>();
		for (final Building landmark : localLandmarks)
			localScores.add(landmark.localLandmarkness);
		return Collections.max(localScores);
	}

	/**
	 * It computes the global Landmarkness of a node in a primal graph route;
	 *
	 * @param targetNode      the node that is being examined;
	 * @param destinationNode the final destination node;
	 * @param onlyAnchors     it indicates whether only landmarks anchoring the
	 *                        destination should be considered distant landmarks;
	 */

	public static double globalLandmarknessNode(NodeGraph targetNode, NodeGraph destinationNode, boolean onlyAnchors) {

		// get the distant landmarks
		ArrayList<Building> distantLandmarks = new ArrayList<>();
		distantLandmarks = targetNode.distantLandmarks;

		if (distantLandmarks.size() == 0)
			return 0.0;

		// get the anchors of the destination
		ArrayList<Building> anchors = new ArrayList<>();
		anchors = destinationNode.anchors;
		double nodeGlobalScore = 0.0;

		for (final Building landmark : distantLandmarks) {
			double score = 0.0;
			{
				if (onlyAnchors && anchors.size() != 0 && !anchors.contains(landmark))
					continue;

				score = landmark.globalLandmarkness;
				// distance factor

				final double distanceLandmark = destinationNode.distances.get(anchors.indexOf(landmark));
				double distanceWeight = NodeGraph.nodesDistance(targetNode, destinationNode) / distanceLandmark;

				if (distanceWeight > 1.0)
					distanceWeight = 1.0;
				score = score * distanceWeight;
				if (onlyAnchors && anchors.size() == 0)
					score = score * 0.90;
			}
			if (score > nodeGlobalScore)
				nodeGlobalScore = score;
		}
		return nodeGlobalScore;
	}

	/**
	 * It computes the global landmarkness of a node in a dual graph route;
	 *
	 * @param centroid        the node that is being examined;
	 * @param targetCentroid  the final destination node;
	 * @param destinationNode the metainformation of the nodes traversed so far;
	 * @param onlyAnchors     it indicates whether only landmarks anchoring the
	 *                        destination should be considered distant landmarks;
	 */

	public static double globalLandmarknessDualNode(NodeGraph centroid, NodeGraph targetCentroid,
			NodeGraph destinationNode, boolean onlyAnchors) {

		// current real segment: identifying the node
		final GeomPlanarGraphDirectedEdge streetSegment = (GeomPlanarGraphDirectedEdge) targetCentroid.primalEdge
				.getDirEdge(0);
		NodeGraph targetNode = (NodeGraph) streetSegment.getToNode(); // targetNode
		if (Path.commonPrimalJunction(centroid, targetCentroid) == targetNode)
			targetNode = (NodeGraph) streetSegment.getFromNode();

		return globalLandmarknessNode(targetNode, destinationNode, onlyAnchors);
	}

	/**
	 * It computes the wayfinding easiness within a space between two nodes - the
	 * ratio between the distance between the passed nodes and a certain maximum
	 * distance; - the legibility complexity, based on the presence of landmarks
	 * within a certain space;
	 *
	 * @param originNode       the origin node of the whole trip;
	 * @param destinationNode  the origin node of the whole trip;
	 * @param typeLandmarkness "global" or "local" landmarks can be used to compute
	 *                         the complexity of the space;
	 */
	public static double wayfindingEasiness(NodeGraph node, NodeGraph destinationNode, String typeLandmarkness) {

		final double distanceComplexity = NodeGraph.nodesDistance(node, destinationNode)
				/ Math.max(PedSimCity.roads.MBR.getHeight(), PedSimCity.roads.MBR.getWidth());

		final ArrayList<MasonGeometry> buildings = getBuildings(node, destinationNode, 999999);
		ArrayList<MasonGeometry> landmarks = new ArrayList<>();

		// global or local landmarks, different thresholds
		if (typeLandmarkness.equals("global"))
			landmarks = getLandmarks(buildings, UserParameters.globalLandmarkThreshold, "global");
		else
			landmarks = getLandmarks(buildings, UserParameters.localLandmarkThreshold, "local");
		// complexity
		double buildingsComplexity = 1.0;
		if (buildings.size() == 0)
			buildingsComplexity = 0.0;
		else
			buildingsComplexity = buildingsComplexity(buildings, landmarks);
		final double wayfindingComplexity = (distanceComplexity + buildingsComplexity) / 2.0;
		// obtain the easiness
		final double easiness = 1.0 - wayfindingComplexity;
		return easiness;
	}

	/**
	 * It computes the complexity of a certain area on the basis of the presence of
	 * landmarks.
	 *
	 * @param buildings the set of buildings;
	 * @param landmarks the set of landmarks;
	 */
	public static double buildingsComplexity(ArrayList<MasonGeometry> buildings, ArrayList<MasonGeometry> landmarks) {
		return ((double) buildings.size() - landmarks.size()) / buildings.size();
	}

	/**
	 * It returns all the buildings enclosed between two points or within a region.
	 * Do not pass originNode when using the region.
	 *
	 * @param originNode      the first node;
	 * @param destinationNode the second node;
	 * @param region          the regionID, when identifying buildings within a
	 *                        region;
	 */
	public static ArrayList<MasonGeometry> getBuildings(NodeGraph originNode, NodeGraph destinationNode, int region) {

		ArrayList<MasonGeometry> buildings = new ArrayList<>();

		// between the origin and the destination
		if (originNode != null) {
			final Geometry smallestCircle = NodeGraph.nodesEnclosingCircle(originNode, destinationNode);
			buildings = PedSimCity.buildings.containedFeatures(smallestCircle);
		}
		// use the region
		else {
			final VectorLayer regionNetwork = PedSimCity.regionsMap.get(region).regionNetwork;
			final Geometry convexHull = regionNetwork.layerConvexHull();
			buildings = PedSimCity.buildings.containedFeatures(convexHull);
		}
		return buildings;
	}

	/**
	 * It returns landmarks (local or global) amongst a set of buildings
	 * (VectorLayer), on the basis of the passed threshold
	 *
	 * @param buildings the set of buildings;
	 * @param threshold the threshold, from 0 to 1;
	 * @param type      "local" or "global";
	 */
	public static ArrayList<MasonGeometry> getLandmarks(VectorLayer buildings, double threshold, String type) {

		final ArrayList<MasonGeometry> landmarks = new ArrayList<>();
		buildings.generateGeometriesList();

		String attribute;
		if (type.equals("local"))
			attribute = "lScore_sc";
		// global
		else
			attribute = "gScore_sc";

		for (final MasonGeometry building : buildings.geometriesList)
			if (building.getDoubleAttribute(attribute) >= threshold)
				landmarks.add(building);
		return landmarks;
	}

	/**
	 * It returns landmarks (local or global) amongst a set of buildings (Bag), on
	 * the basis of the passe threshold
	 *
	 * @param buildings the set of buildings;
	 * @param threshold the threshold, from 0 to 1;
	 * @param type      "local" or "global";
	 */
	public static ArrayList<MasonGeometry> getLandmarks(ArrayList<MasonGeometry> buildings, double threshold,
			String type) {

		final ArrayList<MasonGeometry> landmarks = new ArrayList<>();

		String attribute;
		if (type.equals("local"))
			attribute = "lScore_sc";
		// global
		else
			attribute = "gScore_sc";

		for (final Object o : buildings) {
			final MasonGeometry building = (MasonGeometry) o;
			if (building.getDoubleAttribute(attribute) >= threshold)
				landmarks.add(building);
		}
		return landmarks;
	}

}
