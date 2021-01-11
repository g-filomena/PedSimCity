/**
 * Series of functions that support landmark-based navigation, landmarkness computation, identification of on-route marks and wayfinding easiness
 * of a certain space.
 *
 * @author Gabriele Filomena
 */

package sim.app.geo.pedsimcity;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;

import com.vividsolutions.jts.geom.Geometry;

import sim.app.geo.urbanmason.Building;
import sim.app.geo.urbanmason.NodeGraph;
import sim.app.geo.urbanmason.Path;
import sim.app.geo.urbanmason.VectorLayer;
import sim.util.Bag;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.MasonGeometry;

public class LandmarkNavigation {

	/**
	 * It generates a sequence of intermediate between two nodes (origin, destination) on the basis of local landmarkness (identification of
	 * "on-route marks"). The nodes are considered are salient junctions within a certain space, namely junctions likely to be cognitively
	 * represented. These are identified on the basis of betweenness centrality values.
	 *
	 * @param originNode the origin node;
	 * @param destinationNode the destination node;
	 * @param ap the AgentProperties;
	 */

	public static ArrayList<NodeGraph> onRouteMarks(NodeGraph originNode, NodeGraph destinationNode, AgentProperties ap) {

		double percentile = UserParameters.salientNodesPercentile;
		ArrayList<NodeGraph> sequence = new ArrayList<NodeGraph>();
		Map<NodeGraph, Double> knownJunctions = PedSimCity.network.salientNodesWithinSpace(originNode, destinationNode, percentile);
		// If no salient junctions are found, the tolerance increases till the 0.50 percentile;
		// if still no salient junctions are found, the agent continues without landmarks
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


		// while the wayfindingEasiness is lower than the threshold the agent looks for intermediate-points.
		while (wayfindingEasiness < UserParameters.wayfindingEasinessThreshold) {
			NodeGraph bestNode = null;
			double attractivness = 0.0;
			ArrayList<NodeGraph> junctions = new ArrayList<NodeGraph>(knownJunctions.keySet());
			double maxCentrality = Collections.max(knownJunctions.values());
			double minCentrality = Collections.min(knownJunctions.values());

			for (NodeGraph tmpNode : junctions) {
				if (sequence.contains(tmpNode) || tmpNode == originNode || tmpNode.getEdgeWith(currentNode) != null ||
						tmpNode.getEdgeWith(originNode)!= null) continue;

				if (NodeGraph.nodesDistance(currentNode, tmpNode) > searchDistance) continue; //only nodes in range
				double score;
				if (ap.landmarkBasedNavigation) score = localLandmarkness(tmpNode);
				else score = (tmpNode.centrality-minCentrality)/(maxCentrality-minCentrality);
				double currentDistance = NodeGraph.nodesDistance(currentNode, destinationNode);
				double distanceGain = (currentDistance - NodeGraph.nodesDistance(tmpNode, destinationNode))/currentDistance;
				double tmpAttractivness = score*0.60 + distanceGain*0.40;
				if (tmpAttractivness > attractivness) {
					attractivness = tmpAttractivness;
					bestNode = tmpNode;
				}
			}

			if (bestNode == null || bestNode == destinationNode) break;
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
		ArrayList<Building> localLandmarks = new ArrayList<Building>();
		localLandmarks = node.localLandmarks;
		if (localLandmarks.size() == 0) return 0.0;
		List<Double> localScores = new ArrayList<Double>();
		for (Building landmark : localLandmarks) localScores.add(landmark.localLandmarkness);
		return Collections.max(localScores);
	}

	/**
	 * It computes the global Landmarkness of a node in a primal graph route;
	 *
	 * @param targetNode the node that is being examined;
	 * @param destinationNode the final destination node;
	 * @param onlyAnchors it indicates whether only landmarks anchoring the destination should be considered distant landmarks;
	 */

	static double globalLandmarknessNode(NodeGraph targetNode, NodeGraph destinationNode, boolean onlyAnchors) {

		// get the distant landmarks
		ArrayList<Building> distantLandmarks = new ArrayList<Building>();
		distantLandmarks = targetNode.distantLandmarks;
		if (distantLandmarks.size() == 0)  return 0.0;

		if (!onlyAnchors) {
			List<Double> distantScores = new ArrayList<Double>();
			for (Building landmark : distantLandmarks) distantScores.add(landmark.globalLandmarkness);
			return Collections.max(distantScores);
		}

		// get the anchors
		ArrayList<Building> anchors = new ArrayList<Building>();
		anchors = destinationNode.anchors;
		if (onlyAnchors & anchors.size() == 0) return 0.0;
		double nodeGlobalScore = 0.0;
		// identify the best landmark, considering also the distance anchor-destination

		for (Building landmark : distantLandmarks) {
			double score = 0.0;
			if (anchors.contains(landmark)) {
				score = landmark.globalLandmarkness;
				// distance factor

				double distanceLandmark = destinationNode.distances.get(anchors.indexOf(landmark));
				double distanceWeight = NodeGraph.nodesDistance(targetNode, destinationNode)/distanceLandmark;

				if (distanceWeight > 1.0) distanceWeight = 1.0;
				score = score*distanceWeight;
			}
			if (score > nodeGlobalScore) nodeGlobalScore = score;
		}
		return nodeGlobalScore;
	}

	/**
	 * It computes the global landmarkness of a node in a dual graph route;
	 *
	 * @param centroid the node that is being examined;
	 * @param targetCentroid the final destination node;
	 * @param destinationNode the metainformation of the nodes traversed so far;
	 * @param onlyAnchors it indicates whether only landmarks anchoring the destination should be considered distant landmarks;
	 */

	static double globalLandmarknessDualNode(NodeGraph centroid, NodeGraph targetCentroid, NodeGraph destinationNode, boolean onlyAnchors) {

		// current real segment: identifying the node
		GeomPlanarGraphDirectedEdge streetSegment = (GeomPlanarGraphDirectedEdge) targetCentroid.primalEdge.getDirEdge(0);
		NodeGraph targetNode = (NodeGraph) streetSegment.getToNode(); // targetNode
		if (Path.commonPrimalJunction(centroid, targetCentroid) == targetNode) targetNode = (NodeGraph) streetSegment.getFromNode();

		// get the distant landmarks
		ArrayList<Building> distantLandmarks = new ArrayList<Building>();
		distantLandmarks = targetNode.distantLandmarks;
		if (distantLandmarks.size() == 0) return 0.0;

		if (!onlyAnchors) {
			List<Double> distantScores = new ArrayList<Double>();
			for (Building landmark : distantLandmarks) distantScores.add(landmark.globalLandmarkness);
			return Collections.max(distantScores);
		}

		// get the anchors of the destination
		ArrayList<Building> anchors = new ArrayList<Building>();
		anchors = destinationNode.anchors;
		if (onlyAnchors & anchors.size() == 0) return 0.0;
		double nodeGlobalScore = 0.0;

		// identify the best landmark, considering also the anchor-destination distance
		for (Building landmark : distantLandmarks) {
			double score = 0.0;
			if (anchors.contains(landmark)) {
				score = landmark.globalLandmarkness;
				// distance factor
				double distanceLandmark = destinationNode.distances.get(anchors.indexOf(landmark));
				double distanceWeight = NodeGraph.nodesDistance(targetNode, destinationNode)/distanceLandmark;

				if (distanceWeight > 1.0) distanceWeight = 1.0;
				score = score*distanceWeight;
			}
			if (score > nodeGlobalScore) nodeGlobalScore = score;
		}
		return nodeGlobalScore;
	}

	/**
	 * It computes the wayfinding easiness within a space between two nodes
	 * - the ratio between the distance between the passed nodes and a certain maximum distance;
	 * - the legibility complexity, based on the presence of landmarks within a certain space;
	 *
	 * @param originNode the origin node of the whole trip;
	 * @param destinationNode the origin node of the whole trip;
	 * @param typeLandmarkness "global" or "local" landmarks can be used to compute the complexity of the space;
	 */
	public static double wayfindingEasiness(NodeGraph originNode, NodeGraph destinationNode, String typeLandmarkness) {

		double distanceComplexity = NodeGraph.nodesDistance(originNode, destinationNode)/Math.max(PedSimCity.roads.MBR.getHeight(),
				PedSimCity.roads.MBR.getWidth());

		Bag buildings = getBuildings(originNode, destinationNode, 999999);
		Bag landmarks = new Bag();

		// global or local landmarks, different thresholds
		if (typeLandmarkness.equals("global")) landmarks = getLandmarks(buildings, UserParameters.globalLandmarkThreshold, "global");
		else landmarks = getLandmarks(buildings, UserParameters.localLandmarkThreshold, "local");
		// complexity
		double buildingsComplexity = 1.0;
		if (buildings.size() == 0) buildingsComplexity = 0.0;
		else buildingsComplexity = buildingsComplexity(buildings, landmarks);
		double wayfindingComplexity = (distanceComplexity + buildingsComplexity)/2.0;
		// obtain the easiness
		double easiness = 1.0 - wayfindingComplexity;
		return easiness;
	}

	/**
	 * It computes the complexity of a certain area on the basis of the presence of landmarks.
	 *
	 * @param buildings the set of buildings;
	 * @param landmarks the set of landmarks;
	 */
	public static double buildingsComplexity(Bag buildings, Bag landmarks) {
		return ((double) buildings.size()-landmarks.size())/buildings.size();
	}

	/**
	 * It returns all the buildings enclosed between two points or within a region.
	 * Do not pass originNode when using the region.
	 *
	 * @param originNode the first node;
	 * @param destinationNode the second node;
	 * @param region the regionID, when identifying buildings within a region;
	 */
	public static Bag getBuildings(NodeGraph originNode, NodeGraph destinationNode, int region) {

		Bag buildings = new Bag();

		// between the origin and the destination
		if (originNode != null) {
			Geometry smallestCircle = NodeGraph.nodesEnclosingCircle(originNode, destinationNode);
			buildings = PedSimCity.buildings.containedFeatures(smallestCircle);
		}
		// use the region
		else {
			VectorLayer regionNetwork = PedSimCity.regionsMap.get(region).regionNetwork;
			Geometry convexHull = regionNetwork.layerConvexHull();
			buildings = PedSimCity.buildings.containedFeatures(convexHull);
		}
		return buildings;
	}

	/**
	 * It returns landmarks (local or global) amongst a set of buildings (VectorLayer), on the basis of the passe threshold
	 *
	 * @param buildings the set of buildings;
	 * @param threshold the threshold, from 0 to 1;
	 * @param type "local" or "global";
	 */
	public static Bag getLandmarks(VectorLayer buildings, double threshold, String type) {

		Bag landmarks = new Bag();
		buildings.generateGeometriesList();

		String attribute;
		if (type.equals("local")) attribute = "lScore_sc";
		// global
		else attribute = "gScore_sc";

		for (MasonGeometry building: buildings.geometriesList) {
			if (building.getDoubleAttribute(attribute) >= threshold) landmarks.add(building);
		}
		return landmarks;
	}

	/**
	 * It returns landmarks (local or global) amongst a set of buildings (Bag), on the basis of the passe threshold
	 *
	 * @param buildings the set of buildings;
	 * @param threshold the threshold, from 0 to 1;
	 * @param type "local" or "global";
	 */
	public static Bag getLandmarks(Bag buildings, double threshold, String type) {

		Bag landmarks = new Bag();

		String attribute;
		if (type.equals("local")) attribute = "lScore_sc";
		// global
		else attribute = "gScore_sc";

		for (Object o : buildings) {
			MasonGeometry building = (MasonGeometry) o;
			if (building.getDoubleAttribute(attribute) >= threshold) landmarks.add(building);
		}
		return landmarks;
	}

}
