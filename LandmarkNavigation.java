/**
 * Series of functions that support landmark-based navigation, landmarkness computation, identification of on-route marks and wayfinding easiness
 * of a certain space.
 *
 * @author Gabriele Filomena
 */

package sim.app.geo.pedSimCity;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;
import java.util.Set;

import com.vividsolutions.jts.geom.Geometry;

import sim.app.geo.urbanSim.EdgeGraph;
import sim.app.geo.urbanSim.NodeGraph;
import sim.app.geo.urbanSim.NodeWrapper;
import sim.app.geo.urbanSim.Utilities;
import sim.app.geo.urbanSim.VectorLayer;
import sim.util.Bag;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.MasonGeometry;

public class LandmarkNavigation
{
	/**
	 * It generates a sequence of intermediate between two nodes (origin, destination) on the basis of local landmarkness (identification of
	 * "on-route marks"). The nodes are considered are salient junctions within a certain space, namely junctions likely to be cognitively
	 * represented. These are identified on the basis of betweenness centrality values.
	 *
	 * @param originNode the origin node;
	 * @param destinationNode the destination node;
	 * @param regionBasedNavigation  if true, when using regions, it examines only salient nodes within the region;
	 * @param typeLandmarkness it indicates whether the wayfinding complexity towards the destination should be computed by using
	 * 		local or global landmarks;
	 */

	public static ArrayList<NodeGraph> findSequenceSubGoals(NodeGraph originNode, NodeGraph destinationNode, boolean regionBasedNavigation,
			String typeLandmarkness)
	{
		double percentile = researchParameters.salientNodesPercentile;
		ArrayList<NodeGraph> knownJunctions;
		ArrayList<NodeGraph> sequence = new ArrayList<NodeGraph>();
		List<Integer> badCandidates = new ArrayList<Integer>();

		if (!regionBasedNavigation) knownJunctions = PedSimCity.network.salientNodesBewteenSpace(originNode, destinationNode,
				0,0, percentile, "local");
		else
		{
			RegionData region = PedSimCity.regionsMap.get(originNode.region);
			knownJunctions = region.primalGraph.salientNodesBewteenSpace(originNode, destinationNode, 0,0, percentile,"local");
		}

		/**
		 * If no salient junctions are found, the tolerance increases till the 0.50 percentile;
		 * if still no salient junctions are found, the agent continues without landmarks
		 */
		while (knownJunctions == null)
		{
			percentile -= 0.05;
			if (percentile < 0.50)
			{
				sequence.add(originNode);
				sequence.add(destinationNode);
				return sequence;
			}
			knownJunctions = PedSimCity.network.salientNodesBewteenSpace(originNode, destinationNode, 0,0, percentile, "local");
		}
		// compute wayfinding complexity and the resulting easinesss
		double wayfindingEasiness = wayfindingEasiness(originNode, destinationNode, typeLandmarkness);
		double searchDistance = Utilities.nodesDistance(originNode, destinationNode) * (wayfindingEasiness);
		NodeGraph currentNode = originNode;

		// while the wayfindingEasiness is lower than the threshold the agent looks for intermediate-points.
		while (wayfindingEasiness < researchParameters.wayfindingEasinessThreshold)
		{
			NodeGraph bestNode = null;
			double attractivness = 0.0;

			for (NodeGraph tmpNode : knownJunctions)
			{

				// bad candidates (candidate is destination, or origin, already visited, etc)
				if (sequence.contains(tmpNode) || tmpNode == originNode || tmpNode.getEdgeBetween(currentNode) != null ||
						tmpNode.getEdgeBetween(destinationNode)!= null || tmpNode.getEdgeBetween(originNode)!= null) continue;

				if (Utilities.nodesDistance(currentNode, tmpNode) > searchDistance)
				{
					badCandidates.add(tmpNode.getID());
					continue; //only nodes in range
				}
				double localScore = 0.0;
				localScore = localLandmarkness(tmpNode, false, null);

				double currentDistance = Utilities.nodesDistance(currentNode, destinationNode);
				double gain = (currentDistance - Utilities.nodesDistance(tmpNode, destinationNode))/currentDistance;

				double landmarkness = localScore*0.60 + gain*0.40;
				if (landmarkness > attractivness)
				{
					attractivness = landmarkness;
					bestNode = tmpNode;
				}
			}

			if (bestNode == null) break;
			if (bestNode == destinationNode) break;
			sequence.add(bestNode);

			/**
			 * Second and third parameter not necessary here (i.e. set to 0,0)
			 * "local" rescales the nodes' betweenness centrality within the search space;
			 *  otherwise use "global" for the actual, global centrality value.
			 */
			percentile = researchParameters.salientNodesPercentile;
			knownJunctions = PedSimCity.network.salientNodesBewteenSpace(bestNode, destinationNode, 0, 0,  percentile, "local");
			while (knownJunctions == null)
			{
				percentile -= 0.05;
				if (percentile < 0.50)
				{
					sequence.add(0, originNode);
					sequence.add(destinationNode);
					return sequence;
				}
				knownJunctions = PedSimCity.network.salientNodesBewteenSpace(bestNode, destinationNode, 0,0, percentile, "local");
			}
			wayfindingEasiness = wayfindingEasiness(bestNode, destinationNode, typeLandmarkness);
			searchDistance = Utilities.nodesDistance(bestNode, destinationNode) * wayfindingEasiness;
			currentNode = bestNode;
			bestNode = null;
		}
		sequence.add(0, originNode);
		sequence.add(destinationNode);
		return sequence;
	}

	/**
	 * It computes the local salience of a node (primal and dual), given the previous traversed nodes;
	 *
	 * @param node, the candidate node
	 * @param advanceVis it indicates whether 2d advance visibility should be considered
	 * @param mapWrappers the metainformation of the nodes traversed so far
	 */
	static double localLandmarkness(NodeGraph node, boolean advanceVis, HashMap<NodeGraph, NodeWrapper>	mapWrappers)
	{

		List<Integer> localLandmarks = new ArrayList<Integer>();
		localLandmarks = node.localLandmarks;
		double localScore = 0.0;
		if (localLandmarks == null) return 0.0;

		if (!advanceVis) return Collections.max(node.localScores); //if not using the complete formula, just return the max score at the node
		else
		{
			NodeWrapper previous = mapWrappers.get(mapWrappers.get(node).nodeFrom);
			for (int lL : localLandmarks)
			{
				NodeGraph nodeTo =  node;
				NodeGraph nodeFrom = null;
				nodeFrom = previous.node;
				double distanceTravelled = 0;
				double cumulativeAdvanceVis = 0;

				//check previous nodes, while < threshold --> update local salience
				while ((nodeFrom != null) & (distanceTravelled <= researchParameters.visibilityThreshold))
				{
					List<Integer> visible = new ArrayList<Integer>();
					visible = nodeFrom.visible2d;
					NodeWrapper nt = mapWrappers.get(nodeTo);
					EdgeGraph segment = (EdgeGraph) nt.edgeFrom.getEdge();
					distanceTravelled += segment.getLine().getLength();
					if (visible.contains(lL)) cumulativeAdvanceVis += segment.getLine().getLength();

					nodeTo = nodeFrom;
					NodeWrapper nf = mapWrappers.get(nodeFrom);
					try {nodeFrom = nf.nodeFrom;}
					catch (java.lang.NullPointerException e) {nodeFrom = null;}
				}

				double aV = cumulativeAdvanceVis/distanceTravelled;
				if (aV > 1.0) aV = 1.0;
				double tmp = node.localScores.get(localLandmarks.indexOf(lL)) * aV;
				if (tmp > localScore) localScore = tmp;
			}
			return localScore;
		}
	}

	/**
	 * It computes the global Landmarkness of a node in a primal graph route;
	 *
	 * @param targetNode the node that is being examined;
	 * @param destinationNode the final destination node;
	 * @param onlyAnchors it indicates whether only landmarks anchoring the destination should be considered distant landmarks;
	 */

	static double globalLandmarknessNode(NodeGraph targetNode, NodeGraph destinationNode, boolean onlyAnchors)
	{
		// get the distant landmarks
		List<Integer> distantLandmarks = new ArrayList<Integer>();
		distantLandmarks = targetNode.distantLandmarks;
		if (distantLandmarks == null) return 1.0;
		if (!onlyAnchors) return Collections.max(targetNode.distantScores);

		// get the anchors
		List<Integer> anchors = new ArrayList<Integer>();
		anchors = destinationNode.anchors;
		if (onlyAnchors & anchors == null) return 0.0;
		double nodeGlobalScore = 0.0;
		// identify the best landmark, considering also the distance anchor-destination
		for (int dL : distantLandmarks)
		{
			double tmp = 0.0;
			if (anchors.contains(dL))
			{
				tmp = targetNode.distantScores.get(distantLandmarks.indexOf(dL));

				// distance factor
				double distanceLandmark = destinationNode.distances.get(anchors.indexOf(dL));
				double distanceWeight = Utilities.nodesDistance(targetNode, destinationNode)/distanceLandmark;
				if (distanceWeight > 1.0) distanceWeight = 1.0;
				tmp = tmp*distanceWeight;
			}
			if (tmp > nodeGlobalScore) nodeGlobalScore = tmp;
		}
		return nodeGlobalScore;
	}

	/**
	 * It computes the global Landmarkness of a node in a dual graph route;
	 *
	 * @param centroid the node that is being examined;
	 * @param targetCentroid the final destination node;
	 * @param destinationNode the metainformation of the nodes traversed so far;
	 * @param onlyAnchors it indicates whether only landmarks anchoring the destination should be considered distant landmarks;
	 */

	static double globalLandmarknessDualNode(NodeGraph centroid, NodeGraph targetCentroid, NodeGraph destinationNode, boolean onlyAnchors)
	{
		// current real segment: identifying the node
		GeomPlanarGraphDirectedEdge streetSegment = (GeomPlanarGraphDirectedEdge) targetCentroid.primalEdge.getDirEdge(0);
		NodeGraph targetNode = (NodeGraph) streetSegment.getToNode(); // targetNode
		if (Utilities.commonPrimalJunction(centroid, targetCentroid) == targetNode) targetNode = (NodeGraph) streetSegment.getFromNode();

		// get the distant landmarks
		List<Integer> distantLandmarks = new ArrayList<Integer>();
		distantLandmarks = targetNode.distantLandmarks;
		if (distantLandmarks == null) return 0.0;
		if (!onlyAnchors) return Collections.max(targetNode.distantScores);

		// get the anchors of the destination
		List<Integer> anchors = new ArrayList<Integer>();
		anchors = destinationNode.anchors;
		if (onlyAnchors & anchors == null) return 0.0;
		double nodeGlobalScore = 0.0;

		// identify the best landmark, considering also the distance anchor-destination
		for (int dL : distantLandmarks)
		{
			double tmp = 0.0;
			if (anchors.contains(dL))
			{
				tmp = targetNode.distantScores.get(distantLandmarks.indexOf(dL));

				// distance factor
				double distanceLandmark = destinationNode.distances.get(anchors.indexOf(dL));
				double distanceWeight = Utilities.nodesDistance(targetNode, destinationNode)/distanceLandmark;
				if (distanceWeight > 1.0) distanceWeight = 1.0;
				tmp = tmp*distanceWeight;
			}
			if (tmp > nodeGlobalScore) nodeGlobalScore = tmp;
		}
		return nodeGlobalScore;
	}

	/**
	 * It computes the wayfinding easiness within a space between two nodes
	 * - the ratio between the distance between the passed nodes and a certain maximum distance;
	 * - the legibility complexity, based on the presence of landmarks within the region;
	 *
	 * @param originNode the origin node of the whole trip;
	 * @param destinationNode the origin node of the whole trip;
	 * @param typeLandmarkness "global" or "local" landmarks can be used to compute the complexity of the space;
	 */
	public static double wayfindingEasiness(NodeGraph originNode, NodeGraph destinationNode, String typeLandmarkness)
	{
		double distanceComplexity = Utilities.nodesDistance(originNode, destinationNode)/Math.max(PedSimCity.roads.MBR.getHeight(),
				PedSimCity.roads.MBR.getWidth());

		ArrayList<MasonGeometry> buildings = getBuildings(originNode, destinationNode, originNode.region);
		ArrayList<MasonGeometry> landmarks = new ArrayList<MasonGeometry>();

		// global or local landmarks, different thresholds
		if (typeLandmarkness == "global")  landmarks = getLandmarks(buildings, researchParameters.globalLandmarkThreshold, "global");
		else landmarks = getLandmarks(buildings, researchParameters.localLandmarkThreshold, "local");
		// complexity
		double buildingsComplexity = 1.0;
		if (buildings.size() == 0 || buildings == null) buildingsComplexity = 0.0;
		else buildingsComplexity = buildingsComplexity(buildings, landmarks);
		double wayfindingComplexity = (distanceComplexity + buildingsComplexity)/2.0;
		// obtain the easiness
		double easiness = 1.0 - wayfindingComplexity;
		return easiness;
	}

	/**
	 * It computes the wayfinding easiness within a region on the basis of:
	 * - the ratio between the distance that would be walked within the region considered and the distance between the origin and the destination;
	 * - the legibility complexity, based on the presence of landmarks within the region;
	 *
	 * @param originNode the origin node of the whole trip;
	 * @param destinationNode the origin node of the whole trip;
	 * @param tmpOrigin the intermediate origin node, within the region;
	 * @param tmpDestination the intermediate destination node, within the region;
	 */
	public static double wayfindingEasinessRegion(NodeGraph originNode, NodeGraph destinationNode, NodeGraph tmpOrigin, NodeGraph tmpDestination,
			String typeLandmarkness)
	{
		double intraRegionDistance = Utilities.nodesDistance(tmpOrigin, tmpDestination);
		double distance = Utilities.nodesDistance(originNode, destinationNode);
		if (intraRegionDistance/distance < 0.10) return 1;

		double distanceComplexity = intraRegionDistance/distance;
		double buildingsComplexity = PedSimCity.regionsMap.get(tmpOrigin.region).computeComplexity(typeLandmarkness);
		double wayfindingComplexity = (distanceComplexity + buildingsComplexity)/2.0;
		double easiness = 1.0 - wayfindingComplexity;
		return easiness;
	}

	/**
	 * It computes the complexity of a certain area/region on the basis of the presence of landmarks.
	 *
	 * @param buildings the set of buildings;
	 * @param landmarks the set of landmarks;
	 */
	public static double buildingsComplexity(ArrayList<MasonGeometry> buildings, ArrayList<MasonGeometry> landmarks)
	{
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
	public static ArrayList<MasonGeometry> getBuildings(NodeGraph originNode, NodeGraph destinationNode, int region)
	{
		ArrayList<MasonGeometry> buildings = new ArrayList<MasonGeometry>();

		if (originNode != null) // between the origin and the destination
		{
			Geometry smallestCircle = Utilities.smallestEnclosingCircle(originNode, destinationNode);
			Bag filterBuildings = PedSimCity.buildings.getContainedObjects(smallestCircle);
			for (Object o: filterBuildings) buildings.add((MasonGeometry) o);
		}
		else // use the region
		{
			VectorLayer regionNetwork = PedSimCity.regionsMap.get(region).regionNetwork;
			Geometry convexHull = regionNetwork.getConvexHull().getGeometry();
			Bag filterBuildings = PedSimCity.buildings.getContainedObjects(convexHull);
			for (Object o: filterBuildings) buildings.add((MasonGeometry) o);
		}
		return buildings;
	}

	/**
	 * It returns landmarks (local or global) amongst a set of buildings, on the basis of the passe threshold
	 *
	 * @param buildings the set of buildings
	 * @param threshold the threshold, from 0 to 1;
	 * @param type "local" or "global";
	 */
	public static ArrayList<MasonGeometry> getLandmarks(ArrayList<MasonGeometry> buildings, double threshold, String type)
	{
		ArrayList<MasonGeometry> landmarks = new ArrayList<MasonGeometry>();
		String attribute;
		if (type == "local") attribute = "lScore_sc";
		else attribute = "gScore_sc"; // global

		for (MasonGeometry b: buildings)
		{
			if (b.getDoubleAttribute(attribute) >= threshold) landmarks.add(b);
		}
		return landmarks;
	}

	/**
	 * @deprecated
	 */
	@Deprecated
	static double globalLandmarknessPaths(NodeGraph destinationNode, NodeGraph tmpNode, HashMap<NodeGraph,
			NodeWrapper> mapWrappers, boolean onlyAnchors, String method)
	{
		List<Integer> anchors = new ArrayList<Integer>();
		anchors = destinationNode.anchors;
		if (onlyAnchors & anchors == null) return 0.0;

		List<Double> nodeGlobalScores = new ArrayList<Double>();
		Set<Entry<NodeGraph, NodeWrapper>> entries = mapWrappers.entrySet();

		if (!onlyAnchors)
		{
			for (Entry<NodeGraph, NodeWrapper> pair : entries)
			{
				NodeGraph node = pair.getKey();
				List<Integer> distantLandmarks = new ArrayList<Integer>();
				distantLandmarks = node.distantLandmarks;

				double nodeGlobalScore = 0.0;
				if (distantLandmarks == null) nodeGlobalScore = 0.0;
				nodeGlobalScore = Collections.max(node.distantScores);
				nodeGlobalScores.add(nodeGlobalScore);
			}
			if (method == "max") return Collections.max(nodeGlobalScores);
			if (method == "mean")return nodeGlobalScores.stream().mapToDouble(i -> i).average().orElse(0.0);
		}
		else
		{
			for (Entry<NodeGraph, NodeWrapper> pair : entries)
			{
				NodeGraph node = pair.getKey();
				List<Integer> distantLandmarks = new ArrayList<Integer>();
				distantLandmarks = node.distantLandmarks;
				double nodeGlobalScore = 0.0;
				if (distantLandmarks == null)
				{
					nodeGlobalScores.add(nodeGlobalScore);
					continue;
				}
				else
				{
					for (int dL : distantLandmarks)
					{
						double tmp = 0.0;
						if (anchors.contains(dL))
						{
							tmp = node.distantScores.get(distantLandmarks.indexOf(dL));
							double distanceLandmark = destinationNode.distances.get(anchors.indexOf(dL));
							double distanceWeight = Utilities.nodesDistance(tmpNode, destinationNode)/
									distanceLandmark;
							if (distanceWeight > 1.0) distanceWeight = 1.0;
							tmp = tmp*distanceWeight;
						}
						if (tmp > nodeGlobalScore) nodeGlobalScore = tmp;
					}
					nodeGlobalScores.add(nodeGlobalScore);
				}
			}
			if (method == "max") return Collections.max(nodeGlobalScores);
			if (method == "mean") return nodeGlobalScores.stream().mapToDouble(i -> i).average().orElse(0.0);
		}
		return 0.0;
	}

	/**
	 * @deprecated
	 */
	@Deprecated
	static double globalLandmarknessDualPath(NodeGraph dualDestinationNode, NodeGraph tmpNode,
			NodeGraph destinationNode, HashMap<NodeGraph, NodeWrapper> mapWrappers,
			boolean onlyAnchors, String method)
	{

		List<Integer> anchors = new ArrayList<Integer>();
		anchors = destinationNode.anchors;
		if (onlyAnchors & anchors == null) return 0.0;

		List<Double> nodeGlobalScores = new ArrayList<Double>();
		Set<Entry<NodeGraph, NodeWrapper>> entries = mapWrappers.entrySet();
		NodeGraph previous = dualDestinationNode;
		NodeGraph throughNode;

		if (!onlyAnchors)
		{
			for (Entry<NodeGraph, NodeWrapper> pair : entries)
			{
				NodeGraph centroid = pair.getKey();
				GeomPlanarGraphDirectedEdge edgeDirP = (GeomPlanarGraphDirectedEdge)
						centroid.primalEdge.getDirEdge(0);
				NodeGraph nToP = (NodeGraph) edgeDirP.getToNode();
				NodeGraph nFromP = (NodeGraph) edgeDirP.getFromNode();
				if (previous == nToP)  throughNode = nFromP;
				else throughNode = nToP;

				List<Integer> distantLandmarks = new ArrayList<Integer>();
				distantLandmarks = throughNode.distantLandmarks;
				double nodeGlobalScore = 0.0;

				if (distantLandmarks == null) nodeGlobalScore = 0.0;
				nodeGlobalScore = Collections.max(throughNode.distantScores);
				nodeGlobalScores.add(nodeGlobalScore);
				previous = throughNode;
			}
			if (method == "max") return Collections.max(nodeGlobalScores);
			if (method == "mean") return nodeGlobalScores.stream().mapToDouble(i -> i).average().orElse(0.0);
		}
		else
		{
			for (Entry<NodeGraph, NodeWrapper> pair : entries)
			{
				NodeGraph centroid = pair.getKey();
				GeomPlanarGraphDirectedEdge edgeDirP = (GeomPlanarGraphDirectedEdge)
						centroid.primalEdge.getDirEdge(0);
				NodeGraph nToP = (NodeGraph) edgeDirP.getToNode();
				NodeGraph nFromP = (NodeGraph) edgeDirP.getFromNode();
				if (previous == nToP)  throughNode = nFromP;
				else throughNode = nToP;

				List<Integer> distantLandmarks = new ArrayList<Integer>();
				distantLandmarks = throughNode.distantLandmarks;
				double nodeGlobalScore = 0.0;
				if (distantLandmarks == null)
				{
					nodeGlobalScores.add(nodeGlobalScore);
					continue;
				}

				else
				{
					for (int dL : distantLandmarks)
					{
						double tmp = 0.0;
						if (anchors.contains(dL))
						{
							tmp = throughNode.distantScores.get(distantLandmarks.indexOf(dL));
							double distanceLandmark = destinationNode.distances.get(anchors.indexOf(dL));
							double distanceWeight = Utilities.nodesDistance(tmpNode, destinationNode)/distanceLandmark;
							if (distanceWeight > 1.0) distanceWeight = 1.0;
							tmp = tmp*distanceWeight;
						}
						if (tmp > nodeGlobalScore) nodeGlobalScore = tmp;
					}
					nodeGlobalScores.add(nodeGlobalScore);
				}
				previous = throughNode;
			}
			if (method == "max") return Collections.max(nodeGlobalScores);
			if (method == "mean") return nodeGlobalScores.stream().mapToDouble(i -> i).average().orElse(0.0);
		}
		return 0.0;
	}

}
