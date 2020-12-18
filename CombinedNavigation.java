package sim.app.geo.PedSimCity;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;

import sim.app.geo.UrbanSim.NodeGraph;
import sim.util.geo.GeomPlanarGraphDirectedEdge;

public class CombinedNavigation{

	NodeGraph originNode, destinationNode;

	AgentProperties ap = new AgentProperties();
	ArrayList<GeomPlanarGraphDirectedEdge> completePath =  new ArrayList<GeomPlanarGraphDirectedEdge>();
	ArrayList<NodeGraph> sequenceNodes = new ArrayList<NodeGraph>();
	boolean regionBasedNavigation;

	public ArrayList<GeomPlanarGraphDirectedEdge> path(NodeGraph originNode, NodeGraph destinationNode, AgentProperties ap) {
		this.ap = ap;
		this.sequenceNodes.clear();
		RoutePlanner planner = new RoutePlanner();

		//regional routing necessary Yes/No based on threshold? -- does not change the general agent's property
		if (NodeGraph.nodesDistance(originNode,  destinationNode) < UserParameters.regionBasedNavigationThreshold
				|| !ap.regionBasedNavigation || originNode.region == destinationNode.region) this.regionBasedNavigation = false;

		if (regionBasedNavigation) {
			RegionBasedNavigation regionsPath = new RegionBasedNavigation();
			sequenceNodes = regionsPath.sequenceRegions(originNode, destinationNode, ap);
		}

		// through barrier (sub-goals), already computed above
		if (ap.barrierBasedNavigation) {;}
		// through local landmarks or important nodes (sub-goals)
		else if (ap.landmarkBasedNavigation || ap.nodeBasedNavigation ) {
			// when ap.nodeBasedNavigation ap.landmarkBasedNavigation is false;
			if (this.regionBasedNavigation) intraRegionMarks();
			else sequenceNodes = LandmarkNavigation.onRouteMarks(originNode, destinationNode, ap);
		}
		// pure global landmark navigation (no heuristic, no sub-goals, it allows)
		else if  (ap.usingGlobalLandmarks && !ap.landmarkBasedNavigation && ap.localHeuristic == "" && !ap.regionBasedNavigation) {
			System.out.println("returning pure global");
			return planner.globalLandmarksPath(originNode, destinationNode, ap);
		}

		if (sequenceNodes.size() == 0) {
			sequenceNodes.add(originNode);
			sequenceNodes.add(destinationNode);
		}

		if (ap.localHeuristic.equals("roadDistance")) {
			System.out.println("Path: "+ap.localHeuristic +" with regions: "+ ap.regionBasedNavigation + ", local "+ ap.landmarkBasedNavigation +
					", natural barriers" + ap.usingNaturalBarriers);
			return planner.roadDistanceSequence(sequenceNodes, ap);
		}
		else if (ap.localHeuristic.equals("angularChange") || ap.localHeuristic.equals("turns")) {
			System.out.println("Path: "+ap.localHeuristic+" with regions: "+ ap.regionBasedNavigation + ", local "+ ap.landmarkBasedNavigation +
					", natural barriers: " + ap.usingNaturalBarriers);
			return planner.angularChangeBasedSequence(sequenceNodes, ap);
		}
		else if (ap.usingGlobalLandmarks && ap.localHeuristic == "") {
			System.out.println("only GL");
			return planner.globalLandmarksPathSequence(sequenceNodes, ap);
		}
		else return null;
	}


	public void intraRegionMarks() {

		NodeGraph entryGateway = originNode;
		for (NodeGraph exitGateway : this.sequenceNodes) {
			if (exitGateway == originNode || entryGateway == destinationNode) continue;
			ArrayList<NodeGraph> onRouteMarks = new ArrayList<NodeGraph>();
			// works also for nodeBasedNavigation only:
			onRouteMarks = onRouteMarksRegion(entryGateway, exitGateway, originNode, destinationNode, ap);

			if (onRouteMarks.size() == 0 && ap.agentKnowledge <= UserParameters.noobAgentThreshold) {
				BarrierBasedNavigation barrierBasedPath = new BarrierBasedNavigation();
				onRouteMarks = barrierBasedPath.sequenceBarriers(entryGateway, exitGateway, ap.typeBarriers);
			}
			sequenceNodes.addAll(onRouteMarks);
			entryGateway = exitGateway;
		}
	}

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
	public static ArrayList<NodeGraph> onRouteMarksRegion(NodeGraph entryGateway, NodeGraph exitGateway,
			NodeGraph originNode, NodeGraph destinationNode, AgentProperties ap) {

		double percentile = UserParameters.salientNodesPercentile;
		ArrayList<NodeGraph> sequence = new ArrayList<NodeGraph>();
		List<Integer> badCandidates = new ArrayList<Integer>();
		Region region = PedSimCity.regionsMap.get(originNode.region);
		Map<NodeGraph, Double> knownJunctions =  region.primalGraph.salientNodesNetwork(percentile);

		// If no salient junctions are found, the tolerance increases till the 0.50 percentile;
		// still no salient junctions are found, the agent continues without landmarks
		while (knownJunctions == null) {
			percentile -= 0.05;
			if (percentile < 0.50) {
				sequence.add(originNode);
				sequence.add(destinationNode);
				return sequence;
			}
			knownJunctions = region.primalGraph.salientNodesNetwork(percentile);
		}
		// compute wayfinding complexity and the resulting easinesss
		double wayfindingEasiness = wayfindingEasinessRegion(entryGateway, exitGateway, originNode, destinationNode, ap.typeLandmarks);
		double searchDistance = NodeGraph.nodesDistance(entryGateway, entryGateway) * (wayfindingEasiness);
		NodeGraph currentNode = originNode;

		// while the wayfindingEasiness is lower than the threshold the agent looks for intermediate-points.
		while (wayfindingEasiness < UserParameters.wayfindingEasinessThreshold) {
			NodeGraph bestNode = null;
			double attractivness = 0.0;

			ArrayList<NodeGraph> junctions = new ArrayList<NodeGraph>(knownJunctions.keySet());
			double maxCentrality = Collections.max(knownJunctions.values());
			double minCentrality = Collections.min(knownJunctions.values());
			for (NodeGraph tmpNode : junctions) {
				// bad candidates (candidate is destination, or origin, already visited, etc)
				if (sequence.contains(tmpNode) || tmpNode == entryGateway || tmpNode.getEdgeWith(currentNode) != null ||
						tmpNode.getEdgeWith(exitGateway)!= null || tmpNode.getEdgeWith(entryGateway)!= null) continue;

				if (NodeGraph.nodesDistance(currentNode, tmpNode) > searchDistance) {
					badCandidates.add(tmpNode.getID());
					continue; //only nodes in range
				}
				double score = 0.0;
				if (ap.landmarkBasedNavigation) score = LandmarkNavigation.localLandmarkness(tmpNode);
				else score = (tmpNode.centrality-minCentrality)/(maxCentrality-minCentrality);
				double currentDistance = NodeGraph.nodesDistance(currentNode, exitGateway);
				double gain = (currentDistance - NodeGraph.nodesDistance(tmpNode, exitGateway))/currentDistance;

				double tmp = score*0.60 + gain*0.40;
				if (tmp > attractivness) {
					attractivness = tmp;
					bestNode = tmpNode;
				}
			}

			if (bestNode == null || bestNode == destinationNode) break;
			sequence.add(bestNode);

			percentile = UserParameters.salientNodesPercentile;
			knownJunctions = region.primalGraph.salientNodesNetwork(percentile);
			while (knownJunctions == null) {
				percentile -= 0.05;
				if (percentile < 0.50) {
					sequence.add(0, originNode);
					sequence.add(destinationNode);
					return sequence;
				}
				knownJunctions = region.primalGraph.salientNodesNetwork(percentile);
			}
			wayfindingEasiness = wayfindingEasinessRegion(bestNode, exitGateway, originNode, destinationNode, ap.typeLandmarks);
			searchDistance = NodeGraph.nodesDistance(bestNode, exitGateway) * wayfindingEasiness;
			currentNode = bestNode;
			bestNode = null;
		}
		sequence.add(0, originNode);
		sequence.add(destinationNode);
		return sequence;
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
	public static double wayfindingEasinessRegion(NodeGraph entryGateway, NodeGraph exitGateway,NodeGraph originNode, NodeGraph destinationNode,  String typeLandmarkness) {

		double intraRegionDistance = NodeGraph.nodesDistance(entryGateway, exitGateway);
		double distance = NodeGraph.nodesDistance(originNode, destinationNode);
		if (intraRegionDistance/distance < 0.10) return 1;

		double distanceComplexity = intraRegionDistance/distance;
		double buildingsComplexity = PedSimCity.regionsMap.get(entryGateway.region).computeComplexity(typeLandmarkness);
		double wayfindingComplexity = (distanceComplexity + buildingsComplexity)/2.0;
		double easiness = 1.0 - wayfindingComplexity;
		return easiness;
	}
}




