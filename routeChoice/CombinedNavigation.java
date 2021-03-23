package pedsimcity.routeChoice;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import pedsimcity.agents.AgentGroupProperties;
import pedsimcity.agents.AgentProperties;
import pedsimcity.elements.Region;
import pedsimcity.main.PedSimCity;
import pedsimcity.main.UserParameters;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import urbanmason.main.NodeGraph;

public class CombinedNavigation {

	NodeGraph originNode, destinationNode;

	AgentGroupProperties ap = new AgentGroupProperties();
	ArrayList<GeomPlanarGraphDirectedEdge> completePath = new ArrayList<>();
	ArrayList<NodeGraph> sequenceNodes = new ArrayList<>();

	public ArrayList<GeomPlanarGraphDirectedEdge> path(NodeGraph originNode, NodeGraph destinationNode,
			AgentGroupProperties agp) {
		this.ap = agp;
		this.originNode = originNode;
		this.destinationNode = destinationNode;
		final RoutePlanner planner = new RoutePlanner();

		// only minimisation
		if (!this.ap.onlyMinimising.equals(""))
			if (this.ap.onlyMinimising.equals("roadDistance"))
				return planner.roadDistance(originNode, destinationNode, this.ap);
			else
				return planner.angularChangeBased(originNode, destinationNode, this.ap);

		// regional routing necessary Yes/No based on threshold? -- does not change the
		// general agent's property
		if (NodeGraph.nodesDistance(originNode, destinationNode) < UserParameters.regionBasedNavigationThreshold
				|| originNode.region == destinationNode.region)
			this.ap.regionBasedNavigation = false;

		if (this.ap.regionBasedNavigation) {
			final RegionBasedNavigation regionsPath = new RegionBasedNavigation();
			this.sequenceNodes = regionsPath.sequenceRegions(originNode, destinationNode, this.ap);
		}

		// region+barriers sub-goals already computed above (if applicable); here just
		// barrier sub-goals:
		if (this.ap.barrierBasedNavigation && !this.ap.regionBasedNavigation) {
			final BarrierBasedNavigation barriersPath = new BarrierBasedNavigation();
			this.sequenceNodes = barriersPath.sequenceBarriers(originNode, destinationNode, this.ap);
		}

		// through local landmarks or important nodes (sub-goals)
		else if (this.ap.landmarkBasedNavigation) {
			if (this.ap.regionBasedNavigation && this.sequenceNodes.size() > 0)
				this.intraRegionMarks();
			else
				this.sequenceNodes = LandmarkNavigation.onRouteMarks(originNode, destinationNode, this.ap);
		}

		// pure global landmark navigation (no heuristic, no sub-goals, it allows)
		else if (this.ap.usingGlobalLandmarks && !this.ap.landmarkBasedNavigation && this.ap.localHeuristic.equals(""))
			if (this.ap.regionBasedNavigation)
				return planner.globalLandmarksPathSequence(this.sequenceNodes, this.ap); // through regions
			else
				return planner.globalLandmarksPath(originNode, destinationNode, this.ap);

		final Set<NodeGraph> set = new HashSet<>(this.sequenceNodes);
		if (set.size() < this.sequenceNodes.size())
			System.out.println("DUPLICATES");

		if (this.sequenceNodes.size() == 0) {
			if (this.ap.localHeuristic.equals("roadDistance"))
				return planner.roadDistance(originNode, destinationNode, this.ap);
			else
				return planner.angularChangeBased(originNode, destinationNode, this.ap);
		} else if (this.ap.localHeuristic.equals("roadDistance"))
			return planner.roadDistanceSequence(this.sequenceNodes, this.ap);
		else
			return planner.angularChangeBasedSequence(this.sequenceNodes, this.ap);
	}

	public void intraRegionMarks() {

		NodeGraph currentLocation = this.originNode;
		final ArrayList<NodeGraph> newSequence = new ArrayList<>();

		for (final NodeGraph exitGateway : this.sequenceNodes) {
			if (exitGateway == this.originNode || currentLocation == this.destinationNode)
				continue;
			newSequence.add(currentLocation);
			if (currentLocation.region != exitGateway.region) {
				currentLocation = exitGateway;
				continue;
			}
			ArrayList<NodeGraph> onRouteMarks = new ArrayList<>();
			// works also for nodeBasedNavigation only:
			onRouteMarks = onRouteMarksRegion(currentLocation, exitGateway, this.originNode, this.destinationNode,
					newSequence, this.ap);

//			if (onRouteMarks.size() == 0 && this.ap.agentKnowledge <= UserParameters.noobAgentThreshold) {
//				System.out.println("using barriers instead");
//				final BarrierBasedNavigation barrierBasedPath = new BarrierBasedNavigation();
//				onRouteMarks = barrierBasedPath.sequenceBarriers(currentLocation, exitGateway, this.ap);
//			}

			newSequence.addAll(onRouteMarks);
			// List<Integer> opo = new ArrayList<Integer>();
			// for (NodeGraph n : onRouteMarks) opo.add(n.getID());
			// System.out.println("control onroutemarks "+Arrays.asList(opo));
			currentLocation = exitGateway;
		}
		newSequence.add(this.destinationNode);
		this.sequenceNodes = newSequence;
	}

	/**
	 * It generates a sequence of intermediate between two nodes (origin,
	 * destination) on the basis of local landmarkness (identification of "on-route
	 * marks"). The nodes are considered are salient junctions within a certain
	 * space, namely junctions likely to be cognitively represented. These are
	 * identified on the basis of betweenness centrality values.
	 *
	 * @param originNode            the origin node;
	 * @param destinationNode       the destination node;
	 * @param regionBasedNavigation if true, when using regions, it examines only
	 *                              salient nodes within the region;
	 * @param typeLandmarkness      it indicates whether the wayfinding complexity
	 *                              towards the destination should be computed by
	 *                              using local or global landmarks;
	 */
	public static ArrayList<NodeGraph> onRouteMarksRegion(NodeGraph currentLocation, NodeGraph exitGateway,
			NodeGraph originNode, NodeGraph destinationNode, ArrayList<NodeGraph> sequenceSoFar, AgentProperties ap) {

		double percentile = UserParameters.salientNodesPercentile;
		final ArrayList<NodeGraph> sequence = new ArrayList<>();
		final Region region = PedSimCity.regionsMap.get(currentLocation.region);
		Map<NodeGraph, Double> knownJunctions = region.primalGraph.salientNodesNetwork(percentile);

		// If no salient junctions are found, the tolerance increases till the 0.50
		// percentile;
		// still no salient junctions are found, the agent continues without landmarks
		while (knownJunctions == null) {
			percentile -= 0.05;
			if (percentile < 0.50)
				return sequence;
			knownJunctions = region.primalGraph.salientNodesNetwork(percentile);
		}
		// compute wayfinding complexity and the resulting easinesss
		double wayfindingEasiness = wayfindingEasinessRegion(currentLocation, exitGateway, originNode, destinationNode,
				ap.typeLandmarks);
		double searchDistance = NodeGraph.nodesDistance(currentLocation, exitGateway) * wayfindingEasiness;

		// while the wayfindingEasiness is lower than the threshold the agent looks for
		// intermediate-points.
		while (wayfindingEasiness < UserParameters.wayfindingEasinessThreshold) {
			NodeGraph bestNode = null;
			double attractivness = 0.0;

			final ArrayList<NodeGraph> junctions = new ArrayList<>(knownJunctions.keySet());
			final double maxCentrality = Collections.max(knownJunctions.values());
			final double minCentrality = Collections.min(knownJunctions.values());
			for (final NodeGraph tmpNode : junctions) {
				// bad candidates (candidate is origin or already visited, etc)
				if (sequence.contains(tmpNode) || tmpNode == currentLocation
						|| tmpNode.getEdgeWith(currentLocation) != null)
					continue;
				if (NodeGraph.nodesDistance(currentLocation, tmpNode) > searchDistance)
					continue; // only nodes in range
				if (NodeGraph.nodesDistance(tmpNode, exitGateway) > NodeGraph.nodesDistance(currentLocation,
						exitGateway))
					continue;
				if (sequenceSoFar.contains(tmpNode))
					continue;

				double score = 0.0;
				if (ap.landmarkBasedNavigation)
					score = LandmarkNavigation.localLandmarkness(tmpNode);
				else
					score = (tmpNode.centrality - minCentrality) / (maxCentrality - minCentrality);
				final double currentDistance = NodeGraph.nodesDistance(currentLocation, exitGateway);
				final double gain = (currentDistance - NodeGraph.nodesDistance(tmpNode, exitGateway)) / currentDistance;

				final double tmp = score * 0.50 + gain * 0.50;
				if (tmp > attractivness) {
					attractivness = tmp;
					bestNode = tmpNode;
				}
			}

			if (bestNode == null || bestNode == exitGateway || bestNode == destinationNode)
				break;
			sequence.add(bestNode);

			percentile = UserParameters.salientNodesPercentile;
			knownJunctions = region.primalGraph.salientNodesNetwork(percentile);
			while (knownJunctions == null) {
				percentile -= 0.05;
				if (percentile < 0.50)
					return sequence;
				knownJunctions = region.primalGraph.salientNodesNetwork(percentile);
			}
			wayfindingEasiness = wayfindingEasinessRegion(bestNode, exitGateway, originNode, destinationNode,
					ap.typeLandmarks);
			searchDistance = NodeGraph.nodesDistance(bestNode, exitGateway) * wayfindingEasiness;
			currentLocation = bestNode;
			bestNode = null;
		}
		return sequence;
	}

	/**
	 * It computes the wayfinding easiness within a region on the basis of: - the
	 * ratio between the distance that would be walked within the region considered
	 * and the distance between the origin and the destination; - the legibility
	 * complexity, based on the presence of landmarks within the region;
	 *
	 * @param originNode      the origin node of the whole trip;
	 * @param destinationNode the origin node of the whole trip;
	 * @param tmpOrigin       the intermediate origin node, within the region;
	 * @param tmpDestination  the intermediate destination node, within the region;
	 */
	public static double wayfindingEasinessRegion(NodeGraph currentLocation, NodeGraph exitGateway,
			NodeGraph originNode, NodeGraph destinationNode, String typeLandmarkness) {

		final double intraRegionDistance = NodeGraph.nodesDistance(currentLocation, exitGateway);
		final double distance = NodeGraph.nodesDistance(originNode, destinationNode);
		final double distanceComplexity = intraRegionDistance / distance;
		if (distanceComplexity < 0.20)
			return 1.0;

		final double buildingsComplexity = PedSimCity.regionsMap.get(currentLocation.region)
				.computeComplexity(typeLandmarkness);
		final double wayfindingComplexity = (distanceComplexity + buildingsComplexity) / 2.0;
		final double easiness = 1.0 - wayfindingComplexity;
		// System.out.println("buildingsComplexity "+ buildingsComplexity + " distance
		// Complexity " + distanceComplexity);
		return easiness;
	}
}
