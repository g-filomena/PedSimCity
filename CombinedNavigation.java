package sim.app.geo.pedSimCity;

import java.util.ArrayList;

import sim.app.geo.urbanSim.NodeGraph;
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

		//regional routing necessary Yes/No based on threshold?
		if (NodeGraph.nodesDistance(originNode,  destinationNode) < UserParameters.regionBasedNavigationThreshold
				|| !ap.regionBasedNavigation || originNode.region == destinationNode.region) this.regionBasedNavigation = false;

		if (regionBasedNavigation) {
			RegionBasedNavigation regionsPath = new RegionBasedNavigation();
			sequenceNodes = regionsPath.sequenceRegions(originNode, destinationNode, ap);
		}

		// through barrier (sub-goals), already computed above
		if (ap.barrierBasedNavigation) {;}
		// through local landmarks or important nodes (sub-goals)
		else if ((ap.landmarkBasedNavigation && ap.usingLocalLandmarks) || ap.nodeBasedNavigation ) {
			// when ap.nodeBasedNavigation ap.landmarkBasedNavigation is false;
			if (this.regionBasedNavigation) intraRegionMarks();
			else sequenceNodes = LandmarkNavigation.onRouteMarks(originNode, destinationNode, ap);
		}
		// pure global landmark navigation (no heuristic, no sub-goals)
		else if  (ap.usingGlobalLandmarks && !ap.usingLocalLandmarks && ap.localHeuristic == ""
				&& !this.regionBasedNavigation) return planner.globalLandmarksPath(originNode, destinationNode, ap);

		if (sequenceNodes.size() == 0) {
			sequenceNodes.add(originNode);
			sequenceNodes.add(destinationNode);
		}

		if (ap.localHeuristic.equals("roadDistance")) return planner.roadDistanceSequence(sequenceNodes, ap);
		else if (ap.localHeuristic.equals("angularChange") || ap.localHeuristic.equals("turns")) return planner.angularChangeBasedSequence(sequenceNodes, ap);
		else if (ap.usingGlobalLandmarks && ap.localHeuristic == "") return planner.globalLandmarksPathSequence(sequenceNodes, ap);
		else return null;
	}


	public void intraRegionMarks() {

		NodeGraph entryGateway = originNode;
		for (NodeGraph exitGateway : this.sequenceNodes) {
			if (exitGateway == originNode || entryGateway == destinationNode) continue;
			ArrayList<NodeGraph> onRouteMarks = new ArrayList<NodeGraph>();
			// works also for nodeBasedNavigation only:
			onRouteMarks = LandmarkNavigation.onRouteMarksRegion(entryGateway, exitGateway, originNode, destinationNode, ap);

			if (onRouteMarks.size() == 0 && ap.agentKnowledge <= UserParameters.noobAgentThreshold) {
				BarrierBasedNavigation barrierBasedPath = new BarrierBasedNavigation();
				onRouteMarks = barrierBasedPath.sequenceBarriers(entryGateway, exitGateway, ap.typeBarriers);
			}
			sequenceNodes.addAll(onRouteMarks);
			entryGateway = exitGateway;
		}
	}
}




