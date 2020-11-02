package sim.app.geo.pedSimCity;

import java.util.ArrayList;

import sim.app.geo.urbanSim.NodeGraph;
import sim.app.geo.urbanSim.SubGraph;
import sim.app.geo.urbanSim.Utilities;
import sim.util.Bag;
import sim.util.geo.GeomPlanarGraphDirectedEdge;

public class CombinedNavigation{

	NodeGraph originNode, destinationNode;
	double wayfindingEasinessThreshold = ResearchParameters.wayfindingEasinessThreshold;

	AgentProperties ap = new AgentProperties();
	ArrayList<NodeGraph> sequenceNodes;
	ArrayList<GeomPlanarGraphDirectedEdge> completePath =  new ArrayList<GeomPlanarGraphDirectedEdge>();
	ArrayList<NodeGraph> combinedSequence = new ArrayList<NodeGraph>();

	public ArrayList<GeomPlanarGraphDirectedEdge> path(NodeGraph originNode, NodeGraph destinationNode, AgentProperties ap) {
		this.ap = ap;
		this.combinedSequence.clear();
		RoutePlanner planner = new RoutePlanner();
		/**
		 * regional routing necessary Yes/No based on threshold?
		 */
		if (Utilities.nodesDistance(originNode,  destinationNode) < ResearchParameters.regionBasedNavigationThreshold &
				ap.regionBasedNavigation) {
			sequenceNodes.add(originNode);
			sequenceNodes.add(destinationNode);
		}

		/**
		 * A) Combined Region- Barrier-based navigation --> Get Path
		 * B) Initialise regions
		 */
		else {
			RegionBasedNavigation regionsPath = new RegionBasedNavigation();
			sequenceNodes = regionsPath.sequenceRegions(originNode, destinationNode, ap.barrierBasedNavigation, ap.typeOfBarriers);
			ap.regionBasedNavigation = true;
		}

		combinedLandmarks();
		if (ap.landmarkBasedNavigation) {
			if (ap.localHeuristic == "roadDistance") completePath = planner.roadDistanceLandmarks(combinedSequence, ap);
			else completePath = planner.angularChangeLandmarks(combinedSequence,ap);
		}

		else if(ap.nodeBasedNavigation) {
			if (ap.localHeuristic == "roadDistance") completePath = planner.roadDistanceSequence(combinedSequence, ap);
			else completePath = planner.angularChangeSequence(combinedSequence, ap);
		}
		return completePath;
	}

	/**
	 * using landmarks?
	 * two types of landmark-based navigation, depending on knowledge:
	 * a) the agent uses all the landmarks, regardless the fact that they anchor the destination --> unexpert agents (more detours);
	 * b) the agent uses only landmarks that anchor the destination;
	 */

	public void combinedLandmarks() {

		NodeGraph tmpOrigin = originNode;

		for (NodeGraph tmpDestination : this.sequenceNodes) {
			if (tmpDestination == originNode || tmpOrigin == destinationNode) continue;
			combinedSequence.add(tmpOrigin);

			double wayfindinEasiness = LandmarkNavigation.wayfindingEasinessRegion(originNode, destinationNode, tmpOrigin, tmpDestination,
					ap.typeLandmarks);
			if (wayfindinEasiness > ResearchParameters.wayfindingEasinessThreshold) {
				tmpOrigin = tmpDestination;
				continue;
			}
			Bag localLandmarks = PedSimCity.regionsMap.get(tmpOrigin.region).localLandmarks;

			/**
			 * Are there landmarks in the region?
			 * if no --> use barriers if agent is noob
			 */

			if ((localLandmarks.size() == 0) || localLandmarks == null) {
				if (ap.agentKnowledge <= ResearchParameters.noobAgentThreshold) {
					BarrierBasedNavigation barrierBasedPath = new BarrierBasedNavigation();
					ArrayList<NodeGraph> sequenceIntraRegion = barrierBasedPath.sequenceBarriers(tmpOrigin, tmpDestination, ap.typeOfBarriers);
					for (NodeGraph tmpNode : sequenceIntraRegion) {
						if (tmpNode == tmpDestination || tmpNode == tmpOrigin) continue;
						combinedSequence.add(tmpNode);
					}
				}
			}

			else {
				ArrayList<NodeGraph> sequenceIntraRegion = LandmarkNavigation.findSequenceSubGoals(tmpOrigin, tmpDestination,
						ap.regionBasedNavigation, ap.typeLandmarks);

				for (NodeGraph tmpNode : sequenceIntraRegion) {
					if (tmpNode == tmpDestination || tmpNode == tmpOrigin) continue;
					combinedSequence.add(tmpNode);
				}
			}
			tmpOrigin = tmpDestination;
		}
		combinedSequence.add(destinationNode);
	}

	public ArrayList<NodeGraph> combinedNodes()
	{

		ArrayList<NodeGraph> combinedSequence = new ArrayList<NodeGraph>();
		NodeGraph tmpOrigin = originNode;

		for (NodeGraph tmpDestination : this.sequenceNodes) {
			if (tmpDestination == originNode || tmpOrigin == destinationNode) continue;
			combinedSequence.add(tmpOrigin);
			NodeGraph intermediateNode = nodeBasedNavigation(tmpOrigin, tmpDestination, tmpOrigin.region);
			if (intermediateNode != null) combinedSequence.add(intermediateNode);
			tmpOrigin = tmpDestination;
		}
		combinedSequence.add(destinationNode);
		return combinedSequence;
	}


	/**
	 * The agent uses junctions to navigate.
	 *
	 */

	public NodeGraph nodeBasedNavigation(NodeGraph originNode, NodeGraph destinationNode, int region) {
		SubGraph regionGraph = PedSimCity.regionsMap.get(region).primalGraph;
		ArrayList<NodeGraph> regionSalientNodes = regionGraph.salientNodesInSubGraph(ResearchParameters.salientNodesPercentile);
		double attractivness = 0.0;
		NodeGraph bestNode = null;

		for (NodeGraph tmpNode : regionSalientNodes) {
			double currentDistance = Utilities.nodesDistance(originNode, destinationNode);
			double gain = (currentDistance - Utilities.nodesDistance(tmpNode, destinationNode))/currentDistance;
			double centrality = tmpNode.centrality_sc; //using rescaled
			double salience = centrality*0.60 + gain*0.40;

			if (salience >  attractivness) {
				attractivness = salience;
				bestNode = tmpNode;
			}
		}
		return bestNode;
	}

}




