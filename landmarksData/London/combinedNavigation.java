package sim.app.geo.pedSimCity;

import java.util.ArrayList;

import sim.app.geo.urbanSim.NodeGraph;
import sim.app.geo.urbanSim.SubGraph;
import sim.app.geo.urbanSim.Utilities;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.MasonGeometry;

public class combinedNavigation{

	NodeGraph originNode, destinationNode;
	double wayfindingEasinessThreshold = PedSimCity.wayfindingEasinessThreshold;
	
	boolean onlyAnchors = true;
	boolean onlyBarriers = true;
	boolean regionBasedNavigation = false;
	boolean barrierBasedNavigation = false;
	boolean landmarkBasedNavigation= false;
	boolean nodeBasedNavigation = false;
			
	double agentKnowledge;
	ArrayList<NodeGraph> sequenceNodes;
	ArrayList<GeomPlanarGraphDirectedEdge> completePath =  new ArrayList<GeomPlanarGraphDirectedEdge>();
	
	public ArrayList<GeomPlanarGraphDirectedEdge> path(NodeGraph originNode, NodeGraph destinationNode, String localHeuristic, 
			boolean barrierBasedNavigation, String algorithm, double agentKnowledge)
	{
		this.agentKnowledge = agentKnowledge;
		
		
		/** 
		 * regional routing necessary Yes/No based on threshold? 
		 */
		if (Utilities.nodesDistance(originNode,  destinationNode) < PedSimCity.regionBasedNavigationThreshold) 
		{
			sequenceNodes.add(originNode);
			sequenceNodes.add(destinationNode);
		}
		
		/** 
		 * A) Combined Region- Barrier-based navigation --> Get Path
		 * B) Initialise regions
		 */
		else
		{
			RegionBasedNavigation regionsPath = new RegionBasedNavigation();
			sequenceNodes = regionsPath.sequenceRegions(originNode, destinationNode, barrierBasedNavigation);
			regionBasedNavigation = true;
		}
		
		if (landmarkBasedNavigation) 
		{
			ArrayList<NodeGraph> combinedSequence = combinedLandmarks();
					
			if (localHeuristic == "roadDistance") completePath = RoutePlanner.roadDistanceLandmarks(combinedSequence, onlyAnchors, 
					regionBasedNavigation, barrierBasedNavigation);
			else completePath = RoutePlanner.angularChangeLandmarks(combinedSequence, onlyAnchors, regionBasedNavigation, barrierBasedNavigation);

		}
		
		else if(nodeBasedNavigation) 
		{
			ArrayList<NodeGraph> combinedSequence = combinedNodes();
					
			if (localHeuristic == "roadDistance") completePath = RoutePlanner.roadDistanceSequence(combinedSequence, regionBasedNavigation,
					barrierBasedNavigation, "dijkstra");
			else completePath = RoutePlanner.angularChangeSequence(combinedSequence, regionBasedNavigation, barrierBasedNavigation,
					"dijkstra");

		}
		return completePath;

	}
		
	/** 
	* using landmarks? 
	* two types of landmark-based navigation, depending on knowledge:
	* a) the agent uses all the landmarks, regardless the fact that they anchor the destination --> unexpert agents (more detours);
	* b) the agent uses only landmarks that anchor the destination;
	*/
	
	public ArrayList<NodeGraph> combinedLandmarks()
	{
		if (agentKnowledge <= PedSimCity.agentNoobThreshold) onlyAnchors = false;
		
		ArrayList<NodeGraph> combinedSequence = new ArrayList<NodeGraph>();
		NodeGraph tmpOrigin = originNode;
		
		for (NodeGraph tmpDestination : this.sequenceNodes)
		{
			if (tmpDestination == originNode || tmpOrigin == destinationNode) continue;
			combinedSequence.add(tmpOrigin);
			
			double wayfindinEasiness = LandmarkNavigation.wayfindingEasinessRegion(originNode, destinationNode, tmpOrigin, tmpDestination);	
			if (wayfindinEasiness > PedSimCity.wayfindingEasinessThreshold)
			{
				tmpOrigin = tmpDestination;
				continue;				
			}
			ArrayList<MasonGeometry> localLandmarks = PedSimCity.regionsMap.get(tmpOrigin.region).localLandmarks;
			
			/** 
			* Are there landmarks in the region?
			* if no --> use barriers if agent is noob
			*/
			
			if ((localLandmarks.size() == 0) || localLandmarks == null)
			{
				if (agentKnowledge <= PedSimCity.agentNoobThreshold)
				{
					BarrierBasedNavigation barrierBasedPath = new BarrierBasedNavigation();
					ArrayList<NodeGraph> sequenceIntraRegion = barrierBasedPath.sequenceBarriers(tmpOrigin, tmpDestination);
					for (NodeGraph tmpNode : sequenceIntraRegion) 
					{
						if (tmpNode == tmpDestination || tmpNode == tmpOrigin) continue;
						combinedSequence.add(tmpNode);
					}
				}
			}
			
			else 
			{
				ArrayList<NodeGraph> sequenceIntraRegion = LandmarkNavigation.findSequenceSubGoals(tmpOrigin,
						tmpDestination, regionBasedNavigation);
				
				for (NodeGraph tmpNode : sequenceIntraRegion) 
				{
					if (tmpNode == tmpDestination || tmpNode == tmpOrigin) continue;
					combinedSequence.add(tmpNode);
				}

			}
			tmpOrigin = tmpDestination;
		}
		combinedSequence.add(destinationNode);
		return combinedSequence;
	}
	
	public ArrayList<NodeGraph> combinedNodes()
	{
		
		ArrayList<NodeGraph> combinedSequence = new ArrayList<NodeGraph>();
		NodeGraph tmpOrigin = originNode;
		
		for (NodeGraph tmpDestination : this.sequenceNodes)
		{
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
	
	public NodeGraph nodeBasedNavigation(NodeGraph originNode, NodeGraph destinationNode, int region)
	{
		SubGraph regionGraph = PedSimCity.regionsMap.get(region).primalGraph;
		ArrayList<NodeGraph> regionSalientNodes = regionGraph.salientNodesInSubGraph(PedSimCity.salientNodesPercentile);
		double attractivness = 0.0;
		NodeGraph bestNode = null;
		
		for (NodeGraph tmpNode : regionSalientNodes)
		{
			double currentDistance = Utilities.nodesDistance(originNode, destinationNode);
	    	double gain = (currentDistance - Utilities.nodesDistance(tmpNode, destinationNode))/currentDistance;
	    	double centrality = tmpNode.centrality_sc; //using rescaled
	    	double salience = centrality*0.60 + gain*0.40;	
	    	if (salience >  attractivness)
	    	{
	    		attractivness = salience;
	    		bestNode = tmpNode;
	    	}
		}
		return bestNode;
	}

}




