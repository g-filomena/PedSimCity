package sim.app.geo.pedestrianSimulation;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import sim.app.geo.pedestrianSimulation.utilities.Path;
import sim.field.geo.GeomVectorField;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.MasonGeometry;

public class routePlanning {
	
	static int barrierID = 999999;
	static boolean regionalRouting = false;
	
    /** Road Distance Shortest Path **/
	public static Path roadDistance(NodeGraph originNode, NodeGraph destinationNode, 
			ArrayList <GeomPlanarGraphDirectedEdge> segmentsToAvoid,  boolean regionalRouting, int barrierID,
			String algorithm, PedestrianSimulation state)
		
	{	
		if (algorithm == "astar")
		{
			AStarRoadDistance pathfinder = new AStarRoadDistance();
		    Path path = pathfinder.astarPath(originNode, destinationNode, segmentsToAvoid, regionalRouting,
		    		barrierID, state);   
		    return path;
		}
		else
		{
			DijkstraRoadDistance pathfinder = new DijkstraRoadDistance();
		    Path path = pathfinder.dijkstraPath(originNode, destinationNode, segmentsToAvoid, regionalRouting,
		    		barrierID, state);    
		    return path;
		}

	}
	
	/** Angular Change Shortest Path **/
	public static Path angularChange(NodeGraph originNode, NodeGraph destinationNode, 
			ArrayList <NodeGraph> centroidsToAvoid, NodeGraph previousJunction, boolean regionalRouting, 
			int barrierID, String algorithm, PedestrianSimulation state)

	{	
		
		NodeGraph dualOrigin = utilities.getDualNode(originNode, regionalRouting);
		NodeGraph dualDestination = null;
    	while (dualDestination == dualOrigin || dualDestination == null) dualDestination = utilities.getDualNode(
    			destinationNode, regionalRouting);
		if (algorithm == "astar")
		{
			AStarAngularChange pathfinder = new AStarAngularChange();
		    Path path = pathfinder.astarPath(dualOrigin, dualDestination, centroidsToAvoid, previousJunction, 
		    		regionalRouting, barrierID,	state);    
		    if (utilities.previousJunction(path.edges, state) == destinationNode) path.edges.remove(path.edges.size()-1);
		    return path;
		}
		else
		{
			DijkstraAngularChange pathfinder = new DijkstraAngularChange();
		    Path path = pathfinder.dijkstraPath(dualOrigin, dualDestination, centroidsToAvoid, previousJunction, barrierID, 
		    		regionalRouting, state); 
		    if (utilities.previousJunction(path.edges, state) == destinationNode) path.edges.remove(path.edges.size()-1);
		    return path;
		}
	}
	
	/**Network Distance Shortest Path **/	
	public static Path topologicalShortestPath (NodeGraph originNode, NodeGraph destinationNode, ArrayList <Integer> segmentsToAvoid, PedestrianSimulation state)
	{	
		DijkstraNetworkDistance pathfinder = new DijkstraNetworkDistance();
	    Path path = pathfinder.dijkstraPath(originNode, destinationNode, segmentsToAvoid, state);    
	    return path;
	}
	
	/** Global Landmarks Path **/
	public static Path globalLandmarksPath (NodeGraph originNode, NodeGraph destinationNode, ArrayList <Integer> segmentsToAvoid, String criteria,
			PedestrianSimulation state)
	{	
//		if (criteria == "road_distance")
//		{
//			DijkstraRoadDistanceLandmarks pathfinder = new DijkstraRoadDistanceLandmarks();
//		    Path path = pathfinder.dijkstraPath(originNode, destinationNode, segmentsToAvoid, state);    
//		    return path;
//		}
//		else if (criteria == "angular_change")
//		{
//	    	Node dualOrigin = utilities.getDualNode(originNode, pedestrianSimulation.dualNetwork);
//	    	Node dualDestination = null;
//	    	while (dualDestination == dualOrigin || dualDestination == null) dualDestination = utilities.getDualNode(
//	    			destinationNode, pedestrianSimulation.dualNetwork);
//			DijkstraAngularChangeLandmarks pathfinder = new DijkstraAngularChangeLandmarks();
//		    Path path = pathfinder.dijkstraPath(dualOrigin, dualDestination, destinationNode, segmentsToAvoid, state);    
//		    return path;
//		}
//		else return null;
		DijkstraGlobalLandmarks pathfinder = new DijkstraGlobalLandmarks();
	    Path path = pathfinder.dijkstraPath(originNode, destinationNode, segmentsToAvoid, state);    
	    return path;		
	}
		
	/** Local and Global Landmarks Path plus Road Distance **/
	public static ArrayList<GeomPlanarGraphDirectedEdge> RoadDistanceLandmarksPath(NodeGraph originNode,
			NodeGraph destinationNode, ArrayList <NodeGraph> sequence, PedestrianSimulation state)
	{   
		ArrayList<GeomPlanarGraphDirectedEdge> newPath = null;
		DijkstraRoadDistanceLandmarks pathFinder = new DijkstraRoadDistanceLandmarks();  
		NodeGraph tmpNode = originNode;
		ArrayList<GeomPlanarGraphDirectedEdge> tmpPath = null;
        ArrayList<NodeGraph> traversedNodes = new ArrayList<NodeGraph>();

		for (NodeGraph intermediateNode : sequence)
    	{	
			if ((traversedNodes != null) && (traversedNodes.contains(intermediateNode))) continue;
			Path tP = null;
			if (sequence.indexOf(intermediateNode) == 0)
			{
				tP = pathFinder.dijkstraPath(tmpNode, intermediateNode, null, state);
				newPath = tP.edges;				
			}
    		else
    		{
				DijkstraRoadDistanceLandmarks tmpPathFinder = new DijkstraRoadDistanceLandmarks(); 
				tP = tmpPathFinder.dijkstraPath(tmpNode, intermediateNode, newPath, state);
    			while (tP.edges == null)
    			{
    				GeomPlanarGraphDirectedEdge lastSegment = newPath.get(newPath.size()-1);
    				if (lastSegment.getFromNode() == tmpNode) tmpNode = (NodeGraph) lastSegment.getToNode();
    				else tmpNode = (NodeGraph) lastSegment.getFromNode();
    				newPath.remove(lastSegment);
    				DijkstraRoadDistanceLandmarks loopPathFinder = new DijkstraRoadDistanceLandmarks(); 
    				tP = loopPathFinder.dijkstraPath(tmpNode, intermediateNode, newPath, state);
    			}
    			tmpPath = tP.edges; 
    			newPath.addAll(tmpPath); 	 
    		}
			
    		for (int i = sequence.indexOf(intermediateNode)+1; i < sequence.size(); i++)
    		{
    			NodeGraph next = sequence.get(i);
				if (tP.mapWrappers.containsKey(next)) traversedNodes.add(next);
    		}
        	tmpNode = intermediateNode;
    	}
		DijkstraRoadDistanceLandmarks tmpPathFinder = new DijkstraRoadDistanceLandmarks(); 
		ArrayList<GeomPlanarGraphDirectedEdge> controlPath = controlPath(destinationNode, newPath);
		if (controlPath != null) return controlPath;
		tmpPath = tmpPathFinder.dijkstraPath(tmpNode, destinationNode, newPath, state).edges; 

		while (tmpPath == null)
		{
			GeomPlanarGraphDirectedEdge lastSegment = newPath.get(newPath.size()-1);
			if (lastSegment.getFromNode() == tmpNode) tmpNode = (NodeGraph) lastSegment.getToNode();
			else tmpNode = (NodeGraph) lastSegment.getFromNode();
			newPath.remove(lastSegment);
			if (tmpNode == destinationNode) return newPath;
			DijkstraRoadDistanceLandmarks loopPathFinder = new DijkstraRoadDistanceLandmarks(); 
			tmpPath = loopPathFinder.dijkstraPath(tmpNode, destinationNode, newPath, state).edges;
		}
    	newPath.addAll(tmpPath);
		return newPath; 
    }
	
	/** Local and Global Landmarks Path plus Angular Change **/
	public static ArrayList<GeomPlanarGraphDirectedEdge> AngularChangeLandmarksPath(NodeGraph originNode, 
			NodeGraph destinationNode, ArrayList <NodeGraph> sequence, PedestrianSimulation state)
	{
		ArrayList<GeomPlanarGraphDirectedEdge> newPath = null;
		NodeGraph dualOrigin = utilities.getDualNode(originNode, regionalRouting);
		NodeGraph dualDestination = utilities.getDualNode(destinationNode, regionalRouting);
		DijkstraAngularChangeLandmarks pathFinder = new DijkstraAngularChangeLandmarks();  
		NodeGraph tmpNode = dualOrigin;
		ArrayList<GeomPlanarGraphDirectedEdge> tmpPath = null;
		ArrayList<NodeGraph> centroidsToAvoid = new ArrayList<NodeGraph>();
		ArrayList<NodeGraph> dualSequence =  new ArrayList<NodeGraph>();
        ArrayList<NodeGraph> traversedCentroids = new ArrayList<NodeGraph>();
        NodeGraph pJ = null;

        for (NodeGraph intermediateNode : sequence) dualSequence.add(utilities.getDualNode(intermediateNode,
        		regionalRouting));
		for (NodeGraph dualIntermediateNode : dualSequence)
    	{

    		if ((traversedCentroids != null) && (traversedCentroids.contains(dualIntermediateNode))) continue;
    		Path tP = null;
    		if (dualSequence.indexOf(dualIntermediateNode) == 0) 
    		{
    			tP = pathFinder.dijkstraPath(tmpNode, dualIntermediateNode, destinationNode, null, pJ, state);
    			newPath = tP.edges;
	        	pJ = utilities.previousJunction(newPath, state);
	        	for (GeomPlanarGraphDirectedEdge e: newPath) centroidsToAvoid.add(
	        			state.centroidsMap.get(((EdgeGraph) e.getEdge()).getID()));
    		}
    		else
    		{
    			DijkstraAngularChangeLandmarks tmpPathFinder = new DijkstraAngularChangeLandmarks(); 
				tP = tmpPathFinder.dijkstraPath(tmpNode, dualIntermediateNode, destinationNode, centroidsToAvoid, pJ, state);
				while (tP.edges == null)
    			{
    				newPath.remove(newPath.size()-1); // remove last one
    				pJ = utilities.previousJunction(newPath, state);
    				GeomPlanarGraphDirectedEdge lastSegment = newPath.get(newPath.size()-1);
    				int lastSegmentID = ((EdgeGraph) lastSegment.getEdge()).getID();
    				tmpNode = state.centroidsMap.get(lastSegmentID);
    				centroidsToAvoid.remove(tmpNode);
    				DijkstraAngularChangeLandmarks loopPathFinder = new DijkstraAngularChangeLandmarks(); 
    				tP = loopPathFinder.dijkstraPath(tmpNode, dualIntermediateNode, destinationNode, centroidsToAvoid, 
    						pJ, state);
    			}
	    		tmpPath = tP.edges; 
	        	for (GeomPlanarGraphDirectedEdge e: tmpPath) centroidsToAvoid.add(
	        			state.centroidsMap.get(((EdgeGraph) e.getEdge()).getID()));
            	pJ = utilities.previousJunction(tmpPath, state);
    			tmpPath.remove(tmpPath.get(0));
    			newPath.addAll(tmpPath); 
    		}
    		
        	for (int i = sequence.indexOf(dualIntermediateNode)+1; i < sequence.size(); i++)
    		{
    			 NodeGraph next = dualSequence.get(i);
    			 if (tP.dualMapWrappers.containsKey(next)) traversedCentroids.add(next);
    		}
        	tmpNode = dualIntermediateNode;
        	centroidsToAvoid.remove(tmpNode);
    	}
    	
		GeomPlanarGraphDirectedEdge destinationEdge = (GeomPlanarGraphDirectedEdge) dualDestination.primalEdge.getDirEdge(0);
		ArrayList<GeomPlanarGraphDirectedEdge> controlPath = controlDualPath(destinationNode, destinationEdge, newPath, state);
		if (controlPath != null) return controlPath;
		
		DijkstraAngularChangeLandmarks tmpPathFinder = new DijkstraAngularChangeLandmarks(); 
		tmpPath = tmpPathFinder.dijkstraPath(tmpNode, dualDestination, destinationNode, centroidsToAvoid, pJ, state).edges; 
		while (tmpPath == null)
		{
			newPath.remove(newPath.size()-1); // remove last one
			pJ = utilities.previousJunction(newPath, state);
			GeomPlanarGraphDirectedEdge lastSegment = newPath.get(newPath.size()-1);
			int lastSegmentID = ((EdgeGraph) lastSegment.getEdge()).getID();
			tmpNode = state.centroidsMap.get(lastSegmentID);
			centroidsToAvoid.remove(tmpNode);
			DijkstraAngularChangeLandmarks loopPathFinder = new DijkstraAngularChangeLandmarks(); 
			tmpPath = loopPathFinder.dijkstraPath(tmpNode, dualDestination, destinationNode, centroidsToAvoid, pJ, state).edges;
		}

		tmpPath.remove(tmpPath.get(0));
    	newPath.addAll(tmpPath); 
    	if (utilities.previousJunction(newPath, state) == destinationNode) newPath.remove(newPath.size()-1);
    	return newPath;
	}
	
	/** Local Landmarks Path plus Road Distance **/
	public static ArrayList<GeomPlanarGraphDirectedEdge> RoadDistanceLocalLandmarks(NodeGraph originNode, 
			NodeGraph destinationNode, ArrayList <NodeGraph> sequence, PedestrianSimulation state)
	{
		
		ArrayList<GeomPlanarGraphDirectedEdge> newPath = null;
		DijkstraRoadDistance pathFinder = new DijkstraRoadDistance();  
		NodeGraph tmpNode = originNode;
		ArrayList<GeomPlanarGraphDirectedEdge> tmpPath = null;
        ArrayList<NodeGraph> traversedNodes = new ArrayList<NodeGraph>();
	        
		for (NodeGraph intermediateNode : sequence)
    	{	
			if ((traversedNodes != null) && (traversedNodes.contains(intermediateNode))) continue;
			Path tP = null;
			if (sequence.indexOf(intermediateNode) == 0) 
			{
				tP = pathFinder.dijkstraPath(tmpNode, intermediateNode, null, regionalRouting,
						barrierID, state);
				newPath = tP.edges;
			}
    		else
    		{
				DijkstraRoadDistance tmpPathFinder = new DijkstraRoadDistance(); 
				tP = tmpPathFinder.dijkstraPath(tmpNode, intermediateNode, newPath, false, barrierID, state);
    			while (tP.edges == null)
    			{
    				GeomPlanarGraphDirectedEdge lastSegment = newPath.get(newPath.size()-1);
    				if (lastSegment.getFromNode() == tmpNode) tmpNode = (NodeGraph) lastSegment.getToNode();
    				else tmpNode = (NodeGraph) lastSegment.getFromNode();
    				newPath.remove(lastSegment);
					DijkstraRoadDistance looPathFinder = new DijkstraRoadDistance(); 
					tP = looPathFinder.dijkstraPath(tmpNode, intermediateNode, newPath, regionalRouting,
							barrierID, state);
    			}
    			tmpPath = tP.edges; 
    			newPath.addAll(tmpPath); 
    		}
    		for (int i = sequence.indexOf(intermediateNode)+1; i < sequence.size(); i++)
    		{
    			NodeGraph next = sequence.get(i);
    			if (tP.mapWrappers.containsKey(next)) traversedNodes.add(next);
    		}	
    		tmpNode = intermediateNode;

    	}
		ArrayList<GeomPlanarGraphDirectedEdge> controlPath = controlPath(destinationNode, newPath);
		if (controlPath != null) return controlPath;
		
		DijkstraRoadDistance tmpPathFinder = new DijkstraRoadDistance(); 
		tmpPath = tmpPathFinder.dijkstraPath(tmpNode, destinationNode, newPath, regionalRouting, barrierID,
				state).edges; 
		while (tmpPath == null)
		{
			
			GeomPlanarGraphDirectedEdge lastSegment = newPath.get(newPath.size()-1);
			if (lastSegment.getFromNode() == tmpNode) tmpNode = (NodeGraph) lastSegment.getToNode();
			else tmpNode = (NodeGraph) lastSegment.getFromNode();
			newPath.remove(lastSegment);
			if (tmpNode == destinationNode) return newPath;
			DijkstraRoadDistance loopPathFinder = new DijkstraRoadDistance(); 
			tmpPath = loopPathFinder.dijkstraPath(tmpNode, destinationNode, newPath,  regionalRouting, barrierID,
					state).edges;
		}
    	newPath.addAll(tmpPath);
		return newPath; 
	}
		
	/** Local Landmarks Path plus angular change **/
		
	public static ArrayList<GeomPlanarGraphDirectedEdge> AngularChangeLocalLandmarks(NodeGraph originNode, 
			NodeGraph destinationNode, ArrayList <NodeGraph> sequence, PedestrianSimulation state)
	{
		ArrayList<GeomPlanarGraphDirectedEdge> newPath = null;
		NodeGraph dualOrigin = utilities.getDualNode(originNode, regionalRouting);
		NodeGraph dualDestination = utilities.getDualNode(destinationNode,regionalRouting);
				
		DijkstraAngularChange pathFinder = new DijkstraAngularChange();  
		NodeGraph tmpNode = dualOrigin;
		ArrayList<GeomPlanarGraphDirectedEdge> tmpPath = null;
		ArrayList<NodeGraph> centroidsToAvoid = new ArrayList<NodeGraph>();
		ArrayList<NodeGraph> dualSequence =  new ArrayList<NodeGraph>();
        ArrayList<NodeGraph> traversedCentroids = new ArrayList<NodeGraph>();
		NodeGraph pJ = null;
		boolean regionalRouting = false;
		        
    	for (NodeGraph intermediateNode : sequence) dualSequence.add(utilities.getDualNode(intermediateNode,
    			regionalRouting));
    	
    	for (NodeGraph dualIntermediateNode : dualSequence)
    	{
    		if ((traversedCentroids != null) && (traversedCentroids.contains(dualIntermediateNode))) continue;
    		Path tP = null;
    		
    		if (dualSequence.indexOf(dualIntermediateNode) == 0) 
    		{
    			tP = pathFinder.dijkstraPath(tmpNode, dualIntermediateNode, null, null, barrierID, regionalRouting, state);
    			newPath = tP.edges; 
	        	pJ = utilities.previousJunction(newPath, state);
	        	centroidsToAvoid = (ArrayList<NodeGraph>) tP.dualMapWrappers.keySet().stream().collect(Collectors.toList());
    		}
    		
			else
    		{
    			DijkstraAngularChange tmpPathFinder = new DijkstraAngularChange(); 
				tP = tmpPathFinder.dijkstraPath(tmpNode, dualIntermediateNode, centroidsToAvoid, pJ, barrierID, regionalRouting, state);
				while (tP.edges == null)
    			{
    				newPath.remove(newPath.size()-1); // remove last one
    				pJ = utilities.previousJunction(newPath, state);
    				GeomPlanarGraphDirectedEdge lastSegment = newPath.get(newPath.size()-1);
    				int lastSegmentID = ((EdgeGraph) lastSegment.getEdge()).getID();
    				tmpNode = state.centroidsMap.get(lastSegmentID);
    				centroidsToAvoid.remove(tmpNode);
    				DijkstraAngularChange loopPathFinder = new DijkstraAngularChange(); 
    				tP = loopPathFinder.dijkstraPath(tmpNode, dualIntermediateNode, centroidsToAvoid, pJ, 
    						barrierID, regionalRouting, state);
    			}
    			
    			for (NodeGraph key : tP.dualMapWrappers.keySet()) {centroidsToAvoid.add(key);}
	    		for (int i = sequence.indexOf(dualIntermediateNode)+1; i < sequence.size(); i++)
	    		{
	    			 NodeGraph next = dualSequence.get(i);
	    			 if (tP.dualMapWrappers.containsKey(next)) traversedCentroids.add(next);
	    		}	 
	    		tmpPath = tP.edges; 
            	pJ = utilities.previousJunction(tmpPath, state);
    			tmpPath.remove(tmpPath.get(0));
    			newPath.addAll(tmpPath); 
    		}
	    	tmpNode = dualIntermediateNode;
	    	centroidsToAvoid.remove(tmpNode);
    		
        	for (int i = sequence.indexOf(dualIntermediateNode)+1; i < sequence.size(); i++)
    		{
        		 NodeGraph next = dualSequence.get(i);
    			 if (tP.dualMapWrappers.containsKey(next)) traversedCentroids.add(next);
    		}
		}
		GeomPlanarGraphDirectedEdge destinationEdge = (GeomPlanarGraphDirectedEdge) dualDestination.primalEdge.getDirEdge(0);
		ArrayList<GeomPlanarGraphDirectedEdge> controlPath = controlDualPath(destinationNode, destinationEdge, newPath, state);
		if (controlPath != null) return controlPath;
		
    	DijkstraAngularChange tmpPathFinder = new DijkstraAngularChange(); 
		tmpPath = tmpPathFinder.dijkstraPath(tmpNode, dualDestination, centroidsToAvoid, pJ, barrierID, regionalRouting, state).edges; 
		while (tmpPath == null)
		{
			newPath.remove(newPath.size()-1); // remove last one
			pJ = utilities.previousJunction(newPath, state);
			GeomPlanarGraphDirectedEdge lastSegment = newPath.get(newPath.size()-1);
			int lastSegmentID = ((EdgeGraph) lastSegment.getEdge()).getID();
			tmpNode = state.centroidsMap.get(lastSegmentID);
			centroidsToAvoid.remove(tmpNode);
			DijkstraAngularChange loopPathFinder = new DijkstraAngularChange(); 
			tmpPath = loopPathFinder.dijkstraPath(tmpNode, dualDestination, centroidsToAvoid, pJ, barrierID, 
					regionalRouting, state).edges;
		}

		tmpPath.remove(tmpPath.get(0));
    	newPath.addAll(tmpPath); 
    	if (utilities.previousJunction(newPath, state) == destinationNode) newPath.remove(newPath.size()-1);
    	return newPath;
    }
	
	
	public static ArrayList<NodeGraph> findSequenceSubGoals(NodeGraph originNode, NodeGraph destinationNode, 
			PedestrianSimulation state)
	{
    	GeomVectorField knownJunctions = landmarkFunctions.relevantNodes(originNode, destinationNode, state);
        double wayfindingComplexity = landmarkFunctions.easinessNavigation(originNode, destinationNode, state);
        double searchRange = utilities.nodesDistance(originNode, destinationNode) * (wayfindingComplexity);
        NodeGraph currentNode = originNode;
        
	    List<Integer> badCandidates = new ArrayList<Integer>();
	    ArrayList<NodeGraph> sequence = new ArrayList<NodeGraph>();
    	while (searchRange >  state.t)
        {
    		NodeGraph bestNode = null;
        	double attractivness = 0.0;
        		        	
        	for (Object o : knownJunctions.getGeometries())
	        {
		    	MasonGeometry geoNode = (MasonGeometry) o;
		    	int nodeID = (int) geoNode.getIntegerAttribute("nodeID");
		    	NodeGraph tmpNode = state.nodesMap.get(nodeID);
		    	
		    	if (sequence.contains(tmpNode)) continue;
		    	if (tmpNode == originNode) continue;
		    	if (Graph.getEdgeBetween(tmpNode, currentNode) != null) continue;
		    	if (Graph.getEdgeBetween(tmpNode, destinationNode)!= null) continue;
		    	if (Graph.getEdgeBetween(tmpNode, originNode)!= null) continue;
		    	
		    	if (utilities.nodesDistance(currentNode, tmpNode) > searchRange)
		    	{
		    		badCandidates.add(nodeID);
		    		continue; //only nodes in range	
		    	}
        		double localScore = 0.0;
        		localScore = landmarkFunctions.localLandmarkness(tmpNode, false, null, state);

		    	double gain = ((utilities.nodesDistance(currentNode, destinationNode) - 
		    			utilities.nodesDistance(tmpNode, destinationNode))/utilities.nodesDistance(currentNode, destinationNode));
		    	
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
        	knownJunctions = landmarkFunctions.relevantNodes(bestNode, destinationNode, state);
        	wayfindingComplexity = landmarkFunctions.easinessNavigation(bestNode, destinationNode, state);
        	searchRange = utilities.nodesDistance(bestNode, destinationNode) * wayfindingComplexity;
            currentNode = bestNode;
            bestNode = null;
        }
    	return sequence;
	}
	
	static ArrayList<GeomPlanarGraphDirectedEdge> regionsBarriersPath (NodeGraph originNode, NodeGraph destinationNode, 
			String criteria, PedestrianSimulation state)
	{
		String localHeuristic = "angularChange";
		if (criteria.contains("roadDistance")) localHeuristic = "roadDistance";
		boolean usingBarriers = false;
		if (criteria.contains("Barriers")) usingBarriers = true;		
		RegionalRouting regionsPath = new RegionalRouting();
		return regionsPath.pathFinderRegions(originNode, destinationNode, localHeuristic, usingBarriers, state);
	}
	
	static ArrayList<GeomPlanarGraphDirectedEdge> barriersPath (NodeGraph originNode, NodeGraph destinationNode, 
			String criteria, PedestrianSimulation state)
	{
		String localHeuristic = "angularChange";
		if (criteria == "roadDistanceBarriers") localHeuristic = "roadDistance";
		BarriersRouting barriersPath = new BarriersRouting();
		return barriersPath.pathFinderBarriers(originNode, destinationNode, localHeuristic, state);
	}
	
	static ArrayList<GeomPlanarGraphDirectedEdge> controlPath(NodeGraph destinationNode, 
			ArrayList <GeomPlanarGraphDirectedEdge> path)
	{

		for (GeomPlanarGraphDirectedEdge e: path)
		{
			if ((e.getFromNode() == destinationNode) || (e.getToNode() == destinationNode))
				{
					int lastIndex = path.indexOf(e);
					path = new ArrayList<GeomPlanarGraphDirectedEdge>(path.subList(0, lastIndex+1));
					return path;			
				}
		}
		return null;
	}
	
	static ArrayList<GeomPlanarGraphDirectedEdge> controlDualPath(NodeGraph destinationNode, 
			GeomPlanarGraphDirectedEdge destinationEdge, ArrayList <GeomPlanarGraphDirectedEdge> path,
			PedestrianSimulation state)
	{
		while (path.contains(destinationEdge))
		{
			if (path.get(path.size()-1) == destinationEdge) 
			{
				if (utilities.previousJunction(path, state) == destinationNode) path.remove(path.size()-1);
				return path;	
			}
			else path.remove(path.size()-1);
		}
		
		NodeGraph destinationToNode = (NodeGraph) destinationEdge.getToNode();
		NodeGraph destinationFromNode = (NodeGraph) destinationEdge.getFromNode();
		for (GeomPlanarGraphDirectedEdge e: path)
		{
			if ((e.getToNode() == destinationToNode) || (e.getToNode() == destinationFromNode) || (e.getFromNode() == destinationFromNode) 
					|| (e.getFromNode() == destinationToNode))
				{
					int lastIndex = path.indexOf(e);
					path = new ArrayList<GeomPlanarGraphDirectedEdge>(path.subList(0, lastIndex+1));
					path.add(destinationEdge);
					if (utilities.previousJunction(path, state) == destinationNode) path.remove(path.size()-1);
					return path;			
				}
		}
		return null;
	}
}

