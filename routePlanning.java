package sim.app.geo.pedestrianSimulation;

import java.util.ArrayList;
import java.util.stream.Collectors;

import sim.app.geo.urbanSim.*;
import sim.app.geo.urbanSim.utilities.Path;

import sim.util.geo.GeomPlanarGraphDirectedEdge;

public class routePlanning {
	
	static int barrierID = 999999;
	static boolean regionalRouting = false;
	static boolean barriersRouting = false;
	
    /** Road Distance Shortest Path **/
	public static Path roadDistance(NodeGraph originNode, NodeGraph destinationNode, 
			ArrayList <GeomPlanarGraphDirectedEdge> segmentsToAvoid,  boolean regionalRouting, boolean barriersRouting, int barrierID,
			String algorithm)
	{	

		if (algorithm == "astar")
		{
			AStarRoadDistance pathfinder = new AStarRoadDistance();
		    Path path = pathfinder.astarPath(originNode, destinationNode, segmentsToAvoid, regionalRouting, barriersRouting, barrierID);   
		    return path;
		}
		else
		{
			DijkstraRoadDistance pathfinder = new DijkstraRoadDistance();
		    Path path = pathfinder.dijkstraPath(originNode, destinationNode, segmentsToAvoid, regionalRouting, barriersRouting,
		    		barrierID);    
		    return path;
		}

	}
	
	/** Angular Change Shortest Path **/
	public static Path angularChange(NodeGraph originNode, NodeGraph destinationNode, 
			ArrayList <NodeGraph> centroidsToAvoid, NodeGraph previousJunction, boolean regionalRouting, boolean barriersRouting, 
			int barrierID, String algorithm)

	{	
//		System.out.println(originNode.getID() +" "+destinationNode.getID());
		NodeGraph dualOrigin = originNode.getDualNode(originNode, destinationNode, regionalRouting);
		NodeGraph dualDestination = null;

    	while (dualDestination == dualOrigin || dualDestination == null) dualDestination = destinationNode.getDualNode(
    			originNode, destinationNode, regionalRouting);

		if (algorithm == "astar")
		{
			AStarAngularChange pathfinder = new AStarAngularChange();
		    Path path = pathfinder.astarPath(dualOrigin, dualDestination, centroidsToAvoid, previousJunction, 
		    		regionalRouting, barriersRouting, barrierID); 
		    path.edges = cleanDualPath(path.edges, originNode, destinationNode);
		    return path;
		}
		else
		{
			DijkstraAngularChange pathfinder = new DijkstraAngularChange();
		    Path path = pathfinder.dijkstraPath(dualOrigin, dualDestination, centroidsToAvoid, previousJunction, 
		    		regionalRouting, barriersRouting, barrierID); 
		    path.edges = cleanDualPath(path.edges, originNode, destinationNode);
		    return path;
		}
	}
	
	
	/**Network Distance Shortest Path **/	
	public static Path topologicalShortestPath (NodeGraph originNode, NodeGraph destinationNode, ArrayList <Integer> segmentsToAvoid)
	{	
		DijkstraNetworkDistance pathfinder = new DijkstraNetworkDistance();
	    Path path = pathfinder.dijkstraPath(originNode, destinationNode, segmentsToAvoid);    
	    return path;
	}
	
	/** Global Landmarks Path **/
	public static Path globalLandmarksPath (NodeGraph originNode, NodeGraph destinationNode, ArrayList <Integer> segmentsToAvoid, String criteria)
	{	
		DijkstraGlobalLandmarks pathfinder = new DijkstraGlobalLandmarks();
	    Path path = pathfinder.dijkstraPath(originNode, destinationNode, segmentsToAvoid);    
	    return path;		
	}
		
	/** Local and Global Landmarks Path plus Road Distance **/
	public static ArrayList<GeomPlanarGraphDirectedEdge> roadDistanceLandmarksPath(NodeGraph originNode, 
			NodeGraph destinationNode, 
			ArrayList <NodeGraph> sequence)
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
				tP = pathFinder.dijkstraPath(tmpNode, intermediateNode, null);
				newPath = tP.edges;				
			}
    		else
    		{
				DijkstraRoadDistanceLandmarks tmpPathFinder = new DijkstraRoadDistanceLandmarks(); 
				tP = tmpPathFinder.dijkstraPath(tmpNode, intermediateNode, newPath);
    			while (tP.edges == null)
    			{
    				GeomPlanarGraphDirectedEdge lastSegment = newPath.get(newPath.size()-1);
    				if (lastSegment.getFromNode() == tmpNode) tmpNode = (NodeGraph) lastSegment.getToNode();
    				else tmpNode = (NodeGraph) lastSegment.getFromNode();
    				newPath.remove(lastSegment);
    				DijkstraRoadDistanceLandmarks loopPathFinder = new DijkstraRoadDistanceLandmarks(); 
    				tP = loopPathFinder.dijkstraPath(tmpNode, intermediateNode, newPath);
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
		tmpPath = tmpPathFinder.dijkstraPath(tmpNode, destinationNode, newPath).edges; 

		while (tmpPath == null)
		{
			GeomPlanarGraphDirectedEdge lastSegment = newPath.get(newPath.size()-1);
			if (lastSegment.getFromNode() == tmpNode) tmpNode = (NodeGraph) lastSegment.getToNode();
			else tmpNode = (NodeGraph) lastSegment.getFromNode();
			newPath.remove(lastSegment);
			if (tmpNode == destinationNode) return newPath;
			DijkstraRoadDistanceLandmarks loopPathFinder = new DijkstraRoadDistanceLandmarks(); 
			tmpPath = loopPathFinder.dijkstraPath(tmpNode, destinationNode, newPath).edges;
		}
    	newPath.addAll(tmpPath);
		return newPath; 
    }
	
	/** Local and Global Landmarks Path plus Angular Change **/
	public static ArrayList<GeomPlanarGraphDirectedEdge> AngularChangeLandmarksPath(NodeGraph originNode, NodeGraph destinationNode, 
			ArrayList <NodeGraph> sequence)
	{
		ArrayList<GeomPlanarGraphDirectedEdge> newPath = null;
		NodeGraph dualOrigin = originNode.getDualNode(originNode, destinationNode, regionalRouting);
		NodeGraph dualDestination = destinationNode.getDualNode(originNode, destinationNode, regionalRouting);
		ArrayList<NodeGraph> dualSequence =  new ArrayList<NodeGraph>();
        for (NodeGraph intermediateNode : sequence) dualSequence.add(intermediateNode.getDualNode(intermediateNode, destinationNode, regionalRouting));
        
		DijkstraAngularChangeLandmarks pathFinder = new DijkstraAngularChangeLandmarks();  
		
		NodeGraph tmpOrigin = dualOrigin;
		ArrayList<GeomPlanarGraphDirectedEdge> localPath = null;
		ArrayList<NodeGraph> centroidsToAvoid = new ArrayList<NodeGraph>();
		
		//to check if the sub-goals have been traversed across the previous local path
        ArrayList<NodeGraph> alreadyTraversed = new ArrayList<NodeGraph>(); 
        NodeGraph pJ = null;


		for (NodeGraph subGoal : dualSequence)
    	{

    		if ((alreadyTraversed != null) && (alreadyTraversed.contains(subGoal))) continue;
    		Path lP = null;
    		
    		if (dualSequence.indexOf(subGoal) == 0) // from origin to first subGoal
    		{
    			lP = pathFinder.dijkstraPath(tmpOrigin, subGoal, destinationNode, null, pJ);
    			newPath = lP.edges;
	        	pJ = utilities.previousJunction(newPath);
	        	for (GeomPlanarGraphDirectedEdge e: newPath) centroidsToAvoid.add(((EdgeGraph) e.getEdge()).getDual());
    		}
    		
    		else
    		{
    			DijkstraAngularChangeLandmarks localPathFinder = new DijkstraAngularChangeLandmarks(); 
    			lP = localPathFinder.dijkstraPath(tmpOrigin, subGoal, destinationNode, centroidsToAvoid, pJ);
				
    			while (lP.edges == null) // backtracking
    			{
    				newPath.remove(newPath.size()-1); // remove last one which did not work!
    				pJ = utilities.previousJunction(newPath); // take new previous junction
    				GeomPlanarGraphDirectedEdge lastSegment = newPath.get(newPath.size()-1);
    				tmpOrigin = ((EdgeGraph) lastSegment.getEdge()).getDual();
    				centroidsToAvoid.remove(tmpOrigin);
    				DijkstraAngularChangeLandmarks tryPathFinder = new DijkstraAngularChangeLandmarks(); 
    				lP = tryPathFinder.dijkstraPath(tmpOrigin, subGoal, destinationNode, centroidsToAvoid, pJ);
    			}
    			
				localPath = lP.edges; 
	        	for (GeomPlanarGraphDirectedEdge e: localPath) centroidsToAvoid.add(((EdgeGraph) e.getEdge()).getDual());
            	pJ = utilities.previousJunction(localPath);
            	localPath.remove(localPath.get(0)); // as it was already at the end of the previous localPath
    			newPath.addAll(localPath); 
    		}
    		
        	for (int i = sequence.indexOf(subGoal)+1; i < sequence.size(); i++)
    		{
    			 NodeGraph next = dualSequence.get(i);
    			 if (lP.mapWrappers.containsKey(next)) alreadyTraversed.add(next);
    		}
        	tmpOrigin = subGoal;
        	centroidsToAvoid.remove(tmpOrigin);
    	}
    	
		GeomPlanarGraphDirectedEdge destinationEdge = (GeomPlanarGraphDirectedEdge) dualDestination.primalEdge.getDirEdge(0);
		
		// control if the destination has been already traversed so far and if yes, clean the path
		ArrayList<GeomPlanarGraphDirectedEdge> controlPath = controlDualLandmarksPath(destinationNode, destinationEdge, newPath);
		if (controlPath != null) return controlPath;
		
		// otherwise add the path towards the final destination
		DijkstraAngularChangeLandmarks tmpPathFinder = new DijkstraAngularChangeLandmarks(); 
		localPath = tmpPathFinder.dijkstraPath(tmpOrigin, dualDestination, destinationNode, centroidsToAvoid, pJ).edges; 
		
		while (localPath == null) // backtracking
		{
			newPath.remove(newPath.size()-1); // remove last one
			pJ = utilities.previousJunction(newPath); // remove last one which did not work!
			GeomPlanarGraphDirectedEdge lastSegment = newPath.get(newPath.size()-1); // take new previous junction
			tmpOrigin = ((EdgeGraph) lastSegment.getEdge()).getDual();
			centroidsToAvoid.remove(tmpOrigin);
			DijkstraAngularChangeLandmarks tryPathFinder = new DijkstraAngularChangeLandmarks(); 
			localPath = tryPathFinder.dijkstraPath(tmpOrigin, dualDestination, destinationNode, centroidsToAvoid, pJ).edges;
		}

		localPath.remove(localPath.get(0)); // as it was already at the end of the previous localPath
    	newPath.addAll(localPath); 
    	if (utilities.previousJunction(newPath) == destinationNode) newPath.remove(newPath.size()-1);
    	return newPath;
	}
	
	/** Local Landmarks Path plus Road Distance **/
	public static ArrayList<GeomPlanarGraphDirectedEdge> roadDistanceLocalLandmarks(NodeGraph originNode, 
			NodeGraph destinationNode, ArrayList <NodeGraph> sequence)
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
				tP = pathFinder.dijkstraPath(tmpNode, intermediateNode, null, regionalRouting, barriersRouting, barrierID);
				newPath = tP.edges;
			}
    		else
    		{
				DijkstraRoadDistance tmpPathFinder = new DijkstraRoadDistance(); 
				tP = tmpPathFinder.dijkstraPath(tmpNode, intermediateNode, newPath, regionalRouting, barriersRouting, barrierID);
    			while (tP.edges == null)
    			{
    				GeomPlanarGraphDirectedEdge lastSegment = newPath.get(newPath.size()-1);
    				if (lastSegment.getFromNode() == tmpNode) tmpNode = (NodeGraph) lastSegment.getToNode();
    				else tmpNode = (NodeGraph) lastSegment.getFromNode();
    				newPath.remove(lastSegment);
					DijkstraRoadDistance looPathFinder = new DijkstraRoadDistance(); 
					tP = looPathFinder.dijkstraPath(tmpNode, intermediateNode, newPath, regionalRouting, barriersRouting, barrierID);
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
		tmpPath = tmpPathFinder.dijkstraPath(tmpNode, destinationNode, newPath, regionalRouting, barriersRouting, barrierID).edges; 
		while (tmpPath == null)
		{
			
			GeomPlanarGraphDirectedEdge lastSegment = newPath.get(newPath.size()-1);
			if (lastSegment.getFromNode() == tmpNode) tmpNode = (NodeGraph) lastSegment.getToNode();
			else tmpNode = (NodeGraph) lastSegment.getFromNode();
			newPath.remove(lastSegment);
			if (tmpNode == destinationNode) return newPath;
			DijkstraRoadDistance loopPathFinder = new DijkstraRoadDistance(); 
			tmpPath = loopPathFinder.dijkstraPath(tmpNode, destinationNode, newPath, regionalRouting, barriersRouting, barrierID).edges;
		}
    	newPath.addAll(tmpPath);
		return newPath; 
	}
		
	/** Local Landmarks Path plus angular change **/
		
	public static ArrayList<GeomPlanarGraphDirectedEdge> angularChangeLocalLandmarks(NodeGraph originNode, 
			NodeGraph destinationNode, ArrayList <NodeGraph> sequence)
	{
		ArrayList<GeomPlanarGraphDirectedEdge> newPath = null;
		NodeGraph dualOrigin = originNode.getDualNode(originNode, destinationNode, regionalRouting);
		NodeGraph dualDestination = destinationNode.getDualNode(originNode, destinationNode, regionalRouting);
				
		DijkstraAngularChange pathFinder = new DijkstraAngularChange();  
		NodeGraph tmpNode = dualOrigin;
		ArrayList<GeomPlanarGraphDirectedEdge> tmpPath = null;
		ArrayList<NodeGraph> centroidsToAvoid = new ArrayList<NodeGraph>();
		ArrayList<NodeGraph> dualSequence =  new ArrayList<NodeGraph>();
        ArrayList<NodeGraph> traversedCentroids = new ArrayList<NodeGraph>();
		NodeGraph pJ = null;
		boolean regionalRouting = false;
		        
    	for (NodeGraph intermediateNode : sequence) dualSequence.add(intermediateNode.getDualNode(intermediateNode, destinationNode, regionalRouting));
    	
    	for (NodeGraph dualIntermediateNode : dualSequence)
    	{
    		if ((traversedCentroids != null) && (traversedCentroids.contains(dualIntermediateNode))) continue;
    		Path tP = null;
    		
    		if (dualSequence.indexOf(dualIntermediateNode) == 0) 
    		{
    			tP = pathFinder.dijkstraPath(tmpNode, dualIntermediateNode, null, null, regionalRouting, barriersRouting, barrierID);
    			newPath = tP.edges; 
	        	pJ = utilities.previousJunction(newPath);
	        	centroidsToAvoid = (ArrayList<NodeGraph>) tP.mapWrappers.keySet().stream().collect(Collectors.toList());
    		}
    		
			else
    		{
    			DijkstraAngularChange tmpPathFinder = new DijkstraAngularChange(); 
				tP = tmpPathFinder.dijkstraPath(tmpNode, dualIntermediateNode, centroidsToAvoid, pJ, regionalRouting, barriersRouting, barrierID);
				while (tP.edges == null)
    			{
    				newPath.remove(newPath.size()-1); // remove last one
    				pJ = utilities.previousJunction(newPath);
    				GeomPlanarGraphDirectedEdge lastSegment = newPath.get(newPath.size()-1);
    				int lastSegmentID = ((EdgeGraph) lastSegment.getEdge()).getID();
    				tmpNode = PedestrianSimulation.centroidsMap.get(lastSegmentID);
    				centroidsToAvoid.remove(tmpNode);
    				DijkstraAngularChange loopPathFinder = new DijkstraAngularChange(); 
    				tP = loopPathFinder.dijkstraPath(tmpNode, dualIntermediateNode, centroidsToAvoid, pJ, regionalRouting, barriersRouting, barrierID);
    			}
    			
    			for (NodeGraph key : tP.mapWrappers.keySet()) {centroidsToAvoid.add(key);}
	    		for (int i = sequence.indexOf(dualIntermediateNode)+1; i < sequence.size(); i++)
	    		{
	    			 NodeGraph next = dualSequence.get(i);
	    			 if (tP.mapWrappers.containsKey(next)) traversedCentroids.add(next);
	    		}	 
	    		tmpPath = tP.edges; 
            	pJ = utilities.previousJunction(tmpPath);
    			tmpPath.remove(tmpPath.get(0));
    			newPath.addAll(tmpPath); 
    		}
	    	tmpNode = dualIntermediateNode;
	    	centroidsToAvoid.remove(tmpNode);
    		
        	for (int i = sequence.indexOf(dualIntermediateNode)+1; i < sequence.size(); i++)
    		{
        		 NodeGraph next = dualSequence.get(i);
    			 if (tP.mapWrappers.containsKey(next)) traversedCentroids.add(next);
    		}
		}
    	
		GeomPlanarGraphDirectedEdge destinationEdge = (GeomPlanarGraphDirectedEdge) dualDestination.primalEdge.getDirEdge(0);
		ArrayList<GeomPlanarGraphDirectedEdge> controlPath = controlDualLandmarksPath(destinationNode, destinationEdge, newPath);
		if (controlPath != null) return controlPath;
		
		// else 
    	DijkstraAngularChange tmpPathFinder = new DijkstraAngularChange(); 
		tmpPath = tmpPathFinder.dijkstraPath(tmpNode, dualDestination, centroidsToAvoid, pJ, regionalRouting, barriersRouting, barrierID).edges; 
		while (tmpPath == null)
		{
			newPath.remove(newPath.size()-1); // remove last one
			pJ = utilities.previousJunction(newPath);
			GeomPlanarGraphDirectedEdge lastSegment = newPath.get(newPath.size()-1);
			int lastSegmentID = ((EdgeGraph) lastSegment.getEdge()).getID();
			tmpNode = PedestrianSimulation.centroidsMap.get(lastSegmentID);
			centroidsToAvoid.remove(tmpNode);
			DijkstraAngularChange loopPathFinder = new DijkstraAngularChange(); 
			tmpPath = loopPathFinder.dijkstraPath(tmpNode, dualDestination, centroidsToAvoid, pJ, regionalRouting, barriersRouting, barrierID).edges;
		}

		tmpPath.remove(tmpPath.get(0));
    	newPath.addAll(tmpPath); 
    	if (utilities.previousJunction(newPath) == destinationNode) newPath.remove(newPath.size()-1);
    	return newPath;
    }
	

	static ArrayList<GeomPlanarGraphDirectedEdge> regionsBarriersPath (NodeGraph originNode, NodeGraph destinationNode,	String criteria)
	{
		String localHeuristic = "angularChange";
		if (criteria.contains("roadDistance")) localHeuristic = "roadDistance";
		boolean usingBarriers = false;
		if (criteria.contains("Barriers")) usingBarriers = true;		
		RegionalRouting regionsPath = new RegionalRouting();
		return regionsPath.pathFinderRegions(originNode, destinationNode, localHeuristic, usingBarriers);
	}
	
	static ArrayList<GeomPlanarGraphDirectedEdge> barriersPath (NodeGraph originNode, NodeGraph destinationNode, String criteria)
	{
		String localHeuristic = "angularChange";
		if (criteria == "roadDistanceBarriers") localHeuristic = "roadDistance";
		BarriersRouting barriersPath = new BarriersRouting();
		return barriersPath.pathFinderBarriers(originNode, destinationNode, localHeuristic);
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
	
	static ArrayList<GeomPlanarGraphDirectedEdge> controlDualLandmarksPath(NodeGraph destinationNode, 
			GeomPlanarGraphDirectedEdge destinationEdge, ArrayList <GeomPlanarGraphDirectedEdge> path)
	{
		while (path.contains(destinationEdge))
		{
			if (path.get(path.size()-1) == destinationEdge) 
			{
				if (utilities.previousJunction(path) == destinationNode) path.remove(path.size()-1);
				return path;	
			}
			else path.remove(path.size()-1);
		}
		
		NodeGraph destinationToNode = (NodeGraph) destinationEdge.getToNode();
		NodeGraph destinationFromNode = (NodeGraph) destinationEdge.getFromNode();
		
		for (GeomPlanarGraphDirectedEdge e: path)
		{
			if ((e.getToNode() == destinationToNode) || (e.getToNode() == destinationFromNode) 
				|| (e.getFromNode() == destinationFromNode) || (e.getFromNode() == destinationToNode))
			{
				path = new ArrayList<GeomPlanarGraphDirectedEdge>(path.subList(0, path.indexOf(e)+1));
				path.add(destinationEdge);
				if (utilities.previousJunction(path) == destinationNode) path.remove(path.size()-1);
				return path;			
			}
		}
		return null;
	}
	
	
	static ArrayList<GeomPlanarGraphDirectedEdge> cleanDualPath(ArrayList<GeomPlanarGraphDirectedEdge> edges, NodeGraph originNode,
			NodeGraph destinationNode)
	{

		if (utilities.previousJunction(edges) == destinationNode) edges.remove(edges.size()-1);
	    if (utilities.commonPrimalJunction(((EdgeGraph) edges.get(0).getEdge()).dualNode,
	    		((EdgeGraph) edges.get(1).getEdge()).dualNode) == originNode) edges.remove(0);
	    return edges;
	}
	
}

