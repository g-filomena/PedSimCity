package sim.app.geo.pedestrianSimulation;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import com.vividsolutions.jts.planargraph.Node;

import sim.app.geo.pedestrianSimulation.utilities.Path;
import sim.field.geo.GeomVectorField;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.GeomPlanarGraphEdge;
import sim.util.geo.MasonGeometry;

public class routePlanning {
	
    /** Road Distance Shortest Path **/
	public static Path routeDistanceShortestPath (Node originNode, Node destinationNode, ArrayList <GeomPlanarGraphDirectedEdge> 
			segmentsToAvoid, PedestrianSimulation state,
			String algorithm)
	{	
		if (algorithm == "astar")
		{
			AStarRoadDistance pathfinder = new AStarRoadDistance();
		    Path path = pathfinder.astarPath(originNode, destinationNode, segmentsToAvoid, state);    
		    return path;
		}
		else
		{
			DijkstraRoadDistance pathfinder = new DijkstraRoadDistance();
		    Path path = pathfinder.dijkstraPath(originNode, destinationNode, segmentsToAvoid, state);    
		    return path;
		}

	}
	
	/** Angular Change Shortest Path **/
	public static Path angularChangeShortestPath (Node originNode, Node destinationNode, ArrayList <Node> centroidsToAvoid, PedestrianSimulation state,
			String algorithm)
	{	
    	Node dualOrigin = utilities.getDualNode(originNode, PedestrianSimulation.dualNetwork);
    	Node dualDestination = null;
    	while (dualDestination == dualOrigin || dualDestination == null) dualDestination = utilities.getDualNode(
    			destinationNode, PedestrianSimulation.dualNetwork);
		
		if (algorithm == "astar")
		{
			AStarAngularChange pathfinder = new AStarAngularChange();
		    Path path = pathfinder.astarPath(dualOrigin, dualDestination, centroidsToAvoid, state);    
		    if (previousJunction(path.edges, state) == destinationNode) path.edges.remove(path.edges.size()-1);
		    return path;
		}
		else
		{
			DijkstraAngularChange pathfinder = new DijkstraAngularChange();
		    Path path = pathfinder.dijkstraPath(dualOrigin, dualDestination, centroidsToAvoid, null, state); 
		    if (previousJunction(path.edges, state) == destinationNode) path.edges.remove(path.edges.size()-1);
		    return path;
		}
	}
	
	/**Network Distance Shortest Path **/	
	public static Path topologicalShortestPath (Node originNode, Node destinationNode, ArrayList <Integer> segmentsToAvoid, PedestrianSimulation state)
	{	
		DijkstraNetworkDistance pathfinder = new DijkstraNetworkDistance();
	    Path path = pathfinder.dijkstraPath(originNode, destinationNode, segmentsToAvoid, state);    
	    return path;
	}
	
	/** Global Landmarks Path plus other weights **/
	public static Path globalLandmarksPath (Node originNode, Node destinationNode, ArrayList <Integer> segmentsToAvoid, String criteria,
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
	public static ArrayList<GeomPlanarGraphDirectedEdge> RoadDistanceLandmarksPath(Node originNode, Node destinationNode, 
			ArrayList <Node> sequence, PedestrianSimulation state)
	{   
		ArrayList<GeomPlanarGraphDirectedEdge> newPath = null;
		DijkstraRoadDistanceLandmarks pathFinder = new DijkstraRoadDistanceLandmarks();  
		Node tmpNode = originNode;
		ArrayList<GeomPlanarGraphDirectedEdge> tmpPath = null;
        ArrayList<Node> traversedNodes = new ArrayList<Node>();

		for (Node intermediateNode : sequence)
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
    				if (lastSegment.getFromNode() == tmpNode) tmpNode = lastSegment.getToNode();
    				else tmpNode = lastSegment.getFromNode();
    				newPath.remove(lastSegment);
    				DijkstraRoadDistanceLandmarks loopPathFinder = new DijkstraRoadDistanceLandmarks(); 
    				tP = loopPathFinder.dijkstraPath(tmpNode, intermediateNode, newPath, state);
    			}
    			tmpPath = tP.edges; 
    			newPath.addAll(tmpPath); 	 
    		}
			
    		for (int i = sequence.indexOf(intermediateNode)+1; i < sequence.size(); i++)
    		{
    			Node next = sequence.get(i);
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
			if (lastSegment.getFromNode() == tmpNode) tmpNode = lastSegment.getToNode();
			else tmpNode = lastSegment.getFromNode();
			newPath.remove(lastSegment);
			if (tmpNode == destinationNode) return newPath;
			DijkstraRoadDistanceLandmarks loopPathFinder = new DijkstraRoadDistanceLandmarks(); 
			tmpPath = loopPathFinder.dijkstraPath(tmpNode, destinationNode, newPath, state).edges;
		}
    	newPath.addAll(tmpPath);
		return newPath; 
    }
	
	/** Local and Global Landmarks Path plus Angular Change **/
	public static ArrayList<GeomPlanarGraphDirectedEdge> AngularChangeLandmarksPath(Node originNode, Node destinationNode, 
			ArrayList <Node> sequence, PedestrianSimulation state)
	{
		ArrayList<GeomPlanarGraphDirectedEdge> newPath = null;
		Node dualOrigin = utilities.getDualNode(originNode, PedestrianSimulation.dualNetwork);
		Node dualDestination = utilities.getDualNode(destinationNode, PedestrianSimulation.dualNetwork);
		DijkstraAngularChangeLandmarks pathFinder = new DijkstraAngularChangeLandmarks();  
		Node tmpNode = dualOrigin;
		ArrayList<GeomPlanarGraphDirectedEdge> tmpPath = null;
		ArrayList<Node> centroidsToAvoid = new ArrayList<Node>();
		ArrayList<Node> dualSequence =  new ArrayList<Node>();
        ArrayList<Node> traversedCentroids = new ArrayList<Node>();
		Node pJ = null;

        for (Node intermediateNode : sequence) dualSequence.add(utilities.getDualNode(intermediateNode, PedestrianSimulation.dualNetwork));
		for (Node dualIntermediateNode : dualSequence)
    	{

    		if ((traversedCentroids != null) && (traversedCentroids.contains(dualIntermediateNode))) continue;
    		Path tP = null;
    		if (dualSequence.indexOf(dualIntermediateNode) == 0) 
    		{
    			tP = pathFinder.dijkstraPath(tmpNode, dualIntermediateNode, destinationNode, null, pJ, state);
    			newPath = tP.edges;
	        	pJ = previousJunction(newPath, state);
	        	for (GeomPlanarGraphDirectedEdge e: newPath) centroidsToAvoid.add(
	        			state.centroidsMap.get(((GeomPlanarGraphEdge) e.getEdge()).getIntegerAttribute("edgeID")).c);
    		}
    		else
    		{
    			DijkstraAngularChangeLandmarks tmpPathFinder = new DijkstraAngularChangeLandmarks(); 
				tP = tmpPathFinder.dijkstraPath(tmpNode, dualIntermediateNode, destinationNode, centroidsToAvoid, pJ, state);
				while (tP.edges == null)
    			{
    				newPath.remove(newPath.size()-1); // remove last one
    				pJ = previousJunction(newPath, state);
    				GeomPlanarGraphDirectedEdge lastSegment = newPath.get(newPath.size()-1);
    				int lastSegmentID = ((GeomPlanarGraphEdge) lastSegment.getEdge()).getIntegerAttribute("edgeID");
    				tmpNode = state.centroidsMap.get(lastSegmentID).c;
    				centroidsToAvoid.remove(tmpNode);
    				DijkstraAngularChangeLandmarks loopPathFinder = new DijkstraAngularChangeLandmarks(); 
    				tP = loopPathFinder.dijkstraPath(tmpNode, dualIntermediateNode, destinationNode, centroidsToAvoid, pJ, state);
    			}
	    		tmpPath = tP.edges; 
	        	for (GeomPlanarGraphDirectedEdge e: tmpPath) centroidsToAvoid.add(
	        			state.centroidsMap.get(((GeomPlanarGraphEdge) e.getEdge()).getIntegerAttribute("edgeID")).c);
            	pJ = previousJunction(tmpPath, state);
    			tmpPath.remove(tmpPath.get(0));
    			newPath.addAll(tmpPath); 
    		}
    		
        	for (int i = sequence.indexOf(dualIntermediateNode)+1; i < sequence.size(); i++)
    		{
    			 Node next = dualSequence.get(i);
    			 if (tP.dualMapWrappers.containsKey(next)) traversedCentroids.add(next);
    		}
        	tmpNode = dualIntermediateNode;
        	centroidsToAvoid.remove(tmpNode);
    	}
    	
		GeomPlanarGraphDirectedEdge destinationEdge = (GeomPlanarGraphDirectedEdge) state.edgesMap.get((int) dualDestination.getData()).planarEdge.getDirEdge(0);
		ArrayList<GeomPlanarGraphDirectedEdge> controlPath = controlDualPath(destinationNode, destinationEdge, newPath, state);
		if (controlPath != null) return controlPath;
		
		DijkstraAngularChangeLandmarks tmpPathFinder = new DijkstraAngularChangeLandmarks(); 
		tmpPath = tmpPathFinder.dijkstraPath(tmpNode, dualDestination, destinationNode, centroidsToAvoid, pJ, state).edges; 
		while (tmpPath == null)
		{
			newPath.remove(newPath.size()-1); // remove last one
			pJ = previousJunction(newPath, state);
			GeomPlanarGraphDirectedEdge lastSegment = newPath.get(newPath.size()-1);
			int lastSegmentID = ((GeomPlanarGraphEdge) lastSegment.getEdge()).getIntegerAttribute("edgeID");
			tmpNode = state.centroidsMap.get(lastSegmentID).c;
			centroidsToAvoid.remove(tmpNode);
			DijkstraAngularChangeLandmarks loopPathFinder = new DijkstraAngularChangeLandmarks(); 
			tmpPath = loopPathFinder.dijkstraPath(tmpNode, dualDestination, destinationNode, centroidsToAvoid, pJ, state).edges;
		}

		tmpPath.remove(tmpPath.get(0));
    	newPath.addAll(tmpPath); 
    	if (previousJunction(newPath, state) == destinationNode) newPath.remove(newPath.size()-1);
    	return newPath;
	}
	
	/** Local Landmarks Path plus Road Distance **/
	public static ArrayList<GeomPlanarGraphDirectedEdge> RoadDistanceLocalLandmarksPath(Node originNode, Node destinationNode, 
			ArrayList <Node> sequence, PedestrianSimulation state)
	{
		ArrayList<GeomPlanarGraphDirectedEdge> newPath = null;
		DijkstraRoadDistance pathFinder = new DijkstraRoadDistance();  
		Node tmpNode = originNode;
		ArrayList<GeomPlanarGraphDirectedEdge> tmpPath = null;
        ArrayList<Node> traversedNodes = new ArrayList<Node>();
	        
		for (Node intermediateNode : sequence)
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
				DijkstraRoadDistance tmpPathFinder = new DijkstraRoadDistance(); 
				tP = tmpPathFinder.dijkstraPath(tmpNode, intermediateNode, newPath, state);
    			while (tP.edges == null)
    			{
    				GeomPlanarGraphDirectedEdge lastSegment = newPath.get(newPath.size()-1);
    				if (lastSegment.getFromNode() == tmpNode) tmpNode = lastSegment.getToNode();
    				else tmpNode = lastSegment.getFromNode();
    				newPath.remove(lastSegment);
					DijkstraRoadDistance looPathFinder = new DijkstraRoadDistance(); 
					tP = looPathFinder.dijkstraPath(tmpNode, intermediateNode, newPath, state);
    			}
    			tmpPath = tP.edges; 
    			newPath.addAll(tmpPath); 
    		}
    		for (int i = sequence.indexOf(intermediateNode)+1; i < sequence.size(); i++)
    		{
    			Node next = sequence.get(i);
    			if (tP.mapWrappers.containsKey(next)) traversedNodes.add(next);
    		}	
    		tmpNode = intermediateNode;

    	}
		ArrayList<GeomPlanarGraphDirectedEdge> controlPath = controlPath(destinationNode, newPath);
		if (controlPath != null) return controlPath;
		
		DijkstraRoadDistance tmpPathFinder = new DijkstraRoadDistance(); 
		tmpPath = tmpPathFinder.dijkstraPath(tmpNode, destinationNode, newPath, state).edges; 
		while (tmpPath == null)
		{
			
			GeomPlanarGraphDirectedEdge lastSegment = newPath.get(newPath.size()-1);
			if (lastSegment.getFromNode() == tmpNode) tmpNode = lastSegment.getToNode();
			else tmpNode = lastSegment.getFromNode();
			newPath.remove(lastSegment);
			if (tmpNode == destinationNode) return newPath;
			DijkstraRoadDistance loopPathFinder = new DijkstraRoadDistance(); 
			tmpPath = loopPathFinder.dijkstraPath(tmpNode, destinationNode, newPath, state).edges;
		}
    	newPath.addAll(tmpPath);
		return newPath; 
	}
		
	/** Local Landmarks Path plus angular change **/
		
	public static ArrayList<GeomPlanarGraphDirectedEdge> AngularChangeLocalLandmarksPath(Node originNode, Node destinationNode, 
			ArrayList <Node> sequence, PedestrianSimulation state)
	{
		ArrayList<GeomPlanarGraphDirectedEdge> newPath = null;
		Node dualOrigin = utilities.getDualNode(originNode, PedestrianSimulation.dualNetwork);
		Node dualDestination = utilities.getDualNode(destinationNode, PedestrianSimulation.dualNetwork);
				
		DijkstraAngularChange pathFinder = new DijkstraAngularChange();  
		Node tmpNode = dualOrigin;
		ArrayList<GeomPlanarGraphDirectedEdge> tmpPath = null;
		ArrayList<Node> centroidsToAvoid = new ArrayList<Node>();
		ArrayList<Node> dualSequence =  new ArrayList<Node>();
        ArrayList<Node> traversedCentroids = new ArrayList<Node>();
		Node pJ = null;
		        
    	for (Node intermediateNode : sequence) dualSequence.add(utilities.getDualNode(intermediateNode, PedestrianSimulation.dualNetwork));
    	
    	for (Node dualIntermediateNode : dualSequence)
    	{
    		if ((traversedCentroids != null) && (traversedCentroids.contains(dualIntermediateNode))) continue;
    		Path tP = null;
    		
    		if (dualSequence.indexOf(dualIntermediateNode) == 0) 
    		{
    			tP = pathFinder.dijkstraPath(tmpNode, dualIntermediateNode, null, null, state);
    			newPath = tP.edges; 
	        	pJ = previousJunction(newPath, state);
	        	centroidsToAvoid = (ArrayList<Node>) tP.dualMapWrappers.keySet().stream().collect(Collectors.toList());
    		}
    		
			else
    		{
    			DijkstraAngularChange tmpPathFinder = new DijkstraAngularChange(); 
				tP = tmpPathFinder.dijkstraPath(tmpNode, dualIntermediateNode, centroidsToAvoid, pJ, state);
				while (tP.edges == null)
    			{
    				newPath.remove(newPath.size()-1); // remove last one
    				pJ = previousJunction(newPath, state);
    				GeomPlanarGraphDirectedEdge lastSegment = newPath.get(newPath.size()-1);
    				int lastSegmentID = ((GeomPlanarGraphEdge) lastSegment.getEdge()).getIntegerAttribute("edgeID");
    				tmpNode = state.centroidsMap.get(lastSegmentID).c;
    				centroidsToAvoid.remove(tmpNode);
    				DijkstraAngularChange loopPathFinder = new DijkstraAngularChange(); 
    				tP = loopPathFinder.dijkstraPath(tmpNode, dualIntermediateNode, centroidsToAvoid, pJ, state);
    			}
    			
    			for (Node key : tP.dualMapWrappers.keySet()) {centroidsToAvoid.add(key);}
	    		for (int i = sequence.indexOf(dualIntermediateNode)+1; i < sequence.size(); i++)
	    		{
	    			 Node next = dualSequence.get(i);
	    			 if (tP.dualMapWrappers.containsKey(next)) traversedCentroids.add(next);
	    		}	 
	    		tmpPath = tP.edges; 
            	pJ = previousJunction(tmpPath, state);
    			tmpPath.remove(tmpPath.get(0));
    			newPath.addAll(tmpPath); 
    		}
	    	tmpNode = dualIntermediateNode;
	    	centroidsToAvoid.remove(tmpNode);
    		
        	for (int i = sequence.indexOf(dualIntermediateNode)+1; i < sequence.size(); i++)
    		{
    			 Node next = dualSequence.get(i);
    			 if (tP.dualMapWrappers.containsKey(next)) traversedCentroids.add(next);
    		}
		}
		GeomPlanarGraphDirectedEdge destinationEdge = (GeomPlanarGraphDirectedEdge) state.edgesMap.get((int) dualDestination.getData()).planarEdge.getDirEdge(0);
		ArrayList<GeomPlanarGraphDirectedEdge> controlPath = controlDualPath(destinationNode, destinationEdge, newPath, state);
		if (controlPath != null) return controlPath;
		
    	DijkstraAngularChange tmpPathFinder = new DijkstraAngularChange(); 
		tmpPath = tmpPathFinder.dijkstraPath(tmpNode, dualDestination, centroidsToAvoid, pJ, state).edges; 
		while (tmpPath == null)
		{
			newPath.remove(newPath.size()-1); // remove last one
			pJ = previousJunction(newPath, state);
			GeomPlanarGraphDirectedEdge lastSegment = newPath.get(newPath.size()-1);
			int lastSegmentID = ((GeomPlanarGraphEdge) lastSegment.getEdge()).getIntegerAttribute("edgeID");
			tmpNode = state.centroidsMap.get(lastSegmentID).c;
			centroidsToAvoid.remove(tmpNode);
			DijkstraAngularChange loopPathFinder = new DijkstraAngularChange(); 
			tmpPath = loopPathFinder.dijkstraPath(tmpNode, dualDestination, centroidsToAvoid, pJ, state).edges;
		}

		tmpPath.remove(tmpPath.get(0));
    	newPath.addAll(tmpPath); 
    	if (previousJunction(newPath, state) == destinationNode) newPath.remove(newPath.size()-1);
    	return newPath;
    }
	
	
	public static ArrayList<Node> findSequenceIntermediateNodes(Node originNode, Node destinationNode, PedestrianSimulation state)
	{
    	GeomVectorField knownJunctions = landmarkFunctions.relevantNodes(originNode, destinationNode, state);
        double wayfindingComplexity = landmarkFunctions.easinessNavigation(originNode, destinationNode, state);
        double searchRange = utilities.nodesDistance(originNode, destinationNode) * (wayfindingComplexity);
        Node currentNode = originNode;
//        ArrayList<Integer> segmentsToAvoid = new ArrayList<Integer>();

	    List<Integer> badNodes = new ArrayList<Integer>();
	    ArrayList<Node> sequence = new ArrayList<Node>();
    	while (searchRange >  state.t)
        {
        	Node bestNode = null;
//        	Node bestDualNode = null;
//        	ArrayList<GeomPlanarGraphDirectedEdge> bestPath = null;
//        	ArrayList<GeomPlanarGraphDirectedEdge> tmpPath = null;
        	double attractivness = 0.0;
        		        	
        	for (Object o : knownJunctions.getGeometries())
	        {
		    	MasonGeometry geoNode = (MasonGeometry) o;
		    	int nodeID = state.nodesGeometryMap.get(geoNode);
		    	Node tmpNode = state.nodesMap.get(nodeID).node;
		    	
		    	if (sequence.contains(tmpNode)) continue;
		    	if (tmpNode == originNode) continue;
		    	if (Node.getEdgesBetween(tmpNode, currentNode).size() > 0) continue;
		    	if (Node.getEdgesBetween(tmpNode, destinationNode).size() > 0) continue;
		    	if (Node.getEdgesBetween(tmpNode, originNode).size() > 0) continue;
		    	
		    	if (utilities.nodesDistance(currentNode, tmpNode) > searchRange)
		    	{
		    		badNodes.add(nodeID);
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
        		        	
        	if (bestNode == null) 
        	{

//        		// reconstruct path till the destination
//        		if (criteria == "angularLand") 
//        		{
//        			Node dualOrigin = utilities.getDualNode(currentNode, pedestrianSimulation.dualNetwork);
//        			Node dualTmpDestination = utilities.getDualNode(destinationNode, pedestrianSimulation.dualNetwork);
//        			
//        			AStarAngular pathfinderAngular = new AStarAngular(); 
//        			Path path = pathfinderAngular.astarPath(dualOrigin, dualTmpDestination, state, segmentsToAvoid, gl);
//        			tmpPath = path.edges;
//        		}
//		        else
//		        {
//		        	AStarEuclidean pathfinderEuclidean = new AStarEuclidean();   
//		        	Path path = pathfinderEuclidean.astarPath(currentNode, destinationNode, state, segmentsToAvoid, gl);
//		        	tmpPath = path.edges;
//		        }
        		//len sequence == 0
//	    		if (currentNode == originNode) newPath = tmpPath;
	    		//add to the rest
//	    		else newPath.addAll(tmpPath);
        		break;
        	}
        	
//    		if (currentNode == originNode) newPath = bestPath;
//    		//add to the rest
//    		else newPath.addAll(bestPath);
//    		Iterator<GeomPlanarGraphDirectedEdge> it = bestPath.iterator();
//	        ArrayList<Integer> segmentsToAvoid = new ArrayList<Integer>();
//    		while (it.hasNext()) segmentsToAvoid.add((int) ((GeomPlanarGraphEdge) 
//					it.next().getEdge()).getIntegerAttribute("edgeID"));
    		
//			if (criteria == "angularLand") segmentsToAvoid.remove(bestDualNode.getData());
			if (bestNode == destinationNode) break;
        	sequence.add(bestNode);
        	knownJunctions = landmarkFunctions.relevantNodes(bestNode, destinationNode, state);
        	wayfindingComplexity = landmarkFunctions.easinessNavigation(bestNode, destinationNode, state);
        	searchRange = utilities.nodesDistance(bestNode, destinationNode) * wayfindingComplexity;
            currentNode = bestNode;
            bestNode = null;
        }
    	for (Node i : sequence) System.out.println(i.getData());
    	return sequence;
	}
	
	static Node previousJunction(ArrayList<GeomPlanarGraphDirectedEdge> path, PedestrianSimulation state)
	{
		Node lastCen = state.centroidsMap.get(((GeomPlanarGraphEdge) path.get(path.size()-1).getEdge()).getIntegerAttribute("edgeID")).c; 
		Node otherCen = state.centroidsMap.get(((GeomPlanarGraphEdge) path.get(path.size()-2).getEdge()).getIntegerAttribute("edgeID")).c;
		return state.nodesMap.get(utilities.commonPrimalJunction(lastCen, otherCen, state)).node;
	}

	static ArrayList<GeomPlanarGraphDirectedEdge> controlPath(Node destinationNode, ArrayList <GeomPlanarGraphDirectedEdge> path)
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
	
	static ArrayList<GeomPlanarGraphDirectedEdge> controlDualPath(Node destinationNode, GeomPlanarGraphDirectedEdge destinationEdge, 
			ArrayList <GeomPlanarGraphDirectedEdge> path, PedestrianSimulation state)
	{
		while (path.contains(destinationEdge))
		{
			if (path.get(path.size()-1) == destinationEdge) 
			{
				if (previousJunction(path, state) == destinationNode) path.remove(path.size()-1);
				return path;	
			}
			else path.remove(path.size()-1);
		}
		
		Node destinationToNode = destinationEdge.getToNode();
		Node destinationFromNode = destinationEdge.getFromNode();
		for (GeomPlanarGraphDirectedEdge e: path)
		{
			if ((e.getToNode() == destinationToNode) || (e.getToNode() == destinationFromNode) || (e.getFromNode() == destinationFromNode) 
					|| (e.getFromNode() == destinationToNode))
				{
					int lastIndex = path.indexOf(e);
					path = new ArrayList<GeomPlanarGraphDirectedEdge>(path.subList(0, lastIndex+1));
					path.add(destinationEdge);
					if (previousJunction(path, state) == destinationNode) path.remove(path.size()-1);
					return path;			
				}
		}
		return null;
	}
	
}

