package sim.app.geo.pedestrianSimulation;

import java.util.ArrayList;
import sim.app.geo.urbanSim.*;
import sim.app.geo.urbanSim.Utilities.Path;

import sim.util.geo.GeomPlanarGraphDirectedEdge;

public class routePlanning {
	
	static boolean regionalRouting = false;
	static boolean barriersRouting = false;
	
    /** O-D Road Distance Shortest Path **/
	public static Path roadDistance(NodeGraph originNode, NodeGraph destinationNode, 
			ArrayList <GeomPlanarGraphDirectedEdge> segmentsToAvoid,  boolean regionalRouting, boolean barriersRouting, String algorithm)
	{	
		Path path = new Path();
		if (algorithm == "astar")
		{
			AStarRoadDistance pathfinder = new AStarRoadDistance();
		    path = pathfinder.astarPath(originNode, destinationNode, segmentsToAvoid, regionalRouting, barriersRouting);   
		    return path;
		}
		else
		{
			DijkstraRoadDistance pathfinder = new DijkstraRoadDistance();
		    path = pathfinder.dijkstraPath(originNode, destinationNode, segmentsToAvoid, regionalRouting, barriersRouting);    
		    return path;
		}
	}
	
	/** Sequence [O, ..., D] Road Distance Shortest Path - works for local landmarks and regional **/
	public static ArrayList<GeomPlanarGraphDirectedEdge> roadDistanceSequence(ArrayList<NodeGraph> sequenceNodes, boolean regionalRouting,
			boolean barriersRouting, String algorithm)
	{	

		NodeGraph tmpOrigin = sequenceNodes.get(0); // originNode
		sequenceNodes.remove(0);
		ArrayList<GeomPlanarGraphDirectedEdge> partialPath =  new ArrayList<GeomPlanarGraphDirectedEdge>();
		ArrayList<GeomPlanarGraphDirectedEdge> completePath =  new ArrayList<GeomPlanarGraphDirectedEdge>();
		
		for (NodeGraph tmpDestination : sequenceNodes)
		{
			//check if this tmpDestination has been traversed already
	    	if (Utilities.nodesFromPath(completePath).contains(tmpDestination))
			{
	    		completePath = controlPath(tmpDestination, completePath);
	    		tmpOrigin = tmpDestination;
	    		continue;
			}
	    		    	
	    	//check if edge in between
			GeomPlanarGraphDirectedEdge edge = tmpOrigin.getDirectedEdgeBetween(tmpDestination);
			if (edge != null)
			{
				if (!completePath.contains(edge)) completePath.add(edge);
				tmpOrigin = tmpDestination;
				continue;
			}	
					
	    	Path path = new Path();
			if (algorithm == "astar")
			{
				AStarRoadDistance pathfinder = new AStarRoadDistance();
			    path = pathfinder.astarPath(tmpOrigin, tmpDestination, completePath, regionalRouting, barriersRouting); 
			}
			else
			{
				DijkstraRoadDistance pathfinder = new DijkstraRoadDistance();
			    path = pathfinder.dijkstraPath(tmpOrigin, tmpDestination, completePath, regionalRouting, barriersRouting); 
			}
			partialPath = checkSequenceEdges(path.edges, tmpOrigin, tmpDestination);
			if (partialPath == null) System.out.println(" null partial path between "+ tmpOrigin.getID() + " "+tmpDestination.getID());
			tmpOrigin = tmpDestination;
			completePath.addAll(partialPath);
		}
		return completePath;
	}
	
	/**Angular-Change + Landmarks (Local and Global) Path **/	
	public static ArrayList<GeomPlanarGraphDirectedEdge> roadDistanceLandmarks(ArrayList<NodeGraph> sequenceNodes, boolean regionalRouting,
			boolean barriersRouting)
	{   
		NodeGraph tmpOrigin = sequenceNodes.get(0); // originNode
		sequenceNodes.remove(0);
		NodeGraph destinationNode = sequenceNodes.get(sequenceNodes.size()-1);
		ArrayList<GeomPlanarGraphDirectedEdge> partialPath =  new ArrayList<GeomPlanarGraphDirectedEdge>();
		ArrayList<GeomPlanarGraphDirectedEdge> completePath =  new ArrayList<GeomPlanarGraphDirectedEdge>();
	
		for (NodeGraph tmpDestination : sequenceNodes)
    	{	
		
			// check if node has been traversed already
	    	if (Utilities.nodesFromPath(completePath).contains(tmpDestination))
			{
	    		completePath = controlPath(tmpDestination, completePath);
	    		tmpOrigin = tmpDestination;
	    		continue;
			}
	    	
	    	//check if edge in between
			GeomPlanarGraphDirectedEdge edge = tmpOrigin.getDirectedEdgeBetween(tmpDestination);
			if (edge != null)
			{
				if (!completePath.contains(edge)) completePath.add(edge);
				tmpOrigin = tmpDestination;
				continue;
			}	    	
			
	    	Path path = new Path();
			DijkstraRoadDistanceLandmarks pathFinder = new DijkstraRoadDistanceLandmarks();
		    path = pathFinder.dijkstraPath(tmpOrigin, tmpDestination, destinationNode, completePath);
		    
			while (path.edges == null)
			{
				tmpOrigin = (NodeGraph) completePath.get(completePath.size()-1).getFromNode();
				completePath.remove(completePath.size()-1);
				edge = tmpOrigin.getDirectedEdgeBetween(tmpDestination);
				if (edge != null)
				{
					if (!completePath.contains(edge)) completePath.add(edge);
					tmpOrigin = tmpDestination;
					break;
				}
				path = pathFinder.dijkstraPath(tmpOrigin, tmpDestination, destinationNode, completePath);
			}
		    if (path.edges == null) continue;
			partialPath = checkSequenceEdges(path.edges, tmpOrigin, tmpDestination);
			tmpOrigin = tmpDestination;
			completePath.addAll(partialPath); 	 
    	}
		return completePath;
	}
	
	
	/** O-D Least Cumulative Angular Change Shortest Path **/
	public static Path angularChange(NodeGraph originNode, NodeGraph destinationNode, ArrayList<NodeGraph> centroidsToAvoid,
			NodeGraph previousJunction, boolean regionalRouting, boolean barriersRouting, String algorithm)
	{	

		NodeGraph dualOrigin = originNode.getDualNode(originNode, destinationNode, regionalRouting, previousJunction);
		NodeGraph dualDestination = null;

    	while (dualDestination == dualOrigin || dualDestination == null) dualDestination = destinationNode.getDualNode(
    			originNode, destinationNode, regionalRouting, previousJunction);
    	
    	if (Utilities.commonPrimalJunction(dualOrigin, dualDestination) != null)
    	{
    		Path path = new Path();
    		ArrayList<GeomPlanarGraphDirectedEdge> edges = new ArrayList<GeomPlanarGraphDirectedEdge>();
    		edges.add(originNode.getDirectedEdgeBetween(Utilities.commonPrimalJunction(dualOrigin, dualDestination)));
    		edges.add(Utilities.commonPrimalJunction(dualOrigin, dualDestination).getDirectedEdgeBetween(destinationNode));
    		path.edges = edges;
    		return path;
    	}
    	
    	Path path = new Path();
		if (algorithm == "astar")
		{
			AStarAngularChange pathfinder = new AStarAngularChange();
		    path = pathfinder.astarPath(dualOrigin, dualDestination, centroidsToAvoid, previousJunction, 
		    		regionalRouting, barriersRouting); 
		}
		else
		{
			DijkstraAngularChange pathfinder = new DijkstraAngularChange();
		    path = pathfinder.dijkstraPath(dualOrigin, dualDestination, centroidsToAvoid, previousJunction, 
		    		regionalRouting, barriersRouting); 
		}

	    path.edges = cleanDualPath(path.edges, originNode, destinationNode);
	    return path;
	}
	
	
	/** Sequence [O, ..., D] Least Cumulative Angular Change Shortest Path - works for local landmarks and regional **/
	public static ArrayList<GeomPlanarGraphDirectedEdge> angularChangeSequence(ArrayList<NodeGraph> sequenceNodes, boolean regionalRouting,
			boolean barriersRouting, String algorithm)
	{	
		NodeGraph tmpOrigin = sequenceNodes.get(0); // originNode
		NodeGraph originNode = tmpOrigin;
		sequenceNodes.remove(0);
		NodeGraph previousJunction = null;

		ArrayList<NodeGraph> centroidsToAvoid = new ArrayList<NodeGraph>();
		ArrayList<GeomPlanarGraphDirectedEdge> partialPath =  new ArrayList<GeomPlanarGraphDirectedEdge>();
		ArrayList<GeomPlanarGraphDirectedEdge> completePath =  new ArrayList<GeomPlanarGraphDirectedEdge>();
		
		for (NodeGraph tmpDestination : sequenceNodes)
		{
			//check if this tmpDestination has been traversed already
//			System.out.println(tmpOrigin.getID() +"  "+ tmpDestination.getID());
			if (tmpOrigin != originNode) 
			{
				previousJunction = Utilities.previousJunction(completePath);
				centroidsToAvoid = Utilities.centroidsFromPath(completePath);
			}
			
	    	if (Utilities.nodesFromPath(completePath).contains(tmpDestination))
			{
	    		completePath = controlPath(tmpDestination, completePath);
	    		tmpOrigin = tmpDestination;
	    		continue;
			}
	    	
	    	//check if edge in between
			GeomPlanarGraphDirectedEdge edge = tmpOrigin.getDirectedEdgeBetween(tmpDestination);
			if (edge != null)
			{
				if (!completePath.contains(edge)) completePath.add(edge);
				tmpOrigin = tmpDestination;
				continue;
			}
			//TO DO centroids toa void in dual
			NodeGraph tmpDualOrigin = tmpOrigin.getDualNode(tmpOrigin, tmpDestination, regionalRouting, previousJunction);

			while (tmpDualOrigin == null && previousJunction !=null)
			{
				tmpOrigin = (NodeGraph) completePath.get(completePath.size()-1).getFromNode();
				completePath.remove(completePath.size()-1); // remove last one which did not work!
			    centroidsToAvoid.remove(centroidsToAvoid.size()-1);
				previousJunction = Utilities.previousJunction(completePath); // take new previous junction
				edge = tmpOrigin.getDirectedEdgeBetween(tmpDestination);
				if (edge != null)
				{
					if (!completePath.contains(edge)) completePath.add(edge);
					tmpOrigin = tmpDestination;
					break;
				}
		    	tmpDualOrigin = tmpOrigin.getDualNode(tmpOrigin, tmpDestination, regionalRouting, previousJunction);
			}
			if (tmpOrigin == tmpDestination) continue;
		
			NodeGraph tmpDualDestination = null;
	    	while ((tmpDualDestination == tmpDualOrigin) || (tmpDualDestination == null)) tmpDualDestination = tmpDestination.getDualNode(
	    			tmpOrigin, tmpDestination, regionalRouting, previousJunction);
	    	
			//check if just one node separates them
	    	if (Utilities.commonPrimalJunction(tmpDualOrigin, tmpDualDestination) != null)
	    	{
	    		completePath.add(tmpOrigin.getDirectedEdgeBetween(Utilities.commonPrimalJunction(tmpDualOrigin, tmpDualDestination)));
	    		completePath.add(Utilities.commonPrimalJunction(tmpDualOrigin, tmpDualDestination).getDirectedEdgeBetween(tmpDestination));
	    		tmpOrigin = tmpDestination;
	    		continue;
	    	}

	    	Path path = new Path();
			if (algorithm == "astar")
			{
				AStarAngularChange pathFinder = new AStarAngularChange();
			    path = pathFinder.astarPath(tmpOrigin, tmpDestination, centroidsToAvoid, previousJunction, 
			    		regionalRouting, barriersRouting); 
			}
			else
			{
				DijkstraAngularChange pathFinder = new DijkstraAngularChange();
			    path = pathFinder.dijkstraPath(tmpDualOrigin, tmpDualDestination, centroidsToAvoid, previousJunction, 
			    		regionalRouting, barriersRouting); 
			    
			}

			while (path.edges == null) // backtracking
			{
				DijkstraAngularChange pathFinder = new DijkstraAngularChange();
				try {
					tmpOrigin = (NodeGraph) completePath.get(completePath.size()-1).getFromNode(); // new tmpOrigin
				}
				catch(java.lang.ArrayIndexOutOfBoundsException e)
				{
					System.out.println("issue");
					path.edges = null;
					break;
					
				}
				completePath.remove(completePath.size()-1); // remove last one which did not work!
			    centroidsToAvoid.remove(centroidsToAvoid.size()-1);
				previousJunction = Utilities.previousJunction(completePath); // take new previous junction
				edge = tmpOrigin.getDirectedEdgeBetween(tmpDestination);
				if (edge != null)
				{
					if (!completePath.contains(edge)) completePath.add(edge);
					tmpOrigin = tmpDestination;
					break;
				}
		    	tmpDualOrigin = tmpOrigin.getDualNode(tmpOrigin, tmpDestination, regionalRouting, previousJunction);
				path = pathFinder.dijkstraPath(tmpDualOrigin, tmpDualDestination, centroidsToAvoid, previousJunction,regionalRouting, barriersRouting);
			}   
			
		    if (path.edges == null) continue;
			partialPath = cleanDualPath(path.edges, tmpOrigin, tmpDestination);
			tmpOrigin = tmpDestination;
			completePath.addAll(partialPath);
		}
		return completePath;
	}
		
//	/**O-D Network Distance Shortest Path **/	
//	public static Path topologicalShortestPath (NodeGraph originNode, NodeGraph destinationNode, ArrayList <Integer> segmentsToAvoid)
//	{	
//		DijkstraNetworkDistance pathfinder = new DijkstraNetworkDistance();
//	    Path path = pathfinder.dijkstraPath(originNode, destinationNode, segmentsToAvoid);    
//	    return path;
//	}
	
	/**Angular-Change + Landmarks (Local and Global) Path **/	
	public static ArrayList<GeomPlanarGraphDirectedEdge> angularChangeLandmarks(ArrayList<NodeGraph> sequenceNodes, boolean regionalRouting,
			boolean barriersRouting)
	{	
		NodeGraph tmpOrigin = sequenceNodes.get(0); // originNode
		NodeGraph originNode = tmpOrigin;
		sequenceNodes.remove(0);
		NodeGraph destinationNode = sequenceNodes.get(sequenceNodes.size()-1);
		NodeGraph previousJunction = null;

		ArrayList<NodeGraph> centroidsToAvoid = new ArrayList<NodeGraph>();
		ArrayList<GeomPlanarGraphDirectedEdge> partialPath =  new ArrayList<GeomPlanarGraphDirectedEdge>();
		ArrayList<GeomPlanarGraphDirectedEdge> completePath =  new ArrayList<GeomPlanarGraphDirectedEdge>();
		
		for (NodeGraph tmpDestination : sequenceNodes)
		{
			if (tmpOrigin != originNode) 
			{
				previousJunction = Utilities.previousJunction(completePath);
				centroidsToAvoid = Utilities.centroidsFromPath(completePath);
			}
						
			//check if this tmpDestination has been traversed already
	    	if (Utilities.nodesFromPath(completePath).contains(tmpDestination))
			{
	    		completePath = controlPath(tmpDestination, completePath);
	    		tmpOrigin = tmpDestination;
	    		continue;
			}
	    	
	    	//check if edge in between
			GeomPlanarGraphDirectedEdge edge = tmpOrigin.getDirectedEdgeBetween(tmpDestination);
			if (edge != null)
			{
				if (!completePath.contains(edge)) completePath.add(edge);
				tmpOrigin = tmpDestination;
				continue;
			}	    	

			//TO DO centroids toa void in dual
			NodeGraph tmpDualOrigin = tmpOrigin.getDualNode(tmpOrigin, tmpDestination, regionalRouting, previousJunction);
			NodeGraph tmpDualDestination = null;
	    	while (tmpDualDestination == tmpDualOrigin || tmpDualDestination == null) tmpDualDestination = tmpDestination.getDualNode(
	    			tmpOrigin, tmpDestination, regionalRouting, previousJunction);

			//check if just one node separates them
	    	if (Utilities.commonPrimalJunction(tmpDualOrigin, tmpDualDestination) != null)
	    	{
	    		completePath.add(tmpOrigin.getDirectedEdgeBetween(Utilities.commonPrimalJunction(tmpDualOrigin, tmpDualDestination)));
	    		completePath.add(Utilities.commonPrimalJunction(tmpDualOrigin, tmpDualDestination).getDirectedEdgeBetween(tmpDestination));
	    		tmpOrigin = tmpDestination;
	    		continue;
	    	}
	    	    	
	    	Path path = new Path();
			DijkstraAngularChangeLandmarks pathFinder = new DijkstraAngularChangeLandmarks();
		    path = pathFinder.dijkstraPath(tmpDualOrigin, tmpDualDestination, destinationNode, centroidsToAvoid, previousJunction);

			while (path.edges == null) // backtracking
			{
				pathFinder = new DijkstraAngularChangeLandmarks();
				tmpOrigin = (NodeGraph) completePath.get(completePath.size()-1).getFromNode(); // new tmpOrigin
				completePath.remove(completePath.size()-1); // remove last one which did not work!
			    centroidsToAvoid.remove(centroidsToAvoid.size()-1);
				previousJunction = Utilities.previousJunction(completePath); // take new previous junction
				edge = tmpOrigin.getDirectedEdgeBetween(tmpDestination);
				if (edge != null)
				{
					if (!completePath.contains(edge)) completePath.add(edge);
					tmpOrigin = tmpDestination;
					break;
				}
		    	tmpDualOrigin = tmpOrigin.getDualNode(tmpOrigin, tmpDestination, regionalRouting, previousJunction);
				path = pathFinder.dijkstraPath(tmpDualOrigin, tmpDualDestination, destinationNode,	centroidsToAvoid, 
						previousJunction);
			}  
			if (path.edges == null) continue;
			partialPath = cleanDualPath(path.edges, tmpOrigin, tmpDestination);
			tmpOrigin = tmpDestination;
			completePath.addAll(partialPath);
		}
		return completePath;
	}
	
		
	/** Global Landmarks Path **/
	public static Path globalLandmarksPath (NodeGraph originNode, NodeGraph destinationNode, ArrayList <Integer> segmentsToAvoid, String criteria)
	{	
		DijkstraGlobalLandmarks pathfinder = new DijkstraGlobalLandmarks();
	    Path path = pathfinder.dijkstraPath(originNode, destinationNode, segmentsToAvoid);    
	    return path;		
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
			if (e.getToNode() == destinationNode)
			{
				int lastIndex = path.indexOf(e);
				path = new ArrayList<GeomPlanarGraphDirectedEdge>(path.subList(0, lastIndex+1));
				if (Utilities.previousJunction(path) == destinationNode) path.remove(path.size()-1);
				return path;			
			}
		}
		return null;
	}
	
	
	static ArrayList<GeomPlanarGraphDirectedEdge> cleanDualPath(ArrayList<GeomPlanarGraphDirectedEdge> edges, 
			NodeGraph originNode,	NodeGraph destinationNode)
	{
		//check if the path is one edge ahead 
		if (Utilities.previousJunction(edges) == destinationNode) edges.remove(edges.size()-1); 
		//check presence of a unnecessary edge at the beginning of the path
		if (Utilities.commonPrimalJunction(((EdgeGraph) edges.get(0).getEdge()).dualNode, 
	    		((EdgeGraph) edges.get(1).getEdge()).dualNode) == originNode) edges.remove(0); 
		
		edges = checkSequenceEdges(edges, originNode, destinationNode);	
	    return edges;
	}
	
	
	static ArrayList<GeomPlanarGraphDirectedEdge> checkSequenceEdges(ArrayList<GeomPlanarGraphDirectedEdge> edges, 
			NodeGraph originNode, NodeGraph destinationNode)
	{
		NodeGraph previousNode = originNode;
		for (GeomPlanarGraphDirectedEdge edge: edges)
		{
			NodeGraph nextNode = (NodeGraph) edge.getToNode();
			if (nextNode == previousNode) //need to swap
			{
				nextNode = (NodeGraph) edge.getFromNode();
				GeomPlanarGraphDirectedEdge correctEdge = previousNode.getDirectedEdgeBetween(nextNode);
				edges.set(edges.indexOf(edge), correctEdge);
			}
			previousNode = nextNode;
		}
		return edges;
	}

}

