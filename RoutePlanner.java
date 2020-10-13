package sim.app.geo.pedSimCity;

import java.util.ArrayList;
import sim.app.geo.urbanSim.*;
import sim.app.geo.urbanSim.Utilities.Path;

import sim.util.geo.GeomPlanarGraphDirectedEdge;

public class RoutePlanner {
	
	boolean regionBasedNavigation = false;
	boolean barrierBasedNavigation = false;
	boolean moveOn = false;
	boolean onlyAnchors = true;
	NodeGraph originNode, destinationNode;
	NodeGraph previousJunction = null;
	
	ArrayList<NodeGraph> sequenceNodes = new ArrayList<NodeGraph>();
	ArrayList<NodeGraph> centroidsToAvoid = new ArrayList<NodeGraph>();
	ArrayList<GeomPlanarGraphDirectedEdge> completePath =  new ArrayList<GeomPlanarGraphDirectedEdge>();
	Path path = new Path();
	
	/** 
	 * Road-distance minimisation
	 * A) Direct O-D Road-distance Shortest Path 
	 * B) Sequence [O, ..., D] Road-distance Shortest Path - works for local landmarks, regional+barriers, barriers only
	 * C) Road-distance minimisation + Landmarks-based (Local and Global) Path
	 */
	
	/**
	 * Road-distance based A)
	 */
	public ArrayList<GeomPlanarGraphDirectedEdge> roadDistance(NodeGraph originNode, NodeGraph destinationNode, 
			ArrayList <GeomPlanarGraphDirectedEdge> segmentsToAvoid,  boolean regionBasedNavigation, boolean barrierBasedNavigation,
			String algorithm)
	{	
		if (algorithm == "astar")
		{
			AStarRoadDistance pathfinder = new AStarRoadDistance();
		    path = pathfinder.astarPath(originNode, destinationNode, segmentsToAvoid, barrierBasedNavigation);   
		    return path.edges;
		}
		else
		{
			DijkstraRoadDistance pathfinder = new DijkstraRoadDistance();
		    path = pathfinder.dijkstraPath(originNode, destinationNode, segmentsToAvoid, regionBasedNavigation, barrierBasedNavigation);    
		    return path.edges;
		}
	}
	
	/**
	 * Road-distance based B)
	 */
	public ArrayList<GeomPlanarGraphDirectedEdge> roadDistanceSequence(ArrayList<NodeGraph> sequence, boolean regionBasedNavigation,
			boolean barrierBasedNavigation, String algorithm)
	{	
		this.sequenceNodes = new ArrayList<NodeGraph> (sequence);
		NodeGraph tmpOrigin = sequenceNodes.get(0); // originNode
		this.destinationNode = sequenceNodes.get(sequenceNodes.size()-1);
		sequenceNodes.remove(0);
		
		for (NodeGraph tmpDestination : sequenceNodes)
		{
			moveOn = false;
			//check if this tmpDestination has been traversed already
	    	if (Utilities.nodesFromPath(completePath).contains(tmpDestination))
			{
	    		controlPath(tmpDestination);
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
					
			if (algorithm == "astar")
			{
				AStarRoadDistance pathfinder = new AStarRoadDistance();
			    path = pathfinder.astarPath(tmpOrigin, tmpDestination, completePath, barrierBasedNavigation); 
			}
			else
			{
				DijkstraRoadDistance pathfinder = new DijkstraRoadDistance();
			    path = pathfinder.dijkstraPath(tmpOrigin, tmpDestination, completePath, regionBasedNavigation, barrierBasedNavigation); 
			}
			
			while (path.edges == null && !moveOn) backtracking(tmpDestination, false);
			tmpOrigin = tmpDestination;
			if (moveOn) continue;
			
			checkSequenceEdges(tmpOrigin, tmpDestination);
			completePath.addAll(path.edges);
		}
		return completePath;
	}
	
	/**
	 * Road-distance based C)
	 */
	public ArrayList<GeomPlanarGraphDirectedEdge> roadDistanceLandmarks(ArrayList<NodeGraph> sequence, boolean onlyAnchors, 
			boolean regionBasedNavigation, boolean barrierBasedNavigation)
	{   
		this.sequenceNodes = new ArrayList<NodeGraph> (sequence);
		NodeGraph tmpOrigin = sequenceNodes.get(0); // originNode
		this.destinationNode = sequenceNodes.get(sequenceNodes.size()-1);
		sequenceNodes.remove(0);
		
		for (NodeGraph tmpDestination : sequenceNodes)
    	{	
			// check if node has been traversed already
			moveOn = false;
	    	if (Utilities.nodesFromPath(completePath).contains(tmpDestination))
			{
	    		controlPath(tmpDestination);
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
			
			DijkstraRoadDistanceLandmarks pathFinder = new DijkstraRoadDistanceLandmarks();
		    path = pathFinder.dijkstraPath(tmpOrigin, tmpDestination, destinationNode, completePath, onlyAnchors);
		    while (path.edges == null && !moveOn) backtracking(tmpDestination, true);
			tmpOrigin = tmpDestination;
			if (moveOn) continue;
			
			checkSequenceEdges(tmpOrigin, tmpDestination);
			completePath.addAll(path.edges); 	 
    	}
		return completePath;
	}
		
	/** 
	 * Cumulative angular-change minimisation
	 * A) Direct O-D Least Cumulative Angular Change Shortest Path 
	 * B) Sequence [O, ..., D] Least Cumulative Angular Change Shortest Path - works for local landmarks, regional+barriers, barriers only
	 * C) Angular Change minimisation + Landmarks-based (Local and Global) Path
	 */
	
	/**
	 * Least angular-changed based A)
	 */
	
	public ArrayList<GeomPlanarGraphDirectedEdge> angularChange(NodeGraph originNode, NodeGraph destinationNode, ArrayList<NodeGraph> 
			centroidsToAvoid,NodeGraph previousJunction, boolean regionBasedNavigation, boolean barrierBasedNavigation, String algorithm)
	{	

		NodeGraph dualOrigin = originNode.getDualNode(originNode, destinationNode, regionBasedNavigation, previousJunction);
		NodeGraph dualDestination = null;

    	while (dualDestination == dualOrigin || dualDestination == null) dualDestination = destinationNode.getDualNode(
    			originNode, destinationNode, regionBasedNavigation, previousJunction);
    	
    	if (Utilities.commonPrimalJunction(dualOrigin, dualDestination) != null)
    	{
    		ArrayList<GeomPlanarGraphDirectedEdge> edges = new ArrayList<GeomPlanarGraphDirectedEdge>();
    		edges.add(originNode.getDirectedEdgeBetween(Utilities.commonPrimalJunction(dualOrigin, dualDestination)));
    		edges.add(Utilities.commonPrimalJunction(dualOrigin, dualDestination).getDirectedEdgeBetween(destinationNode));
    		return edges;
    	}
    	
		if (algorithm == "astar")
		{
			AStarAngularChange pathfinder = new AStarAngularChange();
		    path = pathfinder.astarPath(dualOrigin, dualDestination, centroidsToAvoid, previousJunction, barrierBasedNavigation); 
		}
		else
		{
			DijkstraAngularChange pathfinder = new DijkstraAngularChange();
		    path = pathfinder.dijkstraPath(dualOrigin, dualDestination, centroidsToAvoid, previousJunction, 
		    		regionBasedNavigation, barrierBasedNavigation); 
		}

	    cleanDualPath(originNode, destinationNode);
	    return path.edges;
	}
	
	/**
	 * Least angular-changed based B)
	 */
	
	public ArrayList<GeomPlanarGraphDirectedEdge> angularChangeSequence(ArrayList<NodeGraph> sequence, boolean regionBasedNavigation,
			boolean barrierBasedNavigation, String algorithm)
	{	
		this.sequenceNodes = new ArrayList<NodeGraph> (sequence);
		NodeGraph tmpOrigin = originNode = sequenceNodes.get(0); // originNode
		this.destinationNode = sequenceNodes.get(sequenceNodes.size()-1);
		sequenceNodes.remove(0);
		
		for (NodeGraph tmpDestination : sequenceNodes)
		{
			//check if this tmpDestination has been traversed already
			moveOn = false;
			if (tmpOrigin != originNode) 
			{
				previousJunction = Utilities.previousJunction(completePath);
				centroidsToAvoid = Utilities.centroidsFromPath(completePath);
			}
			
	    	if (Utilities.nodesFromPath(completePath).contains(tmpDestination))
			{
	    		controlPath(tmpDestination);
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
			//TO DO centroids to avoid in dual
			NodeGraph tmpDualOrigin = tmpOrigin.getDualNode(tmpOrigin, tmpDestination, regionBasedNavigation, previousJunction);

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
		    	tmpDualOrigin = tmpOrigin.getDualNode(tmpOrigin, tmpDestination, regionBasedNavigation, previousJunction);
			}
			if (tmpOrigin == tmpDestination) continue;
		
			NodeGraph tmpDualDestination = null;
	    	while ((tmpDualDestination == tmpDualOrigin) || (tmpDualDestination == null)) tmpDualDestination = tmpDestination.getDualNode(
	    			tmpOrigin, tmpDestination, regionBasedNavigation, previousJunction);
	    	
			//check if just one node separates them
	    	if (Utilities.commonPrimalJunction(tmpDualOrigin, tmpDualDestination) != null)
	    	{
	    		completePath.add(tmpOrigin.getDirectedEdgeBetween(Utilities.commonPrimalJunction(tmpDualOrigin, tmpDualDestination)));
	    		completePath.add(Utilities.commonPrimalJunction(tmpDualOrigin, tmpDualDestination).getDirectedEdgeBetween(tmpDestination));
	    		tmpOrigin = tmpDestination;
	    		continue;
	    	}

			if (algorithm == "astar")
			{
				AStarAngularChange pathFinder = new AStarAngularChange();
			    path = pathFinder.astarPath(tmpOrigin, tmpDestination, centroidsToAvoid, previousJunction, barrierBasedNavigation); 
			}
			else
			{
				DijkstraAngularChange pathFinder = new DijkstraAngularChange();
			    path = pathFinder.dijkstraPath(tmpDualOrigin, tmpDualDestination, centroidsToAvoid, previousJunction, 
			    		regionBasedNavigation, barrierBasedNavigation); 
			}

			while (path.edges == null && !moveOn) backtrackingDual(tmpDualOrigin, tmpDualDestination, tmpDestination, true);
		    if (path.edges == null) continue;
			tmpOrigin = tmpDestination;
			if (moveOn) continue;
			
			cleanDualPath(tmpOrigin, tmpDestination);
			completePath.addAll(path.edges);
		}
		return completePath;
	}
		
		
	/**
	 * Least angular-changed based C)
	 */
	public ArrayList<GeomPlanarGraphDirectedEdge> angularChangeLandmarks(ArrayList<NodeGraph> sequence, boolean onlyAnchors,
			boolean regionBasedNavigation, boolean barrierBasedNavigation)
	{	
		this.sequenceNodes = new ArrayList<NodeGraph> (sequence);
		NodeGraph tmpOrigin = originNode = sequenceNodes.get(0); // originNode
		this.destinationNode = sequenceNodes.get(sequenceNodes.size()-1);
		sequenceNodes.remove(0);
		
		for (NodeGraph tmpDestination : sequenceNodes)
		{
			moveOn = false;
			if (tmpOrigin != originNode) 
			{
				previousJunction = Utilities.previousJunction(completePath);
				centroidsToAvoid = Utilities.centroidsFromPath(completePath);
			}
						
			//check if this tmpDestination has been traversed already
	    	if (Utilities.nodesFromPath(completePath).contains(tmpDestination))
			{
	    		controlPath(tmpDestination);
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

			//TO DO centroids to avoid in dual
			NodeGraph tmpDualOrigin = tmpOrigin.getDualNode(tmpOrigin, tmpDestination, regionBasedNavigation, previousJunction);
			NodeGraph tmpDualDestination = null;
	    	while (tmpDualDestination == tmpDualOrigin || tmpDualDestination == null) tmpDualDestination = tmpDestination.getDualNode(
	    			tmpOrigin, tmpDestination, regionBasedNavigation, previousJunction);

			//check if just one node separates them
	    	if (Utilities.commonPrimalJunction(tmpDualOrigin, tmpDualDestination) != null)
	    	{
	    		completePath.add(tmpOrigin.getDirectedEdgeBetween(Utilities.commonPrimalJunction(tmpDualOrigin, tmpDualDestination)));
	    		completePath.add(Utilities.commonPrimalJunction(tmpDualOrigin, tmpDualDestination).getDirectedEdgeBetween(tmpDestination));
	    		tmpOrigin = tmpDestination;
	    		continue;
	    	}
	    	    	
			DijkstraAngularChangeLandmarks pathFinder = new DijkstraAngularChangeLandmarks();
		    path = pathFinder.dijkstraPath(tmpDualOrigin, tmpDualDestination, destinationNode, centroidsToAvoid, previousJunction, onlyAnchors);

			while (path.edges == null && !moveOn) backtrackingDual(tmpDualOrigin, tmpDualDestination, tmpDestination, true);	
		    if (path.edges == null) continue;
			tmpOrigin = tmpDestination;
			if (moveOn) continue;
			
			cleanDualPath(tmpOrigin, tmpDestination);
			completePath.addAll(path.edges);
		}
		return completePath;
	}
	
	/** 
	 * Global Landmarks Path 
	 */
	public ArrayList<GeomPlanarGraphDirectedEdge> globalLandmarksPath (NodeGraph originNode, NodeGraph destinationNode, ArrayList <Integer> segmentsToAvoid,
			boolean onlyAnchors)
	{	
		DijkstraGlobalLandmarks pathfinder = new DijkstraGlobalLandmarks();
	    path = pathfinder.dijkstraPath(originNode, destinationNode, segmentsToAvoid, onlyAnchors);    
	    return path.edges;		
	}
			
	public ArrayList<GeomPlanarGraphDirectedEdge> regionBarrierBasedPath (NodeGraph originNode, NodeGraph destinationNode,	
			boolean barrierBasedNavigation,	String localHeuristic)
	{
		RegionBasedNavigation regionsPath = new RegionBasedNavigation();
		ArrayList<NodeGraph> regionsSequence = regionsPath.sequenceRegions(originNode, destinationNode, barrierBasedNavigation);
		ArrayList<GeomPlanarGraphDirectedEdge> path =  new ArrayList<GeomPlanarGraphDirectedEdge>();
		
		if (localHeuristic == "roadDistance") path = roadDistanceSequence(regionsSequence, true, barrierBasedNavigation, "dijkstra");
        else if (localHeuristic == "angularChange") path = angularChangeSequence(regionsSequence, true, barrierBasedNavigation, "dijkstra");
		return path;
	}
	
	public ArrayList<GeomPlanarGraphDirectedEdge> barrierBasedPath (NodeGraph originNode, NodeGraph destinationNode, String localHeuristic)
	{
		ArrayList<GeomPlanarGraphDirectedEdge> path =  new ArrayList<GeomPlanarGraphDirectedEdge>();
		BarrierBasedNavigation barrierBasedPath = new BarrierBasedNavigation();
		ArrayList<NodeGraph> sequenceBarriers = barrierBasedPath.sequenceBarriers(originNode, destinationNode);
		if (localHeuristic == "roadDistance") 
			path = roadDistanceSequence(sequenceBarriers, false, true, "dijkstra");
		else if (localHeuristic == "angularChange") path = angularChangeSequence(sequenceBarriers, false, true, "dijkstra");
		return path;
	}
	
	/** 
	 * Utility functions to clean paths and check dual-paths
	 */
		

	
	private void backtracking (NodeGraph tmpDestination, boolean landmarkBasedNavigation)
	{
		NodeGraph tmpOrigin = (NodeGraph) completePath.get(completePath.size()-1).getFromNode();
		completePath.remove(completePath.size()-1);
		GeomPlanarGraphDirectedEdge edge = tmpOrigin.getDirectedEdgeBetween(tmpDestination);
		
		if (edge != null)
		{
			if (!completePath.contains(edge)) completePath.add(edge);
			moveOn = true;
			return;
		}
		if (landmarkBasedNavigation)
		{
			DijkstraRoadDistanceLandmarks pathFinder = new DijkstraRoadDistanceLandmarks();
			path = pathFinder.dijkstraPath(tmpOrigin, tmpDestination, destinationNode, completePath, onlyAnchors);
		}
		else
		{
			DijkstraRoadDistance pathFinder = new DijkstraRoadDistance();
			path = pathFinder.dijkstraPath(tmpOrigin, tmpDestination, completePath, false, false);
		}
	}
	
	
	private void backtrackingDual(NodeGraph tmpDualOrigin, NodeGraph tmpDualDestination, NodeGraph tmpDestination, boolean landmarkBasedNavigation)
	{
		
		NodeGraph tmpOrigin;
		try {
			tmpOrigin = (NodeGraph) completePath.get(completePath.size()-1).getFromNode(); // new tmpOrigin
		}
		catch(java.lang.ArrayIndexOutOfBoundsException e)
		{
			path.edges = null;
			return;
		}
		
		completePath.remove(completePath.size()-1); // remove last one which did not work!
	    centroidsToAvoid.remove(centroidsToAvoid.size()-1);
		previousJunction = Utilities.previousJunction(completePath); // take new previous junction
		GeomPlanarGraphDirectedEdge edge = tmpOrigin.getDirectedEdgeBetween(tmpDestination);
		
		if (edge != null)
		{
			if (!completePath.contains(edge)) completePath.add(edge);
			moveOn = true;
			return;
		}
		
    	tmpDualOrigin = tmpOrigin.getDualNode(tmpOrigin, tmpDestination, regionBasedNavigation, previousJunction);
		
    	if (landmarkBasedNavigation)
    	{
    		DijkstraAngularChangeLandmarks pathFinder = new DijkstraAngularChangeLandmarks();
    		path = pathFinder.dijkstraPath(tmpDualOrigin, tmpDualDestination, destinationNode,	centroidsToAvoid, 
    				previousJunction, onlyAnchors);
    	}
    	else
    	{
	    	DijkstraAngularChange pathFinder = new DijkstraAngularChange();
			path = pathFinder.dijkstraPath(tmpDualOrigin, tmpDualDestination, centroidsToAvoid, previousJunction,regionBasedNavigation, 
					barrierBasedNavigation);
    	}
	}  
	
	private void controlPath(NodeGraph destinationNode)
	{

		for (GeomPlanarGraphDirectedEdge e: completePath)
		{
			if (e.getToNode() == destinationNode)
			{
				int lastIndex = completePath.indexOf(e);
				completePath = new ArrayList<GeomPlanarGraphDirectedEdge>(completePath.subList(0, lastIndex+1));
				if (Utilities.previousJunction(completePath) == destinationNode) completePath.remove(completePath.size()-1);
				return;			
			}
		}
	}
	
	
	private void cleanDualPath(NodeGraph originNode, NodeGraph destinationNode)
	{
		//check if the path is one edge ahead 
		if (Utilities.previousJunction(path.edges) == destinationNode) path.edges.remove(path.edges.size()-1); 
		//check presence of a unnecessary edge at the beginning of the path
		if (Utilities.commonPrimalJunction(((EdgeGraph) path.edges.get(0).getEdge()).dualNode, 
	    		((EdgeGraph) path.edges.get(1).getEdge()).dualNode) == originNode) path.edges.remove(0); 
		checkSequenceEdges(originNode, destinationNode);	
	}
	
	
	private void checkSequenceEdges(NodeGraph originNode, NodeGraph destinationNode)
	{
		NodeGraph previousNode = originNode;
		for (GeomPlanarGraphDirectedEdge edge: path.edges)
		{
			NodeGraph nextNode = (NodeGraph) edge.getToNode();
			if (nextNode == previousNode) //need to swap
			{
				nextNode = (NodeGraph) edge.getFromNode();
				GeomPlanarGraphDirectedEdge correctEdge = previousNode.getDirectedEdgeBetween(nextNode);
				path.edges.set(path.edges.indexOf(edge), correctEdge);
			}
			previousNode = nextNode;
		}
	}

}

