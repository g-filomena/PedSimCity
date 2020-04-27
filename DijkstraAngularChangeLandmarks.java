package sim.app.geo.pedestrianSimulation;

import java.util.ArrayList;
import java.util.HashMap;

import sim.app.geo.urbanSim.*;
import sim.app.geo.urbanSim.utilities.Path;
import sim.util.geo.GeomPlanarGraphDirectedEdge;

public class DijkstraAngularChangeLandmarks {
    
	NodeGraph originNode, destinationNode, primalDestinationNode;
	ArrayList<NodeGraph> visitedNodes;
	ArrayList<NodeGraph> unvisitedNodes;
	NodeGraph previousJunction = null;
	NodeGraph safe;
    HashMap<NodeGraph, NodeWrapper> mapWrappers =  new HashMap<NodeGraph, NodeWrapper>();
    ArrayList<NodeGraph> centroidsToAvoid = new ArrayList<NodeGraph>();
    
    public Path dijkstraPath (NodeGraph originNode, NodeGraph destinationNode, 
    		NodeGraph primalDestinationNode, ArrayList<NodeGraph> centroidsToAvoid, NodeGraph previousJunction)
	{
    	this.originNode = originNode;
    	this.destinationNode = destinationNode;
    	this.primalDestinationNode = primalDestinationNode;		
    	this.centroidsToAvoid = centroidsToAvoid;
    	this.previousJunction = previousJunction;
		
		visitedNodes = new ArrayList<NodeGraph>();
		unvisitedNodes = new ArrayList<NodeGraph>();
		unvisitedNodes.add(originNode);
		
		NodeWrapper NodeWrapper = new NodeWrapper(originNode);
		NodeWrapper.gx = 0.0;
		if (previousJunction != null) NodeWrapper.commonPrimalJunction = previousJunction;
        mapWrappers.put(originNode, NodeWrapper);
        
        if (centroidsToAvoid != null) for (NodeGraph c : centroidsToAvoid) visitedNodes.add(c);

		while (unvisitedNodes.size() > 0) 
		{
			NodeGraph currentNode = getClosest(unvisitedNodes); // at the beginning it takes originNode
			visitedNodes.add(currentNode);
			unvisitedNodes.remove(currentNode);
			findMinDistances(currentNode);
		}
		
		return reconstructPath(originNode, destinationNode);
	}

	private void findMinDistances(NodeGraph currentNode) 
	{
		ArrayList<NodeGraph> adjacentNodes = currentNode.getAdjacentNodes();        
	    for (NodeGraph targetNode : adjacentNodes) 
	    {    
	    	if (visitedNodes.contains(targetNode)) continue;
            if (utilities.commonPrimalJunction(targetNode, currentNode) ==  mapWrappers.get(currentNode).commonPrimalJunction) 
            	continue;

            EdgeGraph commonEdge = null;
            commonEdge = currentNode.getEdgeBetween(targetNode);
	    	double segmentCost = commonEdge.getDeflectionAngle() + utilities.fromNormalDistribution(0, 5, null);                
            if (segmentCost > 180) segmentCost = 180;
            if (segmentCost < 0) segmentCost = 0;
	    	GeomPlanarGraphDirectedEdge outEdge = (GeomPlanarGraphDirectedEdge) commonEdge.getDirEdge(0);
	    	
            double globalLandmarkness = LandmarksNavigation.globalLandmarknessDualNode(targetNode, primalDestinationNode, true);
        	double nodeLandmarkness = 1-globalLandmarkness*0.70;
        	double nodeCost = nodeLandmarkness*segmentCost;
        	double tentativeCost = getBest(currentNode) + nodeCost;
	    	
	    	if (getBest(currentNode) > tentativeCost)
	    		{
	    			NodeWrapper NodeWrapper = mapWrappers.get(targetNode);
	                if (NodeWrapper == null) NodeWrapper = new NodeWrapper(targetNode);
	                NodeWrapper.nodeFrom = currentNode;
	                NodeWrapper.edgeFrom = outEdge;
	                NodeWrapper.commonPrimalJunction = utilities.commonPrimalJunction(currentNode, targetNode);
	                NodeWrapper.gx = tentativeCost;
	                mapWrappers.put(targetNode, NodeWrapper);
	                unvisitedNodes.add(targetNode);
	    		}    			
		}
	}


	private NodeGraph getClosest(ArrayList<NodeGraph> nodes) //amongst unvisited (they have to have been explored)
	{
		NodeGraph closest = null;
		for (NodeGraph node : nodes) 
		{
			if (closest == null) closest = node;
			else if (getBest(node) < getBest(closest)) closest = node;
	    }
	    return closest;
	}

	Double getBest(NodeGraph target)
	{
		if (mapWrappers.get(target) == null) return Double.MAX_VALUE;
	    else return mapWrappers.get(target).gx;
	}

	public Path reconstructPath(NodeGraph originNode, NodeGraph destinationNode) 
	{
    	Path path = new Path();
		path.edges = null;
		path.mapWrappers = null;
		
		HashMap<NodeGraph, NodeWrapper> mapTraversedWrappers =  new HashMap<NodeGraph, NodeWrapper>();
		ArrayList<GeomPlanarGraphDirectedEdge> sequenceEdges = new ArrayList<GeomPlanarGraphDirectedEdge>();
		NodeGraph step = destinationNode;
		mapTraversedWrappers.put(destinationNode, mapWrappers.get(destinationNode));
		
		if ((step == null) || (mapWrappers.size() == 1)) return path;
		try 
		{
			while (mapWrappers.get(step).nodeFrom != null)
	    	{
				GeomPlanarGraphDirectedEdge de = (GeomPlanarGraphDirectedEdge) step.primalEdge.getDirEdge(0);
	        	step = mapWrappers.get(step).nodeFrom;
	        	mapTraversedWrappers.put(step, mapWrappers.get(step));
	        	sequenceEdges.add(0, de); 
	        	if (step == originNode)
	        	{
					GeomPlanarGraphDirectedEdge lastDe = (GeomPlanarGraphDirectedEdge) 
							step.primalEdge.getDirEdge(0);
		        	sequenceEdges.add(0, lastDe); 
					break;
	        	}
	    	}
		}
		catch(java.lang.NullPointerException e)  {return path;}
		
        path.edges = sequenceEdges;
        path.mapWrappers = mapTraversedWrappers;
	    return path;
	 }
}


