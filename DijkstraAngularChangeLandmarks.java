package sim.app.geo.pedSimCity;

import java.util.ArrayList;
import java.util.HashMap;

import sim.app.geo.urbanSim.*;
import sim.app.geo.urbanSim.Utilities.Path;
import sim.util.geo.GeomPlanarGraphDirectedEdge;

public class DijkstraAngularChangeLandmarks {
    
	NodeGraph originNode, destinationNode, primalDestinationNode;
	ArrayList<NodeGraph> visitedNodes, unvisitedNodes;
	NodeGraph previousJunction = null;
    HashMap<NodeGraph, NodeWrapper> mapWrappers =  new HashMap<NodeGraph, NodeWrapper>();
    ArrayList<NodeGraph> centroidsToAvoid = new ArrayList<NodeGraph>();
    boolean onlyAnchors;
    
    public Path dijkstraPath (NodeGraph originNode, NodeGraph destinationNode, 
    		NodeGraph primalDestinationNode, ArrayList<NodeGraph> centroidsToAvoid, NodeGraph previousJunction, boolean onlyAnchors)
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
            if (Utilities.commonPrimalJunction(targetNode, currentNode) ==  mapWrappers.get(currentNode).commonPrimalJunction) continue;

            EdgeGraph commonEdge = null;
            commonEdge = currentNode.getEdgeBetween(targetNode);
//	    	double edgeCost = commonEdge.getDeflectionAngle() + Utilities.fromNormalDistribution(0, 5, null);      
	    	
            double error = Utilities.fromNormalDistribution(1, 0.10, null);
            if (error < 0) error = 0.00;
	    	double edgeCost = commonEdge.getDeflectionAngle()*error;
	    	
            if (edgeCost > 180) edgeCost = 180;
            if (edgeCost < 0) edgeCost = 0;
	    	GeomPlanarGraphDirectedEdge outEdge = (GeomPlanarGraphDirectedEdge) commonEdge.getDirEdge(0);
	    	
            double globalLandmarkness = 0.0;
	    	if (onlyAnchors) globalLandmarkness = LandmarkNavigation.globalLandmarknessDualNode(currentNode, targetNode, 
	    			primalDestinationNode, true);
	    	else globalLandmarkness = LandmarkNavigation.globalLandmarknessDualNode(currentNode, targetNode, primalDestinationNode, false);
	    	
        	double nodeLandmarkness = 1-globalLandmarkness*PedSimCity.globalLandmarknessWeight;
        	double nodeCost = nodeLandmarkness*edgeCost;
        	double tentativeCost = getBest(currentNode) + nodeCost;
	    	
	    	if (getBest(targetNode) > tentativeCost)
    		{
    			NodeWrapper NodeWrapper = mapWrappers.get(targetNode);
                if (NodeWrapper == null) NodeWrapper = new NodeWrapper(targetNode);
                NodeWrapper.nodeFrom = currentNode;
                NodeWrapper.edgeFrom = outEdge;
                NodeWrapper.commonPrimalJunction = Utilities.commonPrimalJunction(currentNode, targetNode);
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
					GeomPlanarGraphDirectedEdge lastDe = (GeomPlanarGraphDirectedEdge) step.primalEdge.getDirEdge(0);
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


