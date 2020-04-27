package sim.app.geo.pedestrianSimulation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import sim.app.geo.urbanSim.*;
import sim.app.geo.urbanSim.utilities.Path;
import sim.util.geo.GeomPlanarGraphDirectedEdge;

public class DijkstraAngularChange {
    
	NodeGraph originNode, destinationNode;
	ArrayList<NodeGraph> visitedNodes;
	ArrayList<NodeGraph> unvisitedNodes;
	NodeGraph previousJunction;
	int barrierID;
	boolean barriersRouting;
	SubGraph graph = new SubGraph();
    HashMap<NodeGraph, NodeWrapper> mapWrappers =  new HashMap<NodeGraph, NodeWrapper>();
    ArrayList<NodeGraph> centroidsToAvoid = new ArrayList<NodeGraph>();
    
    public Path dijkstraPath (NodeGraph originNode, NodeGraph destinationNode, 
    		ArrayList<NodeGraph> centroidsToAvoid, NodeGraph previousJunction, boolean regionalRouting, boolean barriersRouting, int barrierID)
	{
    	this.originNode = originNode;
    	this.destinationNode = destinationNode;
    	this.centroidsToAvoid = centroidsToAvoid;
		this.barrierID = barrierID;
		this.barriersRouting = barriersRouting;

    	if ((originNode.district == destinationNode.district) && regionalRouting)
    	{
    		graph = PedestrianSimulation.districtsMap.get(originNode.district).dualGraph;
    		originNode = graph.findNode(originNode.getCoordinate());
    		destinationNode = graph.findNode(destinationNode.getCoordinate());
    		if (centroidsToAvoid != null) centroidsToAvoid = graph.getChildNodes(centroidsToAvoid);
    		// primalJunction is always the same;
    	}
    	else // create graph from convex hull
    	{
    		ArrayList<EdgeGraph> containedEdges = PedestrianSimulation.dualNetwork.edgesWithinSpace(originNode, destinationNode); 
    		SubGraph graph = new SubGraph(PedestrianSimulation.dualNetwork, containedEdges);
    		originNode = graph.findNode(originNode.getCoordinate());
    		destinationNode = graph.findNode(destinationNode.getCoordinate());
    		if (centroidsToAvoid != null) centroidsToAvoid = graph.getChildNodes(centroidsToAvoid);
    	}
  
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
            if (utilities.commonPrimalJunction(targetNode, currentNode) == mapWrappers.get(currentNode).commonPrimalJunction) 
            	continue;
	    	
            EdgeGraph commonEdge = null;
            commonEdge = currentNode.getEdgeBetween(targetNode);
	    	
	    	double error = 0.0;
	    	List<Integer> positiveBarriers = targetNode.primalEdge.positiveBarriers;
	    	List<Integer> negativeBarriers = targetNode.primalEdge.negativeBarriers;
	    	if (barriersRouting) 
	    	{
	    		if ((barrierID != 999999) && (positiveBarriers != null))
	    		{ 
	    			if (positiveBarriers.contains(barrierID)) error = utilities.fromNormalDistribution(1, 0.20, "left");
	    			else if (negativeBarriers != null) error = utilities.fromNormalDistribution(1, 0.20, "right");
	    			else error = utilities.fromNormalDistribution(1, 0.10, null);
	    		}
	    		if (positiveBarriers != null) error = utilities.fromNormalDistribution(0.80, 0.10, "left");
	    		else if (negativeBarriers != null) error = utilities.fromNormalDistribution(1.20, 0.10, "right");
	    		else error = utilities.fromNormalDistribution(1, 0.10, null);
	    	}
	    	else error = utilities.fromNormalDistribution(1, 0.10, null);

//	    	double segmentCost = commonEdge.getDeflectionAngle() + utilities.fromNormalDistribution(0, 5); 	
	    	double edgeCost = commonEdge.getDeflectionAngle() * error; 	
            if (edgeCost > 180) edgeCost = 180;
            if (edgeCost < 0) edgeCost = 0;
	    	GeomPlanarGraphDirectedEdge outEdge = (GeomPlanarGraphDirectedEdge) commonEdge.getDirEdge(0);
	   
        	double tentativeCost = getBest(currentNode) + edgeCost;
	    	if (getBest(targetNode) > tentativeCost)
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


