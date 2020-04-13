package sim.app.geo.pedestrianSimulation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import sim.app.geo.pedestrianSimulation.utilities.Path;
import sim.util.geo.GeomPlanarGraphDirectedEdge;

public class DijkstraAngularChange {
    
	NodeGraph originNode, destinationNode;
	ArrayList<NodeGraph> visitedNodes;
	ArrayList<NodeGraph> unvisitedNodes;
	NodeGraph previousJunction;
	int barrierID;
	SubGraph graph = new SubGraph();
    HashMap<NodeGraph, DualNodeWrapper> mapWrappers =  new HashMap<NodeGraph, DualNodeWrapper>();
    ArrayList<NodeGraph> centroidsToAvoid = new ArrayList<NodeGraph>();
    PedestrianSimulation state;
    
    public Path dijkstraPath (NodeGraph originNode, NodeGraph destinationNode, 
    		ArrayList<NodeGraph> centroidsToAvoid, NodeGraph previousJunction, int barrierID, 
    		boolean regionalRouting, PedestrianSimulation state)
	{
    	this.originNode = originNode;
    	this.destinationNode = destinationNode;
    	this.centroidsToAvoid = centroidsToAvoid;
    	this.state = state;
		this.barrierID = barrierID;

    	if ((originNode.district == destinationNode.district) && regionalRouting)
    	{
    		graph = state.districtsMap.get(originNode.district).dualGraph;
    		originNode = graph.findNode(originNode.getCoordinate());
    		destinationNode = graph.findNode(destinationNode.getCoordinate());
    		if (centroidsToAvoid != null) centroidsToAvoid = graph.getChildNodes(centroidsToAvoid);
    		// primalJunction is always the same;
    		System.out.println("after.."+ centroidsToAvoid);
    	}

		visitedNodes = new ArrayList<NodeGraph>();
		unvisitedNodes = new ArrayList<NodeGraph>();
		unvisitedNodes.add(originNode);
		
		DualNodeWrapper dualNodeWrapper = new DualNodeWrapper(originNode);
		dualNodeWrapper.gx = 0.0;
		if (previousJunction != null) dualNodeWrapper.commonPrimalJunction = previousJunction;
        mapWrappers.put(originNode, dualNodeWrapper);
        
        if (centroidsToAvoid != null) for (NodeGraph c : centroidsToAvoid) visitedNodes.add(c);

		while (unvisitedNodes.size() > 0) 
		{
			NodeGraph node = getClosest(unvisitedNodes); // at the beginning it takes originNode
			visitedNodes.add(node);
			unvisitedNodes.remove(node);
			findMinDistances(node);
		}
		
		return reconstructPath(originNode, destinationNode);
	}

	private void findMinDistances(NodeGraph node) 
	{
		ArrayList<NodeGraph> adjacentNodes = utilities.getAdjacentNodes(node);        
	    for (NodeGraph target : adjacentNodes) 
	    {    
	    	if (visitedNodes.contains(target)) continue;
            if (utilities.commonPrimalJunction(target, node) == mapWrappers.get(node).commonPrimalJunction) 
            	continue;
	    	
            EdgeGraph d = null;
            d = Graph.getEdgeBetween(node, target);
	    	double segmentCost = d.getDeflectionAngle() + state.fromNormalDistribution(0, 5);                
            if (segmentCost > 180) segmentCost = 180;
            if (segmentCost < 0) segmentCost = 0;
            List<Integer> positiveBarriers = target.primalEdge.positiveBarriers;
            if (barrierID != 999999 && positiveBarriers != null && positiveBarriers.contains(barrierID))
            	segmentCost = segmentCost * 0.90;
	    	GeomPlanarGraphDirectedEdge lastSegment = (GeomPlanarGraphDirectedEdge) d.getDirEdge(0);
	   
        	double nodeCost = segmentCost;
        	double tentativeCost = getBest(node) + nodeCost;
	    	if (getBest(target) > tentativeCost)
    		{
    			DualNodeWrapper dualNodeWrapper = mapWrappers.get(target);
                if (dualNodeWrapper == null) dualNodeWrapper = new DualNodeWrapper(target);
                dualNodeWrapper.nodeFrom = node;
                dualNodeWrapper.edgeFrom = lastSegment;
                dualNodeWrapper.commonPrimalJunction = utilities.commonPrimalJunction(node, target);
                dualNodeWrapper.gx = tentativeCost;
                mapWrappers.put(target, dualNodeWrapper);
                unvisitedNodes.add(target);
    		}
	     }
	}


	private NodeGraph getClosest(ArrayList<NodeGraph> nodes) //amongst unvisited (they have to have been explored)
	{
		NodeGraph closest = null;
		for (NodeGraph node : nodes) 
		{
			if (closest == null) closest = node;
			else 
			{
				if (getBest(node) < getBest(closest)) closest = node;
			}
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
		
		HashMap<NodeGraph, DualNodeWrapper> mapTraversedWrappers =  new HashMap<NodeGraph, DualNodeWrapper>();
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
        path.dualMapWrappers = mapTraversedWrappers;
	    return path;
	 }
}


