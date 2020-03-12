package sim.app.geo.pedestrianSimulation;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import com.vividsolutions.jts.planargraph.Node;

import sim.app.geo.pedestrianSimulation.utilities.Path;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.GeomPlanarGraphEdge;

public class DijkstraAngularChange {
    
	Node originNode, destinationNode;
	ArrayList<Node> visitedNodes;
	ArrayList<Node> unvisitedNodes;
	Node previousJunction;

    HashMap<Node, DualNodeWrapper> mapWrappers =  new HashMap<Node, DualNodeWrapper>();
    ArrayList<Node> centroidsToAvoid = new ArrayList<Node>();
    PedestrianSimulation state;
    
    public Path dijkstraPath (Node originNode, Node destinationNode, ArrayList<Node> centroidsToAvoid, 
    		Node previousJunction, PedestrianSimulation state)
	{
    	this.originNode = originNode;
    	this.destinationNode = destinationNode;
    	this.centroidsToAvoid = centroidsToAvoid;
    	this.state = state;
		
		visitedNodes = new ArrayList<Node>();
		unvisitedNodes = new ArrayList<Node>();
		unvisitedNodes.add(originNode);
		
		DualNodeWrapper dualNodeWrapper = new DualNodeWrapper(originNode);
		dualNodeWrapper.gx = 0.0;
		if (previousJunction != null) dualNodeWrapper.commonPrimalJunction = (int) previousJunction.getData();
        mapWrappers.put(originNode, dualNodeWrapper);
        
        if (centroidsToAvoid != null) for (Node c : centroidsToAvoid) visitedNodes.add(c);
		while (unvisitedNodes.size() > 0) 
		{
			Node node = getClosest(unvisitedNodes); // at the beginning it takes originNode
			visitedNodes.add(node);
			unvisitedNodes.remove(node);
			findMinDistances(node);
		}
		
		return reconstructPath(originNode, destinationNode);
	}

	private void findMinDistances(Node node) 
	{
		ArrayList<Node> adjacentNodes = utilities.getAdjacentNodes(node);        
	    for (Node target : adjacentNodes) 
	    {    
	    	if (visitedNodes.contains(target)) continue;
            if (utilities.commonPrimalJunction(target, node, state) == mapWrappers.get(node).commonPrimalJunction) continue;
	    	
            GeomPlanarGraphEdge d = null;
	    	Collection edgesBetween = Node.getEdgesBetween(node, target); //should be one
	    	for (Object o : edgesBetween) d = (GeomPlanarGraphEdge) o;
	    	double segmentCost = d.getDoubleAttribute("deg") + state.fromNormalDistribution(0, 5);                
            if (segmentCost > 180) segmentCost = 180;
            if (segmentCost < 0) segmentCost = 0;
	    	GeomPlanarGraphDirectedEdge lastSegment = (GeomPlanarGraphDirectedEdge) d.getDirEdge(0);
	   
        	double nodeCost = segmentCost;
        	double tentativeCost = getBest(node) + nodeCost;
	    	if (getBest(target) > tentativeCost)
	    		{
	    			DualNodeWrapper dualNodeWrapper = mapWrappers.get(target);
	                if (dualNodeWrapper == null) dualNodeWrapper = new DualNodeWrapper(target);
	                dualNodeWrapper.nodeFrom = node;
	                dualNodeWrapper.edgeFrom = lastSegment;
	                dualNodeWrapper.commonPrimalJunction = utilities.commonPrimalJunction(node, target, state);
	                dualNodeWrapper.gx = tentativeCost;
	                mapWrappers.put(target, dualNodeWrapper);
	                unvisitedNodes.add(target);
	    		}
	     }
	}


	private Node getClosest(ArrayList<Node> nodes) //amongst unvisited (they have to have been explored)
	{
		Node closest = null;
		for (Node node : nodes) 
		{
			if (closest == null) closest = node;
			else 
			{
				if (getBest(node) < getBest(closest)) closest = node;
			}
	     }
	        return closest;
	}

	Double getBest(Node target)
	{
		if (mapWrappers.get(target) == null) return Double.MAX_VALUE;
	    else return mapWrappers.get(target).gx;
	}

	public Path reconstructPath(Node originNode, Node destinationNode) 
	{
        Path path = new Path();
		path.edges = null;
		path.mapWrappers = null;
		
		HashMap<Node, DualNodeWrapper> mapTraversedWrappers =  new HashMap<Node, DualNodeWrapper>();
		ArrayList<GeomPlanarGraphDirectedEdge> sequenceEdges = new ArrayList<GeomPlanarGraphDirectedEdge>();
		Node step = destinationNode;
		mapTraversedWrappers.put(destinationNode, mapWrappers.get(destinationNode));
		if ((step == null) || (mapWrappers.size() == 1)) return path;

		try
		{
			while (mapWrappers.get(step).nodeFrom != null)
			{
				int edgeID = (int)step.getData(); //extract edgeID
				GeomPlanarGraphDirectedEdge de = (GeomPlanarGraphDirectedEdge) state.edgesMap.get(edgeID).planarEdge.getDirEdge(0);
		    	step = mapWrappers.get(step).nodeFrom;
		    	mapTraversedWrappers.put(step, mapWrappers.get(step));
		    	sequenceEdges.add(0, de); 
		    	if (step == originNode)
		    	{
					int lastEdgeID = (int)step.getData(); //extract edgeID
					GeomPlanarGraphDirectedEdge lastDe = (GeomPlanarGraphDirectedEdge) state.edgesMap.get(lastEdgeID).planarEdge.getDirEdge(0);
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


