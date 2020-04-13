/**
 ** Dikstra for shortest-Euclidean distance path 
 **
 ** Copyright 2019 by Gabriele Filomena
 ** Licensed under the Academic Free License version 3.0
 **
 **/

package sim.app.geo.pedestrianSimulation;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;

import sim.app.geo.pedestrianSimulation.utilities.Path;
import sim.util.geo.GeomPlanarGraphDirectedEdge;


public class DijkstraRoadDistance {
    
	NodeGraph destinationNode;
	ArrayList<NodeGraph> visitedNodes;
	ArrayList<NodeGraph> unvisitedNodes;
	HashMap<NodeGraph, NodeWrapper> mapWrappers =  new HashMap<NodeGraph, NodeWrapper>();
    PedestrianSimulation state;
    ArrayList<GeomPlanarGraphDirectedEdge> segmentsToAvoid = new ArrayList<GeomPlanarGraphDirectedEdge>();
	int barrierID;
    
    public Path dijkstraPath(NodeGraph originNode, NodeGraph destinationNode,
    		ArrayList<GeomPlanarGraphDirectedEdge> segmentsToAvoid, boolean regionalRouting,
    		int barrierID, PedestrianSimulation state)
	{
    	this.segmentsToAvoid = segmentsToAvoid;
    	this.destinationNode = destinationNode;
    	this.state = state;
    	this.barrierID = barrierID;
		visitedNodes = new ArrayList<NodeGraph>();
		unvisitedNodes = new ArrayList<NodeGraph>();
		unvisitedNodes.add(originNode);
        
		NodeWrapper nodeWrapper = new NodeWrapper(originNode);
		nodeWrapper.gx = 0.0;
        mapWrappers.put(originNode, nodeWrapper);

		while (unvisitedNodes.size() > 0) 
		{
			NodeGraph node = getClosest(unvisitedNodes); // at the beginning it takes originNode
			visitedNodes.add(node);
			unvisitedNodes.remove(node);
			findMinDistances(node);
		}
		return reconstructPath(originNode, destinationNode);
	}

	void findMinDistances(NodeGraph node) 
	{
		ArrayList<NodeGraph> adjacentNodes = utilities.getAdjacentNodes(node);   
	    for (NodeGraph target : adjacentNodes) 
	    {    
	    	if (visitedNodes.contains(target)) continue;	
            EdgeGraph d = null;
            d = Graph.getEdgeBetween(node, target);
            double error = state.fromNormalDistribution(1, 0.10);
            if (error < 0) error = 0.00;
	    	double segmentCost = d.getLength()*error;
            List<Integer> positiveBarriers = d.positiveBarriers;
            if (barrierID != 999999 && positiveBarriers != null && positiveBarriers.contains(barrierID))
	        	segmentCost = segmentCost * 0.90;
	    	
	    	GeomPlanarGraphDirectedEdge lastSegment = (GeomPlanarGraphDirectedEdge) d.getDirEdge(0);

			if (segmentsToAvoid == null);
            else if (segmentsToAvoid.contains(lastSegment)) continue;

        	double tentativeCost = getBest(node) + segmentCost;
	    	if (getBest(target) > tentativeCost)
	    	{
	    		NodeWrapper nodeWrapper = mapWrappers.get(target);
                if (nodeWrapper == null) nodeWrapper = new NodeWrapper(target);
                nodeWrapper.nodeFrom = node;
                nodeWrapper.edgeFrom = lastSegment;
                nodeWrapper.gx = tentativeCost;
                mapWrappers.put(target, nodeWrapper);
                unvisitedNodes.add(target);
	    	}
	     }
	}


	NodeGraph getClosest(ArrayList<NodeGraph> nodes) //amongst unvisited (they have to have been explored)
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


	Path reconstructPath(NodeGraph originNode, NodeGraph destinationNode) 
	
	{
        Path path = new Path();
		path.edges = null;
		path.mapWrappers = null;
		
		HashMap<NodeGraph, NodeWrapper> mapTraversedWrappers =  new HashMap<NodeGraph, NodeWrapper>();
		ArrayList<GeomPlanarGraphDirectedEdge> sequenceEdges = new ArrayList<GeomPlanarGraphDirectedEdge>();
		NodeGraph step = destinationNode;
		mapTraversedWrappers.put(destinationNode, mapWrappers.get(destinationNode));
		
		if ((step == null) || (mapWrappers.size() == 1)) return path; //no path
		try 
		{
			while (mapWrappers.get(step).nodeFrom != null)
	    	{
				GeomPlanarGraphDirectedEdge dd = mapWrappers.get(step).edgeFrom;
				step = mapWrappers.get(step).nodeFrom;
				sequenceEdges.add(0, dd);
				mapTraversedWrappers.put(step, mapWrappers.get(step));
	        }
		}
		catch(java.lang.NullPointerException e)	{return path;} //no path
        path.edges = sequenceEdges;
        path.mapWrappers = mapTraversedWrappers;
	    return path;
    }
	
	public NodeGraph getKeysByValue(HashMap<NodeGraph, NodeGraph> map, NodeGraph node) 
	{
        for (Entry<NodeGraph, NodeGraph> entry : map.entrySet()) 
        {
            if (entry.getValue().equals(node)) return entry.getKey();
        }
        return null;
	}
}


