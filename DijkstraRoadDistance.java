/**
 ** Dikstra for shortest-Euclidean distance path 
 **
 ** Copyright 2019 by Gabriele Filomena
 ** Licensed under the Academic Free License version 3.0
 **
 **/

package sim.app.geo.pedSimCity;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;

import sim.app.geo.urbanSim.*;
import sim.app.geo.urbanSim.Utilities.Path;
import sim.util.geo.GeomPlanarGraphDirectedEdge;


public class DijkstraRoadDistance {
    
	NodeGraph destinationNode;
	ArrayList<NodeGraph> visitedNodes;
	ArrayList<NodeGraph> unvisitedNodes;
	HashMap<NodeGraph, NodeWrapper> mapWrappers =  new HashMap<NodeGraph, NodeWrapper>();
    ArrayList<GeomPlanarGraphDirectedEdge> segmentsToAvoid = new ArrayList<GeomPlanarGraphDirectedEdge>();
	boolean barriersRouting;
    
    public Path dijkstraPath(NodeGraph originNode, NodeGraph destinationNode,
    		ArrayList<GeomPlanarGraphDirectedEdge> segmentsToAvoid, boolean regionalRouting, boolean barriersRouting)
	{
    	this.segmentsToAvoid = segmentsToAvoid;
    	this.destinationNode = destinationNode;
    	this.barriersRouting = barriersRouting;
		visitedNodes = new ArrayList<NodeGraph>();
		unvisitedNodes = new ArrayList<NodeGraph>();
		unvisitedNodes.add(originNode);
        
		NodeWrapper nodeWrapper = new NodeWrapper(originNode);
		nodeWrapper.gx = 0.0;
        mapWrappers.put(originNode, nodeWrapper);

		while (unvisitedNodes.size() > 0) 
		{
			NodeGraph currentNode = getClosest(unvisitedNodes); // at the beginning it takes originNode
			visitedNodes.add(currentNode);
			unvisitedNodes.remove(currentNode);
			findMinDistances(currentNode);
		}
		return reconstructPath(originNode, destinationNode);
	}

	void findMinDistances(NodeGraph currentNode) 
	{
		ArrayList<NodeGraph> adjacentNodes = currentNode.getAdjacentNodes();   
	    for (NodeGraph targetNode : adjacentNodes) 
	    {    
	    	if (visitedNodes.contains(targetNode)) continue;	
            EdgeGraph commonEdge = null;
            commonEdge = currentNode.getEdgeBetween(targetNode);
            GeomPlanarGraphDirectedEdge outEdge = (GeomPlanarGraphDirectedEdge) commonEdge.getDirEdge(0);

			if (segmentsToAvoid == null);
            else if (segmentsToAvoid.contains(outEdge))	continue;
            else
            {
            	boolean toContinue = false;
            	for (GeomPlanarGraphDirectedEdge otherEdge : segmentsToAvoid)
            	{
            		EdgeGraph oe = (EdgeGraph) otherEdge.getEdge();
            		if (oe.getID().equals(commonEdge.getID())) 
            		{
            			toContinue = true;
            			break;
            		}
            	}
            	if (toContinue) continue;
            }
			
	    	double error = 0.0;
	    	
	    	List<Integer> positiveBarriers = commonEdge.positiveBarriers;
	    	List<Integer> negativeBarriers = commonEdge.negativeBarriers;
	    	if (barriersRouting) 
	    	{
	    		if (positiveBarriers != null) error = Utilities.fromNormalDistribution(0.70, 0.10, "left");
	    		else if (negativeBarriers != null) error = Utilities.fromNormalDistribution(1.30, 0.10, "right");
	    		else error = Utilities.fromNormalDistribution(1, 0.10, null);
    		}
	    	else error = Utilities.fromNormalDistribution(1, 0.10, null);
	    	double edgeCost = commonEdge.getLength()*error;
	    	

        	double tentativeCost = getBest(currentNode) + edgeCost;
	    	if (getBest(targetNode) > tentativeCost)
	    	{
	    		NodeWrapper nodeWrapper = mapWrappers.get(targetNode);
                if (nodeWrapper == null) nodeWrapper = new NodeWrapper(targetNode);
                nodeWrapper.nodeFrom = currentNode;
                nodeWrapper.edgeFrom = outEdge;
                nodeWrapper.gx = tentativeCost;
                mapWrappers.put(targetNode, nodeWrapper);
                unvisitedNodes.add(targetNode);
	    	}
	     }
	}


	NodeGraph getClosest(ArrayList<NodeGraph> nodes) //amongst unvisited (they have to have been explored)
	{
		NodeGraph closest = null;
		for (NodeGraph node : nodes) 
		{
			if (closest == null) closest = node;
			else if (getBest(node) < getBest(closest)) closest = node;
	     }
	    return closest;
	}

	Double getBest(NodeGraph targetNode)
	{
		if (mapWrappers.get(targetNode) == null) return Double.MAX_VALUE;
	    else return mapWrappers.get(targetNode).gx;
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


