/**
 ** Dikstra for shortest-Euclidean distance path 
 **
 ** Copyright 2019 by Gabriele Filomena
 ** Licensed under the Academic Free License version 3.0
 **
 **/

package sim.app.geo.pedestrianSimulation;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import com.vividsolutions.jts.planargraph.Node;
import sim.util.geo.GeomPlanarGraph;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.GeomPlanarGraphEdge;

public class DijkstraEuclidean {
    
	Node destinationNode;
	private ArrayList<Node> visitedNodes;
	private ArrayList<Node> unvisitedNodes;
	private HashMap<Node, Node> predecessors;
	private HashMap<Node, Double> costsMap = new HashMap<Node, Double>();
    private GeomPlanarGraph network = new GeomPlanarGraph();
	HashMap<Integer, NodeData> nodesMap;
    HashMap<Node, nodeWrapper> mapWrappers =  new HashMap<Node, nodeWrapper>();
    
    public ArrayList<GeomPlanarGraphDirectedEdge> dijkstraPath (Node originNode, Node destinationNode, pedestrianSimulation state)
	{
    	this.destinationNode = destinationNode;
    	this.nodesMap = state.nodesMap;
		this.network = state.network;
		
		visitedNodes = new ArrayList<Node>();
		unvisitedNodes = new ArrayList<Node>();
		predecessors = new HashMap<Node, Node>();
		costsMap.put(originNode, 0.0);
		unvisitedNodes.add(originNode);
        
		nodeWrapper nodeWrapper = new nodeWrapper(originNode);
        mapWrappers.put(originNode, nodeWrapper);

		while (unvisitedNodes.size() > 0) 
		{
			Node node = getClosest(unvisitedNodes); // at the beginning it takes originNode
//			node.setVisited(true);
			visitedNodes.add(node);
			unvisitedNodes.remove(node);
			findMinDistances(node);
		}
		return reconstructPath(originNode, destinationNode);
	}

	private void findMinDistances(Node node) 
	{
		ArrayList<Node> adjacentNodes = utilities.getAdjacentNodes(node, network);        
	    for (Node target : adjacentNodes) 
	    {    
	    	if (visitedNodes.contains(target)) continue;

	    	GeomPlanarGraphEdge d = null;
	    	Collection edgesBetween = Node.getEdgesBetween(node, target); //should be one
	    	for (Object o : edgesBetween) d = (GeomPlanarGraphEdge) o;
	    	double segmentCost = d.getDoubleAttribute("length");
	    	GeomPlanarGraphDirectedEdge lastSegment = (GeomPlanarGraphDirectedEdge) d.getDirEdge(0);
        	double tentativeCost = getBest(node) + segmentCost;

	    	if (getBest(target) > tentativeCost)
	    		{
	                nodeWrapper nodeWrapper = new nodeWrapper(target);
	                nodeWrapper.cameFrom = node;
	                nodeWrapper.edgeFrom = lastSegment;
	                mapWrappers.put(target, nodeWrapper);
	                costsMap.put(target, tentativeCost);
	                predecessors.put(target, node);
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

	private Double getBest(Node target)
	{
		Double d = costsMap.get(target);
	    if (d == null) return Double.MAX_VALUE;
	    else return d;
	}


	public ArrayList<GeomPlanarGraphDirectedEdge> reconstructPath(Node originNode, Node destinationNode) 
	{
		ArrayList<GeomPlanarGraphDirectedEdge> path = new ArrayList<GeomPlanarGraphDirectedEdge>();
		Node step = destinationNode;
		while (predecessors.get(step) != null)
	    	{
				GeomPlanarGraphDirectedEdge dd = mapWrappers.get(step).edgeFrom;
	        	Node previous = predecessors.get(step);
	            path.add(0, dd);
	            step = previous;
	            if (step == originNode) break;
            
//        	  GeomPlanarGraphEdge d = null;
//            Collection edgesBetween = Node.getEdgesBetween(previous, step);
//            for (Object o : edgesBetween) d = (GeomPlanarGraphEdge) o;
//            GeomPlanarGraphDirectedEdge dd = (GeomPlanarGraphDirectedEdge) d.getDirEdge(0);
	        }
		
		if (path.size() == 0) 
		{
		System.out.println("problems with Euclidean Dijstra "+originNode.getData()+ " "+destinationNode.getData());
		System.out.println("map "+ mapWrappers.size()+ " unvis "+unvisitedNodes.size()+" visited " +visitedNodes.size());
		
		}
	    return path;
	    }


	class nodeWrapper
	{
	
	    // the underlying Node associated with the metainformation
	    Node node;
	    // the Node from which this Node was most profitably linked
	    Node cameFrom;
	    // the edge by which this Node was discovered
	    GeomPlanarGraphDirectedEdge edgeFrom;

	    public nodeWrapper(Node n)
	    {
	        node = n;
	        cameFrom = null;
	        edgeFrom = null;
	    }
	}
}


