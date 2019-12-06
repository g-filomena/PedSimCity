/**
 ** Dikstra for shortest-cumulative angular-change path 
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

public class DijkstraAngular {
    
	Node destinationNode, primalDestinationNode;
	private ArrayList<Node> visitedNodes;
	private ArrayList<Node> unvisitedNodes;
	private HashMap<Node, Node> predecessors;
	private HashMap<Node, Double> costsMap = new HashMap<Node, Double>();
    private GeomPlanarGraph dualNetwork = new GeomPlanarGraph();
	
	HashMap<Integer, NodeData> nodesMap;
	HashMap<Integer, EdgeData> edgesMap;
	HashMap<Integer, CentroidData> centroidsMap;
	
    HashMap<Node, nodeWrapper> mapWrappers =  new HashMap<Node, nodeWrapper>();
    
    public ArrayList<GeomPlanarGraphDirectedEdge> dijkstraPath (Node originNode, Node destinationNode, pedestrianSimulation state)
	{
    	this.destinationNode = destinationNode;
    	this.edgesMap = state.edgesMap;
    	this.nodesMap = state.nodesMap;
    	this.centroidsMap = state.centroidsMap;

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
			visitedNodes.add(node);
			unvisitedNodes.remove(node);
			findMinDistances(node);
		}
		
		return reconstructPath(originNode, destinationNode);
	}

	private void findMinDistances(Node node) 
	{
		ArrayList<Node> adjacentNodes = utilities.getAdjacentNodes(node, dualNetwork);        
	    for (Node target : adjacentNodes) 
	    {    
	    	if (visitedNodes.contains(target)) continue;
            
	    	GeomPlanarGraphEdge d = null;
	    	Collection edgesBetween = Node.getEdgesBetween(node, target); //should be one
	    	for (Object o : edgesBetween) d = (GeomPlanarGraphEdge) o;
	    	double segmentCost = d.getDoubleAttribute("rad");
	    	GeomPlanarGraphDirectedEdge lastSegment = (GeomPlanarGraphDirectedEdge) d.getDirEdge(0);
        	double tentativeCost = getBest(node) + segmentCost;
	    	
	    	if (getBest(target) > tentativeCost)
	    		{
	                nodeWrapper nodeWrapper = new nodeWrapper(target);
	                nodeWrapper.cameFrom = node;
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
		while (step != null)
	    	{
				int edgeID = (int)step.getData(); //extract edgeID
				GeomPlanarGraphDirectedEdge dd = (GeomPlanarGraphDirectedEdge) edgesMap.get(edgeID).planarEdge.getDirEdge(0);
	        	step = predecessors.get(step);
	            path.add(0, dd);
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

	    public nodeWrapper(Node n)
	    {
	        node = n;
	        cameFrom = null;
	    }
	}
}


