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
import java.util.List;
import java.util.Map.Entry;
import com.vividsolutions.jts.planargraph.Node;

import sim.app.geo.pedestrianSimulation.utilities.Path;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.GeomPlanarGraphEdge;

public class DijkstraGlobalLandmarks {
    
	Node destinationNode;
	ArrayList<Node> visitedNodes;
	ArrayList<Node> unvisitedNodes;
	HashMap<Node, NodeWrapper> mapWrappers =  new HashMap<Node, NodeWrapper>();
    PedestrianSimulation state;
    ArrayList<Integer> segmentsToAvoid = new ArrayList<Integer>();
    
    public Path dijkstraPath (Node originNode, Node destinationNode,
    		ArrayList<Integer> segmentsToAvoid, PedestrianSimulation state)
	{
    	this.segmentsToAvoid = segmentsToAvoid;
    	this.destinationNode = destinationNode;
    	this.state = state;
		visitedNodes = new ArrayList<Node>();
		unvisitedNodes = new ArrayList<Node>();
		unvisitedNodes.add(originNode);
        
		NodeWrapper nodeWrapper = new NodeWrapper(originNode);
		nodeWrapper.gx = 0.0;
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

	void findMinDistances(Node node) 
	{
		ArrayList<Node> adjacentNodes = utilities.getAdjacentNodes(node);   
	    for (Node target : adjacentNodes) 
	    {    
	    	
	    	if (visitedNodes.contains(target)) continue;	
	    	GeomPlanarGraphEdge d = null;
	    	Collection edgesBetween = Node.getEdgesBetween(node, target); //should be one
	    	for (Object o : edgesBetween) d = (GeomPlanarGraphEdge) o;
	    	GeomPlanarGraphDirectedEdge lastSegment = (GeomPlanarGraphDirectedEdge) d.getDirEdge(0);

			if (segmentsToAvoid == null);
            else if (segmentsToAvoid.contains(((GeomPlanarGraphEdge) lastSegment.getEdge()).getIntegerAttribute("edgeID"))) 
            	continue;
	    	
            double globalLandmarkness = landmarkFunctions.globalLandmarknessNode(target, destinationNode, false, state);
        	double nodeLandmarkness = (1-globalLandmarkness)/d.getDoubleAttribute("length");
        	
        	double tentativeCost = getBest(node) + nodeLandmarkness;

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


	Node getClosest(ArrayList<Node> nodes) //amongst unvisited (they have to have been explored)
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


	Path reconstructPath(Node originNode, Node destinationNode) 
	{
		HashMap<Node, NodeWrapper> mapTraversedWrappers =  new HashMap<Node, NodeWrapper>();
		ArrayList<GeomPlanarGraphDirectedEdge> sequenceEdges = new ArrayList<GeomPlanarGraphDirectedEdge>();
		Node step = destinationNode;
		mapTraversedWrappers.put(destinationNode, mapWrappers.get(destinationNode));
		
		while (mapWrappers.get(step).nodeFrom != null)
    	{
			GeomPlanarGraphDirectedEdge dd = mapWrappers.get(step).edgeFrom;
			step = mapWrappers.get(step).nodeFrom;
			sequenceEdges.add(0, dd);
			mapTraversedWrappers.put(step, mapWrappers.get(step));
        }
		if (sequenceEdges.size() == 0)  System.out.println("DE_path hasn't worked "+originNode.getData()+ " "+destinationNode.getData()); 
		
        Path path = new Path();
        path.edges = sequenceEdges;
        path.mapWrappers = mapTraversedWrappers;
	    return path;
    }
	
	public Node getKeysByValue(HashMap<Node, Node> map, Node node) 
	{
        for (Entry<Node, Node> entry : map.entrySet()) 
        {
            if (entry.getValue().equals(node)) return entry.getKey();
        }
        return null;
	}
}


