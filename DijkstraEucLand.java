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
import java.util.Map.Entry;
import com.vividsolutions.jts.planargraph.Node;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.GeomPlanarGraphEdge;

public class DijkstraEucLand {
    
	Node destinationNode;
	ArrayList<Node> visitedNodes;
	ArrayList<Node> unvisitedNodes;
	HashMap<Node, Double> costsMap = new HashMap<Node, Double>();
	HashMap<Node, NodeWrapper> mapWrappers =  new HashMap<Node, NodeWrapper>();
    pedestrianSimulation state;

    
    public ArrayList<GeomPlanarGraphDirectedEdge> dijkstraPath (Node originNode, Node destinationNode, 
    		pedestrianSimulation state)
	{
    	this.destinationNode = destinationNode;
    	this.state = state;
		visitedNodes = new ArrayList<Node>();
		unvisitedNodes = new ArrayList<Node>();
		costsMap.put(originNode, 0.0);
		unvisitedNodes.add(originNode);
        
		NodeWrapper nodeWrapper = new NodeWrapper(originNode);
        mapWrappers.put(originNode, nodeWrapper);

		while (unvisitedNodes.size() > 0) 
		{
			
			Node node = getClosest(unvisitedNodes); // at the beginning it takes originNode
			if (node == null) System.out.println("the node is null");
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
//            int nodesSoFar = mapWrappers.get(node).nodesSoFar+1;
//            double pathLandmarkness = mapWrappers.get(node).pathLandmarkness;
//            double pathCost = mapWrappers.get(node).pathCost;

	    	GeomPlanarGraphEdge d = null;
	    	Collection edgesBetween = Node.getEdgesBetween(node, target); //should be one
	    	for (Object o : edgesBetween) d = (GeomPlanarGraphEdge) o;
            double error = state.fromNormalDistribution(1, 0.10);
            if (error < 0) error = 0.00;
	    	double segmentCost = d.getDoubleAttribute("length")*error;

	    	GeomPlanarGraphDirectedEdge lastSegment = (GeomPlanarGraphDirectedEdge) d.getDirEdge(0);
            double globalLandmarkness = landmarkFunctions.globalLandmarknessNode(target, destinationNode, false, state);
        	double nodeLandmarkness = 1-(globalLandmarkness*0.5);
        	double nodeCost = segmentCost*nodeLandmarkness;
        	
        	double tentativeCost = getBest(node) + nodeCost;
//        	if (costsMap.get(node) < 1000) System.out.println(" null? c"+costsMap.get(node));
	    	if (getBest(target) > tentativeCost)
	    	{
	    		
	    		NodeWrapper nodeWrapper = mapWrappers.get(target);
                if (nodeWrapper == null) nodeWrapper = new NodeWrapper(target);
                nodeWrapper.nodeFrom = node;
                nodeWrapper.edgeFrom = lastSegment;
//                nodeWrapper.nodesSoFar = nodesSoFar;
//                nodeWrapper.pathCost = pathCost + segmentCost;
//                nodeWrapper.nodeLandmarkness = nodeLandmarkness;
//                nodeWrapper.pathLandmarkness = pathLandmarkness + nodeLandmarkness;

                mapWrappers.put(target, nodeWrapper);
                costsMap.put(target, tentativeCost);


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
		Double d = costsMap.get(target);
	    if (d == null) return Double.MAX_VALUE;
	    else return d;
	}


	ArrayList<GeomPlanarGraphDirectedEdge> reconstructPath(Node originNode, Node destinationNode) 
	{
		ArrayList<GeomPlanarGraphDirectedEdge> path = new ArrayList<GeomPlanarGraphDirectedEdge>();
		Node step = destinationNode;
		while (mapWrappers.get(step).nodeFrom != null)
    	{
			GeomPlanarGraphDirectedEdge dd = mapWrappers.get(step).edgeFrom;
			step = mapWrappers.get(step).nodeFrom;
            path.add(0, dd);
            if (step == originNode) break;
        }
		if (path.size() == 0)  System.out.println("DE_path hasn't worked "+originNode.getData()+ " "+destinationNode.getData()); 
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


