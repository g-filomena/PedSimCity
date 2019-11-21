/**
 ** Dikstra for Landmark Maximisation
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
import com.vividsolutions.jts.planargraph.Node;
import sim.util.geo.GeomPlanarGraph;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.GeomPlanarGraphEdge;

public class DijkstraLand {
    
	Node destinationNode;
	private ArrayList<Node> visitedNodes;
	private ArrayList<Node> unvisitedNodes;
	private HashMap<Node, Node> predecessors;
	private HashMap<Node, Double> costsMap = new HashMap<Node, Double>();
    private GeomPlanarGraph network = new GeomPlanarGraph();
	HashMap<Integer, NodeData> nodesMap;
    HashMap<Node, nodeWrapper> mapWrappers =  new HashMap<Node, nodeWrapper>();
    double wL, wG, t;
    
    public ArrayList<GeomPlanarGraphDirectedEdge> dijkstraLandmarkPath (Node originNode, Node destinationNode, 
    		double wL, double wG, pedestrianSimulation state)
	{
    	this.destinationNode = destinationNode;
    	this.nodesMap = state.nodesMap;
		this.network = state.network;
		this.wL = wL;
		this.wG = wG;
		this.t = state.t;
		
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
            int nodesSoFar = mapWrappers.get(node).nodesSoFar+1;
            double pathLandmarkness = mapWrappers.get(node).pathLandmarkness;
            
	    	GeomPlanarGraphEdge d = null;
	    	Collection edgesBetween = Node.getEdgesBetween(node, target); //should be one
	    	for (Object o : edgesBetween) d = (GeomPlanarGraphEdge) o;
            
	    	GeomPlanarGraphDirectedEdge lastSegment = (GeomPlanarGraphDirectedEdge) d.getDirEdge(0);
	    	double localLandmarkness = localLandmarkness(target, node, mapWrappers, lastSegment);
            double globalLandmarkness = globalLandmarkness(target, destinationNode);
        	double nodeLandmarkness = localLandmarkness*wL + globalLandmarkness*wG;
        	double nodeCost = (pathLandmarkness+nodeLandmarkness)/nodesSoFar;
        	double tentativeCost = nodeCost; 

	    	if (getBest(target) > tentativeCost)
	    		{
	                nodeWrapper nodeWrapper = new nodeWrapper(target);
	                nodeWrapper.nodeFrom = node;
	                nodeWrapper.edgeFrom = lastSegment;
	                nodeWrapper.nodesSoFar = nodesSoFar;
	                nodeWrapper.nodeLandmarkness = nodeLandmarkness;
	                nodeWrapper.pathLandmarkness = pathLandmarkness + nodeLandmarkness;
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
	            
//	        	GeomPlanarGraphEdge d = null;
//	            Collection edgesBetween = Node.getEdgesBetween(previous, step);
//	            for (Object o : edgesBetween) d = (GeomPlanarGraphEdge) o;
//	            GeomPlanarGraphDirectedEdge dd = (GeomPlanarGraphDirectedEdge) d.getDirEdge(0);
	        }
	    return path;
	    
	 }


	class nodeWrapper
	{
	
	    // the underlying Node associated with the metainformation
	    Node node;
	    // the Node from which this Node was most profitably linked
	    Node nodeFrom;
	    // the edge by which this Node was discovered
	    GeomPlanarGraphDirectedEdge edgeFrom;
	    
	    int nodesSoFar;
	    double pathLength, nodeLandmarkness, pathLandmarkness;
	
	    public nodeWrapper(Node n)
	    {
	        node = n;
	        nodeFrom = null;
	        edgeFrom = null;
	        pathLength = 0.0;
	        nodeLandmarkness = 0.0;
	        pathLandmarkness = 0.0;
	    }
	}
	
	
    double localLandmarkness(Node target, Node current, HashMap<Node, nodeWrapper> foundNodes, 
    		GeomPlanarGraphDirectedEdge lastSegment) 
    {   	
    	Node targetNode =  target;              
        Integer nodeID = (Integer) targetNode.getData();
        NodeData nd = nodesMap.get(nodeID);
        List<Integer> localLandmarks = new ArrayList<Integer>();
        localLandmarks = nd.localLandmarks;
        double localScore = 0.0;
        
        if (localLandmarks == null) return 1.0;
        else
        {
        	for (int i = 0; i < localLandmarks.size(); i++)
	        {
	            Node nodeTo =  target;
	            Node nodeFrom = current;
	            double distanceTravelled = 0;
	            double cumulativeAdvanceVis = 0;
	            if (nd.localScores.get(i) < 0.30 ) continue;
	            while ((nodeFrom != null) & (distanceTravelled <= t))
	            {
	            	
	            	Integer nodeIDLoop = (Integer) nodeFrom.getData();
	                NodeData ndLoop = nodesMap.get(nodeIDLoop);
	                List<Integer> visible = new ArrayList<Integer>();
	                visible = ndLoop.visible2d;
	                GeomPlanarGraphEdge segment;
	                if (nodeTo == targetNode) segment = (GeomPlanarGraphEdge) lastSegment.getEdge();
	                else
	                {
	                	nodeWrapper nt = foundNodes.get(nodeTo);
	                	segment = (GeomPlanarGraphEdge) nt.edgeFrom.getEdge();
	                }
	                distanceTravelled += segment.getDoubleAttribute("length");	                
	                if (visible.contains(localLandmarks.get(i))) cumulativeAdvanceVis += segment.getDoubleAttribute("length");
	                nodeTo = nodeFrom;
	                nodeWrapper nf = foundNodes.get(nodeFrom);
	                try {nodeFrom = nf.nodeFrom;}
	                catch (java.lang.NullPointerException e) {nodeFrom = null;}
	            }
	              
	          double aV = cumulativeAdvanceVis/distanceTravelled;
	          if (aV > 1.0) aV = 1.0;
	          double tmp = nd.localScores.get(i) * aV;
	          if (tmp > localScore) localScore = tmp;
	        }
        	return (1.0 -localScore);
        }
    }
        
    double globalLandmarkness(Node target, Node destination) 
    {   	
       Node targetNode =  target;              
       Integer nodeID = (Integer) targetNode.getData();
       NodeData nd = nodesMap.get(nodeID);
       List<Integer> distantLandmarks = new ArrayList<Integer>();
       distantLandmarks = nd.distantLandmarks;
       if (distantLandmarks == null) return 1.0;
    		
   	   // destination segment: identifying node
       Node destNode = destination;
       Integer nodeIDdest = (Integer) destNode.getData();
       NodeData dd = nodesMap.get(nodeIDdest);
       List<Integer> anchors = new ArrayList<Integer>();
       anchors = dd.anchors;
       if (anchors == null) return 1.0;

       double globalScore = 0.0;
       for (int i = 0; i < distantLandmarks.size(); i++)
       {
    	   double tmp = 0.0;
    	   if (anchors.contains(distantLandmarks.get(i)))
    	   {
    		   tmp = nd.distantScores.get(i);
    		   double distance = dd.distances.get(anchors.lastIndexOf(distantLandmarks.get(i)));
    		   double distanceWeight = utilities.nodesDistance(target, destination)/distance;
    		   if (distanceWeight > 1.0) distanceWeight = 1.0;
    		   tmp = tmp*distanceWeight;
    	   }
    	   if (tmp > globalScore) globalScore = tmp;
       }
	   return 1.0 - globalScore;
    }

}


