/**
 ** AStarAngularChange.java 
 * This is built on AStar.java 2011 (by Sarah Wise, Mark Coletti, Andrew Crooks)
 * It computes cumulative angular change shortest path by employing the A* algorithm
 * It uses the dual graph of the street network
 **/


package sim.app.geo.pedSimCity;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map.Entry;

import sim.app.geo.urbanSim.*;
import sim.app.geo.urbanSim.Utilities.Path;
import sim.util.geo.GeomPlanarGraphDirectedEdge;


public class DijkstraRoadDistanceLandmarks {
    
	NodeGraph tmpDestinationNode, destinationNode;
	ArrayList<NodeGraph> visitedNodes, unvisitedNodes;
	HashMap<NodeGraph, NodeWrapper> mapWrappers =  new HashMap<NodeGraph, NodeWrapper>();
    ArrayList<GeomPlanarGraphDirectedEdge> segmentsToAvoid = new ArrayList<GeomPlanarGraphDirectedEdge>();
    boolean onlyAnchors; 
    
	/**
	 * 
	 * @param originNode
	 * @param destinationNode
	 * @param centroidsToAvoid
	 * @param previousJunction
	 * @param regionBasedNavigation
	 * @param barrierBasedNavigation
	 * @return
	 */
    public Path dijkstraPath(NodeGraph originNode, NodeGraph tmpDestinationNode, NodeGraph destinationNode,
    		ArrayList<GeomPlanarGraphDirectedEdge> segmentsToAvoid, boolean onlyAnchors)
	{
    	this.segmentsToAvoid = segmentsToAvoid;
    	this.tmpDestinationNode = tmpDestinationNode;
    	this.destinationNode = destinationNode;
    	this.onlyAnchors = onlyAnchors;
    	
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
		return reconstructPath(originNode, tmpDestinationNode);
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

            double error = Utilities.fromNormalDistribution(1, 0.10, null);
            if (error < 0) error = 0.00;
	    	double segmentCost = commonEdge.getLength()*error;
	    	
            double globalLandmarkness = 0.0;
	    	if (onlyAnchors) globalLandmarkness = LandmarkNavigation.globalLandmarknessNode(targetNode, destinationNode, true);
	    	else globalLandmarkness = LandmarkNavigation.globalLandmarknessNode(targetNode, destinationNode, false);

        	double nodeLandmarkness = 1-globalLandmarkness*PedSimCity.globalLandmarknessWeight;
        	double nodeCost = segmentCost*nodeLandmarkness;
        	
        	double tentativeCost = getBest(currentNode) + nodeCost;
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

	Double getBest(NodeGraph target)
	{
		if (mapWrappers.get(target) == null) return Double.MAX_VALUE;
	    else return mapWrappers.get(target).gx;
	}


	Path reconstructPath(NodeGraph originNode, NodeGraph tmpDestinationNode) 
	{
		Path path = new Path();
		path.edges = null;
		path.mapWrappers = null;

		HashMap<NodeGraph, NodeWrapper> mapTraversedWrappers =  new HashMap<NodeGraph, NodeWrapper>();
		ArrayList<GeomPlanarGraphDirectedEdge> sequenceEdges = new ArrayList<GeomPlanarGraphDirectedEdge>();
		NodeGraph step = tmpDestinationNode;
		mapTraversedWrappers.put(tmpDestinationNode, mapWrappers.get(tmpDestinationNode));
		
		if ((step == null) || (mapWrappers.size() == 1))  return path;
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
		catch(java.lang.NullPointerException e) {return path;}

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


