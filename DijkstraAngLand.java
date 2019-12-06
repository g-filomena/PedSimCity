package sim.app.geo.pedestrianSimulation;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import com.vividsolutions.jts.planargraph.Node;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.GeomPlanarGraphEdge;

public class DijkstraAngLand {
    
	Node destinationNode, primalDestinationNode, originNode;
	ArrayList<Node> visitedNodes;
	ArrayList<Node> unvisitedNodes;
	HashMap<Node, Double> costsMap = new HashMap<Node, Double>();
	HashMap<Integer, NodeData> nodesMap;
	HashMap<Integer, EdgeData> edgesMap;
	HashMap<Integer, CentroidData> centroidsMap;
    HashMap<Node, DualNodeWrapper> mapWrappers =  new HashMap<Node, DualNodeWrapper>();
    pedestrianSimulation state;
    
    public ArrayList<GeomPlanarGraphDirectedEdge> dijkstraPath (Node originNode, Node destinationNode, 
    		Node primalDestinationNode, pedestrianSimulation state)
	{
    	this.originNode = originNode;
    	this.destinationNode = destinationNode;
    	this.primalDestinationNode = primalDestinationNode;		
    	this.edgesMap = state.edgesMap;
    	this.nodesMap = state.nodesMap;
    	this.centroidsMap = state.centroidsMap;
		this.state = state;
		
		visitedNodes = new ArrayList<Node>();
		unvisitedNodes = new ArrayList<Node>();
		costsMap.put(originNode, 0.0);
		unvisitedNodes.add(originNode);
				
		DualNodeWrapper DualNodeWrapper = new DualNodeWrapper(originNode);
        mapWrappers.put(originNode, DualNodeWrapper);
        
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
//            int nodesSoFar = mapWrappers.get(node).nodesSoFar+1;
//            double pathLandmarkness = mapWrappers.get(node).pathLandmarkness;
//            double pathCost = mapWrappers.get(node).pathCost;
            
	    	GeomPlanarGraphEdge d = null;
	    	Collection edgesBetween = Node.getEdgesBetween(node, target); //should be one
	    	for (Object o : edgesBetween) d = (GeomPlanarGraphEdge) o;
	    	double segmentCost = d.getDoubleAttribute("deg") + state.fromNormalDistribution(0, 5);                
            if (segmentCost > 180) segmentCost = 180;
            if (segmentCost < 0) segmentCost = 0;
	    	GeomPlanarGraphDirectedEdge lastSegment = (GeomPlanarGraphDirectedEdge) d.getDirEdge(0);
	    	
            double globalLandmarkness = landmarkFunctions.globalLandmarknessDualNode(target, destinationNode, false, state);
        	double nodeLandmarkness = 1-globalLandmarkness*0.5;

//        	double nodeCost = ((pathLandmarkness+nodeLandmarkness)/nodesSoFar) * ((pathCost+segmentCost)/nodesSoFar);
        	double nodeCost = segmentCost*nodeLandmarkness;
        	double tentativeCost = getBest(node) + nodeCost;
	    	
	    	if (getBest(target) > tentativeCost)
	    		{
	    			DualNodeWrapper dualNodeWrapper = mapWrappers.get(target);
	                if (dualNodeWrapper == null) dualNodeWrapper = new DualNodeWrapper(target);
	                dualNodeWrapper.nodeFrom = node;
	                dualNodeWrapper.edgeFrom = lastSegment;
//	                DualNodeWrapper.nodesSoFar = nodesSoFar;
//	                DualNodeWrapper.pathCost = pathCost + segmentCost;
//	                dualNodeWrapper.nodeLandmarkness = nodeLandmarkness;
//	                DualNodeWrapper.pathLandmarkness = pathLandmarkness + nodeLandmarkness;
	                mapWrappers.put(target, dualNodeWrapper);
	                costsMap.put(target, tentativeCost);
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
		while (mapWrappers.get(step).nodeFrom != null)
	    	{
				int edgeID = (int)step.getData(); //extract edgeID
				GeomPlanarGraphDirectedEdge dd = (GeomPlanarGraphDirectedEdge) edgesMap.get(edgeID).planarEdge.getDirEdge(0);
	        	step = mapWrappers.get(step).nodeFrom;
	            path.add(0, dd); 
	        }
		
		if (path.size() == 0) System.out.println("DA_path hasn't worked "+originNode.getData()+ " "+destinationNode.getData());
	    return path;
	    }
}


