/**
 ** AStar.java
 **
 ** Copyright 2011 by Sarah Wise, Mark Coletti, Andrew Crooks, and
 ** George Mason University.
 **
 ** Licensed under the Academic Free License version 3.0
 **
 ** See the file "LICENSE" for more information
 **
 ** $Id: AStar.java 842 2012-12-18 01:09:18Z mcoletti $
 **/
package sim.app.geo.pedestrianSimulation;

import java.util.List;

import org.javatuples.Pair;
import com.vividsolutions.jts.planargraph.DirectedEdgeStar;
import com.vividsolutions.jts.planargraph.Node;
import java.util.ArrayList;
import java.util.HashMap;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.GeomPlanarGraphEdge;




@SuppressWarnings("restriction")
public class AStarEucLand
{
	
	HashMap<Integer, EdgeData> edgesMap;
	HashMap<Integer, NodeData> nodesMap;
    double wL, wG, t;
    public HashMap<Pair, Double> euclideanDistancesMap = new HashMap<Pair, Double>();
	
    public ArrayList<GeomPlanarGraphDirectedEdge> astarPath(Node originNode, Node destinationNode, 
    		double wL, double wG, pedestrianSimulation state)
    {
    	this.edgesMap = state.edgesMap;
    	this.nodesMap = state.nodesMap;
		this.wL = wL;
		this.wG = wG;
    	this.t = state.t;
    	
//        for (int y = 0; y < state.geometriesNodes.size(); y++) //street junctions and betweenness centrality
//        {
//
//        	MasonGeometry geometryNode = (MasonGeometry) state.geometriesNodes.objs[y]; 
//        	Integer nodeID = geometryNode.getIntegerAttribute("nodeID");
//        	if (destinationID == nodeID) continue;
//        	Node node = state.network.findNode(geometryNode.geometry.getCoordinate());
//        	DijkstraEuclidean pathfinderEuclidean = new DijkstraEuclidean();
//        	ArrayList<GeomPlanarGraphDirectedEdge> newPath = pathfinderEuclidean.dijkstraPath(node, destinationNode, state);  
//            double euclideanDistance = utilities.finalDistance(newPath);
//    		Pair<Integer, Integer> pair = new Pair<Integer, Integer> (nodeID, destinationID);
//    		euclideanDistancesMap.put(pair, euclideanDistance);
//        }
    	
    	
        // set up the containers for the result
        ArrayList<GeomPlanarGraphDirectedEdge> result =  new ArrayList<GeomPlanarGraphDirectedEdge>();

        // containers for the metainformation about the Nodes relative to the
        // A* search
        HashMap<Node, NodeWrapper> mapWrappers =  new HashMap<Node, NodeWrapper>();

        NodeWrapper originNodeWrapper = new NodeWrapper(originNode);
        NodeWrapper destinationNodeWrapper = new NodeWrapper(destinationNode);
        mapWrappers.put(originNode, originNodeWrapper);
        mapWrappers.put(destinationNode, destinationNodeWrapper);

        originNodeWrapper.gx = 0;
        originNodeWrapper.hx = 0;
        originNodeWrapper.fx = originNodeWrapper.gx + originNodeWrapper.hx;
        
        // A* containers: nodes to be investigated
        ArrayList<NodeWrapper> closedSet = new ArrayList<NodeWrapper>();
        // nodes that have been investigated
        ArrayList<NodeWrapper> openSet = new ArrayList<NodeWrapper>();
        openSet.add(originNodeWrapper); //adding the originNode Wrapper         
       
        while (openSet.size() > 0)
        { // while there are reachable nodes to investigate
        	
            NodeWrapper currentNodeWrapper = findMin(openSet); // find the shortest path so far
            if (currentNodeWrapper.node == destinationNode) return reconstructPath(destinationNodeWrapper); 
            // we have found the shortest possible path to the goal! Reconstruct the path and send it back.
            openSet.remove(currentNodeWrapper); // maintain the lists
            closedSet.add(currentNodeWrapper);

            // check all the edges out from this Node
            DirectedEdgeStar des = currentNodeWrapper.node.getOutEdges();
            
            for (Object o : des.getEdges().toArray())
            {
                GeomPlanarGraphDirectedEdge lastSegment = (GeomPlanarGraphDirectedEdge) o;
                Node nextNode = null;
                nextNode = lastSegment.getToNode();
                // get the A* meta information about this Node
                NodeWrapper nextNodeWrapper;

                if (mapWrappers.containsKey(nextNode)) nextNodeWrapper =  mapWrappers.get(nextNode);
                else
                {
                	nextNodeWrapper = new NodeWrapper(nextNode);
                	mapWrappers.put(nextNode, nextNodeWrapper);
                }

                if (closedSet.contains(nextNodeWrapper)) continue; // it has already been considered
                double tentativeCost = currentNodeWrapper.gx + scaledLength(lastSegment);
                boolean better = false;

                if (!openSet.contains(nextNodeWrapper))
                {
                	openSet.add(nextNodeWrapper);
                	nextNodeWrapper.hx = 1.0 -((landmarkFunctions.localLandmarkness(nextNode, state) *wL) + 
                			(landmarkFunctions.globalLandmarkness(nextNode, destinationNode, state) *wG));
                    better = true;
                }
                
                else if (tentativeCost < nextNodeWrapper.gx) better = true;

                // store A* information about this promising candidate node
                if (better)
                {
                	nextNodeWrapper.cameFrom = currentNodeWrapper;
                	nextNodeWrapper.edgeFrom = lastSegment;
                	nextNodeWrapper.gx = tentativeCost;
//                	System.out.println("euclidean "+nextNodeWrapper.gx + "  "+ nextNodeWrapper.hx);
                    nextNodeWrapper.fx = nextNodeWrapper.gx + nextNodeWrapper.hx*10*nextNodeWrapper.gx;
                }
            }
        }

        return result;
    }
    

    /**
     * Takes the information about the given node n and returns the path that
     * found it.
     * @param n the end point of the path
     * @return an ArrayList of GeomPlanarGraphDirectedEdges that lead from the
     * given Node to the Node from which the search began
     */
    ArrayList<GeomPlanarGraphDirectedEdge> reconstructPath(NodeWrapper n)
    {
        ArrayList<GeomPlanarGraphDirectedEdge> result =  new ArrayList<GeomPlanarGraphDirectedEdge>();
        NodeWrapper x = n;
        
        while (x.cameFrom != null)
        {
            result.add(0, x.edgeFrom); // add this edge to the front of the list
            x = x.cameFrom;
        }
        return result;
    }
    
    double length(GeomPlanarGraphDirectedEdge lastSegment)
    {
    	GeomPlanarGraphEdge d = (GeomPlanarGraphEdge) lastSegment.getEdge();
    	return d.getLine().getLength();
    }
    
    

    /**
     * 	Considers the list of Nodes open for consideration and returns the node
     *  with minimum fx value
     * @param set list of open Nodes
     * @return
     */
    
    NodeWrapper findMin(ArrayList<NodeWrapper> set)
    {
        double min = 100000;
        NodeWrapper minNode = null;
        for (NodeWrapper n : set)
        {
        	
            if (n.fx < min)
            {
                min = n.fx;
                minNode = n;
            }
        }
        return minNode;
    }


    /**
     * A wrapper to contain the A* meta information about the Nodes
     *
     */
    
    double scaledLength(GeomPlanarGraphDirectedEdge lastIntersection)
    {
    	GeomPlanarGraphEdge d = (GeomPlanarGraphEdge) lastIntersection.getEdge();
    	return d.getDoubleAttribute("length_sc");
    }
            

}