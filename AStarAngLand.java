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


import com.vividsolutions.jts.planargraph.DirectedEdgeStar;
import com.vividsolutions.jts.planargraph.Node;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.GeomPlanarGraphEdge;
import org.javatuples.Pair;


public class AStarAngLand
{
	
	HashMap<Integer, NodeData> nodesMap;
	HashMap<Integer, EdgeData> edgesMap;
	HashMap<Integer, CentroidData> centroidsMap;
    double wL, wG, t;
    public HashMap<Pair, Double> angularDistancesMap = new HashMap<Pair, Double>();
    
    public ArrayList<GeomPlanarGraphDirectedEdge> astarPath(Node originNode, Node destinationNode, Node primalDestinationNode,
    		double wL, double wG, pedestrianSimulation state)
    {
    	this.edgesMap = state.edgesMap;
    	this.nodesMap = state.nodesMap;
    	this.centroidsMap = state.centroidsMap;
		this.wL = wL;
		this.wG = wG;
    	this.t = state.t;

   	
        // set up the containers for the result
        ArrayList<GeomPlanarGraphDirectedEdge> result =  new ArrayList<GeomPlanarGraphDirectedEdge>();

        // containers for the metainformation about the Nodes relative to the
        // A* search
        HashMap<Node, nodeWrapper> mapWrappers =  new HashMap<Node, nodeWrapper>();

        nodeWrapper originNodeWrapper = new nodeWrapper(originNode);
        nodeWrapper destinationNodeWrapper = new nodeWrapper(destinationNode);
        mapWrappers.put(originNode, originNodeWrapper);
        mapWrappers.put(destinationNode, destinationNodeWrapper);

        originNodeWrapper.gx = 0.0;
        originNodeWrapper.hx = 0.0;
        originNodeWrapper.fx = originNodeWrapper.gx + originNodeWrapper.hx;

        // A* containers: nodes to be investigated
        ArrayList<nodeWrapper> closedSet = new ArrayList<nodeWrapper>();
        // nodes that have been investigated
        ArrayList<nodeWrapper> openSet = new ArrayList<nodeWrapper>();
        openSet.add(originNodeWrapper); //adding the startNode Wrapper 

        while (openSet.size() > 0)
        { // while there are reachable nodes to investigate
        	
            nodeWrapper currentNodeWrapper = findMin(openSet); // find the shortest path so far
            if (currentNodeWrapper.node == destinationNode) return reconstructPath(destinationNodeWrapper, centroidsMap, edgesMap);
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
                nodeWrapper nextNodeWrapper;
                
                if (mapWrappers.containsKey(nextNode)) nextNodeWrapper =  mapWrappers.get(nextNode);
                else
                {
                    nextNodeWrapper = new nodeWrapper(nextNode);
                    mapWrappers.put(nextNode, nextNodeWrapper);
                }

                if (closedSet.contains(nextNodeWrapper)) continue; // it has already been considered
                double tentativeCost = currentNodeWrapper.gx + angle(lastSegment);
                boolean better = false;

                if (!openSet.contains(nextNodeWrapper))
                {
                    openSet.add(nextNodeWrapper);
                    nextNodeWrapper.hx = 1.0 -((landmarkFunctions.localLandmarknessDualGraph(nextNode, currentNodeWrapper.node, state) *wL) + 
                			(landmarkFunctions.globalLandmarknessDualGraph(nextNode, primalDestinationNode, state) *wG));
                    better = true;
                }
                
                else if (tentativeCost < nextNodeWrapper.gx) better = true;

                // store A* information about this promising candidate node
                if (better)
                {
                    nextNodeWrapper.cameFrom = currentNodeWrapper;
                    nextNodeWrapper.edgeFrom = lastSegment;
                    nextNodeWrapper.gx = tentativeCost;    
//                	System.out.println("angular "+nextNodeWrapper.gx + "  "+ nextNodeWrapper.hx);
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
    ArrayList<GeomPlanarGraphDirectedEdge> reconstructPath(nodeWrapper n, HashMap<Integer, CentroidData> centroidsMap, 
    		HashMap<Integer, EdgeData> edgesMap  )
    {
        ArrayList<GeomPlanarGraphDirectedEdge> result =  new ArrayList<GeomPlanarGraphDirectedEdge>();
        nodeWrapper x = n;
        Node previousCommon = null;
        Node common = null;
        Node previousFrom = null;
        Node previousTo = null;
         
        while (x.cameFrom != null)
        {
        	int streetID = (int) x.node.getData(); //extract streetID
        	//extract edge from streetID   ("e" is the edge in the edgeData structure)
        	
         	GeomPlanarGraphDirectedEdge edge = (GeomPlanarGraphDirectedEdge) edgesMap.get(streetID).planarEdge.getDirEdge(0); 
         	
         	if (previousFrom != null)
         	{
	         	if (edge.getFromNode() == previousFrom || edge.getFromNode() == previousTo) common = edge.getFromNode();
	         	else if (edge.getToNode() == previousFrom || edge.getToNode() == previousTo) common = edge.getToNode();
	         	if (common == previousCommon) result.remove(0);
         	}
         	result.add(0, edge); // add this edge to the front of the list

        	previousFrom = edge.getFromNode();
        	previousTo = edge.getToNode();
         	previousCommon = common;
         	x = x.cameFrom;
        }
        return result;
    }

       
    
    double angle(GeomPlanarGraphDirectedEdge lastIntersection)
    {
    	GeomPlanarGraphEdge d = (GeomPlanarGraphEdge) lastIntersection.getEdge();
    	return d.getDoubleAttribute("rad_sc");
    }

    /**
     * 	Considers the list of Nodes open for consideration and returns the node
     *  with minimum fx value
     * @param set list of open Nodes
     * @return
     */
    
    nodeWrapper findMin(ArrayList<nodeWrapper> set)
    {
        double min = 100000;
        nodeWrapper minNode = null;
        
        for (nodeWrapper n : set)
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
    class nodeWrapper
    {

        Node node;
        nodeWrapper cameFrom;
        GeomPlanarGraphDirectedEdge edgeFrom;
        
        double gx, hx, fx, landmarkness;

        public nodeWrapper(Node n)
        {
            node = n;
            gx = 0;
            hx = 0;
            fx = 0;
            cameFrom = null;
            edgeFrom = null;
            landmarkness = 0;
        }

    }
    
       
}