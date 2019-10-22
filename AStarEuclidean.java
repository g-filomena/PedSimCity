/**
 ** AStarEuclidean.java
 **
 ** Copyright 2019 by Sarah Wise, Mark Coletti, Andrew Crooks, and
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
import sim.engine.SimState;


@SuppressWarnings("restriction")
public class AStarEuclidean
{
	
	HashMap<Integer, edgeData> edgesMap;
	HashMap<Integer, nodeData> nodesMap;
    HashMap<Node, nodeWrapper> mapWrappers =  new HashMap<Node, nodeWrapper>();
	
    public ArrayList<GeomPlanarGraphDirectedEdge> astarPath(Node originNode, Node destinationNode, pedestrianSimulation state, 
    		List segmentsToAvoid)
    {
    	this.edgesMap = state.edgesMap;
    	this.nodesMap = state.nodesMap;
        // set up the containers for the result
        ArrayList<GeomPlanarGraphDirectedEdge> result =  new ArrayList<GeomPlanarGraphDirectedEdge>();
        
        // containers for the metainformation about the Nodes relative to the
        // A* search


        nodeWrapper originNodeWrapper = new nodeWrapper(originNode);
        nodeWrapper destinationNodeWrapper = new nodeWrapper(destinationNode);
        mapWrappers.put(originNode, originNodeWrapper);
        mapWrappers.put(destinationNode, destinationNodeWrapper);

        originNodeWrapper.gx = 0;
        originNodeWrapper.hx = 0;
        originNodeWrapper.fx = originNodeWrapper.gx + originNodeWrapper.hx;
        
        // A* containers: nodes to be investigated
        ArrayList<nodeWrapper> closedSet = new ArrayList<nodeWrapper>();
        // nodes that have been investigated
        ArrayList<nodeWrapper> openSet = new ArrayList<nodeWrapper>();
        openSet.add(originNodeWrapper); //adding the originNode Wrapper         
       
        while (openSet.size() > 0)
        { // while there are reachable nodes to investigate
        	
            nodeWrapper currentNodeWrapper = findMin(openSet); // find the shortest path so far
            if (currentNodeWrapper.node == destinationNode) return reconstructPath(destinationNodeWrapper);
            // we have found the shortest possible path to the goal! Reconstruct the path and send it back.
            openSet.remove(currentNodeWrapper); // maintain the lists
            closedSet.add(currentNodeWrapper);
            // check all the edges out from this Node
            DirectedEdgeStar des = currentNodeWrapper.node.getOutEdges();
            
            for (Object o : des.getEdges().toArray())
            {
                GeomPlanarGraphDirectedEdge lastSegment = (GeomPlanarGraphDirectedEdge) o;
                if (segmentsToAvoid == null);
                else if (segmentsToAvoid.contains(((GeomPlanarGraphEdge) lastSegment.getEdge()).getIntegerAttribute("streetID"))) continue;
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

                // otherwise evaluate the cost of this node/edge combo
                                       
                double tentativeCost = currentNodeWrapper.gx + length(lastSegment)*state.fromNormalDistribution();
                boolean better = false;

                if (!openSet.contains(nextNodeWrapper))
                {
                    openSet.add(nextNodeWrapper);
                    nextNodeWrapper.hx = utilities.nodesDistance(nextNode, destinationNode);
                    better = true;
                }
                
                else if (tentativeCost < nextNodeWrapper.gx) {better = true;}

                // store A* information about this promising candidate node
                if (better)
                {
                	nextNodeWrapper.nodeFrom = currentNodeWrapper.node;
                	nextNodeWrapper.edgeFrom = lastSegment;
                	nextNodeWrapper.gx = tentativeCost;
                	nextNodeWrapper.fx = nextNodeWrapper.gx + nextNodeWrapper.hx;
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
    ArrayList<GeomPlanarGraphDirectedEdge> reconstructPath(nodeWrapper nodeWrapper)
    {
        ArrayList<GeomPlanarGraphDirectedEdge> result =  new ArrayList<GeomPlanarGraphDirectedEdge>();
        nodeWrapper currentWrapper = nodeWrapper;
        while (currentWrapper.nodeFrom != null)
        {
            result.add(0, currentWrapper.edgeFrom); // add this edge to the front of the list
            currentWrapper = mapWrappers.get(currentWrapper.nodeFrom);
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

}