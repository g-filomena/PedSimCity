/**
 ** AStarAngularChange.java
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

import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.GeomPlanarGraphEdge;

public class AStarAngularNode
{
	
	HashMap<Integer, nodeData> nodesMap;
	HashMap<Integer, edgeData> edgesMap;
	HashMap<Integer, centroidData> centroidsMap;
    HashMap<Node, dualNodeWrapper> mapWrappers =  new HashMap<Node, dualNodeWrapper>();
	
    public HashMap<Node, dualNodeWrapper> astarPathNodes(Node originNode, Node destinationNode, pedestrianSimulation state, boolean gl)
    {
    	this.edgesMap = state.edgesMap;
    	this.nodesMap = state.nodesMap;
    	this.centroidsMap = state.centroidsMap;

        // set up the containers for the result
        ArrayList<GeomPlanarGraphDirectedEdge> result =  new ArrayList<GeomPlanarGraphDirectedEdge>();

        // containers for the metainformation about the Nodes relative to the
        // A* search


        dualNodeWrapper originNodeWrapper = new dualNodeWrapper(originNode);
        dualNodeWrapper goalNodeWrapper = new dualNodeWrapper(destinationNode);
        mapWrappers.put(originNode, originNodeWrapper);
        mapWrappers.put(destinationNode, goalNodeWrapper);

        originNodeWrapper.gx = 0;
        originNodeWrapper.hx = 0;
        originNodeWrapper.fx = originNodeWrapper.gx + originNodeWrapper.hx;

        // A* containers: nodes to be investigated
        ArrayList<dualNodeWrapper> closedSet = new ArrayList<dualNodeWrapper>();
        // nodes that have been investigated
        ArrayList<dualNodeWrapper> openSet = new ArrayList<dualNodeWrapper>();
        
        
        openSet.add(originNodeWrapper); //adding the startNode Wrapper 

        while (openSet.size() > 0)
        { // while there are reachable nodes to investigate
        	
            dualNodeWrapper currentNodeWrapper = findMin(openSet); // find the shortest path so far
            if (currentNodeWrapper.node == destinationNode)  return (mapWrappers);
            	
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
                if (utilities.commonPrimalJunction(nextNode, currentNodeWrapper.node, state) == currentNodeWrapper.commonPrimalJunction) continue;
                // get the A* meta information about this Node
                dualNodeWrapper nextNodeWrapper;
                
                if (mapWrappers.containsKey(nextNode)) nextNodeWrapper =  mapWrappers.get(nextNode);
                else
                {
                   nextNodeWrapper = new dualNodeWrapper(nextNode);
                   mapWrappers.put(nextNode, nextNodeWrapper);
                }

                if (closedSet.contains(nextNodeWrapper)) continue; // it has already been considered
                // otherwise evaluate the cost of this node/edge combo

                double currentCost = angle(lastSegment) + state.fromNormalDistribution(1, 5);
                if (gl == false) currentCost = angle(lastSegment) + state.fromNormalDistribution(1, 7);
                if (currentCost > 180) currentCost = 180;
                if (currentCost < 0) currentCost = 0;
                
                double tentativeCost = currentNodeWrapper.gx + currentCost;
                boolean better = false;

                if (!openSet.contains(nextNodeWrapper))
                {
                    openSet.add(nextNodeWrapper);
                    nextNodeWrapper.hx = 0;
					better = true;
                }
                
                else if (tentativeCost < nextNodeWrapper.gx) better = true;
                // store A* information about this promising candidate node
                if (better)
                {
                    nextNodeWrapper.nodeFrom = currentNodeWrapper.node;
                    nextNodeWrapper.edgeFrom = lastSegment;
                    nextNodeWrapper.gx = tentativeCost;
                    nextNodeWrapper.fx = nextNodeWrapper.gx + nextNodeWrapper.hx;
                    nextNodeWrapper.commonPrimalJunction = utilities.commonPrimalJunction(nextNodeWrapper.node, currentNodeWrapper.node, state);
                }
            }
        }
        return mapWrappers;
    }

    
    /**
     * Takes the information about the given node n and returns the path that
     * found it.
     * @param n the end point of the path
     * @return an ArrayList of GeomPlanarGraphDirectedEdges that lead from the
     * given Node to the Node from which the search began
     */
    ArrayList<GeomPlanarGraphDirectedEdge> reconstructPath(dualNodeWrapper nodeWrapper, HashMap<Integer, centroidData> centroidsMap, 
    		HashMap<Integer, edgeData> edgesMap  )
    {
        ArrayList<GeomPlanarGraphDirectedEdge> result =  new ArrayList<GeomPlanarGraphDirectedEdge>();
        dualNodeWrapper currentWrapper = nodeWrapper;

        while (currentWrapper.nodeFrom != null)
        {
        	int streetID = (int) currentWrapper.node.getData(); //extract streetID
        	//extract edge from streetID   ("e" is the edge in the edgeData structure)
         	GeomPlanarGraphDirectedEdge edge = (GeomPlanarGraphDirectedEdge) edgesMap.get(streetID).planarEdge.getDirEdge(0); 
         	result.add(0, edge); // add this edge to the front of the list
            currentWrapper = mapWrappers.get(currentWrapper.nodeFrom);
        }
        return result;
    }

       
    
    double angle(GeomPlanarGraphDirectedEdge lastIntersection)
    {
    	GeomPlanarGraphEdge d = (GeomPlanarGraphEdge) lastIntersection.getEdge();
    	return d.getDoubleAttribute("rad");
    }

    /**
     * 	Considers the list of Nodes open for consideration and returns the node
     *  with minimum fx value
     * @param set list of open Nodes
     * @return
     */
    
    dualNodeWrapper findMin(ArrayList<dualNodeWrapper> set)
    {
        double min = 100000;
        dualNodeWrapper minNode = null;
        
        for (dualNodeWrapper n : set)
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
}