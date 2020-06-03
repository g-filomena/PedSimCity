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

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import sim.app.geo.urbanSim.*;
import sim.app.geo.urbanSim.Utilities.Path;
import sim.util.geo.GeomPlanarGraphDirectedEdge;

public class AStarAngularChange
{
	
	HashMap<Integer, NodeGraph> nodesMap;
	HashMap<Integer, EdgeGraph> edgesMap;
	HashMap<Integer, NodeGraph> centroidsMap;
    HashMap<NodeGraph, NodeWrapper> mapWrappers =  new HashMap<NodeGraph, NodeWrapper>();
	NodeGraph previousJunction;
	
    public Path astarPath(NodeGraph originNode, NodeGraph destinationNode, 
    		ArrayList<NodeGraph> centroidsToAvoid, NodeGraph previousJunction, boolean regionalRouting, boolean barriersRouting)
    {
        // set up the containers for the sequenceEdges
        ArrayList<GeomPlanarGraphDirectedEdge> sequenceEdges =  new ArrayList<GeomPlanarGraphDirectedEdge>();

        // containers for the metainformation about the Nodes relative to the
        // A* search
        NodeWrapper originNodeWrapper = new NodeWrapper(originNode);
		if (previousJunction != null) originNodeWrapper.commonPrimalJunction = previousJunction;
        NodeWrapper goalNodeWrapper = new NodeWrapper(destinationNode);
        mapWrappers.put(originNode, originNodeWrapper);
        mapWrappers.put(destinationNode, goalNodeWrapper);

        originNodeWrapper.gx = 0;
        originNodeWrapper.hx = 0;
        originNodeWrapper.fx = originNodeWrapper.gx + originNodeWrapper.hx;

        // A* containers: nodes to be investigated
        ArrayList<NodeWrapper> closedSet = new ArrayList<NodeWrapper>();
        // nodes that have been investigated
        ArrayList<NodeWrapper> openSet = new ArrayList<NodeWrapper>();
        openSet.add(originNodeWrapper); //adding the startNode Wrapper 
        
        while (openSet.size() > 0)
        { 
        	// while there are reachable nodes to investigate
            NodeWrapper currentNodeWrapper = findMin(openSet); // find the shortest path so far
            // we have found the shortest possible path to the goal! Reconstruct the path and send it back.
            NodeGraph currentNode = currentNodeWrapper.node;
            if (currentNode == destinationNode)  return reconstructPath(goalNodeWrapper);
            if (centroidsToAvoid == null);
            else if (centroidsToAvoid.contains(currentNode))
            { 
            	openSet.remove(currentNodeWrapper);
            	closedSet.add(currentNodeWrapper);
            	continue;
            }

            openSet.remove(currentNodeWrapper); // maintain the lists
            closedSet.add(currentNodeWrapper);
            
            // check all the edges out from this Node
            ArrayList<GeomPlanarGraphDirectedEdge> outEdges = new ArrayList<GeomPlanarGraphDirectedEdge> (currentNode.getOutDirectedEdges());
    		
            for (GeomPlanarGraphDirectedEdge outEdge : outEdges)
            {
                NodeGraph targetNode = null;
                EdgeGraph commonEdge = (EdgeGraph) outEdge.getEdge();
                targetNode = commonEdge.getOtherNode(currentNode);
                if (Utilities.commonPrimalJunction(targetNode, currentNode) == currentNodeWrapper.commonPrimalJunction) continue;
                // get the A* meta information about this Node
                NodeWrapper nextNodeWrapper;
                
                if (mapWrappers.containsKey(targetNode)) nextNodeWrapper = mapWrappers.get(targetNode);
                else
                {
                   nextNodeWrapper = new NodeWrapper(targetNode);
                   mapWrappers.put(targetNode, nextNodeWrapper);
                }

                if (closedSet.contains(nextNodeWrapper)) continue; // it has already been considered
                
                // otherwise evaluate the cost of this node/edge combo
    	    	double error = 0.0;
    	    	List<Integer> positiveBarriers = targetNode.primalEdge.positiveBarriers;
    	    	List<Integer> negativeBarriers = targetNode.primalEdge.negativeBarriers;
    	    	if (barriersRouting) 
    	    	{
    	    		if (positiveBarriers != null) error = Utilities.fromNormalDistribution(0.70, 0.10, "left");
    	    		else if (negativeBarriers != null) error = Utilities.fromNormalDistribution(1.30, 0.10, "right");
    	    		else error = Utilities.fromNormalDistribution(1, 0.10, null);
    	    	}
    	    	else error = Utilities.fromNormalDistribution(1, 0.10, null);
                double edgeCost = commonEdge.getDeflectionAngle() * error;
                if (edgeCost > 180) edgeCost = 180;
                if (edgeCost < 0) edgeCost = 0;
                
                double tentativeCost = currentNodeWrapper.gx + edgeCost;
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
                    nextNodeWrapper.nodeFrom = currentNode;
                    nextNodeWrapper.edgeFrom = outEdge;
                    nextNodeWrapper.gx = tentativeCost;
                    nextNodeWrapper.fx = nextNodeWrapper.gx + nextNodeWrapper.hx;
                    nextNodeWrapper.commonPrimalJunction = Utilities.commonPrimalJunction(targetNode, currentNode);
                }
            }
        }
        Path path = new Path();
        path.edges = sequenceEdges;
        path.mapWrappers = mapWrappers;
        return path;
    }

    
    /**
     * Takes the information about the given node n and returns the path that
     * found it.
     * @param n the end point of the path
     * @return an ArrayList of GeomPlanarGraphDirectedEdges that lead from the
     * given Node to the Node from which the search began
     */
    Path reconstructPath(NodeWrapper nodeWrapper)
    {
        ArrayList<GeomPlanarGraphDirectedEdge> sequenceEdges =  new ArrayList<GeomPlanarGraphDirectedEdge>();
        NodeWrapper currentWrapper = nodeWrapper;

        while (currentWrapper.nodeFrom != null)
        {
         	GeomPlanarGraphDirectedEdge edge = (GeomPlanarGraphDirectedEdge) 
         			currentWrapper.node.primalEdge.getDirEdge(0); 
         	sequenceEdges.add(0, edge); // add this edge to the front of the list
            currentWrapper = mapWrappers.get(currentWrapper.nodeFrom);
        }
        Path path = new Path();
        path.edges = sequenceEdges;
        path.mapWrappers = mapWrappers;
        return path;
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

}