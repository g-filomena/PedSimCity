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

import java.util.ArrayList;
import java.util.HashMap;
import sim.app.geo.pedestrianSimulation.utilities.Path;
import sim.util.geo.GeomPlanarGraphDirectedEdge;

public class AStarAngularChange
{
	
	HashMap<Integer, NodeGraph> nodesMap;
	HashMap<Integer, EdgeGraph> edgesMap;
	HashMap<Integer, NodeGraph> centroidsMap;
    HashMap<NodeGraph, DualNodeWrapper> mapWrappers =  new HashMap<NodeGraph, DualNodeWrapper>();
	NodeGraph previousJunction;
	int barrierID;
	
    public Path astarPath(NodeGraph originNode, NodeGraph destinationNode, 
    		ArrayList<NodeGraph> centroidsToAvoid, NodeGraph previousJunction, boolean regionalRouting, int barrierID,
    		PedestrianSimulation state)
    {
    	this.nodesMap = state.nodesMap;
    	this.edgesMap = state.edgesMap;
        // set up the containers for the sequenceEdges
        ArrayList<GeomPlanarGraphDirectedEdge> sequenceEdges =  new ArrayList<GeomPlanarGraphDirectedEdge>();

        // containers for the metainformation about the Nodes relative to the
        // A* search
        DualNodeWrapper originNodeWrapper = new DualNodeWrapper(originNode);
		if (previousJunction != null) originNodeWrapper.commonPrimalJunction = previousJunction;
        DualNodeWrapper goalNodeWrapper = new DualNodeWrapper(destinationNode);
        mapWrappers.put(originNode, originNodeWrapper);
        mapWrappers.put(destinationNode, goalNodeWrapper);

        originNodeWrapper.gx = 0;
        originNodeWrapper.hx = 0;
        originNodeWrapper.fx = originNodeWrapper.gx + originNodeWrapper.hx;

        // A* containers: nodes to be investigated
        ArrayList<DualNodeWrapper> closedSet = new ArrayList<DualNodeWrapper>();
        // nodes that have been investigated
        ArrayList<DualNodeWrapper> openSet = new ArrayList<DualNodeWrapper>();
        openSet.add(originNodeWrapper); //adding the startNode Wrapper 
        while (openSet.size() > 0)
        { 
        	// while there are reachable nodes to investigate
            DualNodeWrapper currentNodeWrapper = findMin(openSet); // find the shortest path so far
            // we have found the shortest possible path to the goal! Reconstruct the path and send it back.
            if (currentNodeWrapper.node == destinationNode)  return reconstructPath(goalNodeWrapper);
            if (centroidsToAvoid == null);
            else if (centroidsToAvoid.contains(currentNodeWrapper.node))
            { 
            	openSet.remove(currentNodeWrapper);
            	closedSet.add(currentNodeWrapper);
            	continue;
            }

            openSet.remove(currentNodeWrapper); // maintain the lists
            closedSet.add(currentNodeWrapper);

            // check all the edges out from this Node
            DirectedEdgeStar des = currentNodeWrapper.node.getOutEdges();
            Object[] outEdges = des.getEdges().toArray();
            for (Object o : outEdges)
            {
                GeomPlanarGraphDirectedEdge lastSegment = (GeomPlanarGraphDirectedEdge) o;

                NodeGraph nextNode = null;
                nextNode = (NodeGraph) lastSegment.getToNode();
                if (utilities.commonPrimalJunction(nextNode, currentNodeWrapper.node) == 
                		currentNodeWrapper.commonPrimalJunction) continue;
                // get the A* meta information about this Node
                DualNodeWrapper nextNodeWrapper;
                
                if (mapWrappers.containsKey(nextNode)) nextNodeWrapper = mapWrappers.get(nextNode);
                else
                {
                   nextNodeWrapper = new DualNodeWrapper(nextNode);
                   mapWrappers.put(nextNode, nextNodeWrapper);
                }

                if (closedSet.contains(nextNodeWrapper)) continue; // it has already been considered
                // otherwise evaluate the cost of this node/edge combo
                
                double currentCost = angle(lastSegment) + state.fromNormalDistribution(0, 5);
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
                    nextNodeWrapper.commonPrimalJunction = utilities.commonPrimalJunction(nextNodeWrapper.node, 
                    		currentNodeWrapper.node);
                }
            }
        }
        Path path = new Path();
        path.edges = sequenceEdges;
        path.dualMapWrappers = mapWrappers;
        return path;
    }

    
    /**
     * Takes the information about the given node n and returns the path that
     * found it.
     * @param n the end point of the path
     * @return an ArrayList of GeomPlanarGraphDirectedEdges that lead from the
     * given Node to the Node from which the search began
     */
    Path reconstructPath(DualNodeWrapper nodeWrapper)
    {
        ArrayList<GeomPlanarGraphDirectedEdge> sequenceEdges =  new ArrayList<GeomPlanarGraphDirectedEdge>();
        DualNodeWrapper currentWrapper = nodeWrapper;

        while (currentWrapper.nodeFrom != null)
        {
         	GeomPlanarGraphDirectedEdge edge = (GeomPlanarGraphDirectedEdge) 
         			currentWrapper.node.primalEdge.getDirEdge(0); 
         	sequenceEdges.add(0, edge); // add this edge to the front of the list
            currentWrapper = mapWrappers.get(currentWrapper.nodeFrom);
        }
        Path path = new Path();
        path.edges = sequenceEdges;
        path.dualMapWrappers = mapWrappers;
        return path;
    }

    double angle(GeomPlanarGraphDirectedEdge lastIntersection)
    {
    	EdgeGraph d = (EdgeGraph) lastIntersection.getEdge();
    	return d.getDeflectionAngle();
    }

    /**
     * 	Considers the list of Nodes open for consideration and returns the node
     *  with minimum fx value
     * @param set list of open Nodes
     * @return
     */
    
    DualNodeWrapper findMin(ArrayList<DualNodeWrapper> set)
    {
        double min = 100000;
        DualNodeWrapper minNode = null;
        
        for (DualNodeWrapper n : set)
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