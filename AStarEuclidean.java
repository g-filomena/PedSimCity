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
import sim.app.geo.pedestrianSimulation.utilities.Path;
import sim.engine.SimState;


@SuppressWarnings("restriction")
public class AStarEuclidean
{
	
	HashMap<Integer, EdgeData> edgesMap;
	HashMap<Integer, NodeData> nodesMap;
    HashMap<Node, NodeWrapper> mapWrappers =  new HashMap<Node, NodeWrapper>();
	
    public Path astarPath(Node originNode, Node destinationNode, pedestrianSimulation state, 
    		List segmentsToAvoid, boolean gl)
    {
    	this.edgesMap = state.edgesMap;
    	this.nodesMap = state.nodesMap;
        // set up the containers for the sequenceEdges
        ArrayList<GeomPlanarGraphDirectedEdge> sequenceEdges =  new ArrayList<GeomPlanarGraphDirectedEdge>();
        
        // containers for the metainformation about the Nodes relative to the
        // A* search


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
        int count = 0;
        while (openSet.size() > 0)
        { 
        	count += 1;
        	// while there are reachable nodes to investigate
//        	
        	if (count > 100000) System.out.println("euclideanPath");
            NodeWrapper currentNodeWrapper = findMin(openSet); // find the shortest path so far
            if (currentNodeWrapper.node == destinationNode) return reconstructPath(destinationNodeWrapper);
            // we have found the shortest possible path to the goal! Reconstruct the path and send it back.
            openSet.remove(currentNodeWrapper); // maintain the lists
            closedSet.add(currentNodeWrapper);
            // check all the edges out from this Node
            DirectedEdgeStar des = currentNodeWrapper.node.getOutEdges();
            
            boolean found = false;
            for (Object o : des.getEdges().toArray())
            {
                GeomPlanarGraphDirectedEdge lastSegment = (GeomPlanarGraphDirectedEdge) o;
                
                if (segmentsToAvoid == null);
                else if (segmentsToAvoid.contains(((GeomPlanarGraphEdge) lastSegment.getEdge()).getIntegerAttribute("streetID"))) 
                	continue;
                
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

                // otherwise evaluate the cost of this node/edge combo
                                       
                double error = state.fromNormalDistribution(1, 0.10);
                if (error < 0) error = 0;
                double tentativeCost = currentNodeWrapper.gx + length(lastSegment)*error;
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
            sequenceEdges.add(0, currentWrapper.edgeFrom); // add this edge to the front of the list
            currentWrapper = mapWrappers.get(currentWrapper.nodeFrom);
        }

        Path path = new Path();
        path.edges = sequenceEdges;
        path.mapWrappers = mapWrappers;
        return path;
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

}