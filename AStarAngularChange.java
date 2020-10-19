/**
 * AStarAngularChange.java
 * This is built on AStar.java 2011 (by Sarah Wise, Mark Coletti, Andrew Crooks)
 *
 * It computes the cumulative angular change shortest path by employing the A* shortest path algorithm.
 * It uses the dual graph of the street network.
 * It does not support navigation based on regions nor landmarks.
 * It does support barrier-based navigation.
 *
 **/

package sim.app.geo.pedSimCity;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import sim.app.geo.urbanSim.EdgeGraph;
import sim.app.geo.urbanSim.NodeGraph;
import sim.app.geo.urbanSim.NodeWrapper;
import sim.app.geo.urbanSim.Utilities;
import sim.app.geo.urbanSim.Utilities.Path;
import sim.util.geo.GeomPlanarGraphDirectedEdge;

public class AStarAngularChange {

	NodeGraph originNode, destinationNode, previousJunction;
	ArrayList<NodeGraph> visitedNodes, unvisitedNodes;
	boolean barrierBasedNavigation;
	ArrayList<NodeGraph> centroidsToAvoid = new ArrayList<NodeGraph>();
	HashMap<NodeGraph, NodeWrapper> mapWrappers =  new HashMap<NodeGraph, NodeWrapper>();

	/**
	 * @param originNode the originNode;
	 * @param destinationNode the destinationNode;
	 * @param centroidsToAvoid the centroids (dual nodes, representing segments) already traversed in previous iterations, if applicable;
	 * @param previousJunction the previous primal junction, if any;
	 * @param ap the set of the properties that describe the agent;
	 */
	public Path astarPath(NodeGraph originNode, NodeGraph destinationNode, ArrayList<NodeGraph> centroidsToAvoid, NodeGraph previousJunction,
			AgentProperties ap) {

		this.originNode = originNode;
		this.destinationNode = destinationNode;
		this.centroidsToAvoid = new ArrayList<NodeGraph>(centroidsToAvoid);
		this.previousJunction = previousJunction;
		this.barrierBasedNavigation = ap.barrierBasedNavigation;

		// set up the containers for the sequenceEdges
		ArrayList<GeomPlanarGraphDirectedEdge> sequenceEdges =  new ArrayList<GeomPlanarGraphDirectedEdge>();

		// NodeWrapper = container for the metainformation about a Node
		NodeWrapper originNodeWrapper = new NodeWrapper(originNode);
		if (previousJunction != null) originNodeWrapper.commonPrimalJunction = previousJunction;
		NodeWrapper goalNodeWrapper = new NodeWrapper(destinationNode);
		mapWrappers.put(originNode, originNodeWrapper);
		mapWrappers.put(destinationNode, goalNodeWrapper);

		originNodeWrapper.gx = 0;
		originNodeWrapper.hx = 0;
		originNodeWrapper.fx = originNodeWrapper.gx + originNodeWrapper.hx;

		// A* containers: nodes to be investigated vs the ones that have been investigated
		ArrayList<NodeWrapper> closedSet = new ArrayList<NodeWrapper>();
		ArrayList<NodeWrapper> openSet = new ArrayList<NodeWrapper>();
		openSet.add(originNodeWrapper);

		while (openSet.size() > 0) {
			// while there are reachable nodes to investigate
			NodeWrapper currentNodeWrapper = findMin(openSet); // find the shortest path so far
			NodeGraph currentNode = currentNodeWrapper.node;
			if (currentNode == destinationNode)  return reconstructPath(goalNodeWrapper); //found
			if (centroidsToAvoid == null);

			//this centroid was already traversed in a previous iteration
			else if (centroidsToAvoid.contains(currentNode)) {
				openSet.remove(currentNodeWrapper);
				closedSet.add(currentNodeWrapper);
				continue;
			}

			openSet.remove(currentNodeWrapper); // maintain the lists
			closedSet.add(currentNodeWrapper);

			// check all the edges out from this Node
			ArrayList<GeomPlanarGraphDirectedEdge> outEdges = new ArrayList<GeomPlanarGraphDirectedEdge> (currentNode.getOutDirectedEdges());

			for (GeomPlanarGraphDirectedEdge outEdge : outEdges) {
				NodeGraph targetNode = null;
				EdgeGraph commonEdge = (EdgeGraph) outEdge.getEdge();
				targetNode = commonEdge.getOtherNode(currentNode);

				/**
				 * Check if the current and the possible next centroid share in the primal graph the same junction as the current with
				 * its previous centroid --> if yes move on. This essential means that the in the primal graph you would go back to an
				 * already traversed node; but the dual graph wouldn't know.
				 */
				if (Utilities.commonPrimalJunction(targetNode, currentNode) == currentNodeWrapper.commonPrimalJunction) continue;

				NodeWrapper nextNodeWrapper;
				if (mapWrappers.containsKey(targetNode)) nextNodeWrapper = mapWrappers.get(targetNode);
				else {
					nextNodeWrapper = new NodeWrapper(targetNode);
					mapWrappers.put(targetNode, nextNodeWrapper);
				}

				if (closedSet.contains(nextNodeWrapper)) continue; // it has already been considered

				// otherwise evaluate the cost of this combo
				// computing errors in perception of road coasts with stochastic variables
				double error = 0.0;
				if (barrierBasedNavigation) {
					List<Integer> positiveBarriers = targetNode.primalEdge.positiveBarriers;
					List<Integer> negativeBarriers = targetNode.primalEdge.negativeBarriers;
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

				if (!openSet.contains(nextNodeWrapper)) {
					openSet.add(nextNodeWrapper);
					nextNodeWrapper.hx = 0;
					better = true;
				}
				else if (tentativeCost < nextNodeWrapper.gx) better = true;

				// store A* information about this promising candidate node
				if (better) {
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

	// path reconstruction, given the last nodeWrapper
	Path reconstructPath(NodeWrapper nodeWrapper) {
		ArrayList<GeomPlanarGraphDirectedEdge> sequenceEdges =  new ArrayList<GeomPlanarGraphDirectedEdge>();
		NodeWrapper currentWrapper = nodeWrapper;

		while (currentWrapper.nodeFrom != null) {
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

	NodeWrapper findMin(ArrayList<NodeWrapper> set) {

		double min = 100000;
		NodeWrapper minNode = null;
		for (NodeWrapper n : set) {
			if (n.fx < min)	{
				min = n.fx;
				minNode = n;
			}
		}
		return minNode;
	}

}