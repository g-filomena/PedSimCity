package pedSim.dijkstra;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;

import org.locationtech.jts.planargraph.DirectedEdge;

import pedSim.agents.Agent;
import sim.graph.EdgeGraph;
import sim.graph.GraphUtils;
import sim.graph.NodeGraph;

/**
 * The class allows computing the least cumulative angular change route by
 * employing the Dijkstra shortest-path algorithm on a dual graph representation
 * of the street network.
 *
 * It furthermore supports combined navigation strategies based on landmark and
 * urban subdivisions (regions, barriers).
 **/
public class DijkstraAngularChange extends Dijkstra {

	/**
	 * Performs the Dijkstra's algorithm to find the least cumulative angular change
	 * path from the origin node to the destination node.
	 *
	 * @param originNode           The starting node for the path.
	 * @param destinationNode      The destination node to reach.
	 * @param finalDestinationNode The final destination node (primal graph) for the
	 *                             path, if different.
	 * @param centroidsToAvoid     A set of centroids (nodes representing segments)
	 *                             to avoid during the path calculation.
	 * @param agent                The agent for which the route is computed.
	 * 
	 * @return An ArrayList of DirectedEdges representing the path.
	 */
	public List<DirectedEdge> dijkstraAlgorithm(NodeGraph originNode, NodeGraph destinationNode,
			NodeGraph finalDestinationNode, Set<NodeGraph> centroidsToAvoid, NodeGraph previousJunction, Agent agent) {

		initialise(originNode, destinationNode, finalDestinationNode, agent);
		initialiseDual(centroidsToAvoid, previousJunction);

		visitedNodes = new HashSet<>();
		unvisitedNodes = new PriorityQueue<>(Comparator.comparingDouble(this::getBest));
		unvisitedNodes.add(this.originNode);

		// NodeWrapper = container for the metainformation about a Node
		NodeWrapper nodeWrapper = new NodeWrapper(this.originNode);
		nodeWrapper.gx = 0.0;

		if (previousJunction != null)
			nodeWrapper.commonPrimalJunction = previousJunction;
		nodeWrappersMap.put(this.originNode, nodeWrapper);
		if (this.centroidsToAvoid != null)
			for (NodeGraph centroid : this.centroidsToAvoid)
				visitedNodes.add(centroid);

		runDijkstra();
		return reconstructSequence();
	}

	/**
	 * Runs the Dijkstra algorithm to find the shortest path.
	 */
	private void runDijkstra() {

		// add centroids to avoid in the visited set
		while (unvisitedNodes.size() > 0) {
			// at the beginning it takes originNode
			NodeGraph currentNode = unvisitedNodes.peek();
			visitedNodes.add(currentNode);
			unvisitedNodes.remove(currentNode);
			findLeastAngularChange(currentNode);
		}
	}

	/**
	 * Finds the least cumulative angular deviations for adjacent nodes of the given
	 * current node in the dual graph.
	 *
	 *
	 * @param currentNode The current node in the dual graph for which to find
	 *                    adjacent nodes.
	 */
	private void findLeastAngularChange(NodeGraph currentNode) {

		List<NodeGraph> adjacentNodes = currentNode.getAdjacentNodes();
		for (NodeGraph targetNode : adjacentNodes) {
			if (visitedNodes.contains(targetNode))
				continue;

			// Check if the current and the possible next centroid share in the primal graph
			// the same junction as the current with its previous centroid
			// --> if yes move on. This essentially means that the in the primal graph you
			// would go back to an
			// already traversed node; but the dual graph wouldn't know.
			if (GraphUtils.getPrimalJunction(targetNode, currentNode)
					.equals(nodeWrappersMap.get(currentNode).commonPrimalJunction))
				continue;

			EdgeGraph commonEdge = agentDualNetwork.getEdgeBetween(currentNode, targetNode);
			DirectedEdge outEdge = agentDualNetwork.getDirectedEdgeBetween(currentNode, targetNode);

			// compute errors in perception of road coasts with stochastic variables
			double error = costPerceptionError(targetNode, commonEdge, true);
			// Use ExecutorService to calculate edgeCost in parallel
			double edgeCost = commonEdge.getDeflectionAngle() * error;
			computeTentativeCostDual(currentNode, targetNode, edgeCost);
			isBestDual(currentNode, targetNode, outEdge);
		}
	}

	/**
	 * Reconstructs the sequence of directed edges composing the path.
	 *
	 * @return An ArrayList of DirectedEdges representing the path sequence.
	 */
	private List<DirectedEdge> reconstructSequence() {

		Map<NodeGraph, NodeWrapper> traversedNodesMap = new HashMap<>();
		List<DirectedEdge> directedEdgesSequence = new ArrayList<>();
		NodeGraph step = destinationNode;
		traversedNodesMap.put(destinationNode, nodeWrappersMap.get(destinationNode));

		// check that the route has been formulated properly
		if (nodeWrappersMap.get(destinationNode) == null || nodeWrappersMap.size() <= 1)
			directedEdgesSequence.clear();
		try {
			while (nodeWrappersMap.get(step).nodeFrom != null) {

				DirectedEdge directedEdge = step.getPrimalEdge().getDirEdge(0); // this refer in any case to the Parent
																				// primal graph
				step = nodeWrappersMap.get(step).nodeFrom;
				traversedNodesMap.put(step, nodeWrappersMap.get(step));
				directedEdgesSequence.add(0, directedEdge);

				if (step.equals(originNode)) {
					DirectedEdge firstEdge = step.getPrimalEdge().getDirEdge(0);
					directedEdgesSequence.add(0, firstEdge);
					break;
				}
			}
		} catch (final java.lang.NullPointerException e) {
			return directedEdgesSequence;
		}

		return directedEdgesSequence;
	}
}
