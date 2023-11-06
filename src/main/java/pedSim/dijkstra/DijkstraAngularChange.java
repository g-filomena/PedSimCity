package pedSim.dijkstra;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;

import org.locationtech.jts.planargraph.DirectedEdge;

import pedSim.agents.Agent;
import pedSim.engine.Parameters;
import sim.graph.EdgeGraph;
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
	 * @param segmentsToAvoid      A set of directed edges (segments) to avoid
	 *                             during the path calculation.
	 * @param agentProperties      The set of agent properties that influence path
	 *                             calculations.
	 * 
	 * @return An ArrayList of DirectedEdges representing the path.
	 */
	public ArrayList<DirectedEdge> dijkstraAlgorithm(NodeGraph originNode, NodeGraph destinationNode,
			NodeGraph finalDestinationNode, HashSet<NodeGraph> centroidsToAvoid, NodeGraph previousJunction,
			Agent agent) {

		initialiseDual(originNode, destinationNode, finalDestinationNode, centroidsToAvoid, previousJunction, agent,
				Parameters.subGraph);

		visitedNodes = new HashSet<>();
		unvisitedNodes = new HashSet<>();
		unvisitedNodes.add(this.originNode);

		// NodeWrapper = container for the metainformation about a Node
		final NodeWrapper nodeWrapper = new NodeWrapper(this.originNode);
		nodeWrapper.gx = 0.0;

		if (previousJunction != null)
			nodeWrapper.commonPrimalJunction = previousJunction;
		nodeWrappersMap.put(this.originNode, nodeWrapper);
		if (this.centroidsToAvoid != null)
			for (final NodeGraph centroid : this.centroidsToAvoid)
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
			final NodeGraph currentNode = getClosest(unvisitedNodes);
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

		final ArrayList<NodeGraph> adjacentNodes = currentNode.getAdjacentNodes();
		for (NodeGraph targetNode : adjacentNodes) {
			if (visitedNodes.contains(targetNode))
				continue;

			// Check if the current and the possible next centroid share in the primal graph
			// the same junction as the current with its previous centroid
			// --> if yes move on. This essentially means that the in the primal graph you
			// would go back to an
			// already traversed node; but the dual graph wouldn't know.
			if (route.commonPrimalJunction(targetNode, currentNode)
					.equals(nodeWrappersMap.get(currentNode).commonPrimalJunction))
				continue;

			final EdgeGraph commonEdge = graph.getEdgeBetween(currentNode, targetNode);
			final DirectedEdge outEdge = graph.getDirectedEdgeBetween(currentNode, targetNode);

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
	private ArrayList<DirectedEdge> reconstructSequence() {

		final HashMap<NodeGraph, NodeWrapper> traversedNodesMap = new HashMap<>();
		final ArrayList<DirectedEdge> directedEdgesSequence = new ArrayList<>();
		NodeGraph step = destinationNode;
		traversedNodesMap.put(destinationNode, nodeWrappersMap.get(destinationNode));
		// If the subgraph navigation hasn't worked, retry by using the full graph
		// --> it switches "subgraph" to false;
		if (nodeWrappersMap.get(destinationNode) == null && usingSubGraph) {
			clear();
			final ArrayList<DirectedEdge> secondAttempt = dijkstraAlgorithm(originNode, destinationNode,
					finalDestinationNode, centroidsToAvoid, previousJunction, agent);
			return secondAttempt;
		}

		// check that the route has been formulated properly
		if (nodeWrappersMap.get(destinationNode) == null || nodeWrappersMap.size() <= 1)
			directedEdgesSequence.clear();
		try {
			while (nodeWrappersMap.get(step).nodeFrom != null) {
				final DirectedEdge directedEdge = step.primalEdge.getDirEdge(0);
				step = nodeWrappersMap.get(step).nodeFrom;
				traversedNodesMap.put(step, nodeWrappersMap.get(step));
				directedEdgesSequence.add(0, directedEdge);

				if (step == originNode) {
					final DirectedEdge firstEdge = step.primalEdge.getDirEdge(0);
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
