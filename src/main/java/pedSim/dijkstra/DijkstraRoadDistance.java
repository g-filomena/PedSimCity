
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
import sim.graph.NodeGraph;

/**
 * The class allows computing the road distance shortest route by employing the
 * Dijkstra shortest-path algorithm on a primal graph representation of the
 * street network.
 *
 * It furthermore supports combined navigation strategies based on landmark and
 * urban subdivisions (regions, barriers).
 **/
public class DijkstraRoadDistance extends Dijkstra {

	/**
	 * Performs the Dijkstra's algorithm to find the shortest path from the origin
	 * node to the destination node.
	 *
	 * This method calculates the shortest path in the network graph from the
	 * specified origin node to the destination node while considering optional
	 * segments to avoid and agent properties.
	 *
	 * @param originNode           The starting node for the path.
	 * @param destinationNode      The destination node to reach.
	 * @param finalDestinationNode The final destination node (primal graph) for the
	 *                             path.
	 * @param directedEdgesToAvoid A set of directed edges (segments) to avoid
	 *                             during the path calculation.
	 * @param agent                The agent for which the route is computed.
	 * 
	 * @return An ArrayList of DirectedEdges representing the shortest path from the
	 *         origin to the destination.
	 */
	public List<DirectedEdge> dijkstraAlgorithm(NodeGraph originNode, NodeGraph destinationNode,
			NodeGraph finalDestinationNode, Set<DirectedEdge> directedEdgesToAvoid, Agent agent) {

		initialise(originNode, destinationNode, finalDestinationNode, agent);
		initialisePrimal(directedEdgesToAvoid);
		visitedNodes = new HashSet<>();
		unvisitedNodes = new PriorityQueue<>(Comparator.comparingDouble(this::getBest));
		unvisitedNodes.add(this.originNode);

		// NodeWrapper = container for the metainformation about a Node
		NodeWrapper nodeWrapper = new NodeWrapper(this.originNode);
		nodeWrapper.gx = 0.0;
		nodeWrappersMap.put(this.originNode, nodeWrapper);
		runDijkstra();
		return reconstructSequence();
	}

	/**
	 * Runs the Dijkstra algorithm to find the shortest path.
	 */
	private void runDijkstra() {
		while (!unvisitedNodes.isEmpty()) {
			NodeGraph currentNode = unvisitedNodes.peek();
			visitedNodes.add(currentNode);
			unvisitedNodes.remove(currentNode);
			findMinDistances(currentNode);
		}
	}

	/**
	 * Finds the minimum distances for adjacent nodes of the given current node in
	 * the primal graph.
	 *
	 * @param currentNode The current node in the primal graph for which to find
	 *                    adjacent nodes.
	 */
	private void findMinDistances(NodeGraph currentNode) {
		List<NodeGraph> adjacentNodes = currentNode.getAdjacentNodes();
		for (NodeGraph targetNode : adjacentNodes) {
			if (visitedNodes.contains(targetNode))
				continue;

			EdgeGraph commonEdge = agentNetwork.getEdgeBetween(currentNode, targetNode);
			DirectedEdge outEdge = commonEdge.getDirEdge(0);
			if (edgesToAvoid.contains(outEdge.getEdge()))
				continue;

			tentativeCost = 0.0;
			double error = costPerceptionError(targetNode, commonEdge, false);
			double edgeCost = commonEdge.getLength() * error;
			computeTentativeCost(currentNode, targetNode, edgeCost);
			isBest(currentNode, targetNode, outEdge);
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

		// Check that the route has been formulated properly
		// No route
		if (nodeWrappersMap.get(destinationNode) == null || nodeWrappersMap.size() <= 1)
			directedEdgesSequence.clear();
		else
			while (nodeWrappersMap.get(step).nodeFrom != null) {
				DirectedEdge directedEdge;
				if (subGraph != null)
					directedEdge = retrieveFromParentGraph(step);
				else
					directedEdge = nodeWrappersMap.get(step).directedEdgeFrom;
				step = nodeWrappersMap.get(step).nodeFrom;
				directedEdgesSequence.add(0, directedEdge);
				traversedNodesMap.put(step, nodeWrappersMap.get(step));
			}
		return directedEdgesSequence;
	}
}
