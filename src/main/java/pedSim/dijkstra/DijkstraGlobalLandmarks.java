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
import pedSim.routeChoice.LandmarkNavigation;
import sim.graph.EdgeGraph;
import sim.graph.NodeGraph;

/**
 * The class allows computing the route that maximises global landmarkness
 * between an origin and a destination on a primal graph representation of the
 * street network.
 */

public class DijkstraGlobalLandmarks extends Dijkstra {

	/**
	 * Performs the Dijkstra's algorithm to find the path that maximise global
	 * landmarkness exposure towards the destination node.
	 * 
	 * @param originNode           The starting node for the path.
	 * @param destinationNode      The destination node to reach.
	 * @param finalDestinationNode The final destination node for the path, if
	 *                             different.
	 * @param directedEdgesToAvoid A set of directed edges (segments) to avoid
	 *                             during the path calculation.
	 * @param agent                The agent for which the route is computed.
	 * 
	 * @return An ArrayList of DirectedEdges representing the path.
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
			findBestLandmarkness(currentNode);
		}
	}

	/**
	 * Finds the highest landmarkness for adjacent nodes of the given current node.
	 *
	 * This method calculates the landmarkness of adjacent nodes and determines if
	 * they are better choices for the route based on landmarkness criteria.
	 *
	 * @param currentNode The current node for which to find adjacent nodes.
	 */
	void findBestLandmarkness(NodeGraph currentNode) {
		List<NodeGraph> adjacentNodes = currentNode.getAdjacentNodes();
		for (NodeGraph targetNode : adjacentNodes) {
			if (visitedNodes.contains(targetNode))
				continue;

			EdgeGraph commonEdge = agentNetwork.getEdgeBetween(currentNode, targetNode);
			DirectedEdge outEdge = commonEdge.getDirEdge(0);
			if (edgesToAvoid.contains(outEdge.getEdge()))
				continue;

			double globalLandmarkness = LandmarkNavigation.globalLandmarknessNode(targetNode, finalDestinationNode);

			// the global landmarkness from the node is divided by the segment's length so
			// to avoid that the route is not affected
			// by network (topological) distance
			double nodeLandmarkness = (1.0 - globalLandmarkness) / commonEdge.getLength();
			tentativeCost = getBest(currentNode) + nodeLandmarkness;
			isBest(currentNode, targetNode, outEdge);
		}
	}

	/**
	 * Reconstructs the sequence of directed edges composing the path.
	 *
	 * @return An ArrayList of DirectedEdges representing the path sequence.
	 */
	List<DirectedEdge> reconstructSequence() {
		Map<NodeGraph, NodeWrapper> traversedNodesMap = new HashMap<>();
		List<DirectedEdge> directedEdgesSequence = new ArrayList<>();
		NodeGraph step = destinationNode;
		traversedNodesMap.put(destinationNode, nodeWrappersMap.get(destinationNode));

		// check that the route has been formulated properly
		if (nodeWrappersMap.get(destinationNode) == null || nodeWrappersMap.size() <= 1)
			directedEdgesSequence.clear();
		try {
			while (nodeWrappersMap.get(step).nodeFrom != null) {
				DirectedEdge directedEdge = nodeWrappersMap.get(step).directedEdgeFrom;
				step = nodeWrappersMap.get(step).nodeFrom;
				directedEdgesSequence.add(0, directedEdge);
				traversedNodesMap.put(step, nodeWrappersMap.get(step));
			}
		}
		// no route
		catch (final java.lang.NullPointerException e) {
			return directedEdgesSequence;
		}
		return directedEdgesSequence;
	}
}
