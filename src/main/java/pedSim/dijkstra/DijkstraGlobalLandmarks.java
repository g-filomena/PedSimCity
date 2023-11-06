package pedSim.dijkstra;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;

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
	 * @param segmentsToAvoid      A set of directed edges (segments) to avoid
	 *                             during the path calculation.
	 * @param agentProperties      The set of agent properties that influence path
	 *                             calculations.
	 * 
	 * @return An ArrayList of DirectedEdges representing the path.
	 */
	public ArrayList<DirectedEdge> dijkstraAlgorithm(NodeGraph originNode, NodeGraph destinationNode,
			NodeGraph finalDestinationNode, HashSet<DirectedEdge> segmentsToAvoid, Agent agent) {

		this.usingSubGraph = false;
		initialise(originNode, destinationNode, finalDestinationNode, segmentsToAvoid, agent, false);
		visitedNodes = new HashSet<>();
		unvisitedNodes = new HashSet<>();
		unvisitedNodes.add(this.originNode);

		// NodeWrapper = container for the metainformation about a Node
		final NodeWrapper nodeWrapper = new NodeWrapper(this.originNode);
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
			final NodeGraph currentNode = getClosest(unvisitedNodes);
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
		final ArrayList<NodeGraph> adjacentNodes = currentNode.getAdjacentNodes();
		for (final NodeGraph targetNode : adjacentNodes) {
			if (this.visitedNodes.contains(targetNode))
				continue;

			final EdgeGraph commonEdge = graph.getEdgeBetween(currentNode, targetNode);
			final DirectedEdge outEdge = commonEdge.getDirEdge(0);
			if (edgesToAvoid.contains(outEdge.getEdge()))
				continue;

			final double globalLandmarkness = LandmarkNavigation.globalLandmarknessNode(targetNode,
					this.finalDestinationNode);

			// the global landmarkness from the node is divided by the segment's length so
			// to avoid that the route is not affected
			// by network (topological) distance
			final double nodeLandmarkness = (1.0 - globalLandmarkness) / commonEdge.getLength();
			tentativeCost = getBest(currentNode) + nodeLandmarkness;
			isBest(currentNode, targetNode, outEdge);
		}
	}

	/**
	 * Reconstructs the sequence of directed edges composing the path.
	 *
	 * @return An ArrayList of DirectedEdges representing the path sequence.
	 */
	ArrayList<DirectedEdge> reconstructSequence() {
		final HashMap<NodeGraph, NodeWrapper> traversedNodesMap = new HashMap<>();
		final ArrayList<DirectedEdge> directedEdgesSequence = new ArrayList<>();
		NodeGraph step = destinationNode;
		traversedNodesMap.put(destinationNode, nodeWrappersMap.get(destinationNode));

		// check that the route has been formulated properly
		if (nodeWrappersMap.get(destinationNode) == null || nodeWrappersMap.size() <= 1)
			directedEdgesSequence.clear();
		try {
			while (nodeWrappersMap.get(step).nodeFrom != null) {
				final DirectedEdge directedEdge = nodeWrappersMap.get(step).directedEdgeFrom;
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