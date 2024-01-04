package pedSim.routeChoice;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.locationtech.jts.planargraph.DirectedEdge;

import pedSim.agents.Agent;
import pedSim.dijkstra.DijkstraAngularChange;
import pedSim.dijkstra.DijkstraRoadDistance;
import sim.graph.EdgeGraph;
import sim.graph.Graph;
import sim.graph.GraphUtils;
import sim.graph.NodeGraph;

/**
 * The `PathFinder` class provides common functionality for computing navigation
 * paths using various algorithms and graph representations.
 */
public class PathFinder {

	Agent agent;
	Route route = new Route();
	protected Graph agentNetwork;
	NodeGraph originNode, destinationNode;
	NodeGraph tmpOrigin, tmpDestination;
	NodeGraph previousJunction = null;

	List<NodeGraph> sequenceNodes = new ArrayList<>();
	// need order here, that's why it's not hashset
	List<NodeGraph> centroidsToAvoid = new ArrayList<>();
	Set<DirectedEdge> directedEdgesToAvoid = new HashSet<>();

	List<DirectedEdge> completeSequence = new ArrayList<>();
	List<DirectedEdge> partialSequence = new ArrayList<>();

	protected boolean regionBased = false;
	boolean moveOn = false;

	/**
	 * Performs backtracking to compute a path in a primal graph from the current
	 * temporary origin node to the given temporary destination node. If the
	 * temporary origin node is the same as the original origin node, it attempts to
	 * skip the temporary destination. Otherwise, it updates the temporary origin
	 * node, checks for the existence of a direct segment between the new origin and
	 * the destination, and if not found, computes a path from the new origin to the
	 * destination while avoiding specified segments.
	 *
	 * @param tmpDestination The temporary destination node.
	 */
	protected void backtracking(NodeGraph tmpDestination) {

		if (tmpOrigin.equals(originNode)) {
			// try skipping this tmpDestination
			moveOn = true;
			return;
		}
		// determine new tmpOrigin
		updateTmpOrigin();

		// check if there's a segment between the new tmpOrigin and the destination
		final DirectedEdge edge = agentNetwork.getDirectedEdgeBetween(tmpOrigin, tmpDestination);
		if (edge != null) {
			if (!completeSequence.contains(edge))
				completeSequence.add(edge);
			moveOn = true; // No need to backtrack anymore
			return;
		}

		// If not, try to compute the path from the new tmpOrigin
		final DijkstraRoadDistance pathFinder = new DijkstraRoadDistance();
		directedEdgesToAvoid = new HashSet<>(completeSequence);
		partialSequence = pathFinder.dijkstraAlgorithm(tmpOrigin, tmpDestination, destinationNode, directedEdgesToAvoid,
				agent);
	}

	/**
	 * Updates the temporary origin node based on the current state of the path. If
	 * there are fewer than two segments in the complete sequence, it clears the
	 * sequence and sets the temporary origin to the original origin node.
	 * Otherwise, it removes the last problematic segment from the complete sequence
	 * and updates the temporary origin accordingly.
	 */
	private void updateTmpOrigin() {
		if (completeSequence.size() < 2) {
			completeSequence.clear();
			tmpOrigin = originNode;
		} else {
			// remove the last problematic segment
			completeSequence.remove(completeSequence.size() - 1);
			tmpOrigin = (NodeGraph) completeSequence.get(completeSequence.size() - 1).getToNode();
		}
	}

	/**
	 * Performs backtracking in the context of dual graph-based pathfinding (angular
	 * change). When the agent gets stuck due to the "centroidsToAvoid" set, this
	 * method iterates back across nodes and retries to compute the path towards the
	 * given tmpDestinationNode.
	 */
	protected void dualBacktracking() {
		// new tmpOrigin
		try {
			tmpOrigin = (NodeGraph) completeSequence.get(completeSequence.size() - 1).getFromNode();
		} catch (final java.lang.ArrayIndexOutOfBoundsException e) {
			partialSequence.clear();
			return;
		}

		// remove last one which did not work!
		completeSequence.remove(completeSequence.size() - 1);
		centroidsToAvoid.remove(centroidsToAvoid.size() - 1);
		// take new previous junction
		previousJunction = route.previousJunction(completeSequence);
		// check if there's a segment between the new tmpOrigin and the destination
		final DirectedEdge edge = agentNetwork.getDirectedEdgeBetween(tmpOrigin, tmpDestination);

		if (edge != null) {
			if (!completeSequence.contains(edge))
				completeSequence.add(edge);
			moveOn = true; // no need to backtracking anymore
			return;
		}

		List<NodeGraph> dualNodesOrigin = getDualNodes(tmpOrigin, previousJunction);
		List<NodeGraph> dualNodesDestination = getDualNodes(tmpDestination, previousJunction);
		for (final NodeGraph tmpDualOrigin : dualNodesOrigin) {
			for (final NodeGraph tmpDualDestination : dualNodesDestination) {
				final DijkstraAngularChange pathfinder = new DijkstraAngularChange();
				Set<NodeGraph> centroidsToAvoidSet = new HashSet<>(centroidsToAvoid);
				partialSequence = pathfinder.dijkstraAlgorithm(tmpDualOrigin, tmpDualDestination, destinationNode,
						centroidsToAvoidSet, tmpOrigin, agent);
				if (!partialSequence.isEmpty())
					break;
			}
			if (!partialSequence.isEmpty())
				break;
		}
	}

	/**
	 * Retrieves a list of dual nodes connected to the given node.
	 *
	 * @param node             The examined primal node.
	 * @param previousJunction The previous junction node used for deriving the
	 *                         direction.
	 * @return A list of dual nodes connected to the given primal node.
	 */
	protected ArrayList<NodeGraph> getDualNodes(NodeGraph node, NodeGraph previousJunction) {
		Set<NodeGraph> dualNodesSet = node.getDualNodes(tmpOrigin, tmpDestination, regionBased, previousJunction)
				.keySet();
		return new ArrayList<NodeGraph>(dualNodesSet);
	}

	/**
	 * Controls and adjusts the sequence of DirectedEdges to ensure that the
	 * destinationNode has not been traversed already. If the destinationNode has
	 * been traversed, it removes any unnecessary edges and ensures that the path is
	 * correctly ordered.
	 *
	 * @param destinationNode The examined primal destination node.
	 */
	protected void controlPath(NodeGraph destinationNode) {
		for (final DirectedEdge directedEdge : completeSequence)
			if (directedEdge.getToNode().equals(destinationNode)) {
				int lastIndex = completeSequence.indexOf(directedEdge);
				completeSequence = new ArrayList<>(completeSequence.subList(0, lastIndex + 1));
				if (route.previousJunction(completeSequence).equals(destinationNode))
					completeSequence.remove(completeSequence.size() - 1);
				return;
			}
	}

	/**
	 * Cleans and adjusts the sequence of DirectedEdges in the dual graph-based
	 * path. It checks if the path is one edge ahead and removes the last edge if
	 * necessary. It also checks for the presence of an unnecessary edge at the
	 * beginning of the path and removes it. This method ensures that the dual
	 * graph-based path is correctly ordered and free of unnecessary edges.
	 *
	 * @param tmpOrigin      The examined primal origin node.
	 * @param tmpDestination The examined primal destination node.
	 */
	protected void cleanDualPath(NodeGraph tmpOrigin, NodeGraph tmpDestination) {
		// check if the path is one edge ahead
		final NodeGraph firstDualNode = ((EdgeGraph) partialSequence.get(0).getEdge()).getDualNode();
		final NodeGraph secondDualNode = ((EdgeGraph) partialSequence.get(1).getEdge()).getDualNode();

		if (route.previousJunction(partialSequence).equals(tmpDestination))
			partialSequence.remove(partialSequence.size() - 1);
		// check presence of a unnecessary edge at the beginning of the path
		if (GraphUtils.getPrimalJunction(firstDualNode, secondDualNode).equals(tmpOrigin))
			partialSequence.remove(0);
		checkEdgesSequence(tmpOrigin);
	}

	/**
	 * Reorders the sequence of DirectedEdges based on their fromNode and toNode,
	 * ensuring that they follow a consistent order. This method is used to correct
	 * the sequence of DirectedEdges when performing region-based navigation to
	 * ensure a smooth path.
	 *
	 * @param tmpOrigin The examined primal origin node.
	 */
	protected void checkEdgesSequence(NodeGraph tmpOrigin) {
		NodeGraph previousNode = tmpOrigin;

		final ArrayList<DirectedEdge> copyPartial = new ArrayList<>(partialSequence);
		for (final DirectedEdge edge : copyPartial) {
			NodeGraph nextNode = (NodeGraph) edge.getToNode();
			// need to swap
			if (nextNode.equals(previousNode)) {
				nextNode = (NodeGraph) edge.getFromNode();
				DirectedEdge correctEdge = agentNetwork.getDirectedEdgeBetween(previousNode, nextNode);
				partialSequence.set(partialSequence.indexOf(edge), correctEdge);
			}
			previousNode = nextNode;
		}
	}

	/**
	 * Checks if there are directed edges between two nodes.
	 *
	 * @return true if edges exist, false otherwise.
	 */
	protected boolean haveEdgesBetween() {
		// check if edge in between
		DirectedEdge edge = agentNetwork.getDirectedEdgeBetween(tmpOrigin, tmpDestination);

		if (edge == null)
			return false;
		else {
			if (!completeSequence.contains(edge))
				completeSequence.add(edge);
			tmpOrigin = tmpDestination;
			return true;
		}
	}
}
