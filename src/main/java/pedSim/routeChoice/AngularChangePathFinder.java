package pedSim.routeChoice;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import org.locationtech.jts.planargraph.DirectedEdge;

import pedSim.agents.Agent;
import pedSim.dijkstra.DijkstraAngularChange;
import sim.graph.GraphUtils;
import sim.graph.NodeGraph;

/**
 * A pathfinder for least cumulative angular change based route calculations.
 * This class extends the functionality of the base class PathFinder.
 */
public class AngularChangePathFinder extends PathFinder {

	/**
	 * Formulates the least cumulative angular change shortest path between an
	 * origin and a destination node.
	 *
	 * @param originNode      The origin node for the route.
	 * @param destinationNode The destination node for the route.
	 * @param agent           The agent for which the route is completed.
	 * @return A Route object representing the calculated route based on angular
	 *         change.
	 */
	public Route angularChangeBased(NodeGraph originNode, NodeGraph destinationNode, Agent agent) {

		this.agent = agent;
		previousJunction = null;
		agentNetwork = agent.getCognitiveMap().getKnownNetwork();

		NodeGraph dualOrigin = originNode.getDualNode(originNode, destinationNode, false, previousJunction);
		NodeGraph dualDestination = null;
		while (dualDestination == null || dualDestination.equals(dualOrigin))
			dualDestination = destinationNode.getDualNode(originNode, destinationNode, false, previousJunction);

		NodeGraph commonJunction = GraphUtils.getPrimalJunction(dualOrigin, dualDestination);
		if (commonJunction != null) {
			route.directedEdgesSequence.add(agentNetwork.getDirectedEdgeBetween(originNode, commonJunction));
			route.directedEdgesSequence.add(agentNetwork.getDirectedEdgeBetween(commonJunction, destinationNode));
			return route;
		}

		DijkstraAngularChange dijkstra = new DijkstraAngularChange();
		partialSequence = dijkstra.dijkstraAlgorithm(dualOrigin, dualDestination, destinationNode,
				new HashSet<NodeGraph>(centroidsToAvoid), previousJunction, agent);
		cleanDualPath(originNode, destinationNode);
		route.directedEdgesSequence = partialSequence;
		route.routeSequences();
		return route;
	}

	/**
	 * Formulates the least cumulative angular change path through a sequence of
	 * intermediate nodes [originNode, ..., destinationNode] using the provided
	 * agent properties. It allows combining the angular-change local minimisation
	 * heuristic with navigational strategies based on the usage of urban elements.
	 * 
	 * @param sequenceNodes A list of nodes representing intermediate nodes.
	 * @param agent         The agent for which the route is completed.
	 * @return A Route object representing the calculated sequence of routes based
	 *         on angular change.
	 */
	public Route angularChangeBasedSequence(List<NodeGraph> sequenceNodes, Agent agent) {

		this.agent = agent;
		agentNetwork = agent.getCognitiveMap().getKnownNetwork();
		this.regionBased = agent.getProperties().regionBasedNavigation;
		this.sequenceNodes = new ArrayList<>(sequenceNodes);

		originNode = sequenceNodes.get(0);
		tmpOrigin = originNode;
		destinationNode = sequenceNodes.get(this.sequenceNodes.size() - 1);
		this.sequenceNodes.remove(0);

		for (final NodeGraph currentNode : this.sequenceNodes) {

			moveOn = false; // for path cleaning and already traversed edges
			tmpDestination = currentNode;
			partialSequence = new ArrayList<>();

			if (tmpOrigin != originNode) {
				centroidsToAvoid = route.centroidsFromEdgesSequence(completeSequence);
				previousJunction = route.previousJunction(completeSequence);

				// check if tmpDestination traversed already
				if (route.nodesFromEdgesSequence(completeSequence).contains(tmpDestination)) {
					controlPath(tmpDestination);
					tmpOrigin = tmpDestination;
					continue;
				}
			}
			// check if edge in between
			if (haveEdgesBetween())
				continue;

			List<NodeGraph> dualNodesOrigin = getDualNodes(tmpOrigin, previousJunction);
			List<NodeGraph> dualNodesDestination = getDualNodes(tmpDestination, null);

			for (NodeGraph tmpDualOrigin : dualNodesOrigin) {
				for (NodeGraph tmpDualDestination : dualNodesDestination) {
					// check if just one node separates them
					NodeGraph sharedJunction = GraphUtils.getPrimalJunction(tmpDualOrigin, tmpDualDestination);

					if (sharedJunction != null) {
						addEdgesCommonJunction(sharedJunction);
					} else {
						final DijkstraAngularChange pathfinder = new DijkstraAngularChange();
						HashSet<NodeGraph> centroidsToAvoidSet = new HashSet<>(centroidsToAvoid);
						partialSequence = pathfinder.dijkstraAlgorithm(tmpDualOrigin, tmpDualDestination,
								destinationNode, centroidsToAvoidSet, tmpOrigin, agent);
					}
					if (!partialSequence.isEmpty())
						break;
				}
				if (!partialSequence.isEmpty())
					break;
			}

			while (partialSequence.isEmpty() && !moveOn)
				dualBacktracking();
			if (moveOn) {
				tmpOrigin = tmpDestination;
				continue;
			}
			cleanDualPath(tmpOrigin, tmpDestination);
			completeSequence.addAll(partialSequence);
			tmpOrigin = tmpDestination;
		}
		route.directedEdgesSequence = completeSequence;
		route.routeSequences();
		return route;
	}

	/**
	 * Adds directed edges between the current origin and the common junction node,
	 * and between the common junction and the current destination to the partial
	 * sequence.
	 *
	 * @param commonJunction The common junction node between the origin and
	 *                       destination.
	 */
	private void addEdgesCommonJunction(NodeGraph commonJunction) {
		DirectedEdge first = agentNetwork.getDirectedEdgeBetween(tmpOrigin, commonJunction);
		DirectedEdge second = agentNetwork.getDirectedEdgeBetween(commonJunction, tmpDestination);
		partialSequence.add(first);
		partialSequence.add(second);
	}
}
