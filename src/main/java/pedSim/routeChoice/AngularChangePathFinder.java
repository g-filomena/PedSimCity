package pedSim.routeChoice;

import java.util.ArrayList;
import java.util.HashSet;

import org.locationtech.jts.planargraph.DirectedEdge;

import pedSim.agents.Agent;
import pedSim.dijkstra.DijkstraAngularChange;
import pedSim.engine.PedSimCity;
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
	 * @param agentProperties The agent properties used for route calculation.
	 * @return A Route object representing the calculated route based on angular
	 *         change.
	 */
	public Route angularChangeBased(NodeGraph originNode, NodeGraph destinationNode, Agent agent) {

		previousJunction = null;
		NodeGraph dualOrigin = originNode.getDualNode(originNode, destinationNode, false, previousJunction);
		NodeGraph dualDestination = null;
		while (dualDestination == dualOrigin || dualDestination == null)
			dualDestination = destinationNode.getDualNode(originNode, destinationNode, false, previousJunction);

		NodeGraph commonJunction = route.commonPrimalJunction(dualOrigin, dualDestination);
		if (commonJunction != null) {
			route.directedEdgesSequence.add(PedSimCity.network.getDirectedEdgeBetween(originNode, commonJunction));
			route.directedEdgesSequence.add(PedSimCity.network.getDirectedEdgeBetween(commonJunction, destinationNode));
			return route;
		}

		DijkstraAngularChange dijkstra = new DijkstraAngularChange();
		HashSet<NodeGraph> centroidsToAvoidSet = new HashSet<>(centroidsToAvoid);
		partialSequence = dijkstra.dijkstraAlgorithm(dualOrigin, dualDestination, destinationNode, centroidsToAvoidSet,
				previousJunction, agent);
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
	 * @param sequenceNodes   A list of nodes representing intermediate nodes.
	 * @param agentProperties The agent properties used for route calculation.
	 * @return A Route object representing the calculated sequence of routes based
	 *         on angular change.
	 */
	public Route angularChangeBasedSequence(ArrayList<NodeGraph> sequenceNodes, Agent agent) {

		this.agent = agent;
		this.regionBased = agent.getProperties().regionBasedNavigation;
		this.sequenceNodes = new ArrayList<>(sequenceNodes);

		originNode = sequenceNodes.get(0);
		tmpOrigin = originNode;
		destinationNode = sequenceNodes.get(this.sequenceNodes.size() - 1);
		this.sequenceNodes.remove(0);

		for (final NodeGraph currentNode : this.sequenceNodes) {

			moveOn = false; // for path cleaning and already traversed edges
			tmpDestination = currentNode;
			partialSequence = new ArrayList<DirectedEdge>();

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

			ArrayList<NodeGraph> dualNodesOrigin = getDualNodes(tmpOrigin, previousJunction);
//			while (dualNodesOrigin.isEmpty() && previousJunction != null) {
//				System.out.println("passing there");
//				tmpOrigin = (NodeGraph) completeSequence.get(completeSequence.size() - 1).getFromNode();
//				// remove last one which did not work!
//				completeSequence.remove(completeSequence.size() - 1);
//				centroidsToAvoid.remove(centroidsToAvoid.size() - 1);
//				// take new previous junction
//				if (completeSequence.isEmpty())
//					previousJunction = null;
//				if (haveEdgesBetween())
//					break;
//				dualNodesOrigin = getDualNodes(tmpOrigin, previousJunction);
//			}
			// check again given the new tmpOrigin
//			if (haveEdgesBetween())
//				continue;

			ArrayList<NodeGraph> dualNodesDestination = getDualNodes(tmpDestination, null);

			for (final NodeGraph tmpDualOrigin : dualNodesOrigin) {
				for (final NodeGraph tmpDualDestination : dualNodesDestination) {
					// check if just one node separates them
					NodeGraph commonJunction = route.commonPrimalJunction(tmpDualOrigin, tmpDualDestination);

					if (commonJunction != null) {
						addEdgesCommonJunction(commonJunction);
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
		DirectedEdge first = PedSimCity.network.getDirectedEdgeBetween(tmpOrigin, commonJunction);
		DirectedEdge second = PedSimCity.network.getDirectedEdgeBetween(commonJunction, tmpDestination);
		partialSequence.add(first);
		partialSequence.add(second);
	}
}