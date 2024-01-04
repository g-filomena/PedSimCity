package pedSim.routeChoice;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import pedSim.agents.Agent;
import pedSim.dijkstra.DijkstraRoadDistance;
import sim.graph.NodeGraph;

/**
 * A pathfinder for road-distance based route calculations. This class extends
 * the functionality of the base class PathFinder.
 */
public class RoadDistancePathFinder extends PathFinder {

	/**
	 * Formulates a route based on road distance between the given origin and
	 * destination nodes using the provided agent properties.
	 * 
	 * @param originNode      the origin node;
	 * @param destinationNode the destination node;
	 * @param agent           The agent for which the route is computed.
	 * @return a {@code Route} object representing the road-distance shortest path.
	 */
	public Route roadDistance(NodeGraph originNode, NodeGraph destinationNode, Agent agent) {

		this.agent = agent;
		agentNetwork = agent.getCognitiveMap().getKnownNetwork();
		final DijkstraRoadDistance pathfinder = new DijkstraRoadDistance();
		partialSequence = pathfinder.dijkstraAlgorithm(originNode, destinationNode, destinationNode,
				directedEdgesToAvoid, agent);
		route.directedEdgesSequence = partialSequence;
		route.routeSequences();
		return route;
	}

	/**
	 * Formulates a route based on road distance minimisation through a sequence of
	 * intermediate nodes [originNode, ..., destinationNode] using the provided
	 * agent properties. It allows combining the road-distance local minimisation
	 * heuristic with navigational strategies based on the usage of urban elements.
	 *
	 * @param sequenceNodes sequence of intermediate nodes (e.g. on-route marks,
	 *                      gateways) including the origin and the destination
	 *                      nodes;
	 * @param agent         The agent for which the route is computed.
	 * @return a `Route' object representing the road-distance shortest path for the
	 *         sequence of nodes.
	 */
	public Route roadDistanceSequence(List<NodeGraph> sequenceNodes, Agent agent) {

		this.agent = agent;
		agentNetwork = agent.getCognitiveMap().getKnownNetwork();
		this.sequenceNodes = new ArrayList<>(sequenceNodes);

		// originNode
		originNode = this.sequenceNodes.get(0);
		tmpOrigin = originNode;
		destinationNode = sequenceNodes.get(sequenceNodes.size() - 1);
		this.sequenceNodes.remove(0);

		for (final NodeGraph currentNode : this.sequenceNodes) {
			moveOn = false;
			tmpDestination = currentNode;

			// check if this tmpDestination has been traversed already
			if (route.nodesFromEdgesSequence(completeSequence).contains(tmpDestination)) {
				controlPath(tmpDestination);
				tmpOrigin = tmpDestination;
				continue;
			}

			if (haveEdgesBetween())
				continue;

			directedEdgesToAvoid = new HashSet<>(completeSequence);
			DijkstraRoadDistance pathfinder = new DijkstraRoadDistance();
			partialSequence = pathfinder.dijkstraAlgorithm(tmpOrigin, tmpDestination, destinationNode,
					directedEdgesToAvoid, agent);
			while (partialSequence.isEmpty() && !moveOn)
				backtracking(tmpDestination);

			if (moveOn) {
				if (tmpOrigin == originNode)
					continue;
				tmpOrigin = tmpDestination;
				continue;
			}
			checkEdgesSequence(tmpOrigin);
			completeSequence.addAll(partialSequence);
			tmpOrigin = tmpDestination;
		}
		route.directedEdgesSequence = completeSequence;
		route.routeSequences();
		return route;
	}
}
