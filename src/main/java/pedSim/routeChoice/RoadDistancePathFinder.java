package pedSim.routeChoice;

import java.util.ArrayList;
import java.util.HashSet;

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
	 * @param agentProperties the agent properties;
	 * @return a {@code Route} object representing the road-distance shortest path.
	 */
	public Route roadDistance(NodeGraph originNode, NodeGraph destinationNode, Agent agent) {

		this.agent = agent;
		final DijkstraRoadDistance pathfinder = new DijkstraRoadDistance();
		partialSequence = pathfinder.dijkstraAlgorithm(originNode, destinationNode, destinationNode, null, agent);
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
	 * @param sequenceNodes   sequence of intermediate nodes (e.g. on-route marks,
	 *                        gateways) including the origin and the destination
	 *                        nodes;
	 * @param agentProperties the agent properties;
	 * @return a {@code Route} object representing the road-distance shortest path
	 *         for the sequence of nodes.
	 */
	public Route roadDistanceSequence(ArrayList<NodeGraph> sequenceNodes, Agent agent) {

		this.agent = agent;
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

			segmentsToAvoid = new HashSet<>(completeSequence);
			final DijkstraRoadDistance pathfinder = new DijkstraRoadDistance();
			partialSequence = pathfinder.dijkstraAlgorithm(tmpOrigin, tmpDestination, destinationNode, segmentsToAvoid,
					agent);
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