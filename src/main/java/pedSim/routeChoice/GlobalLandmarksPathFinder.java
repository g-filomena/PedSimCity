/**
 * Series of functions that support the generation of routes calling different methods.
 *
 */

package pedSim.routeChoice;

import java.util.ArrayList;

import pedSim.agents.Agent;
import pedSim.dijkstra.DijkstraGlobalLandmarks;
import sim.graph.NodeGraph;

public class GlobalLandmarksPathFinder extends PathFinder {

	public GlobalLandmarksPathFinder(NodeGraph originNode, NodeGraph destinationNode, Agent agent) {
		this.originNode = originNode;
		this.destinationNode = destinationNode;
		this.agent = agent;
	}

	/**
	 * Formulates a route based on global landmarkness maximisation between the
	 * origin and destination nodes only.
	 *
	 * @return The computed route.
	 */
	public Route globalLandmarksPath() {
		DijkstraGlobalLandmarks pathfinder = new DijkstraGlobalLandmarks();
		partialSequence = pathfinder.dijkstraAlgorithm(originNode, destinationNode, destinationNode, null, agent);
		route.directedEdgesSequence = partialSequence;
		route.routeSequences();
		return route;
	}

	/**
	 * Formulates a route based on global landmarkness maximisation through a
	 * sequence of intermediate nodes [originNode, ..., destinationNode]. It allows
	 * combining global landmarkness maximisation with a sequence of nodes resulting
	 * for example from the region-based navigation.
	 *
	 * @param sequenceNodes A list of nodes representing the sequence to follow.
	 * @return The computed route.
	 */
	public Route globalLandmarksPathSequence(ArrayList<NodeGraph> sequenceNodes) {

		this.sequenceNodes = new ArrayList<>(sequenceNodes);
		// originNode
		tmpOrigin = this.sequenceNodes.get(0);
		this.sequenceNodes.remove(0);

		for (NodeGraph tmpDestination : this.sequenceNodes) {
			moveOn = false;
			// check if this tmpDestination has been traversed already
			if (route.nodesFromEdgesSequence(completeSequence).contains(tmpDestination)) {
				controlPath(tmpDestination);
				tmpOrigin = tmpDestination;
				continue;
			}

			// check if edge in between
			if (haveEdgesBetween())
				continue;

			final DijkstraGlobalLandmarks pathfinder = new DijkstraGlobalLandmarks();
			partialSequence = pathfinder.dijkstraAlgorithm(tmpOrigin, tmpDestination, destinationNode, null, agent);

			while (partialSequence.isEmpty() && !moveOn)
				backtracking(tmpDestination);
			tmpOrigin = tmpDestination;
			if (moveOn)
				continue;

			checkEdgesSequence(tmpOrigin);
			completeSequence.addAll(partialSequence);
		}

		route.directedEdgesSequence = completeSequence;
		route.routeSequences();
		return route;
	}
}
