/**
 * Series of functions that support the generation of routes calling different methods.
 *
 */

package sim.app.geo.PedSimCity;
import java.util.ArrayList;

import sim.app.geo.UrbanSim.EdgeGraph;
import sim.app.geo.UrbanSim.NodeGraph;
import sim.app.geo.UrbanSim.Path;
import sim.util.geo.GeomPlanarGraphDirectedEdge;

public class RoutePlanner {

	boolean moveOn = false;
	NodeGraph originNode, destinationNode;
	NodeGraph previousJunction = null;

	ArrayList<NodeGraph> sequenceNodes = new ArrayList<NodeGraph>();
	ArrayList<NodeGraph> centroidsToAvoid = new ArrayList<NodeGraph>();
	ArrayList<GeomPlanarGraphDirectedEdge> completePath =  new ArrayList<GeomPlanarGraphDirectedEdge>();
	AgentProperties ap = new AgentProperties();
	Path path = new Path();

	/**
	 * Road-distance based A) Direct O-D road-distance shortest path
	 *
	 * @param originNode the origin node;
	 * @param destinationNode the destination node;
	 * @param ap the agent properties;
	 */
	public ArrayList<GeomPlanarGraphDirectedEdge> roadDistance(NodeGraph originNode, NodeGraph destinationNode,
			AgentProperties ap) {

		this.ap = ap;
		DijkstraRoadDistance pathfinder = new DijkstraRoadDistance();
		path = pathfinder.dijkstraPath(originNode, destinationNode, destinationNode, null, ap);
		return path.edges;
	}

	/**
	 * Road-distance based B) Sequence [O, ..., D] road-distance shortest path.
	 * It allows to combine multiple navigation strategies (on the basis of on-route marks with road-distance based heuristics).
	 *
	 * @param sequence sequence of intermediate nodes (e.g. on-route marks, gateways) including the origin and the destination nodes;
	 * @param ap the agent properties;
	 */
	public ArrayList<GeomPlanarGraphDirectedEdge> roadDistanceSequence(ArrayList<NodeGraph> sequence, AgentProperties ap) {

		this.ap = ap;
		this.sequenceNodes = new ArrayList<NodeGraph> (sequence);
		// originNode
		NodeGraph tmpOrigin = sequenceNodes.get(0);
		this.destinationNode = sequenceNodes.get(sequenceNodes.size()-1);
		sequenceNodes.remove(0);

		for (NodeGraph tmpDestination : sequenceNodes) {
			moveOn = false;
			// check if this tmpDestination has been traversed already
			if (Path.nodesFromPath(completePath).contains(tmpDestination)) {
				controlPath(tmpDestination);
				tmpOrigin = tmpDestination;
				continue;
			}

			// check if edge in between
			GeomPlanarGraphDirectedEdge edge = tmpOrigin.getDirectedEdgeWith(tmpDestination);
			if (edge != null) {
				if (!completePath.contains(edge)) completePath.add(edge);
				tmpOrigin = tmpDestination;
				continue;
			}

			DijkstraRoadDistance pathfinder = new DijkstraRoadDistance();
			path = pathfinder.dijkstraPath(tmpOrigin, tmpDestination, destinationNode, completePath, ap);

			while (path.edges == null && !moveOn) backtracking(tmpDestination);
			tmpOrigin = tmpDestination;
			if (moveOn) continue;
			checkSequenceEdges(tmpOrigin, tmpDestination);
			completePath.addAll(path.edges);
		}
		return completePath;
	}

	/**
	 * Angular change-based A) Direct O-D Least cumulative angular change shortest path.
	 *
	 * @param originNode the origin node;
	 * @param destinationNode the destination node;
	 * @param centroidsToAvoid possible segments (centroids) to avoid (e.g. already traversed in previous iterations);
	 * @param ap the agent properties;
	 */

	public ArrayList<GeomPlanarGraphDirectedEdge> angularChangeBased(NodeGraph originNode, NodeGraph destinationNode,
			AgentProperties ap) {

		NodeGraph dualOrigin = originNode.getDualNode(originNode, destinationNode, ap.regionBasedNavigation, previousJunction);
		NodeGraph dualDestination = null;
		this.ap = ap;
		previousJunction = null;
		while (dualDestination == dualOrigin || dualDestination == null) dualDestination = destinationNode.getDualNode(
				originNode, destinationNode, ap.regionBasedNavigation, previousJunction);

		if (Path.commonPrimalJunction(dualOrigin, dualDestination) != null) {
			ArrayList<GeomPlanarGraphDirectedEdge> edges = new ArrayList<GeomPlanarGraphDirectedEdge>();
			edges.add(originNode.getDirectedEdgeWith(Path.commonPrimalJunction(dualOrigin, dualDestination)));
			edges.add(Path.commonPrimalJunction(dualOrigin, dualDestination).getDirectedEdgeWith(destinationNode));
			return edges;
		}

		if (ap.localHeuristic.equals("angularChange")) {
			DijkstraAngularChange pathfinder = new DijkstraAngularChange();
			path = pathfinder.dijkstraPath(dualOrigin, dualDestination, destinationNode, centroidsToAvoid, previousJunction, ap);
		}
		else if (ap.localHeuristic.equals("turns")) {
			DijkstraTurns pathfinder = new DijkstraTurns();
			path = pathfinder.dijkstraPath(dualOrigin, dualDestination, destinationNode, centroidsToAvoid, previousJunction, ap);
		}

		cleanDualPath(originNode, destinationNode);
		return path.edges;
	}

	/**
	 * Angular change-based B) Sequence [O, ..., D] Least cumulative angular change shortest path.
	 * It allows to combine multiple navigation strategies with angular change based heuristics.
	 *
	 * @param sequence sequence of intermediate nodes (e.g. on-route marks, gateways) including the origin and the destination nodes;
	 * @param ap the agent properties;
	 */

	public ArrayList<GeomPlanarGraphDirectedEdge> angularChangeBasedSequence(ArrayList<NodeGraph> sequence,	AgentProperties ap) {

		this.ap = ap;
		this.sequenceNodes = new ArrayList<NodeGraph> (sequence);
		// originNode
		NodeGraph tmpOrigin = originNode = sequenceNodes.get(0);
		this.destinationNode = sequenceNodes.get(sequenceNodes.size()-1);
		sequenceNodes.remove(0);

		for (NodeGraph tmpDestination : sequenceNodes) {


			moveOn = false; //for path cleaning and already traversed edges
			if (tmpOrigin != originNode) {
				previousJunction = Path.previousJunction(completePath);
				centroidsToAvoid = Path.centroidsFromPath(completePath);
			}
			// check if tmpDestination traversed already
			if (Path.nodesFromPath(completePath).contains(tmpDestination)) {
				controlPath(tmpDestination);
				tmpOrigin = tmpDestination;
				continue;
			}

			// check if edge in between
			GeomPlanarGraphDirectedEdge edge = tmpOrigin.getDirectedEdgeWith(tmpDestination);
			if (edge != null) {
				if (!completePath.contains(edge)) completePath.add(edge);
				tmpOrigin = tmpDestination;
				continue;
			}

			NodeGraph tmpDualOrigin = tmpOrigin.getDualNode(tmpOrigin, tmpDestination, ap.regionBasedNavigation, previousJunction);

			while (tmpDualOrigin == null && previousJunction != null) {
				tmpOrigin = (NodeGraph) completePath.get(completePath.size()-1).getFromNode();
				// remove last one which did not work!
				completePath.remove(completePath.size()-1);
				centroidsToAvoid.remove(centroidsToAvoid.size()-1);
				// take new previous junction
				previousJunction = Path.previousJunction(completePath);
				edge = tmpOrigin.getDirectedEdgeWith(tmpDestination);

				if (edge != null) {
					if (!completePath.contains(edge)) completePath.add(edge);
					tmpOrigin = tmpDestination;
					break;
				}
				tmpDualOrigin = tmpOrigin.getDualNode(tmpOrigin, tmpDestination, ap.regionBasedNavigation, previousJunction);
			}
			if (tmpOrigin == tmpDestination) continue;

			NodeGraph tmpDualDestination = null;
			while ((tmpDualDestination == tmpDualOrigin) || (tmpDualDestination == null)) tmpDualDestination = tmpDestination.getDualNode(
					tmpOrigin, tmpDestination, ap.regionBasedNavigation, previousJunction);

			// check if just one node separates them
			if (Path.commonPrimalJunction(tmpDualOrigin, tmpDualDestination) != null) {
				completePath.add(tmpOrigin.getDirectedEdgeWith(Path.commonPrimalJunction(tmpDualOrigin, tmpDualDestination)));
				completePath.add(Path.commonPrimalJunction(tmpDualOrigin, tmpDualDestination).getDirectedEdgeWith(tmpDestination));
				tmpOrigin = tmpDestination;
				continue;
			}

			if (ap.localHeuristic.equals("angularChange")) {
				DijkstraAngularChange pathfinder = new DijkstraAngularChange();
				path = pathfinder.dijkstraPath(tmpDualOrigin, tmpDualDestination, destinationNode, centroidsToAvoid, tmpOrigin, ap);
			}
			else if (ap.localHeuristic.equals("turns")) {
				DijkstraTurns pathfinder = new DijkstraTurns();
				path = pathfinder.dijkstraPath(tmpDualOrigin, tmpDualDestination, destinationNode, centroidsToAvoid, tmpOrigin, ap);
			}

			while (path.edges == null && !moveOn) backtrackingDual(tmpDualOrigin, tmpDualDestination, tmpDestination);
			if (path.edges == null) continue;
			tmpOrigin = tmpDestination;
			if (moveOn) continue;

			cleanDualPath(tmpOrigin, tmpDestination);
			completePath.addAll(path.edges);
		}
		return completePath;
	}

	/**
	 * Global landmarks path a)
	 *
	 * It returns the path that maximises global landmarkness between an origin and a destination.
	 * @param originNode the origin node;
	 * @param destinationNode the destination node;
	 * @param ap the agent properties;
	 *
	 */
	public ArrayList<GeomPlanarGraphDirectedEdge> globalLandmarksPath (NodeGraph originNode, NodeGraph destinationNode, AgentProperties ap) {
		this.ap = ap;
		DijkstraGlobalLandmarks pathfinder = new DijkstraGlobalLandmarks();
		path = pathfinder.dijkstraPath(originNode, destinationNode, destinationNode, null, ap);
		return path.edges;
	}

	/**
	 * Global landmarks path B) Sequence [O, ..., D] Maximise global landmarkness through a sequence of intermediate nodes
	 *
	 * @param sequence sequence of intermediate nodes (e.g. on-route marks, gateways) including the origin and the destination nodes;
	 * @param ap the agent properties;
	 *
	 */
	public ArrayList<GeomPlanarGraphDirectedEdge> globalLandmarksPathSequence(ArrayList<NodeGraph> sequence, AgentProperties ap) {

		this.ap = ap;
		this.sequenceNodes = new ArrayList<NodeGraph> (sequence);
		// originNode
		NodeGraph tmpOrigin = sequenceNodes.get(0);
		this.destinationNode = sequenceNodes.get(sequenceNodes.size()-1);
		sequenceNodes.remove(0);

		for (NodeGraph tmpDestination : sequenceNodes) {
			moveOn = false;
			// check if this tmpDestination has been traversed already
			if (Path.nodesFromPath(completePath).contains(tmpDestination)) {
				controlPath(tmpDestination);
				tmpOrigin = tmpDestination;
				continue;
			}

			// check if edge in between
			GeomPlanarGraphDirectedEdge edge = tmpOrigin.getDirectedEdgeWith(tmpDestination);
			if (edge != null) {
				if (!completePath.contains(edge)) completePath.add(edge);
				tmpOrigin = tmpDestination;
				continue;
			}

			DijkstraGlobalLandmarks pathfinder = new DijkstraGlobalLandmarks();
			path = pathfinder.dijkstraPath(tmpOrigin, tmpDestination, destinationNode, completePath, ap);

			while (path.edges == null && !moveOn) backtracking(tmpDestination);
			tmpOrigin = tmpDestination;
			if (moveOn) continue;
			checkSequenceEdges(tmpOrigin, tmpDestination);
			completePath.addAll(path.edges);
		}
		return completePath;
	}

	/**
	 * Region- or Region- and barrier-based path
	 *
	 * @param originNode the origin node;
	 * @param destinationNode the destination node;
	 * @param ap the agent properties;
	 *
	 */
	public ArrayList<GeomPlanarGraphDirectedEdge> regionBarrierBasedPath (NodeGraph originNode, NodeGraph destinationNode, AgentProperties ap) {
		this.ap = ap;
		RegionBasedNavigation regionsPath = new RegionBasedNavigation();
		ArrayList<NodeGraph> sequenceRegions = regionsPath.sequenceRegions(originNode, destinationNode, ap);
		ArrayList<GeomPlanarGraphDirectedEdge> path =  new ArrayList<GeomPlanarGraphDirectedEdge>();
		if (ap.localHeuristic.equals("roadDistance")) path = roadDistanceSequence(sequenceRegions, ap);
		else path = angularChangeBasedSequence(sequenceRegions, ap);
		return path;
	}

	/**
	 * Barrier-based path
	 *
	 * @param originNode the origin node;
	 * @param destinationNode the destination node;
	 * @param ap the agent properties;
	 *
	 */
	public ArrayList<GeomPlanarGraphDirectedEdge> barrierBasedPath (NodeGraph originNode, NodeGraph destinationNode, AgentProperties ap) {

		this.ap = ap;
		ArrayList<GeomPlanarGraphDirectedEdge> path =  new ArrayList<GeomPlanarGraphDirectedEdge>();
		BarrierBasedNavigation barrierBasedPath = new BarrierBasedNavigation();
		ArrayList<NodeGraph> sequenceBarriers = barrierBasedPath.sequenceBarriers(originNode, destinationNode, ap.typeBarriers);
		if (ap.localHeuristic.equals("roadDistance")) path = roadDistanceSequence(sequenceBarriers, ap);
		else if (ap.localHeuristic.equals("angularChange")) path = angularChangeBasedSequence(sequenceBarriers, ap);
		return path;
	}

	//Utility functions to clean paths and check dual-paths

	/**
	 * Backtracking function for sequences and landmarks, primal graph-based path (road distance).
	 * When the agent gets stuck because of the "segmentsToAvoid", the function iterates back across the nodes
	 * and retries to compute the path towards the given tmpDestinationNode.
	 *
	 * @param tmpDestination the examined intermediate destination node;
	 */
	private void backtracking (NodeGraph tmpDestination)
	{
		// new tmpOrigin
		NodeGraph tmpOrigin = (NodeGraph) completePath.get(completePath.size()-1).getFromNode();
		// remove the last problematic segment;
		completePath.remove(completePath.size()-1);

		// check if there's a segment between the new tmpOrigin and the destination
		GeomPlanarGraphDirectedEdge edge = tmpOrigin.getDirectedEdgeWith(tmpDestination);
		if (edge != null) {
			if (!completePath.contains(edge)) completePath.add(edge);
			moveOn = true; // no need to backtracking anymore
			return;
		}
		// if not, try to compute the path from the new tmpOrigin
		DijkstraRoadDistance pathFinder = new DijkstraRoadDistance();
		path = pathFinder.dijkstraPath(tmpOrigin, tmpDestination, destinationNode, completePath, ap);
	}

	/**
	 * Backtracking function for sequences and landmarks, dual graph-based path (angular change).
	 * When the agent gets stuck because of the "centroidsToAvoid", the function iterates back across the nodes
	 * and retries to compute the path towards the given tmpDestinationNode.
	 *
	 * @param tmpDualOrigin the examined dual intermediate origin node;
	 * @param tmpDualDestination the examined dual intermediate destination node;
	 * @param tmpDestination the examined (primal) intermediate destination node;
	 */
	private void backtrackingDual(NodeGraph tmpDualOrigin, NodeGraph tmpDualDestination, NodeGraph tmpDestination)
	{
		// new tmpOrigin
		NodeGraph tmpOrigin;
		try {
			tmpOrigin = (NodeGraph) completePath.get(completePath.size()-1).getFromNode();
		}
		catch(java.lang.ArrayIndexOutOfBoundsException e) {
			path.edges = null;
			return;
		}

		// remove last one which did not work!
		completePath.remove(completePath.size()-1);
		centroidsToAvoid.remove(centroidsToAvoid.size()-1);
		// take new previous junction
		previousJunction = Path.previousJunction(completePath);
		// check if there's a segment between the new tmpOrigin and the destination
		GeomPlanarGraphDirectedEdge edge = tmpOrigin.getDirectedEdgeWith(tmpDestination);

		if (edge != null) {
			if (!completePath.contains(edge)) completePath.add(edge);
			moveOn = true; // no need to backtracking anymore
			return;
		}
		tmpDualOrigin = tmpOrigin.getDualNode(tmpOrigin, tmpDestination, ap.regionBasedNavigation, previousJunction);

		if (ap.localHeuristic.equals("angularChange")) {
			DijkstraAngularChange pathfinder = new DijkstraAngularChange();
			path = pathfinder.dijkstraPath(tmpDualOrigin, tmpDualDestination, destinationNode, centroidsToAvoid, tmpOrigin, ap);
		}
		if (ap.localHeuristic.equals("turns")) {
			DijkstraTurns pathfinder = new DijkstraTurns();
			path = pathfinder.dijkstraPath(tmpDualOrigin, tmpDualDestination, destinationNode, centroidsToAvoid, tmpOrigin, ap);
		}
	}

	/**
	 * It checks whether the destinationNode has been traversed already when formulating a path.
	 *
	 * @param destinationNode the examined primal destination node;
	 */

	private void controlPath(NodeGraph destinationNode)
	{
		for (GeomPlanarGraphDirectedEdge e: completePath) {
			if (e.getToNode() == destinationNode) {
				int lastIndex = completePath.indexOf(e);
				completePath = new ArrayList<GeomPlanarGraphDirectedEdge>(completePath.subList(0, lastIndex+1));
				if (Path.previousJunction(completePath) == destinationNode) completePath.remove(completePath.size()-1);
				return;
			}
		}
	}

	/**
	 * It checks whether the primal destinationNode has been passed in the dual graph (since nodes represent entire
	 * segments in this representation). It also checks the presence of unnecessary edges at the beginning,
	 * in relation to the originNode.
	 *
	 * @param originNode the examined primal origin node;
	 * @param destinationNode the examined primal destination node;
	 */

	private void cleanDualPath(NodeGraph originNode, NodeGraph destinationNode)
	{
		// check if the path is one edge ahead
		if (Path.previousJunction(path.edges) == destinationNode) path.edges.remove(path.edges.size()-1);
		// check presence of a unnecessary edge at the beginning of the path
		if (Path.commonPrimalJunction(((EdgeGraph) path.edges.get(0).getEdge()).dualNode,
				((EdgeGraph) path.edges.get(1).getEdge()).dualNode) == originNode) path.edges.remove(0);
		checkSequenceEdges(originNode, destinationNode);
	}

	/**
	 * It readjusts the order of the GeomPlanarGraphDirectedEdge on the basis of toNode and fromNode.
	 *
	 * @param originNode the examined primal origin node;
	 * @param destinationNode the examined primal destination node;
	 */

	private void checkSequenceEdges(NodeGraph originNode, NodeGraph destinationNode) {

		NodeGraph previousNode = originNode;
		for (GeomPlanarGraphDirectedEdge edge: path.edges) {
			NodeGraph nextNode = (NodeGraph) edge.getToNode();

			// need to swap
			if (nextNode == previousNode) {
				nextNode = (NodeGraph) edge.getFromNode();
				GeomPlanarGraphDirectedEdge correctEdge = previousNode.getDirectedEdgeWith(nextNode);
				path.edges.set(path.edges.indexOf(edge), correctEdge);
			}
			previousNode = nextNode;
		}
	}
}

