/**
 * Series of functions that support the generation of routes calling different methods.
 *
 */

package sim.app.geo.pedSimCity;
import java.util.ArrayList;

import sim.app.geo.urbanSim.EdgeGraph;
import sim.app.geo.urbanSim.NodeGraph;
import sim.app.geo.urbanSim.Utilities;
import sim.app.geo.urbanSim.Utilities.Path;
import sim.util.geo.GeomPlanarGraphDirectedEdge;

public class RoutePlanner {

	boolean regionBasedNavigation = false;
	boolean barrierBasedNavigation = false;
	boolean landmarkBasedNavigation = false;
	boolean moveOn = false;

	NodeGraph originNode, destinationNode;
	NodeGraph previousJunction = null;

	ArrayList<NodeGraph> sequenceNodes = new ArrayList<NodeGraph>();
	ArrayList<NodeGraph> centroidsToAvoid = new ArrayList<NodeGraph>();
	ArrayList<GeomPlanarGraphDirectedEdge> completePath =  new ArrayList<GeomPlanarGraphDirectedEdge>();
	Path path = new Path();
	AgentProperties ap = new AgentProperties();

	/**
	 * Road-distance based A) Direct O-D road-distance shortest path
	 *
	 * @param originNode the origin node;
	 * @param destinationNode the destination node;
	 * @param segmentsToAvoid possible segments to avoid (e.g. already traversed in previous iterations);
	 * @param ap the agent properties;
	 */
	public ArrayList<GeomPlanarGraphDirectedEdge> roadDistance(NodeGraph originNode, NodeGraph destinationNode,
			ArrayList <GeomPlanarGraphDirectedEdge> segmentsToAvoid,  AgentProperties ap) {

		if (ap.algorithm == "astar") {
			AStarRoadDistance pathfinder = new AStarRoadDistance();
			path = pathfinder.astarPath(originNode, destinationNode, segmentsToAvoid, ap);
			return path.edges;
		}
		else {
			DijkstraRoadDistance pathfinder = new DijkstraRoadDistance();
			path = pathfinder.dijkstraPath(originNode, destinationNode, null, segmentsToAvoid, ap);
			return path.edges;
		}
	}

	/**
	 * Road-distance based B) Sequence [O, ..., D] road-distance shortest path - works for local landmarks, regional+barriers, barriers only
	 *
	 * @param sequence sequence of intermediate nodes (e.g. on-route marks, gateways) including the origin and the destination nodes;
	 * @param ap the agent properties;
	 */
	public ArrayList<GeomPlanarGraphDirectedEdge> roadDistanceSequence(ArrayList<NodeGraph> sequence, AgentProperties ap) {

		this.sequenceNodes = new ArrayList<NodeGraph> (sequence);
		// originNode
		NodeGraph tmpOrigin = sequenceNodes.get(0);
		this.destinationNode = sequenceNodes.get(sequenceNodes.size()-1);
		this.regionBasedNavigation = ap.regionBasedNavigation;
		sequenceNodes.remove(0);

		for (NodeGraph tmpDestination : sequenceNodes) {

			moveOn = false;
			// check if this tmpDestination has been traversed already
			if (Utilities.nodesFromPath(completePath).contains(tmpDestination)) {
				controlPath(tmpDestination);
				tmpOrigin = tmpDestination;
				continue;
			}

			// check if edge in between
			GeomPlanarGraphDirectedEdge edge = tmpOrigin.getDirectedEdgeBetween(tmpDestination);
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
	 * Road-distance based C) road-distance minimisation + landmark-based (local and global) path
	 *
	 * @param sequence sequence of intermediate nodes (e.g. on-route marks, gateways) including the origin and the destination nodes;
	 * @param ap the agent properties;
	 */
	public ArrayList<GeomPlanarGraphDirectedEdge> roadDistanceLandmarks(ArrayList<NodeGraph> sequence,
			AgentProperties ap) {

		this.sequenceNodes = new ArrayList<NodeGraph> (sequence);
		// originNode
		NodeGraph tmpOrigin = sequenceNodes.get(0);
		this.destinationNode = sequenceNodes.get(sequenceNodes.size()-1);
		this.regionBasedNavigation = ap.regionBasedNavigation;
		this.landmarkBasedNavigation = true;
		sequenceNodes.remove(0);

		// check if node has been traversed already
		for (NodeGraph tmpDestination : sequenceNodes) {

			moveOn = false;
			if (Utilities.nodesFromPath(completePath).contains(tmpDestination)) {
				controlPath(tmpDestination);
				tmpOrigin = tmpDestination;
				continue;
			}

			// check if edge in between
			GeomPlanarGraphDirectedEdge edge = tmpOrigin.getDirectedEdgeBetween(tmpDestination);
			if (edge != null) {
				if (!completePath.contains(edge)) completePath.add(edge);
				tmpOrigin = tmpDestination;
				continue;
			}

			DijkstraRoadDistance pathFinder = new DijkstraRoadDistance();
			path = pathFinder.dijkstraPath(tmpOrigin, tmpDestination, destinationNode, completePath, ap);
			while (path.edges == null && !moveOn) backtracking(tmpDestination);
			tmpOrigin = tmpDestination;
			if (moveOn) continue;

			checkSequenceEdges(tmpOrigin, tmpDestination);
			completePath.addAll(path.edges);
		}
		return completePath;
	}


	/**
	 * Angular change based A) Direct O-D Least cumulative angular change shortest path
	 *
	 * @param originNode the origin node;
	 * @param destinationNode the destination node;
	 * @param centroidsToAvoid possible segments (centroids) to avoid (e.g. already traversed in previous iterations);
	 * @param previouJunction the last, if any, primal junction traversed, so to avoid to cross it again;
	 * @param ap the agent properties;
	 */

	public ArrayList<GeomPlanarGraphDirectedEdge> angularChange(NodeGraph originNode, NodeGraph destinationNode, ArrayList<NodeGraph>
	centroidsToAvoid, NodeGraph previousJunction, AgentProperties ap) {

		this.regionBasedNavigation = ap.regionBasedNavigation;

		NodeGraph dualOrigin = originNode.getDualNode(originNode, destinationNode, regionBasedNavigation, previousJunction);
		NodeGraph dualDestination = null;

		while (dualDestination == dualOrigin || dualDestination == null) dualDestination = destinationNode.getDualNode(
				originNode, destinationNode, regionBasedNavigation, previousJunction);

		if (Utilities.commonPrimalJunction(dualOrigin, dualDestination) != null) {
			ArrayList<GeomPlanarGraphDirectedEdge> edges = new ArrayList<GeomPlanarGraphDirectedEdge>();
			edges.add(originNode.getDirectedEdgeBetween(Utilities.commonPrimalJunction(dualOrigin, dualDestination)));
			edges.add(Utilities.commonPrimalJunction(dualOrigin, dualDestination).getDirectedEdgeBetween(destinationNode));
			return edges;
		}

		if (ap.algorithm == "astar") {
			AStarAngularChange pathfinder = new AStarAngularChange();
			path = pathfinder.astarPath(dualOrigin, dualDestination, centroidsToAvoid, previousJunction, ap);
		}
		else {
			DijkstraAngularChange pathfinder = new DijkstraAngularChange();
			path = pathfinder.dijkstraPath(dualOrigin, dualDestination, destinationNode, centroidsToAvoid, previousJunction, ap);

		}

		cleanDualPath(originNode, destinationNode);
		return path.edges;
	}

	/**
	 * Angular-change based B) Sequence [O, ..., D] Least cumulative angular change shortest path - works for local landmarks,
	 * regional+barriers, barriers only
	 *
	 * @param sequence sequence of intermediate nodes (e.g. on-route marks, gateways) including the origin and the destination nodes;
	 * @param ap the agent properties;
	 */

	public ArrayList<GeomPlanarGraphDirectedEdge> angularChangeSequence(ArrayList<NodeGraph> sequence,
			AgentProperties ap) {
		this.regionBasedNavigation = ap.regionBasedNavigation;
		this.sequenceNodes = new ArrayList<NodeGraph> (sequence);
		// originNode
		NodeGraph tmpOrigin = originNode = sequenceNodes.get(0);
		this.destinationNode = sequenceNodes.get(sequenceNodes.size()-1);
		sequenceNodes.remove(0);

		for (NodeGraph tmpDestination : sequenceNodes) {
			// check if this tmpDestination has been traversed already
			moveOn = false;
			if (tmpOrigin != originNode) {
				previousJunction = Utilities.previousJunction(completePath);
				centroidsToAvoid = Utilities.centroidsFromPath(completePath);
			}

			if (Utilities.nodesFromPath(completePath).contains(tmpDestination)) {
				controlPath(tmpDestination);
				tmpOrigin = tmpDestination;
				continue;
			}

			// check if edge in between
			GeomPlanarGraphDirectedEdge edge = tmpOrigin.getDirectedEdgeBetween(tmpDestination);
			if (edge != null) {
				if (!completePath.contains(edge)) completePath.add(edge);
				tmpOrigin = tmpDestination;
				continue;
			}
			//TO DO centroids to avoid in dual
			NodeGraph tmpDualOrigin = tmpOrigin.getDualNode(tmpOrigin, tmpDestination, regionBasedNavigation, previousJunction);

			while (tmpDualOrigin == null && previousJunction !=null) {
				tmpOrigin = (NodeGraph) completePath.get(completePath.size()-1).getFromNode();
				// remove last one which did not work!
				completePath.remove(completePath.size()-1);
				centroidsToAvoid.remove(centroidsToAvoid.size()-1);
				// take new previous junction
				previousJunction = Utilities.previousJunction(completePath);
				edge = tmpOrigin.getDirectedEdgeBetween(tmpDestination);
				if (edge != null) {
					if (!completePath.contains(edge)) completePath.add(edge);
					tmpOrigin = tmpDestination;
					break;
				}
				tmpDualOrigin = tmpOrigin.getDualNode(tmpOrigin, tmpDestination, regionBasedNavigation, previousJunction);
			}
			if (tmpOrigin == tmpDestination) continue;

			NodeGraph tmpDualDestination = null;
			while ((tmpDualDestination == tmpDualOrigin) || (tmpDualDestination == null)) tmpDualDestination = tmpDestination.getDualNode(
					tmpOrigin, tmpDestination, regionBasedNavigation, previousJunction);

			// check if just one node separates them
			if (Utilities.commonPrimalJunction(tmpDualOrigin, tmpDualDestination) != null) {
				completePath.add(tmpOrigin.getDirectedEdgeBetween(Utilities.commonPrimalJunction(tmpDualOrigin, tmpDualDestination)));
				completePath.add(Utilities.commonPrimalJunction(tmpDualOrigin, tmpDualDestination).getDirectedEdgeBetween(tmpDestination));
				tmpOrigin = tmpDestination;
				continue;
			}

			DijkstraAngularChange pathFinder = new DijkstraAngularChange();
			path = pathFinder.dijkstraPath(tmpDualOrigin, tmpDualDestination, destinationNode, centroidsToAvoid, previousJunction, ap);

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
	 * Angular change based C) Cumulative angular change minimisation + landmark-based (local and global) path
	 *
	 * @param sequence sequence of intermediate nodes (e.g. on-route marks, gateways) including the origin and the destination nodes;
	 * @param ap the agent properties;
	 */
	public ArrayList<GeomPlanarGraphDirectedEdge> angularChangeLandmarks(ArrayList<NodeGraph> sequence,
			AgentProperties ap) {

		this.regionBasedNavigation = ap.regionBasedNavigation;
		this.landmarkBasedNavigation = true;
		this.sequenceNodes = new ArrayList<NodeGraph> (sequence);

		// originNode
		NodeGraph tmpOrigin = originNode = sequenceNodes.get(0);
		this.destinationNode = sequenceNodes.get(sequenceNodes.size()-1);
		sequenceNodes.remove(0);

		for (NodeGraph tmpDestination : sequenceNodes) {
			moveOn = false;

			if (tmpOrigin != originNode) {
				previousJunction = Utilities.previousJunction(completePath);
				centroidsToAvoid = Utilities.centroidsFromPath(completePath);
			}

			// check if this tmpDestination has been traversed already
			if (Utilities.nodesFromPath(completePath).contains(tmpDestination)) {
				controlPath(tmpDestination);
				tmpOrigin = tmpDestination;
				continue;
			}

			// check if edge in between
			GeomPlanarGraphDirectedEdge edge = tmpOrigin.getDirectedEdgeBetween(tmpDestination);
			if (edge != null) {
				if (!completePath.contains(edge)) completePath.add(edge);
				tmpOrigin = tmpDestination;
				continue;
			}

			// TO DO centroids to avoid in dual
			NodeGraph tmpDualOrigin = tmpOrigin.getDualNode(tmpOrigin, tmpDestination, regionBasedNavigation, previousJunction);
			NodeGraph tmpDualDestination = null;
			while (tmpDualDestination == tmpDualOrigin || tmpDualDestination == null) tmpDualDestination = tmpDestination.getDualNode(
					tmpOrigin, tmpDestination, regionBasedNavigation, previousJunction);

			// check if just one node separates them
			if (Utilities.commonPrimalJunction(tmpDualOrigin, tmpDualDestination) != null) {
				completePath.add(tmpOrigin.getDirectedEdgeBetween(Utilities.commonPrimalJunction(tmpDualOrigin, tmpDualDestination)));
				completePath.add(Utilities.commonPrimalJunction(tmpDualOrigin, tmpDualDestination).getDirectedEdgeBetween(tmpDestination));
				tmpOrigin = tmpDestination;
				continue;
			}

			DijkstraAngularChange pathFinder = new DijkstraAngularChange();
			path = pathFinder.dijkstraPath(tmpDualOrigin, tmpDualDestination, destinationNode, centroidsToAvoid, previousJunction, ap);

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
	 * Global landmarks path
	 *
	 * It returns the path that maximises global landmarkness between an origin and a destination.
	 * @param originNode the origin node;
	 * @param destinationNode the destination node;
	 * @param segmentsToAvoid possible segments to avoid (e.g. already traversed in previous iterations);
	 * @param ap the agent properties;
	 *
	 */
	public ArrayList<GeomPlanarGraphDirectedEdge> globalLandmarksPath (NodeGraph originNode, NodeGraph destinationNode,
			ArrayList<GeomPlanarGraphDirectedEdge> segmentsToAvoid, AgentProperties ap) {

		DijkstraGlobalLandmarks pathfinder = new DijkstraGlobalLandmarks();
		path = pathfinder.dijkstraPath(originNode, destinationNode, segmentsToAvoid, ap.onlyAnchors);
		return path.edges;
	}

	/**
	 * Region- or Region- and barrier-based path
	 *
	 * @param originNode the origin node;
	 * @param destinationNode the destination node;
	 * @param ap the agent properties;
	 *
	 */
	public ArrayList<GeomPlanarGraphDirectedEdge> regionBarrierBasedPath (NodeGraph originNode, NodeGraph destinationNode,
			AgentProperties ap) {
		this.regionBasedNavigation = ap.regionBasedNavigation;
		this.barrierBasedNavigation = ap.barrierBasedNavigation;

		RegionBasedNavigation regionsPath = new RegionBasedNavigation();
		ArrayList<NodeGraph> regionsSequence = regionsPath.sequenceRegions(originNode, destinationNode, barrierBasedNavigation, ap.typeOfBarriers);
		ArrayList<GeomPlanarGraphDirectedEdge> path =  new ArrayList<GeomPlanarGraphDirectedEdge>();

		if (ap.localHeuristic == "roadDistance") path = roadDistanceSequence(regionsSequence, ap);
		else if (ap.localHeuristic == "angularChange") path = angularChangeSequence(regionsSequence, ap);
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
	public ArrayList<GeomPlanarGraphDirectedEdge> barrierBasedPath (NodeGraph originNode, NodeGraph destinationNode,
			AgentProperties ap) {

		this.landmarkBasedNavigation = ap.landmarkBasedNavigation;
		this.regionBasedNavigation = ap.regionBasedNavigation;
		this.barrierBasedNavigation = ap.barrierBasedNavigation;

		ArrayList<GeomPlanarGraphDirectedEdge> path =  new ArrayList<GeomPlanarGraphDirectedEdge>();
		BarrierBasedNavigation barrierBasedPath = new BarrierBasedNavigation();
		ArrayList<NodeGraph> sequenceBarriers = barrierBasedPath.sequenceBarriers(originNode, destinationNode, ap.typeOfBarriers);
		if (ap.localHeuristic == "roadDistance") path = roadDistanceSequence(sequenceBarriers, ap);
		else if (ap.localHeuristic == "angularChange") path = angularChangeSequence(sequenceBarriers, ap);
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
		GeomPlanarGraphDirectedEdge edge = tmpOrigin.getDirectedEdgeBetween(tmpDestination);
		if (edge != null) {
			if (!completePath.contains(edge)) completePath.add(edge);
			moveOn = true;
			return;
		}
		// if not, try to compute the path from the new tmpOrigin
		if (this.landmarkBasedNavigation) {
			DijkstraRoadDistance pathFinder = new DijkstraRoadDistance();
			path = pathFinder.dijkstraPath(tmpOrigin, tmpDestination, destinationNode, completePath, ap);
		}
		else {
			DijkstraRoadDistance pathFinder = new DijkstraRoadDistance();
			path = pathFinder.dijkstraPath(tmpOrigin, tmpDestination, null,  completePath, ap);
		}
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
		previousJunction = Utilities.previousJunction(completePath);
		// check if there's a segment between the new tmpOrigin and the destination
		GeomPlanarGraphDirectedEdge edge = tmpOrigin.getDirectedEdgeBetween(tmpDestination);

		if (edge != null) {
			if (!completePath.contains(edge)) completePath.add(edge);
			moveOn = true;
			return;
		}
		tmpDualOrigin = tmpOrigin.getDualNode(tmpOrigin, tmpDestination, this.regionBasedNavigation, previousJunction);

		if (this.landmarkBasedNavigation) {
			DijkstraAngularChange pathFinder = new DijkstraAngularChange();
			path = pathFinder.dijkstraPath(tmpDualOrigin, tmpDualDestination, destinationNode,	centroidsToAvoid, previousJunction, ap);
		}
		else {
			DijkstraAngularChange pathFinder = new DijkstraAngularChange();
			path = pathFinder.dijkstraPath(tmpDualOrigin, tmpDualDestination, destinationNode, centroidsToAvoid, previousJunction, ap);
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
				if (Utilities.previousJunction(completePath) == destinationNode) completePath.remove(completePath.size()-1);
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
		if (Utilities.previousJunction(path.edges) == destinationNode) path.edges.remove(path.edges.size()-1);
		// check presence of a unnecessary edge at the beginning of the path
		if (Utilities.commonPrimalJunction(((EdgeGraph) path.edges.get(0).getEdge()).dualNode,
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
				GeomPlanarGraphDirectedEdge correctEdge = previousNode.getDirectedEdgeBetween(nextNode);
				path.edges.set(path.edges.indexOf(edge), correctEdge);
			}
			previousNode = nextNode;
		}
	}

}

