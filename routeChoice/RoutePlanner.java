/**
 * Series of functions that support the generation of routes calling different methods.
 *
 */

package pedsimcity.routeChoice;

import java.util.ArrayList;

import pedsimcity.agents.AgentProperties;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import urbanmason.main.EdgeGraph;
import urbanmason.main.NodeGraph;
import urbanmason.main.Path;

public class RoutePlanner {

	boolean moveOn = false;
	NodeGraph originNode, destinationNode;
	NodeGraph tmpOrigin;
	NodeGraph previousJunction = null;

	ArrayList<NodeGraph> sequenceNodes = new ArrayList<>();
	ArrayList<NodeGraph> centroidsToAvoid = new ArrayList<>();
	ArrayList<GeomPlanarGraphDirectedEdge> completePath = new ArrayList<>();
	AgentProperties ap = new AgentProperties();
	Path path = new Path();

	/**
	 * Road-distance based A) Direct O-D road-distance shortest path
	 *
	 * @param originNode      the origin node;
	 * @param destinationNode the destination node;
	 * @param ap              the agent properties;
	 */
	public ArrayList<GeomPlanarGraphDirectedEdge> roadDistance(NodeGraph originNode, NodeGraph destinationNode,
			AgentProperties ap) {

		this.ap = ap;
		final DijkstraRoadDistance pathfinder = new DijkstraRoadDistance();
		this.path = pathfinder.dijkstraPath(originNode, destinationNode, destinationNode, null, ap);
		return this.path.edges;
	}

	/**
	 * Road-distance based B) Sequence [O, ..., D] road-distance shortest path. It
	 * allows to combine multiple navigation strategies (on the basis of on-route
	 * marks with road-distance based heuristics).
	 *
	 * @param sequence sequence of intermediate nodes (e.g. on-route marks,
	 *                 gateways) including the origin and the destination nodes;
	 * @param ap       the agent properties;
	 */
	public ArrayList<GeomPlanarGraphDirectedEdge> roadDistanceSequence(ArrayList<NodeGraph> sequence,
			AgentProperties ap) {

		this.ap = ap;
		this.sequenceNodes = new ArrayList<>(sequence);
		// originNode
		this.tmpOrigin = this.sequenceNodes.get(0);
		this.destinationNode = this.sequenceNodes.get(this.sequenceNodes.size() - 1);
		this.sequenceNodes.remove(0);

		for (final NodeGraph tmpDestination : this.sequenceNodes) {
			this.moveOn = false;
			// check if this tmpDestination has been traversed already
			if (Path.nodesFromPath(this.completePath).contains(tmpDestination)) {
				this.controlPath(tmpDestination);
				this.tmpOrigin = tmpDestination;
				continue;
			}

			// check if edge in between
			final GeomPlanarGraphDirectedEdge edge = this.tmpOrigin.getDirectedEdgeWith(tmpDestination);
			if (edge != null) {
				if (!this.completePath.contains(edge))
					this.completePath.add(edge);
				this.tmpOrigin = tmpDestination;
				continue;
			}

			final DijkstraRoadDistance pathfinder = new DijkstraRoadDistance();
			this.path = pathfinder.dijkstraPath(this.tmpOrigin, tmpDestination, this.destinationNode, this.completePath,
					ap);

			while (this.path.edges.size() == 0 && !this.moveOn)
				this.backtracking(tmpDestination);
			this.tmpOrigin = tmpDestination;
			if (this.moveOn)
				continue;
			this.checkSequenceEdges(this.tmpOrigin, tmpDestination);
			this.completePath.addAll(this.path.edges);
		}
		return this.completePath;
	}

	/**
	 * Angular change-based A) Direct O-D Least cumulative angular change shortest
	 * path.
	 *
	 * @param originNode       the origin node;
	 * @param destinationNode  the destination node;
	 * @param centroidsToAvoid possible segments (centroids) to avoid (e.g. already
	 *                         traversed in previous iterations);
	 * @param ap               the agent properties;
	 */
	public ArrayList<GeomPlanarGraphDirectedEdge> angularChangeBased(NodeGraph originNode, NodeGraph destinationNode,
			AgentProperties ap) {

		final NodeGraph dualOrigin = originNode.getDualNode(originNode, destinationNode, ap.regionBasedNavigation,
				this.previousJunction);
		NodeGraph dualDestination = null;
		this.ap = ap;
		this.previousJunction = null;
		while (dualDestination == dualOrigin || dualDestination == null)
			dualDestination = destinationNode.getDualNode(originNode, destinationNode, ap.regionBasedNavigation,
					this.previousJunction);

		if (Path.commonPrimalJunction(dualOrigin, dualDestination) != null) {
			final ArrayList<GeomPlanarGraphDirectedEdge> edges = new ArrayList<>();
			edges.add(originNode.getDirectedEdgeWith(Path.commonPrimalJunction(dualOrigin, dualDestination)));
			edges.add(Path.commonPrimalJunction(dualOrigin, dualDestination).getDirectedEdgeWith(destinationNode));
			return edges;
		}

		if (ap.localHeuristic.equals("angularChange") || ap.onlyMinimising.equals("angularChange")) {
			final DijkstraAngularChange pathfinder = new DijkstraAngularChange();
			this.path = pathfinder.dijkstraPath(dualOrigin, dualDestination, destinationNode, this.centroidsToAvoid,
					this.previousJunction, ap);
		} else if (ap.localHeuristic.equals("turns") || ap.onlyMinimising.equals("turns")) {
			final DijkstraIntersections pathfinder = new DijkstraIntersections();
			this.path = pathfinder.dijkstraPath(dualOrigin, dualDestination, destinationNode, this.centroidsToAvoid,
					this.previousJunction, ap);
		}
//		System.out.println(originNode.nodeID + " " + destinationNode.nodeID + "  " + this.path.edges.size());

		this.cleanDualPath(originNode, destinationNode);
		return this.path.edges;
	}

	/**
	 * Angular change-based B) Sequence [O, ..., D] Least cumulative angular change
	 * shortest path. It allows to combine multiple navigation strategies with
	 * angular change based heuristics.
	 *
	 * @param sequence sequence of intermediate nodes (e.g. on-route marks,
	 *                 gateways) including the origin and the destination nodes;
	 * @param ap       the agent properties;
	 */

	public ArrayList<GeomPlanarGraphDirectedEdge> angularChangeBasedSequence(ArrayList<NodeGraph> sequence,
			AgentProperties ap) {

		this.ap = ap;
		this.sequenceNodes = new ArrayList<>(sequence);
		// List<Integer> opo = new ArrayList<Integer>();
		// for (NodeGraph n : sequenceNodes) opo.add(n.getID());
		// System.out.println(Arrays.asList(opo));

		// originNode
		this.tmpOrigin = this.originNode = this.sequenceNodes.get(0);
		this.destinationNode = this.sequenceNodes.get(this.sequenceNodes.size() - 1);
		this.sequenceNodes.remove(0);

		for (final NodeGraph tmpDestination : this.sequenceNodes) {
			this.moveOn = false; // for path cleaning and already traversed edges

			if (this.tmpOrigin != this.originNode) {
				this.path.edges.clear();
				this.path.mapWrappers.clear();
				this.centroidsToAvoid = Path.centroidsFromPath(this.completePath);
				this.previousJunction = Path.previousJunction(this.completePath);

				// check if tmpDestination traversed already
				if (Path.nodesFromPath(this.completePath).contains(tmpDestination)) {
					this.controlPath(tmpDestination);
					this.tmpOrigin = tmpDestination;
					continue;
				}
			}

			// check if edge in between
			GeomPlanarGraphDirectedEdge edge = this.tmpOrigin.getDirectedEdgeWith(tmpDestination);
			if (edge != null) {
				if (!this.completePath.contains(edge))
					this.completePath.add(edge);
				this.tmpOrigin = tmpDestination;
				continue;
			}

			ArrayList<NodeGraph> dualNodesTO = new ArrayList<>(this.tmpOrigin
					.getDualNodes(this.tmpOrigin, tmpDestination, ap.regionBasedNavigation, this.previousJunction)
					.keySet());

			while (dualNodesTO.size() == 0 && this.previousJunction != null) {
				this.tmpOrigin = (NodeGraph) this.completePath.get(this.completePath.size() - 1).getFromNode();
				// remove last one which did not work!
				this.completePath.remove(this.completePath.size() - 1);
				this.centroidsToAvoid.remove(this.centroidsToAvoid.size() - 1);
				// take new previous junction
				this.previousJunction = Path.previousJunction(this.completePath);
				edge = this.tmpOrigin.getDirectedEdgeWith(tmpDestination);

				if (edge != null) {
					if (!this.completePath.contains(edge))
						this.completePath.add(edge);
					this.tmpOrigin = tmpDestination;
					break;
				}
				dualNodesTO = new ArrayList<>(this.tmpOrigin
						.getDualNodes(this.tmpOrigin, tmpDestination, ap.regionBasedNavigation, this.previousJunction)
						.keySet());
			}

			ArrayList<NodeGraph> dualNodesTD = null;
			dualNodesTD = new ArrayList<>(tmpDestination
					.getDualNodes(this.tmpOrigin, tmpDestination, ap.regionBasedNavigation, null).keySet());

			boolean found = false;
			for (final NodeGraph tmpDualOrigin : dualNodesTO) {
				for (final NodeGraph tmpDualDestination : dualNodesTD) {
					// check if just one node separates them
					if (Path.commonPrimalJunction(tmpDualOrigin, tmpDualDestination) != null) {
						final GeomPlanarGraphDirectedEdge first = this.tmpOrigin
								.getDirectedEdgeWith(Path.commonPrimalJunction(tmpDualOrigin, tmpDualDestination));
						final GeomPlanarGraphDirectedEdge second = Path
								.commonPrimalJunction(tmpDualOrigin, tmpDualDestination)
								.getDirectedEdgeWith(tmpDestination);
						this.path.edges.add(first);
						this.path.edges.add(second);
					} else if (ap.localHeuristic.equals("angularChange")) {
						final DijkstraAngularChange pathfinder = new DijkstraAngularChange();
						this.path = pathfinder.dijkstraPath(tmpDualOrigin, tmpDualDestination, this.destinationNode,
								this.centroidsToAvoid, this.tmpOrigin, ap);
					} else if (ap.localHeuristic.equals("turns")) {
						final DijkstraIntersections pathfinder = new DijkstraIntersections();
						this.path = pathfinder.dijkstraPath(tmpDualOrigin, tmpDualDestination, this.destinationNode,
								this.centroidsToAvoid, this.tmpOrigin, ap);
					}
					if (this.path.edges.size() != 0) {
						found = true;
						break;
					}
				}
				if (found)
					break;
			}
			while (this.path.edges.size() == 0 && !this.moveOn)
				this.backtrackingDual(tmpDestination);
			if (this.moveOn) {
				this.tmpOrigin = tmpDestination;
				continue;
			}

			this.cleanDualPath(this.tmpOrigin, tmpDestination);
			this.tmpOrigin = tmpDestination;
			this.completePath.addAll(this.path.edges);
		}
		return this.completePath;
	}

	/**
	 * Global landmarks path a)
	 *
	 * It returns the path that maximises global landmarkness between an origin and
	 * a destination.
	 * 
	 * @param originNode      the origin node;
	 * @param destinationNode the destination node;
	 * @param ap              the agent properties;
	 *
	 */
	public ArrayList<GeomPlanarGraphDirectedEdge> globalLandmarksPath(NodeGraph originNode, NodeGraph destinationNode,
			AgentProperties ap) {
		this.ap = ap;
		final DijkstraGlobalLandmarks pathfinder = new DijkstraGlobalLandmarks();
		this.path = pathfinder.dijkstraPath(originNode, destinationNode, destinationNode, null, ap);
		return this.path.edges;
	}

	/**
	 * Global landmarks path B) Sequence [O, ..., D] Maximise global landmarkness
	 * through a sequence of intermediate nodes
	 *
	 * @param sequence sequence of intermediate nodes (e.g. on-route marks,
	 *                 gateways) including the origin and the destination nodes;
	 * @param ap       the agent properties;
	 *
	 */
	public ArrayList<GeomPlanarGraphDirectedEdge> globalLandmarksPathSequence(ArrayList<NodeGraph> sequence,
			AgentProperties ap) {

		this.ap = ap;
		this.sequenceNodes = new ArrayList<>(sequence);
		// originNode
		this.tmpOrigin = this.sequenceNodes.get(0);
		this.destinationNode = this.sequenceNodes.get(this.sequenceNodes.size() - 1);
		this.sequenceNodes.remove(0);

		for (final NodeGraph tmpDestination : this.sequenceNodes) {
			this.moveOn = false;
			// check if this tmpDestination has been traversed already
			if (Path.nodesFromPath(this.completePath).contains(tmpDestination)) {
				this.controlPath(tmpDestination);
				this.tmpOrigin = tmpDestination;
				continue;
			}

			// check if edge in between
			final GeomPlanarGraphDirectedEdge edge = this.tmpOrigin.getDirectedEdgeWith(tmpDestination);
			if (edge != null) {
				if (!this.completePath.contains(edge))
					this.completePath.add(edge);
				this.tmpOrigin = tmpDestination;
				continue;
			}

			final DijkstraGlobalLandmarks pathfinder = new DijkstraGlobalLandmarks();
			this.path = pathfinder.dijkstraPath(this.tmpOrigin, tmpDestination, this.destinationNode, this.completePath,
					ap);

			while (this.path.edges.size() == 0 && !this.moveOn)
				this.backtracking(tmpDestination);
			this.tmpOrigin = tmpDestination;
			if (this.moveOn)
				continue;
			this.checkSequenceEdges(this.tmpOrigin, tmpDestination);
			this.completePath.addAll(this.path.edges);
		}
		return this.completePath;
	}

	/**
	 * Region- or Region- and barrier-based path
	 *
	 * @param originNode      the origin node;
	 * @param destinationNode the destination node;
	 * @param ap              the agent properties;
	 *
	 */
	public ArrayList<GeomPlanarGraphDirectedEdge> regionBarrierBasedPath(NodeGraph originNode,
			NodeGraph destinationNode, AgentProperties ap) {
		this.ap = ap;
		final RegionBasedNavigation regionsPath = new RegionBasedNavigation();
		final ArrayList<NodeGraph> sequenceRegions = regionsPath.sequenceRegions(originNode, destinationNode, ap);
		ArrayList<GeomPlanarGraphDirectedEdge> path = new ArrayList<>();
		if (ap.localHeuristic.equals("roadDistance"))
			path = this.roadDistanceSequence(sequenceRegions, ap);
		else
			path = this.angularChangeBasedSequence(sequenceRegions, ap);
		return path;
	}

	/**
	 * Barrier-based path
	 *
	 * @param originNode      the origin node;
	 * @param destinationNode the destination node;
	 * @param ap              the agent properties;
	 *
	 */
	public ArrayList<GeomPlanarGraphDirectedEdge> barrierBasedPath(NodeGraph originNode, NodeGraph destinationNode,
			AgentProperties ap) {

		this.ap = ap;
		ArrayList<GeomPlanarGraphDirectedEdge> path = new ArrayList<>();
		final BarrierBasedNavigation barrierBasedPath = new BarrierBasedNavigation();
		final ArrayList<NodeGraph> sequenceBarriers = barrierBasedPath.sequenceBarriers(originNode, destinationNode,
				ap);
		if (ap.localHeuristic.equals("roadDistance"))
			path = this.roadDistanceSequence(sequenceBarriers, ap);
		else if (ap.localHeuristic.equals("angularChange"))
			path = this.angularChangeBasedSequence(sequenceBarriers, ap);
		return path;
	}

	// Utility functions to clean paths and check dual-paths

	/**
	 * Backtracking function for sequences and landmarks, primal graph-based path
	 * (road distance). When the agent gets stuck because of the "segmentsToAvoid",
	 * the function iterates back across the nodes and retries to compute the path
	 * towards the given tmpDestinationNode.
	 *
	 * @param tmpDestination the examined intermediate destination node;
	 */
	private void backtracking(NodeGraph tmpDestination) {
		// new tmpOrigin
		this.tmpOrigin = (NodeGraph) this.completePath.get(this.completePath.size() - 1).getFromNode();
		// remove the last problematic segment;
		this.completePath.remove(this.completePath.size() - 1);

		// check if there's a segment between the new tmpOrigin and the destination
		final GeomPlanarGraphDirectedEdge edge = this.tmpOrigin.getDirectedEdgeWith(tmpDestination);
		if (edge != null) {
			if (!this.completePath.contains(edge))
				this.completePath.add(edge);
			this.moveOn = true; // no need to backtracking anymore
			return;
		}
		// if not, try to compute the path from the new tmpOrigin
		final DijkstraRoadDistance pathFinder = new DijkstraRoadDistance();
		this.path = pathFinder.dijkstraPath(this.tmpOrigin, tmpDestination, this.destinationNode, this.completePath,
				this.ap);
	}

	/**
	 * Backtracking function for sequences and landmarks, dual graph-based path
	 * (angular change). When the agent gets stuck because of the
	 * "centroidsToAvoid", the function iterates back across the nodes and retries
	 * to compute the path towards the given tmpDestinationNode.
	 *
	 * @param tmpDualOrigin      the examined dual intermediate origin node;
	 * @param tmpDualDestination the examined dual intermediate destination node;
	 * @param tmpDestination     the examined (primal) intermediate destination
	 *                           node;
	 */
	private void backtrackingDual(NodeGraph tmpDestination) {
		// new tmpOrigin
		System.out.println("backtracking -- from " + this.tmpOrigin.getID() + " to " + tmpDestination.getID()
				+ " previous junction " + this.previousJunction.getID());
		try {
			this.tmpOrigin = (NodeGraph) this.completePath.get(this.completePath.size() - 1).getFromNode();
		} catch (final java.lang.ArrayIndexOutOfBoundsException e) {
			this.path.edges.clear();
			return;
		}

		// remove last one which did not work!
		this.completePath.remove(this.completePath.size() - 1);
		this.centroidsToAvoid.remove(this.centroidsToAvoid.size() - 1);
		// take new previous junction
		this.previousJunction = Path.previousJunction(this.completePath);
		// check if there's a segment between the new tmpOrigin and the destination
		final GeomPlanarGraphDirectedEdge edge = this.tmpOrigin.getDirectedEdgeWith(tmpDestination);

		if (edge != null) {
			if (!this.completePath.contains(edge))
				this.completePath.add(edge);
			this.moveOn = true; // no need to backtracking anymore
			return;
		}

		final ArrayList<NodeGraph> dualNodesTO = new ArrayList<>(this.tmpOrigin
				.getDualNodes(this.tmpOrigin, tmpDestination, this.ap.regionBasedNavigation, this.previousJunction)
				.keySet());
		final ArrayList<NodeGraph> dualNodesTD = new ArrayList<>(tmpDestination
				.getDualNodes(this.tmpOrigin, tmpDestination, this.ap.regionBasedNavigation, this.previousJunction)
				.keySet());

		boolean found = false;
		for (final NodeGraph tmpDualOrigin : dualNodesTO) {
			for (final NodeGraph tmpDualDestination : dualNodesTD) {
				if (this.ap.localHeuristic.equals("angularChange")) {
					final DijkstraAngularChange pathfinder = new DijkstraAngularChange();
					this.path = pathfinder.dijkstraPath(tmpDualOrigin, tmpDualDestination, this.destinationNode,
							this.centroidsToAvoid, this.tmpOrigin, this.ap);
				} else if (this.ap.localHeuristic.equals("turns")) {
					final DijkstraIntersections pathfinder = new DijkstraIntersections();
					this.path = pathfinder.dijkstraPath(tmpDualOrigin, tmpDualDestination, this.destinationNode,
							this.centroidsToAvoid, this.tmpOrigin, this.ap);
				}
				if (this.path.edges.size() != 0) {
					found = true;
					break;
				}
			}
			if (found)
				break;
		}
	}

	/**
	 * It checks whether the destinationNode has been traversed already when
	 * formulating a path.
	 *
	 * @param destinationNode the examined primal destination node;
	 */

	private void controlPath(NodeGraph destinationNode) {
		for (final GeomPlanarGraphDirectedEdge e : this.completePath)
			if (e.getToNode() == destinationNode) {
				final int lastIndex = this.completePath.indexOf(e);
				this.completePath = new ArrayList<>(this.completePath.subList(0, lastIndex + 1));
				if (Path.previousJunction(this.completePath) == destinationNode)
					this.completePath.remove(this.completePath.size() - 1);
				return;
			}
	}

	/**
	 * It checks whether the primal destinationNode has been passed in the dual
	 * graph (since nodes represent entire segments in this representation). It also
	 * checks the presence of unnecessary edges at the beginning, in relation to the
	 * originNode.
	 *
	 * @param originNode      the examined primal origin node;
	 * @param destinationNode the examined primal destination node;
	 */

	private void cleanDualPath(NodeGraph originNode, NodeGraph destinationNode) {
		// check if the path is one edge ahead
		if (Path.previousJunction(this.path.edges) == destinationNode)
			this.path.edges.remove(this.path.edges.size() - 1);
		// check presence of a unnecessary edge at the beginning of the path
		if (Path.commonPrimalJunction(((EdgeGraph) this.path.edges.get(0).getEdge()).dualNode,
				((EdgeGraph) this.path.edges.get(1).getEdge()).dualNode) == originNode)
			this.path.edges.remove(0);
		this.checkSequenceEdges(originNode, destinationNode);
	}

	/**
	 * It readjusts the order of the GeomPlanarGraphDirectedEdge on the basis of
	 * toNode and fromNode.
	 *
	 * @param originNode      the examined primal origin node;
	 * @param destinationNode the examined primal destination node;
	 */

	private void checkSequenceEdges(NodeGraph originNode, NodeGraph destinationNode) {

		NodeGraph previousNode = originNode;
		for (final GeomPlanarGraphDirectedEdge edge : this.path.edges) {
			NodeGraph nextNode = (NodeGraph) edge.getToNode();

			// need to swap
			if (nextNode == previousNode) {
				nextNode = (NodeGraph) edge.getFromNode();
				final GeomPlanarGraphDirectedEdge correctEdge = previousNode.getDirectedEdgeWith(nextNode);
				this.path.edges.set(this.path.edges.indexOf(edge), correctEdge);
			}
			previousNode = nextNode;
		}
	}
}
