/**
 * DijkstraAngularChange.java
 * It computes the cumulative angular change shortest path by employing the Dijkstra shortest-path algorithm
 * It uses the dual graph of the street network.
 *
 * It supports: landmark-, region-, barrier-based navigation.
 **/

package pedsimcity.routeChoice;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import pedsimcity.agents.AgentProperties;
import pedsimcity.graph.EdgeGraph;
import pedsimcity.graph.NodeGraph;
import pedsimcity.graph.SubGraph;
import pedsimcity.main.PedSimCity;
import pedsimcity.main.UserParameters;
import pedsimcity.utilities.NodeWrapper;
import pedsimcity.utilities.Path;
import pedsimcity.utilities.Utilities;
import sim.util.geo.GeomPlanarGraphDirectedEdge;

public class DijkstraIntersections {

	NodeGraph originNode, destinationNode, primalDestinationNode, previousJunction;
	ArrayList<NodeGraph> visitedNodes, unvisitedNodes, centroidsToAvoid;
	HashMap<NodeGraph, NodeWrapper> mapWrappers = new HashMap<>();
	SubGraph graph = new SubGraph();
	// it contemplates an attempt where navigation takes place by the convex-hull
	// method (see below).
	AgentProperties ap = new AgentProperties();

	/**
	 * @param originNode            the origin node (dual graph);
	 * @param destinationNode       the destination node (dual graph);
	 * @param primalDestinationNode the actual final, primal destination Node;
	 * @param centroidsToAvoid      the centroids (dual nodes, representing
	 *                              segments) already traversed in previous
	 *                              iterations, if applicable;
	 * @param previousJunction      the previous primal junction, if any;
	 * @param ap                    the set of the properties that describe the
	 *                              agent;
	 */
	public Path dijkstraPath(NodeGraph originNode, NodeGraph destinationNode, NodeGraph primalDestinationNode,
			ArrayList<NodeGraph> centroidsToAvoid, NodeGraph previousJunction, AgentProperties ap) {

		this.ap = ap;
		this.originNode = originNode;
		this.destinationNode = destinationNode;
		this.primalDestinationNode = primalDestinationNode;
		if (centroidsToAvoid != null)
			this.centroidsToAvoid = new ArrayList<>(centroidsToAvoid);
		this.previousJunction = previousJunction;

		// If region-based navigation, navigate only within the region subgraph, if
		// origin and destination nodes belong to the same region.

		if (originNode.region == destinationNode.region && ap.regionBasedNavigation) {
			this.graph = PedSimCity.regionsMap.get(originNode.region).dualGraph;
			originNode = this.graph.findNode(originNode.getCoordinate());
			destinationNode = this.graph.findNode(destinationNode.getCoordinate());
			if (centroidsToAvoid != null)
				centroidsToAvoid = this.graph.getChildNodes(centroidsToAvoid);
			// primalJunction is always the same;
		}

		this.visitedNodes = new ArrayList<>();
		this.unvisitedNodes = new ArrayList<>();
		this.unvisitedNodes.add(originNode);

		// NodeWrapper = container for the metainformation about a Node
		final NodeWrapper NodeWrapper = new NodeWrapper(originNode);
		NodeWrapper.gx = 0.0;
		if (previousJunction != null)
			NodeWrapper.commonPrimalJunction = previousJunction;
		this.mapWrappers.put(originNode, NodeWrapper);

		// add centroids to avoid in the visited set
		if (centroidsToAvoid != null)
			for (final NodeGraph c : centroidsToAvoid)
				this.visitedNodes.add(c);

		while (this.unvisitedNodes.size() > 0) {
			// at the beginning it takes originNode
			final NodeGraph currentNode = this.getClosest(this.unvisitedNodes);
			this.visitedNodes.add(currentNode);
			this.unvisitedNodes.remove(currentNode);
			this.findMinDistances(currentNode);
		}
		return this.reconstructPath(originNode, destinationNode);
	}

	private void findMinDistances(NodeGraph currentNode) {
		final ArrayList<NodeGraph> adjacentNodes = currentNode.getAdjacentNodes();
		for (final NodeGraph targetNode : adjacentNodes) {
			if (this.visitedNodes.contains(targetNode))
				continue;

			/**
			 * Check if the current and the possible next centroid share in the primal graph
			 * the same junction as the current with its previous centroid --> if yes move
			 * on. This essential means that the in the primal graph you would go back to an
			 * already traversed node; but the dual graph wouldn't know.
			 */
			if (Path.commonPrimalJunction(targetNode,
					currentNode) == this.mapWrappers.get(currentNode).commonPrimalJunction)
				continue;

			final EdgeGraph commonEdge = currentNode.getEdgeWith(targetNode);

			// compute costs based on the navigation strategies.
			// compute errors in perception of road coasts with stochastic variables
			double error = 1.0;
			double tentativeCost = 0.0;
			if (this.ap.barrierBasedNavigation) {
				final List<Integer> positiveBarriers = targetNode.primalEdge.positiveBarriers;
				final List<Integer> negativeBarriers = targetNode.primalEdge.negativeBarriers;
				if (positiveBarriers != null)
					error = Utilities.fromDistribution(0.70, 0.10, "left");
				else if (negativeBarriers != null && positiveBarriers == null)
					error = Utilities.fromDistribution(1.30, 0.10, "right");
				else
					error = Utilities.fromDistribution(1.0, 0.10, null);
			}
			// else error = Utilities.fromDistribution(1, 0.10, null);

			double turnCost = commonEdge.getDeflectionAngle() * error;
			if (turnCost > 180.0)
				turnCost = 180.0;
			if (turnCost < 0.0)
				turnCost = 0.0;
			final GeomPlanarGraphDirectedEdge outEdge = currentNode.getDirectedEdgeWith(targetNode);
			if (turnCost <= UserParameters.thresholdTurn)
				tentativeCost = this.getBest(currentNode);
			else {
				double edgeCost = 1.0;
				if (this.ap.usingDistantLandmarks && NodeGraph.nodesDistance(targetNode,
						this.primalDestinationNode) > UserParameters.threshold3dVisibility) {
					final double globalLandmarkness = LandmarkNavigation.globalLandmarknessDualNode(currentNode,
							targetNode, this.primalDestinationNode, this.ap.onlyAnchors);
					final double nodeLandmarkness = 1.0
							- globalLandmarkness * UserParameters.globalLandmarknessWeightAngular;
					edgeCost = nodeLandmarkness * 1;
				}
				tentativeCost = this.getBest(currentNode) + edgeCost; // no turn
			}

			if (this.getBest(targetNode) > tentativeCost) {
				NodeWrapper NodeWrapper = this.mapWrappers.get(targetNode);
				if (NodeWrapper == null)
					NodeWrapper = new NodeWrapper(targetNode);
				NodeWrapper.nodeFrom = currentNode;
				NodeWrapper.edgeFrom = outEdge;
				NodeWrapper.commonPrimalJunction = Path.commonPrimalJunction(currentNode, targetNode);
				NodeWrapper.gx = tentativeCost;
				this.mapWrappers.put(targetNode, NodeWrapper);
				this.unvisitedNodes.add(targetNode);
			}
		}
	}

	private NodeGraph getClosest(ArrayList<NodeGraph> nodes) {

		NodeGraph closest = null;
		for (final NodeGraph node : nodes)
			if (closest == null)
				closest = node;
			else if (this.getBest(node) < this.getBest(closest))
				closest = node;
		return closest;
	}

	Double getBest(NodeGraph target) {

		if (this.mapWrappers.get(target) == null)
			return Double.MAX_VALUE;
		else
			return this.mapWrappers.get(target).gx;
	}

	public Path reconstructPath(NodeGraph originNode, NodeGraph destinationNode) {

		final Path path = new Path();

		final HashMap<NodeGraph, NodeWrapper> mapTraversedWrappers = new HashMap<>();
		final ArrayList<GeomPlanarGraphDirectedEdge> sequenceEdges = new ArrayList<>();
		NodeGraph step = destinationNode;
		mapTraversedWrappers.put(destinationNode, this.mapWrappers.get(destinationNode));

		// check that the path has been formulated properly
		if (this.mapWrappers.get(destinationNode) == null || this.mapWrappers.size() <= 1)
			path.invalidPath();
		try {

			while (this.mapWrappers.get(step).nodeFrom != null) {
				final GeomPlanarGraphDirectedEdge de = (GeomPlanarGraphDirectedEdge) step.primalEdge.getDirEdge(0);
				step = this.mapWrappers.get(step).nodeFrom;
				mapTraversedWrappers.put(step, this.mapWrappers.get(step));
				sequenceEdges.add(0, de);

				if (step == originNode) {
					final GeomPlanarGraphDirectedEdge lastDe = (GeomPlanarGraphDirectedEdge) step.primalEdge
							.getDirEdge(0);
					sequenceEdges.add(0, lastDe);
					break;
				}
			}
		} catch (final java.lang.NullPointerException e) {
			return path;
		}

		path.edges = sequenceEdges;
		path.mapWrappers = mapTraversedWrappers;
		return path;
	}
}
