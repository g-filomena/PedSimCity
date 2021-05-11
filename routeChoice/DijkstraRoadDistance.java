/**
 * DijkstraRoadDistance.java
 * It computes the road (metric) distance shortest path by employing the Dijkstra shortest-path algorithm.
 * It uses the primal graph of the street network.
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

public class DijkstraRoadDistance {

	NodeGraph originNode, destinationNode, finalDestinationNode;
	ArrayList<NodeGraph> visitedNodes, unvisitedNodes;
	HashMap<NodeGraph, NodeWrapper> mapWrappers = new HashMap<>();
	ArrayList<GeomPlanarGraphDirectedEdge> segmentsToAvoid = new ArrayList<>();
	ArrayList<EdgeGraph> edgesToAvoid = new ArrayList<>();
	SubGraph graph = new SubGraph();
	AgentProperties ap = new AgentProperties();
	boolean subGraph = UserParameters.subGraph;

	/**
	 * @param originNode           the origin node (it may be a sequence
	 *                             intermediate origin node, e.g. in landmark
	 *                             navigation);
	 * @param destinationNode      the destination node (it may be a sequence
	 *                             intermediate destination node, e.g. in landmark
	 *                             navigation);
	 * @param finalDestinationNode the actual final destination Node;
	 * @param segmentsToAvoid      street segments already traversed in previous
	 *                             iterations, if applicable;
	 * @param ap                   the set of the properties that describe the
	 *                             agent;
	 */
	public Path dijkstraPath(NodeGraph originNode, NodeGraph destinationNode, NodeGraph finalDestinationNode,
			ArrayList<GeomPlanarGraphDirectedEdge> segmentsToAvoid, AgentProperties ap) {
		this.ap = ap;
		this.originNode = originNode;
		this.destinationNode = destinationNode;
		this.finalDestinationNode = finalDestinationNode;
		if (segmentsToAvoid != null)
			this.segmentsToAvoid = new ArrayList<>(segmentsToAvoid);

		// from the GeomPlanarGraphDirectedEdge get the actual EdgeGraph (safer)
		if (segmentsToAvoid != null)
			for (final GeomPlanarGraphDirectedEdge e : segmentsToAvoid)
				this.edgesToAvoid.add((EdgeGraph) e.getEdge());

		// If region-based navigation, navigate only within the region subgraph, if
		// origin and destination nodes belong to the same region.
		if (originNode.region == destinationNode.region && ap.regionBasedNavigation) {
			this.graph = PedSimCity.regionsMap.get(originNode.region).primalGraph;
			originNode = this.graph.findNode(originNode.getCoordinate());
			destinationNode = this.graph.findNode(destinationNode.getCoordinate());
			if (segmentsToAvoid != null)
				this.edgesToAvoid = this.graph.getChildEdges(this.edgesToAvoid);
		}
		// otherwise create graph from convex hull
		else if (this.subGraph) {
			final ArrayList<EdgeGraph> containedEdges = PedSimCity.network.edgesWithinSpace(originNode,
					destinationNode);
			this.graph = new SubGraph(PedSimCity.network, containedEdges);
			originNode = this.graph.findNode(originNode.getCoordinate());
			destinationNode = this.graph.findNode(destinationNode.getCoordinate());
			if (segmentsToAvoid != null)
				this.edgesToAvoid = this.graph.getChildEdges(this.edgesToAvoid);
		}

		this.visitedNodes = new ArrayList<>();
		this.unvisitedNodes = new ArrayList<>();
		this.unvisitedNodes.add(originNode);

		// NodeWrapper = container for the metainformation about a Node
		final NodeWrapper nodeWrapper = new NodeWrapper(originNode);
		nodeWrapper.gx = 0.0;
		this.mapWrappers.put(originNode, nodeWrapper);

		while (this.unvisitedNodes.size() > 0) {
			// at the beginning it takes originNode
			final NodeGraph currentNode = this.getClosest(this.unvisitedNodes);
			this.visitedNodes.add(currentNode);
			this.unvisitedNodes.remove(currentNode);
			this.findMinDistances(currentNode);
		}
		return this.reconstructPath(originNode, destinationNode);
	}

	void findMinDistances(NodeGraph currentNode) {
		final ArrayList<NodeGraph> adjacentNodes = currentNode.getAdjacentNodes();
		for (final NodeGraph targetNode : adjacentNodes) {

			if (this.visitedNodes.contains(targetNode))
				continue;
			final EdgeGraph commonEdge = currentNode.getEdgeWith(targetNode);
			final GeomPlanarGraphDirectedEdge outEdge = (GeomPlanarGraphDirectedEdge) commonEdge.getDirEdge(0);
			if (this.segmentsToAvoid == null)
				;
			else if (this.edgesToAvoid.contains(outEdge.getEdge()))
				continue;
			double error = 1.0;
			double tentativeCost = 0.0;

			// compute costs based on the navigation strategies.
			// compute errors in perception of road coasts with stochastic variables
			final List<Integer> pBarriers = commonEdge.positiveBarriers;
			final List<Integer> nBarriers = commonEdge.negativeBarriers;
			if (this.ap.onlyMinimising.equals("") && this.ap.preferenceNaturalBarriers && pBarriers.size() > 0)
				error = Utilities.fromDistribution(this.ap.naturalBarriers, this.ap.naturalBarriersSD, "left");
			else if (!this.ap.onlyMinimising.equals("") && this.ap.aversionSeveringBarriers && nBarriers.size() > 0)
				error = Utilities.fromDistribution(this.ap.severingBarriers, this.ap.severingBarriersSD, "right");
			else
				error = Utilities.fromDistribution(1.0, 0.10, null);

//			if (this.ap.onlyMinimising.equals("") && this.ap.preferenceNaturalBarriers && pBarriers.size() > 0)
//				error = 0.50;
//			else if (this.ap.onlyMinimising.equals("") && this.ap.aversionSeveringBarriers && nBarriers.size() > 0)
//				error = 1.50;
//			else
//				error = Utilities.fromDistribution(1.0, 0.10, null);
			final double edgeCost = commonEdge.getLength() * error;

			if (this.ap.onlyMinimising.equals("") && this.ap.usingDistantLandmarks && NodeGraph
					.nodesDistance(targetNode, this.finalDestinationNode) > UserParameters.threshold3dVisibility) {
				final double globalLandmarkness = LandmarkNavigation.globalLandmarknessNode(targetNode,
						this.finalDestinationNode, this.ap.onlyAnchors);
				final double nodeLandmarkness = 1.0
						- globalLandmarkness * UserParameters.globalLandmarknessWeightDistance;

				final double nodeCost = edgeCost * nodeLandmarkness;
				tentativeCost = this.getBest(currentNode) + nodeCost;
			} else
				tentativeCost = this.getBest(currentNode) + edgeCost;

			if (this.getBest(targetNode) > tentativeCost) {
				NodeWrapper nodeWrapper = this.mapWrappers.get(targetNode);
				if (nodeWrapper == null)
					nodeWrapper = new NodeWrapper(targetNode);
				nodeWrapper.nodeFrom = currentNode;
				nodeWrapper.edgeFrom = outEdge;
				nodeWrapper.gx = tentativeCost;
				this.mapWrappers.put(targetNode, nodeWrapper);
				this.unvisitedNodes.add(targetNode);
			}
		}
	}

	// get the closest amongst unvisited nodes -- the ones that haven't been
	// explored yet
	NodeGraph getClosest(ArrayList<NodeGraph> nodes) {
		NodeGraph closest = null;
		for (final NodeGraph node : nodes)
			if (closest == null)
				closest = node;
			else if (this.getBest(node) < this.getBest(closest))
				closest = node;
		return closest;
	}

	Double getBest(NodeGraph targetNode) {
		if (this.mapWrappers.get(targetNode) == null)
			return Double.MAX_VALUE;
		else
			return this.mapWrappers.get(targetNode).gx;
	}

	Path reconstructPath(NodeGraph originNode, NodeGraph destinationNode) {
		final Path path = new Path();

		final HashMap<NodeGraph, NodeWrapper> mapTraversedWrappers = new HashMap<>();
		final ArrayList<GeomPlanarGraphDirectedEdge> sequenceEdges = new ArrayList<>();
		NodeGraph step = destinationNode;
		mapTraversedWrappers.put(destinationNode, this.mapWrappers.get(destinationNode));

		// If the subgraph navigation hasn't worked, retry by using the full graph
		// --> it switches "subgraph" to false;

		if (this.mapWrappers.get(destinationNode) == null && this.subGraph == true) {
			this.subGraph = false;
			this.visitedNodes.clear();
			this.unvisitedNodes.clear();
			this.mapWrappers.clear();
			originNode = this.graph.getParentNode(originNode);
			destinationNode = this.graph.getParentNode(destinationNode);
			final Path secondAttempt = this.dijkstraPath(originNode, destinationNode, this.finalDestinationNode,
					this.segmentsToAvoid, this.ap);
			return secondAttempt;
		}

		// check that the path has been formulated properly
		// no path
		if (this.mapWrappers.get(destinationNode) == null || this.mapWrappers.size() <= 1)
			path.invalidPath();
		try {
			while (this.mapWrappers.get(step).nodeFrom != null) {
				final GeomPlanarGraphDirectedEdge dd;
				if (this.ap.regionBasedNavigation && originNode.region == destinationNode.region) {
					final NodeGraph nodeTo = this.graph.getParentNode(step);
					final NodeGraph nodeFrom = this.graph.getParentNode(this.mapWrappers.get(step).nodeFrom);
					dd = nodeFrom.getDirectedEdgeWith(nodeTo);
				} else
					dd = this.mapWrappers.get(step).edgeFrom;
				step = this.mapWrappers.get(step).nodeFrom;
				sequenceEdges.add(0, dd);
				mapTraversedWrappers.put(step, this.mapWrappers.get(step));
			}
		}
		// no path
		catch (final java.lang.NullPointerException e) {
			return path;
		}
		if (this.subGraph) {

		}
		path.edges = sequenceEdges;
		path.mapWrappers = mapTraversedWrappers;
		return path;
	}

}
