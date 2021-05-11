/**
 * DijkstraGlobalLandmarks.java
 *
 * It computes the path that maximises global landmarkness between an origin and a destination.
 * It uses the primal graph of the street network.
 *
 */

package pedsimcity.routeChoice;

import java.util.ArrayList;
import java.util.HashMap;

import pedsimcity.agents.AgentProperties;
import pedsimcity.graph.EdgeGraph;
import pedsimcity.graph.NodeGraph;
import pedsimcity.graph.SubGraph;
import pedsimcity.main.PedSimCity;
import pedsimcity.utilities.NodeWrapper;
import pedsimcity.utilities.Path;
import sim.util.geo.GeomPlanarGraphDirectedEdge;

public class DijkstraGlobalLandmarks {

	NodeGraph originNode, destinationNode, finalDestinationNode;
	ArrayList<NodeGraph> visitedNodes, unvisitedNodes;
	HashMap<NodeGraph, NodeWrapper> mapWrappers = new HashMap<>();
	ArrayList<GeomPlanarGraphDirectedEdge> segmentsToAvoid;
	ArrayList<EdgeGraph> edgesToAvoid = new ArrayList<>();
	AgentProperties ap = new AgentProperties();
	SubGraph graph = new SubGraph();

	/**
	 * @param originNode      the origin node;
	 * @param destinationNode the final destination Node;
	 * @param segmentsToAvoid street segments already traversed in previous
	 *                        iterations, if applicable;
	 * @param ap              the agent properties;
	 */
	public Path dijkstraPath(NodeGraph originNode, NodeGraph destinationNode, NodeGraph finalDestinationNode,
			ArrayList<GeomPlanarGraphDirectedEdge> segmentsToAvoid, AgentProperties ap) {

		this.ap = ap;
		this.originNode = originNode;
		this.destinationNode = destinationNode;
		this.finalDestinationNode = finalDestinationNode;
		if (segmentsToAvoid != null)
			this.segmentsToAvoid = new ArrayList<>(segmentsToAvoid);

		// If region-based navigation, navigate only within the region subgraph, if
		// origin and destination nodes belong to the same region.
		if (originNode.region == destinationNode.region && ap.regionBasedNavigation) {
			this.graph = PedSimCity.regionsMap.get(originNode.region).primalGraph;
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

		// from the GeomPlanarGraphDirectedEdge get the actual EdgeGraph (safer)
		if (segmentsToAvoid != null)
			for (final GeomPlanarGraphDirectedEdge e : segmentsToAvoid)
				this.edgesToAvoid.add((EdgeGraph) e.getEdge());

		while (this.unvisitedNodes.size() > 0) {
			// at the beginning it takes originNode
			final NodeGraph node = this.getClosest(this.unvisitedNodes);
			this.visitedNodes.add(node);
			this.unvisitedNodes.remove(node);
			this.findMinDistances(node);
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

			final double globalLandmarkness = LandmarkNavigation.globalLandmarknessNode(targetNode,
					this.finalDestinationNode, this.ap.onlyAnchors);

			// the global landmarkness from the node is divided by the segment's length so
			// to avoid that the path is not affected
			// by network (topological) distance
			final double nodeLandmarkness = (1.0 - globalLandmarkness) / commonEdge.getLength();
			final double tentativeCost = this.getBest(currentNode) + nodeLandmarkness;

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

	// amongst unvisited (they have to have been explored)
	NodeGraph getClosest(ArrayList<NodeGraph> nodes) {
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

	Path reconstructPath(NodeGraph originNode, NodeGraph destinationNode) {
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
				final GeomPlanarGraphDirectedEdge dd = this.mapWrappers.get(step).edgeFrom;
				step = this.mapWrappers.get(step).nodeFrom;
				sequenceEdges.add(0, dd);
				mapTraversedWrappers.put(step, this.mapWrappers.get(step));
			}
		}
		// no path
		catch (final java.lang.NullPointerException e) {
			return path;
		}

		path.edges = sequenceEdges;
		path.mapWrappers = mapTraversedWrappers;
		return path;
	}
}
