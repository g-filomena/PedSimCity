package pedSim.routeChoice;

import java.util.ArrayList;
import java.util.HashMap;

import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.planargraph.DirectedEdge;

import sim.graph.EdgeGraph;
import sim.graph.NodeGraph;
import sim.util.geo.MasonGeometry;

/**
 * A class for storing the sequence of GeomPlanarGraphDirectedEdge in a path and
 * the sequence of NodeWrappers. It supports shortest-path algorithms and
 * provides some utilities.
 */
public class Route {

	// always primal
	public NodeGraph originNode;
	public NodeGraph destinationNode;
	public ArrayList<DirectedEdge> directedEdgesSequence = new ArrayList<>();
	public ArrayList<EdgeGraph> edgesSequence = new ArrayList<>();
	public ArrayList<NodeGraph> nodesSequence = new ArrayList<>();
	public ArrayList<NodeGraph> dualNodesSequence = new ArrayList<>();

	// on-route marks or relevant nodes
	public LineString lineString;
	public ArrayList<NodeGraph> visitedLocations = new ArrayList<>();
	ArrayList<MasonGeometry> buildingsAlong;
	ArrayList<MasonGeometry> localLandmarksAlong;
	ArrayList<MasonGeometry> globalLandmarksAlong;
	public int turns;
	public int intersections;
	public HashMap<Integer, Double> regionsTraversed = new HashMap<>();

	/**
	 * Identifies the previous junction traversed in a dual graph path to avoid
	 * traversing an unnecessary segment in the primal graph.
	 *
	 * @param sequenceDirectedEdges A sequence of GeomPlanarGraphDirectedEdge
	 *                              representing the path.
	 * @return The previous junction node.
	 */
	public static NodeGraph previousJunction(ArrayList<DirectedEdge> sequenceDirectedEdges) {
		// from global graph
		if (sequenceDirectedEdges.size() == 1)
			return (NodeGraph) sequenceDirectedEdges.get(0).getFromNode();

		final NodeGraph lastCen = ((EdgeGraph) sequenceDirectedEdges.get(sequenceDirectedEdges.size() - 1).getEdge())
				.getDual();
		final NodeGraph otherCen = ((EdgeGraph) sequenceDirectedEdges.get(sequenceDirectedEdges.size() - 2).getEdge())
				.getDual();

		return commonPrimalJunction(lastCen, otherCen);
	}

	/**
	 * Returns all the primal nodes traversed in a path.
	 *
	 * @param directedEdgesSequence A sequence of GeomPlanarGraphDirectedEdge
	 *                              representing the path.
	 * @return A list of primal nodes.
	 */
	public static ArrayList<NodeGraph> nodesFromEdgesSequence(ArrayList<DirectedEdge> directedEdgesSequence) {
		final ArrayList<NodeGraph> nodes = new ArrayList<>();
		for (final DirectedEdge planarDirectedEdge : directedEdgesSequence) {
			nodes.add(((EdgeGraph) planarDirectedEdge.getEdge()).fromNode);
			nodes.add(((EdgeGraph) planarDirectedEdge.getEdge()).toNode);
		}
		return nodes;
	}

	/**
	 * Returns all the centroids (nodes in the dual graph) traversed in a path.
	 *
	 * @param sequenceDirectedEdges A sequence of GeomPlanarGraphDirectedEdge
	 *                              representing the path.
	 * @return A list of centroids (dual nodes).
	 */
	public static ArrayList<NodeGraph> centroidsFromEdgesSequence(ArrayList<DirectedEdge> sequenceDirectedEdges) {
		final ArrayList<NodeGraph> centroids = new ArrayList<>();
		for (final DirectedEdge planarDirectedEdge : sequenceDirectedEdges)
			centroids.add(((EdgeGraph) planarDirectedEdge.getEdge()).getDual());
		return centroids;
	}

	/**
	 * Given two centroids (nodes in the dual graph), identifies their common
	 * junction (i.e., the junction shared by the corresponding primal segments).
	 *
	 * @param centroid      A dual node.
	 * @param otherCentroid Another dual node.
	 * @return The common primal junction node.
	 */
	public static NodeGraph commonPrimalJunction(NodeGraph centroid, NodeGraph otherCentroid) {

		final EdgeGraph edge = centroid.primalEdge;
		final EdgeGraph otherEdge = otherCentroid.primalEdge;
		if (edge.fromNode.equals(otherEdge.fromNode) | edge.fromNode.equals(otherEdge.toNode))
			return edge.fromNode;
		else if (edge.toNode.equals(otherEdge.toNode) | edge.toNode.equals(otherEdge.fromNode))
			return edge.toNode;
		else
			return null;
	}

	/**
	 * Constructs the sequences of nodes and edges from the directed edges sequence.
	 * This method populates the {@code nodesSequence} and {@code edgesSequence}
	 * lists.
	 */
	public void routeSequences() {
		nodesSequence = nodesFromEdgesSequence(directedEdgesSequence);
		for (DirectedEdge directedEdge : directedEdgesSequence)
			edgesSequence.add((EdgeGraph) directedEdge.getEdge());
	}
}
