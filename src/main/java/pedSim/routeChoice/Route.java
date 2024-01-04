package pedSim.routeChoice;

import java.util.ArrayList;
import java.util.List;

import org.locationtech.jts.planargraph.DirectedEdge;

import sim.graph.EdgeGraph;
import sim.graph.GraphUtils;
import sim.graph.NodeGraph;

/**
 * A class for storing the sequence of GeomPlanarGraphDirectedEdge in a path and
 * the sequence of NodeWrappers. It supports shortest-path algorithms and
 * provides some utilities.
 */
public class Route {

	// always primal
	public NodeGraph originNode;
	public NodeGraph destinationNode;
	public List<DirectedEdge> directedEdgesSequence = new ArrayList<>();
	private List<EdgeGraph> edgesSequence = new ArrayList<>();
	private List<NodeGraph> nodesSequence = new ArrayList<>();

	/**
	 * Identifies the previous junction traversed in a dual graph path to avoid
	 * traversing an unnecessary segment in the primal graph.
	 *
	 * @param sequenceDirectedEdges A sequence of GeomPlanarGraphDirectedEdge
	 *                              representing the path.
	 * @return The previous junction node.
	 */
	public NodeGraph previousJunction(List<DirectedEdge> sequenceDirectedEdges) {
		// from global graph
		if (sequenceDirectedEdges.size() == 1)
			return (NodeGraph) sequenceDirectedEdges.get(0).getFromNode();

		NodeGraph lastCentroid = ((EdgeGraph) sequenceDirectedEdges.get(sequenceDirectedEdges.size() - 1).getEdge())
				.getDualNode();
		NodeGraph otherCentroid = ((EdgeGraph) sequenceDirectedEdges.get(sequenceDirectedEdges.size() - 2).getEdge())
				.getDualNode();

		return GraphUtils.getPrimalJunction(lastCentroid, otherCentroid);
	}

	/**
	 * Returns all the primal nodes traversed in a path.
	 *
	 * @param directedEdgesSequence A sequence of GeomPlanarGraphDirectedEdge
	 *                              representing the path.
	 * @return A list of primal nodes.
	 */
	public List<NodeGraph> nodesFromEdgesSequence(List<DirectedEdge> directedEdgesSequence) {

		List<NodeGraph> nodes = new ArrayList<>();
		if (directedEdgesSequence.isEmpty())
			return nodes;

		for (DirectedEdge directedEdge : directedEdgesSequence)
			nodes.add(((EdgeGraph) directedEdge.getEdge()).getFromNode());

		EdgeGraph lastEdge = (EdgeGraph) directedEdgesSequence.get(directedEdgesSequence.size() - 1).getEdge();
		nodes.add(lastEdge.getToNode());
		return nodes;
	}

	/**
	 * Returns all the centroids (nodes in the dual graph) traversed in a path.
	 *
	 * @param sequenceDirectedEdges A sequence of GeomPlanarGraphDirectedEdge
	 *                              representing the path.
	 * @return A list of centroids (dual nodes).
	 */
	public List<NodeGraph> centroidsFromEdgesSequence(List<DirectedEdge> sequenceDirectedEdges) {
		List<NodeGraph> centroids = new ArrayList<>();
		for (DirectedEdge planarDirectedEdge : sequenceDirectedEdges)
			centroids.add(((EdgeGraph) planarDirectedEdge.getEdge()).getDualNode());
		return centroids;
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
		originNode = nodesSequence.get(0);
		destinationNode = nodesSequence.get(nodesSequence.size() - 1);
	}
}
