package pedsimcity.utilities;

import java.util.ArrayList;
import java.util.HashMap;

import pedsimcity.graph.EdgeGraph;
import pedsimcity.graph.NodeGraph;
import sim.util.geo.GeomPlanarGraphDirectedEdge;

/**
 * A class for storing the sequence of GeomPlanarGraphDirectedEdge in a path and
 * the sequence of NodeWrappers. It supports shortest-path algorithms and
 * provides some utilities.
 *
 */
public class Path {

	public ArrayList<GeomPlanarGraphDirectedEdge> edges = new ArrayList<>();
	public HashMap<NodeGraph, NodeWrapper> mapWrappers = new HashMap<>();

	/**
	 * It identifies the previous junction traversed in a dual graph path, so to
	 * avoid traversing in the actual primal graph an unecessary segment. This is
	 * due to the peculiarity representation of the dual graph, wherein primal
	 * segments are represented by nodes.
	 *
	 * @param path a sequence of GeomPlanarGraphDirectedEdge representing the path;
	 */
	public static NodeGraph previousJunction(ArrayList<GeomPlanarGraphDirectedEdge> path) {
		// from global graph
		if (path.size() == 1)
			return (NodeGraph) path.get(0).getFromNode();

		final NodeGraph lastCen = ((EdgeGraph) path.get(path.size() - 1).getEdge()).getDual();
		final NodeGraph otherCen = ((EdgeGraph) path.get(path.size() - 2).getEdge()).getDual();
		return commonPrimalJunction(lastCen, otherCen);
	}

	/**
	 * It return all the centroids (nodes in the dual graph) traversed in a path;
	 *
	 * @param path a sequence of GeomPlanarGraphDirectedEdge representing the path;
	 */
	public static ArrayList<NodeGraph> centroidsFromPath(ArrayList<GeomPlanarGraphDirectedEdge> path) {
		final ArrayList<NodeGraph> centroids = new ArrayList<>();
		for (final GeomPlanarGraphDirectedEdge e : path)
			centroids.add(((EdgeGraph) e.getEdge()).getDual());
		return centroids;
	}

	/**
	 * It return all the primal nodes traversed in a path;
	 *
	 * @param path a sequence of GeomPlanarGraphDirectedEdge representing the path;
	 */
	public static ArrayList<NodeGraph> nodesFromPath(ArrayList<GeomPlanarGraphDirectedEdge> path) {
		final ArrayList<NodeGraph> nodes = new ArrayList<>();
		for (final GeomPlanarGraphDirectedEdge e : path) {
			nodes.add(((EdgeGraph) e.getEdge()).u);
			nodes.add(((EdgeGraph) e.getEdge()).v);
		}
		return nodes;
	}

	/**
	 * Given two centroids (nodes in the dual graph ), it identifies their common
	 * junction (i.e. the junction shared by the corresponding primal segments).
	 *
	 * @param centroid      a dual node;
	 * @param otherCentroid an other dual node;
	 */
	public static NodeGraph commonPrimalJunction(NodeGraph centroid, NodeGraph otherCentroid) {

		final EdgeGraph edge = centroid.primalEdge;
		final EdgeGraph otherEdge = otherCentroid.primalEdge;

		if (edge.u == otherEdge.u | edge.u == otherEdge.v)
			return edge.u;
		else if (edge.v == otherEdge.v | edge.v == otherEdge.u)
			return edge.v;
		else
			return null;
	}

	public void invalidPath() {
		this.edges.clear();
		this.mapWrappers.clear();
	}
}
