package pedsimcity.graph;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.CoordinateArrays;
import com.vividsolutions.jts.geom.LineString;

import pedsimcity.utilities.Utilities;
import sim.util.geo.GeomPlanarGraphDirectedEdge;

/**
 * This class represents sub graphs derived from a graph. It establish links
 * between the graph component and the corresponding child components, allowing
 * faster operations and the creation of "regional" or "district" graphs.
 *
 * Navigation throughout a SubGraph is straightforward and can be easily
 * retraced to the parent graph.
 */

public class SubGraph extends Graph {
	private final SubGraphNodesMap subGraphNodesMap = new SubGraphNodesMap();
	private final SubGraphEdgesMap subGraphEdgesMap = new SubGraphEdgesMap();
	private ArrayList<Integer> graphBarriers = new ArrayList<>();
	LinkedHashMap<NodeGraph, Double> centralityMap = new LinkedHashMap<>();
	Graph parentGraph = new Graph();

	/**
	 * The SubGraph constructor, when passing the parent graph and the list of
	 * EdgeGraphs in the parent graph that should be included in the SubGraph.
	 *
	 * @param parentGraph the original parent graph.
	 * @param edges       the list of edges to include.
	 */
	public SubGraph(Graph parentGraph, ArrayList<EdgeGraph> edges) {
		// this.parentGraph = parentGraph;
		for (final EdgeGraph edge : edges)
			this.addFromOtherGraph(parentGraph, edge);
		for (final NodeGraph node : this.getNodesList())
			node.setNeighbouringComponents();
	}

	public SubGraph() {
	}

	/**
	 * It adds an EdgeGraph and the corresponding nodes to the SubGraph, with all
	 * the necessary attributes.
	 *
	 * @param parentGraph the parent graph;
	 * @param parentEdge  the parent edge that it's being added;
	 */
	public void addFromOtherGraph(Graph parentGraph, EdgeGraph parentEdge) {

		final NodeGraph u = parentEdge.u;
		final NodeGraph v = parentEdge.v;
		final Coordinate uCoord = u.getCoordinate();
		final Coordinate vCoord = v.getCoordinate();

		final NodeGraph childU = this.getNode(uCoord);
		final NodeGraph childV = this.getNode(vCoord);
		childU.nodeID = u.getID();
		childV.nodeID = v.getID();

		final LineString line = parentEdge.getLine();
		final Coordinate[] coords = CoordinateArrays.removeRepeatedPoints(line.getCoordinates());
		final EdgeGraph childEdge = new EdgeGraph(line);

		final GeomPlanarGraphDirectedEdge de0 = new GeomPlanarGraphDirectedEdge(childU, childV, coords[1], true);
		final GeomPlanarGraphDirectedEdge de1 = new GeomPlanarGraphDirectedEdge(childV, childU,
				coords[coords.length - 2], false);
		childEdge.setDirectedEdges(de0, de1);
		childEdge.setNodes(childU, childV);
		this.setAttributesChildEdge(childEdge, parentEdge);

		this.subGraphNodesMap.add(childU, u);
		this.subGraphNodesMap.add(childV, v);
		this.subGraphEdgesMap.add(childEdge, parentEdge);
		childU.primalEdge = u.primalEdge;
		childV.primalEdge = v.primalEdge;
		this.add(childEdge);
		this.edgesGraph.add(childEdge);
	}

	/**
	 * It maps the nodes of the SubGraph with its parent graph's nodes.
	 *
	 */
	private class SubGraphNodesMap {
		public HashMap<NodeGraph, NodeGraph> map = new HashMap<>();

		private void add(NodeGraph node, NodeGraph parentNode) {
			this.map.put(node, parentNode);
		}

		private NodeGraph findParent(NodeGraph nodeSubGraph) {
			return this.map.get(nodeSubGraph);
		}

		private NodeGraph findChild(NodeGraph nodeGraph) {
			return Utilities.getKeyFromValue(this.map, nodeGraph);
		}
	}

	/**
	 * It maps the edges of the SubGraph with its parent graph's edges.
	 *
	 */
	private class SubGraphEdgesMap {

		private final HashMap<EdgeGraph, EdgeGraph> map = new HashMap<>();

		public void add(EdgeGraph edge, EdgeGraph parentEdge) {
			this.map.put(edge, parentEdge);
		}

		private EdgeGraph findParent(EdgeGraph edgeSubGraph) {
			return this.map.get(edgeSubGraph);
		}

		private EdgeGraph findChild(EdgeGraph edgeSubGraph) {
			return Utilities.getKeyFromValue(this.map, edgeSubGraph);
		}
	}

	/**
	 * It returns the parent node of a child node;
	 *
	 * @param childNode a child node in the SubGraph;
	 */
	public NodeGraph getParentNode(NodeGraph childNode) {
		return this.subGraphNodesMap.findParent(childNode);
	}

	/**
	 * It returns the parent nodes of all the child nodes in the SubGraph;
	 *
	 */
	public ArrayList<NodeGraph> getParentNodes() {
		final ArrayList<NodeGraph> parentNodes = new ArrayList<>();
		parentNodes.addAll(this.subGraphNodesMap.map.values());
		return parentNodes;
	}

	/**
	 * It returns all the parent nodes associated with the child nodes contained in
	 * the list passed;
	 *
	 * @param childNodes a list of child nodes contained in the SubGraph;
	 */
	public ArrayList<NodeGraph> getParentNodes(ArrayList<NodeGraph> childNodes) {

		final ArrayList<NodeGraph> parentNodes = new ArrayList<>();
		for (final NodeGraph child : childNodes) {
			final NodeGraph parent = this.subGraphNodesMap.findParent(child);
			if (parent != null)
				parentNodes.add(parent);
		}
		return parentNodes;
	}

	/**
	 * It returns all the child nodes associated with the parent nodes contained in
	 * the list passed;
	 *
	 * @param parentNodes a list of parent nodes, who may be associated with child
	 *                    nodes in the SubGraph;
	 */
	public ArrayList<NodeGraph> getChildNodes(ArrayList<NodeGraph> parentNodes) {
		final ArrayList<NodeGraph> childNodes = new ArrayList<>();
		for (final NodeGraph parent : parentNodes) {
			final NodeGraph child = this.subGraphNodesMap.findChild(parent);
			if (child != null)
				childNodes.add(child);
		}
		return childNodes;
	}

	/**
	 * It returns all the child edges associated with the parent edges contained in
	 * the list passed;
	 *
	 * @param parentEdges a list of parent edges, who may be associated with child
	 *                    edges in the SubGraph;
	 */
	public ArrayList<EdgeGraph> getChildEdges(ArrayList<EdgeGraph> parentEdges) {
		final ArrayList<EdgeGraph> childEdges = new ArrayList<>();
		for (final EdgeGraph parent : parentEdges) {
			final EdgeGraph child = this.subGraphEdgesMap.findChild(parent);
			if (child != null)
				childEdges.add(child);
		}
		return childEdges;
	}

	/**
	 * It returns the parent edge of a child edge;
	 *
	 * @param childEdge a child edge in the SubGraph;
	 */
	public EdgeGraph getParentEdge(EdgeGraph childEdge) {
		return this.subGraphEdgesMap.findParent(childEdge);
	}

	/**
	 * It returns all the parent edges associated with the child edges contained in
	 * the list passed;
	 *
	 * @param childEdges a list of child edges contained in the SubGraph;
	 */
	public ArrayList<EdgeGraph> getParentEdges(ArrayList<EdgeGraph> childEdges) {
		final ArrayList<EdgeGraph> parentEdges = new ArrayList<>();
		for (final EdgeGraph child : childEdges) {
			final EdgeGraph parent = this.subGraphEdgesMap.findParent(child);
			if (parent != null)
				parentEdges.add(parent);
		}
		return parentEdges;
	}

	/**
	 * It returns the ArrayList of NodeGraphs contained in this SubGraph.
	 */
	public ArrayList<NodeGraph> getNodesList() {
		final ArrayList<NodeGraph> nodesList = new ArrayList<>();
		nodesList.addAll(this.subGraphNodesMap.map.keySet());
		return nodesList;
	}

	/**
	 * It sets the attribute of a child edge, passing the corresponding parent edge.
	 *
	 * @param childEdge  the child edge;
	 * @param parentEdge the parent edge;
	 */
	public void setAttributesChildEdge(EdgeGraph childEdge, EdgeGraph parentEdge) {

		childEdge.setID(parentEdge.getID());
		childEdge.dualNode = parentEdge.getDual();
		// for dual edges:
		childEdge.deflectionDegrees = parentEdge.deflectionDegrees;
	}

	/**
	 * It stores information about the barriers within this SubGraph.
	 *
	 */
	public void setSubGraphBarriers() {

		final ArrayList<Integer> graphBarriers = new ArrayList<>();
		for (final EdgeGraph childEdge : this.getEdges()) {
			childEdge.barriers = this.getParentEdge(childEdge).barriers;
			childEdge.positiveBarriers = this.getParentEdge(childEdge).positiveBarriers;
			childEdge.negativeBarriers = this.getParentEdge(childEdge).negativeBarriers;
			childEdge.waterBodies = this.getParentEdge(childEdge).waterBodies;
			childEdge.parks = this.getParentEdge(childEdge).parks;
			graphBarriers.addAll(childEdge.barriers);
		}
		final Set<Integer> setBarriers = new HashSet<>(graphBarriers);
		this.graphBarriers = new ArrayList<>(setBarriers);
	}

	/**
	 * It stores information about landmark at nodes, within the SubGraph.
	 *
	 */
	public void setSubGraphLandmarks() {
		final ArrayList<NodeGraph> childNodes = this.getNodesList();

		for (final NodeGraph node : childNodes) {
			final NodeGraph parentNode = this.getParentNode(node);
			node.visible2d = parentNode.visible2d;
			node.localLandmarks = parentNode.localLandmarks;
			node.distantLandmarks = parentNode.distantLandmarks;
			node.anchors = parentNode.anchors;
			node.distances = parentNode.distances;
		}
	}

	/**
	 * It gets information about the barriers from the parent graph.
	 *
	 */
	public ArrayList<Integer> getSubGraphBarriers() {
		return this.graphBarriers;
	}

	/**
	 * It generates the centrality map of the SubGraph.
	 *
	 */
	public void generateSubGraphCentralityMap() {
		final LinkedHashMap<NodeGraph, Double> centralityMap = new LinkedHashMap<>();
		final Collection<NodeGraph> nodes = this.subGraphNodesMap.map.keySet();
		for (final NodeGraph n : nodes) {
			final NodeGraph parentNode = this.getParentNode(n);
			centralityMap.put(n, parentNode.centrality);
		}
		this.centralityMap = (LinkedHashMap<NodeGraph, Double>) Utilities.sortByValue(centralityMap, false);
	}

	/**
	 * It returns a Map of salient nodes, on the basis of centrality values in the
	 * parent graph. The returned Map is in the format <NodeGraph, Double>, where
	 * the values represent centrality values. The percentile determines the
	 * threshold used to identify salient nodes. For example, if 0.75 is provided,
	 * only the nodes whose centrality value is higher than the value at the 75th
	 * percentile are returned.
	 *
	 * The keys represent NodeGraph in the parent Graph (parent nodes).
	 * 
	 * @param percentile the percentile use to identify salient nodes;
	 */
	public ArrayList<NodeGraph> globalSalientNodesInSubGraph(double percentile) {
		final Map<NodeGraph, Double> salientParentGraph = this.parentGraph.salientNodesNetwork(percentile);
		final ArrayList<NodeGraph> salientParentNodes = new ArrayList<>(salientParentGraph.keySet());
		salientParentNodes.retainAll(this.getParentNodes());
		return salientParentNodes;
	}

	@Override
	/**
	 * It returns a Map of salient nodes, on the basis of centrality values. The
	 * returned Map is in the format <NodeGraph, Double>, where the values represent
	 * centrality values. The percentile determines the threshold used to identify
	 * salient nodes. For example, if 0.75 is provided, only the nodes whose
	 * centrality value is higher than the value at the 75th percentile are
	 * returned. This is computed within the SubGraph. The keys represent NodeGraph
	 * in the SubGraph (child nodes).
	 *
	 * @param percentile the percentile use to identify salient nodes;
	 */
	public Map<NodeGraph, Double> salientNodesNetwork(double percentile) {
		int position;
		double min_value = 0.0;

		position = (int) (this.centralityMap.size() * percentile);
		min_value = new ArrayList<>(this.centralityMap.values()).get(position);

		final double boundary = min_value;
		final Map<NodeGraph, Double> filteredMap = this.centralityMap.entrySet().stream()
				.filter(entry -> entry.getValue() >= boundary)
				.collect(Collectors.toMap(entry -> entry.getKey(), entry -> entry.getValue()));

		if (filteredMap.size() == 0 || filteredMap == null)
			return null;
		final Map<NodeGraph, Double> parentMap = new HashMap<>();

		for (final Map.Entry<NodeGraph, Double> entry : filteredMap.entrySet()) {
			final NodeGraph parentNode = this.getParentNode(entry.getKey());
			parentMap.put(parentNode, entry.getValue());
		}
		return parentMap;
	}
}