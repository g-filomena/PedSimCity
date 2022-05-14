package pedsimcity.graph;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.CoordinateArrays;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.planargraph.DirectedEdge;

import pedsimcity.elements.Building;
import pedsimcity.utilities.Utilities;
import pedsimcity.utilities.VectorLayer;
import sim.field.network.Network;
import sim.util.Bag;
import sim.util.geo.GeomPlanarGraph;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.MasonGeometry;

/**
 * A planar graph that extends the GeomPlanarGraph (GeoMason) and PlanarGraph
 * (JTS) classes. Its basic components are NodeGraph and EdgeGraph.
 *
 */
public class Graph extends GeomPlanarGraph {
	public ArrayList<EdgeGraph> edgesGraph = new ArrayList<>();
	public ArrayList<NodeGraph> nodesGraph = new ArrayList<>();
	public LinkedHashMap<NodeGraph, Double> centralityMap = new LinkedHashMap<>();
	public HashMap<Integer, NodeGraph> nodesMap = new HashMap<>();
	public VectorLayer junctions = new VectorLayer();

	public Graph() {
		super();
	}

	/**
	 * It populates the network with lines from a GeomVectorField (GeoMason) or
	 * VectorLayer
	 *
	 * @param streetSegments the street segments layer;
	 */
	public void fromGeomField(VectorLayer streetSegments) {
		final Bag geometries = streetSegments.getGeometries();
		for (final Object o : geometries)
			if (((MasonGeometry) o).geometry instanceof LineString)
				this.addLineString((MasonGeometry) o);
	}

	/**
	 * It populates the network with lines from a GeomVectorField (GeoMason) or
	 * VectorLayer. It also stores the geometries of the junctions, for convenience.
	 *
	 * @param streetJunctions the street junctions layer;
	 * @param streetSegments the street segments layer;
	 */
	public void fromGeomField(VectorLayer streetJunctions, VectorLayer streetSegments) {
		final Bag geometries = streetSegments.getGeometries();
		for (final Object o : geometries)
			if (((MasonGeometry) o).geometry instanceof LineString)
				this.addLineString((MasonGeometry) o);
		this.junctions = streetJunctions;
	}

	/**
	 * It adds an EdgeGraph and its NodeGraphs to the graph. It also stores the
	 * geometries of the junctions, for convenience.
	 *
	 * @param wrappedLine the MasonGeometry corresponding to a street segment;
	 */
	private void addLineString(MasonGeometry wrappedLine) {
		final LineString line = (LineString) wrappedLine.geometry;
		if (line.isEmpty())
			return;

		final Coordinate[] coords = CoordinateArrays.removeRepeatedPoints(line.getCoordinates());
		if (coords.length < 2)
			return;

		final Coordinate uCoord = coords[0];
		final Coordinate vCoord = coords[coords.length - 1];
		final NodeGraph u = this.getNode(uCoord);
		final NodeGraph v = this.getNode(vCoord);

		this.nodesMap.put(u.getID(), u);
		this.nodesMap.put(v.getID(), v);
		this.nodesGraph.add(u);
		this.nodesGraph.add(v);

		final EdgeGraph edge = new EdgeGraph(line);
		final GeomPlanarGraphDirectedEdge de0 = new GeomPlanarGraphDirectedEdge(u, v, coords[1], true);
		final GeomPlanarGraphDirectedEdge de1 = new GeomPlanarGraphDirectedEdge(v, u, coords[coords.length - 2], false);

		edge.setDirectedEdges(de0, de1);
		edge.setAttributes(wrappedLine.getAttributes());
		edge.setNodes(u, v);
		edge.masonGeometry = wrappedLine;
		this.add(edge);
		this.edgesGraph.add(edge);
	}

	/**
	 * It returns the NodeGraph corresponding to the given coordinate.
	 *
	 * @param pt the coordinates;
	 * @note Override as the original methods returns a Node;
	 */
	public NodeGraph getNode(Coordinate pt) {

		NodeGraph node = this.findNode(pt);
		if (node == null) {
			node = new NodeGraph(pt);
			// ensure node is only added once to graph
			this.add(node);
		}
		return node;
	}

	/**
	 * It finds the NodeGraph corresponding to the given coordinates.
	 *
	 * @param pt the coordinates;
	 * @note Override as the original methods returns a Node;
	 */
	@Override
	public NodeGraph findNode(Coordinate pt) {
		return (NodeGraph) this.nodeMap.find(pt);
	}

	/**
	 * It returns this graph's network;
	 *
	 */
	@Override
	public Network getNetwork() {

		final Network network = new Network(false); // false == not directed
		final Collection edges = this.getEdges();
		for (final Object object : edges) {
			final DirectedEdge edge = (DirectedEdge) object;
			network.addEdge(edge.getFromNode(), edge.getToNode(), edge);
		}
		return network;
	}

	/**
	 * It generates the map of the nodes of this graph. Namely, a <nodeID, NodeGraph> map.
	 *
	 */
	private void generateNodesMap() {
		for (final NodeGraph node : this.nodesGraph)
			this.nodesMap.put(node.getID(), node);
	}

	/**
	 * It generates the centrality map of the nodes of this graph. Namely a <NodeGraph, centrality> map.
	 *
	 */
	public void generateCentralityMap() {

		this.generateNodesMap();
		final LinkedHashMap<NodeGraph, Double> centralityMap = new LinkedHashMap<>();

		for (final NodeGraph node : this.nodesGraph)
			centralityMap.put(node, node.centrality);
		this.centralityMap = (LinkedHashMap<NodeGraph, Double>) Utilities.sortByValue(centralityMap, false);

		// rescale
		for (final NodeGraph node : this.nodesGraph) {
			final double rescaled = (node.centrality - Collections.min(centralityMap.values()))
					/ (Collections.max(centralityMap.values()) - Collections.min(centralityMap.values()));
			node.centrality_sc = rescaled;
		}
	}

	/**
	 * It returns a Map of salient nodes in the graph, on the basis of centrality values. The
	 * returned Map is in the format <NodeGraph, Double>, where the values represent
	 * centrality values. The percentile determines the threshold used to identify
	 * salient nodes. For example, if 0.75 is provided, only the nodes whose
	 * centrality value is higher than the value at the 75th percentile are
	 * returned. This is computed within the space (smallest enclosing circle)
	 * between two given nodes; The keys represent NodeGraph in the SubGraph (child
	 * nodes).
	 *
	 * @param node       a node;
	 * @param otherNode  an other node;
	 * @param percentile the percentile use to identify salient nodes;
	 */
	public Map<NodeGraph, Double> salientNodesWithinSpace(NodeGraph node, NodeGraph otherNode, double percentile) {
		ArrayList<NodeGraph> containedNodes = new ArrayList<>();
		final Geometry smallestEnclosingCircle = NodeGraph.nodesEnclosingCircle(node, otherNode);
		containedNodes = this.getContainedNodes(smallestEnclosingCircle);

		if (containedNodes.size() == 0)
			return null;
		LinkedHashMap<NodeGraph, Double> spatialfilteredMap = new LinkedHashMap<>();
		spatialfilteredMap = filterCentralityMap(this.centralityMap, containedNodes);
		if (spatialfilteredMap.size() == 0 || spatialfilteredMap == null)
			return null;

		final int position = (int) (spatialfilteredMap.size() * percentile);
		final double boundary = new ArrayList<>(spatialfilteredMap.values()).get(position);
		final Map<NodeGraph, Double> valueFilteredMap = spatialfilteredMap.entrySet().stream()
				.filter(entry -> entry.getValue() >= boundary)
				.collect(Collectors.toMap(entry -> entry.getKey(), entry -> entry.getValue()));

		if (valueFilteredMap.size() == 0 || valueFilteredMap == null)
			return null;
		else
			return valueFilteredMap;
	}

	/**
	 * It returns a Map of salient nodes, on the basis of centrality values. The
	 * returned Map is in the format <NodeGraph, Double>, where the values represent
	 * centrality values. The percentile determines the threshold used to identify
	 * salient nodes. For example, if 0.75 is provided, only the nodes whose
	 * centrality value is higher than the value at the 75th percentile are
	 * returned. This is computed within the entire graph.
	 *
	 * @param percentile the percentile used to identify salient nodes;
	 */
	public Map<NodeGraph, Double> salientNodesNetwork(double percentile) {
		int position;
		position = (int) (this.centralityMap.size() * percentile);
		final double boundary = new ArrayList<>(this.centralityMap.values()).get(position);
		final Map<NodeGraph, Double> valueFilteredMap = this.centralityMap.entrySet().stream()
				.filter(entry -> entry.getValue() >= boundary)
				.collect(Collectors.toMap(entry -> entry.getKey(), entry -> entry.getValue()));
		if (valueFilteredMap.size() == 0 || valueFilteredMap == null)
			return null;
		else
			return valueFilteredMap;
	}

	/**
	 * It returns a list of nodes contained within a given geometry.
	 *
	 * @param g the given geometry;
	 */
	public ArrayList<NodeGraph> getContainedNodes(Geometry g) {
		final ArrayList<NodeGraph> containedNodes = new ArrayList<>();
		final Collection<NodeGraph> nodes = this.nodesMap.values();

		for (final NodeGraph node : nodes) {
			final Geometry geoNode = node.masonGeometry.geometry;
			if (g.contains(geoNode))
				containedNodes.add(node);
		}
		return containedNodes;
	}

	/**
	 * It returns a list of edges contained within a given geometry.
	 *
	 * @param g the given geometry;
	 */
	public ArrayList<EdgeGraph> getContainedEdges(Geometry g) {
		final ArrayList<EdgeGraph> containedEdges = new ArrayList<>();
		final ArrayList<EdgeGraph> edges = this.edgesGraph;

		for (final EdgeGraph edge : edges) {
			final Geometry geoEdge = edge.masonGeometry.geometry;
			if (g.contains(geoEdge))
				containedEdges.add(edge);
		}
		return containedEdges;
	}

	/**
	 * It returns the list of edges contained in the graph.
	 *
	 * @param g the given geometry;
	 */
	@Override
	public ArrayList<EdgeGraph> getEdges() {
		return this.edgesGraph;
	}

	@Override
	public ArrayList<NodeGraph> getNodes() {
		return this.nodesGraph;
	}

	/**
	 * Given a certain map of nodes and values, and an ArrayList of fewer nodes, It returns the initial map
	 * filtered on the basis of the passed ArrayList.
	 *
	 *
	 * @param map the map to filter;
	 * @param filter the ArrayList of NodeGraph containing only the nodes to keep in the map;
	 */
	public static LinkedHashMap<NodeGraph, Double> filterCentralityMap(LinkedHashMap<NodeGraph, Double> map,
			ArrayList<NodeGraph> filter) {

		final LinkedHashMap<NodeGraph, Double> mapFiltered = new LinkedHashMap<>(map);
		final ArrayList<NodeGraph> result = new ArrayList<>();
		for (final NodeGraph key : mapFiltered.keySet())
			if (filter.contains(key))
				result.add(key);
		mapFiltered.keySet().retainAll(result);
		return mapFiltered;
	}

	/**
	 * Given two nodes, it returns all the nodes contained
	 *
	 * @param percentile the percentile used to identify salient nodes;
	 */
	public ArrayList<EdgeGraph> edgesWithinSpace(NodeGraph originNode, NodeGraph destinationNode) {

		Double radius = NodeGraph.nodesDistance(originNode, destinationNode) * 1.50;
		if (radius < 500)
			radius = 500.0;
		final Geometry bufferOrigin = originNode.masonGeometry.geometry.buffer(radius);
		final Geometry bufferDestination = destinationNode.masonGeometry.geometry.buffer(radius);
		final Geometry convexHull = bufferOrigin.union(bufferDestination).convexHull();

		final ArrayList<EdgeGraph> containedEdges = this.getContainedEdges(convexHull);
		return containedEdges;
	}

	/**
	 * Given a node, a lower and an upper limit, it returns all the nodes whose distance from the given node
	 * is between the lower and the upper limit.
	 *
	 * @param originNode;
	 * @param lowerLimit;
	 * @param upperLimit;
	 */
	public ArrayList<NodeGraph> getNodesBetweenLimits(NodeGraph originNode, double lowerLimit, double upperLimit) {

		final ArrayList<NodeGraph> containedNodes = new ArrayList<>();
		final MasonGeometry originGeometry = originNode.masonGeometry;
		final Bag containedGeometries = this.junctions.featuresBetweenLimits(originGeometry.geometry, lowerLimit,
				upperLimit);
		for (final Object o : containedGeometries)
			containedNodes.add(this.findNode(((MasonGeometry) o).geometry.getCoordinate()));
		return containedNodes;
	}

	/**
	 * Given a node of the graph, a lower and an upper limit, it returns all the nodes whose distance from
	 * the given node is between the lower and the upper limit. Moreover, such nodes's centrality is higher
	 * than the value at the passed percentile
	 *
	 * @param originNode;
	 * @param lowerLimit;
	 * @param upperLimit;
	 * @param percentile the percentile to use as threshold;
	 */
	public ArrayList<NodeGraph> getSalientNodesBetweenLimits(NodeGraph originNode, double lowerLimit,
			double upperLimit, double percentile) {

		final ArrayList<NodeGraph> containedNodes = new ArrayList<>();
		final ArrayList<NodeGraph> containedSalientNodes = new ArrayList<>();
		final MasonGeometry originGeometry = originNode.masonGeometry;
		final Bag containedGeometries = this.junctions.featuresBetweenLimits(originGeometry.geometry, lowerLimit,
				upperLimit);
		for (final Object o : containedGeometries)
			containedNodes.add(this.findNode(((MasonGeometry) o).geometry.getCoordinate()));
		final ArrayList<NodeGraph> salientNodes = new ArrayList<>(this.salientNodesNetworkReach(percentile).keySet());

		for (final NodeGraph n : containedNodes)
			if (salientNodes.contains(n))
				containedSalientNodes.add(n);
		return containedSalientNodes;
	}

	/**
	 * Given a node in the graph, a lower and an upper limit, it returns all the nodes whose distance from
	 * the given node is between the lower and the upper limit. Moreover, the returned nodes must be in
	 * a different region from the originNode.
	 *
	 * @param originNode;
	 * @param lowerLimit;
	 * @param upperLimit;
	 */
	public ArrayList<NodeGraph> getNodesBetweenLimitsOtherRegion(NodeGraph originNode, double lowerLimit,
			double upperLimit) {
		ArrayList<NodeGraph> containedNodes = new ArrayList<>();
		containedNodes = this.getNodesBetweenLimits(originNode, lowerLimit, upperLimit);
		return this.filterOutRegion(containedNodes, originNode.region);
	}

	/**

	 *
	 * @param percentile the percentile used to identify salient nodes;
	 */
	public ArrayList<NodeGraph> filterOutRegion(ArrayList<NodeGraph> nodes, int region) {
		final ArrayList<NodeGraph> newNodes = new ArrayList<>(nodes);
		for (final NodeGraph node : nodes)
			if (node.region == region)
				newNodes.remove(node);
		return newNodes;

	}

	/**
	 * It assigns to each node in the graph a list of local landmarks.
	 *
	 * @param localLandmarks the layer containing all the buildings possibly considered as local landmarks;
	 * @param buildingsMap the map of buildings (buildingID, Building);
	 * @param radius the maximum distance from a node to a building for the building to be considered a local
	 * 			landmark at the junction;
	 */
	public void setLocalLandmarkness(VectorLayer localLandmarks, HashMap<Integer, Building> buildingsMap,
			double radius) {

		final Collection<NodeGraph> nodes = this.nodesMap.values();

		nodes.forEach((node) -> {

			final Bag containedLandmarks = localLandmarks.featuresWithinDistance(node.masonGeometry.geometry, radius);
			for (final Object l : containedLandmarks) {
				final MasonGeometry building = (MasonGeometry) l;
				node.localLandmarks.add(buildingsMap.get((int) building.getUserData()));
			}
		});
	}

	/**
	 * It assigns to each node in the graph a list of distant landmarks and their corresponding
	 * global landmarkness values.
	 *
	 * @param localLandmarks the layer containing all the buildings possibly considered as global landmarks;
	 * @param buildingsMap the map of buildings (buildingID, Building);
	 * @param radiusAnchors the distance radius within which a global landmark is considered to be an anchor
	 * 		  of a node (when intended as destination node);
	 * @param sightLines the layer containing the sight lines;
	 * @param nrAnchors the max number of anchors per node, sorted by global landmarkness;
	 */
	public void setGlobalLandmarkness(VectorLayer globalLandmarks, HashMap<Integer, Building> buildingsMap,
			double radiusAnchors, VectorLayer sightLines, int nrAnchors) {

		final Collection<NodeGraph> nodes = this.nodesMap.values();

		nodes.forEach((node) -> {
			final MasonGeometry nodeGeometry = node.masonGeometry;

			final Bag containedLandmarks = globalLandmarks.featuresWithinDistance(node.masonGeometry.geometry,
					radiusAnchors);
			final List<Double> gScores = new ArrayList<>();

			if (nrAnchors != 999999) {
				for (final Object l : containedLandmarks) {
					final MasonGeometry building = (MasonGeometry) l;
					gScores.add(building.getDoubleAttribute("gScore_sc"));

				}
				Collections.sort(gScores);
				Collections.reverse(gScores);
			}

			for (final Object l : containedLandmarks) {
				final MasonGeometry building = (MasonGeometry) l;
				if (nrAnchors != 999999 & building.getDoubleAttribute("gScore_sc") < gScores.get(nrAnchors - 1))
					continue;
				final int buildingID = (int) building.getUserData();
				node.anchors.add(buildingsMap.get(buildingID));
				node.distances.add(building.geometry.distance(nodeGeometry.geometry));
			}
		});

		final ArrayList<MasonGeometry> sightLinesGeometries = sightLines.geometriesList;
		for (final MasonGeometry sl : sightLinesGeometries) {
			final Building building = buildingsMap.get(sl.getIntegerAttribute("buildingID"));
			final NodeGraph node = this.nodesMap.get(sl.getIntegerAttribute("nodeID"));
			if (node != null)
				node.distantLandmarks.add(building);

		}
	}

}
