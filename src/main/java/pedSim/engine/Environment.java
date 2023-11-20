package pedSim.engine;

import java.util.ArrayList;
import java.util.Map.Entry;

import org.javatuples.Pair;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.planargraph.DirectedEdge;
import org.locationtech.jts.planargraph.DirectedEdgeStar;

import pedSim.cognitiveMap.Barrier;
import pedSim.cognitiveMap.BarrierIntegration;
import pedSim.cognitiveMap.CognitiveMap;
import pedSim.cognitiveMap.Gateway;
import pedSim.cognitiveMap.Region;
import sim.field.geo.VectorLayer;
import sim.graph.Building;
import sim.graph.EdgeGraph;
import sim.graph.GraphUtils;
import sim.graph.NodeGraph;
import sim.graph.SubGraph;
import sim.util.geo.Angles;
import sim.util.geo.MasonGeometry;

/**
 * The Environment class is responsible for preparing the simulation
 * environment, including junctions, buildings, barriers, and regions.
 */
public class Environment {

	/**
	 * Prepares the simulation environment by initializing junctions, buildings,
	 * barriers, attributes, dual graph, and regions (if barriers are present).
	 */
	public static void prepare() {

		getJunctions();
		if (!PedSimCity.buildings.getGeometries().isEmpty())
			prepareBuildings();
		if (!PedSimCity.barriers.getGeometries().isEmpty())
			prepareGateways();
		createEdgesMap();
		prepareDualGraph();

		if (!PedSimCity.barriers.getGeometries().isEmpty())
			prepareRegionsBarriers();

		CognitiveMap cognitiveMap = new CognitiveMap();
		cognitiveMap.setCommunityCognitiveMap();
	}

	/**
	 * Nodes: Assigns scores and attributes to nodes.
	 */
	static private void getJunctions() {

		ArrayList<MasonGeometry> geometries = PedSimCity.junctions.getGeometries();

		for (final MasonGeometry nodeGeometry : geometries) {
			// street junctions and betweenness centrality
			final NodeGraph node = PedSimCity.network.findNode(nodeGeometry.geometry.getCoordinate());
			node.setID(nodeGeometry.getIntegerAttribute("nodeID"));
			node.masonGeometry = nodeGeometry;
			node.primalEdge = null;
			// centrality
			try {
				node.centrality = nodeGeometry.getDoubleAttribute("Bc_multi");
			} catch (final java.lang.NullPointerException e) {
				node.centrality = nodeGeometry.getDoubleAttribute("Bc_Rd");
			}
			// set adjacent edges and nodes
			node.setNeighbouringComponents();
			PedSimCity.nodesMap.put(node.getID(), node);
		}
		// generate the centrality map of the graph
		PedSimCity.network.generateGraphStructures();
		PedSimCity.network.generateCentralityMap();
		PedSimCity.network.setGraphSalientNodes(Parameters.salientNodesPercentile);
	}

	/**
	 * Landmarks: Assign landmark scores to buildings.
	 */
	static private void prepareBuildings() {

		ArrayList<MasonGeometry> geometries = PedSimCity.buildings.getGeometries();
		for (final MasonGeometry buildingGeometry : geometries) {
			final Building building = new Building();
			building.buildingID = buildingGeometry.getIntegerAttribute("buildingID");
			building.landUse = buildingGeometry.getStringAttribute("land_use");
			building.DMA = buildingGeometry.getStringAttribute("DMA");
			building.geometry = buildingGeometry;
			building.attributes.put("globalLandmarkness", buildingGeometry.getAttributes().get("gScore_sc"));
			building.attributes.put("localLandmarkness", buildingGeometry.getAttributes().get("lScore_sc"));

			final ArrayList<MasonGeometry> nearestNodes = PedSimCity.junctions
					.featuresWithinDistance(buildingGeometry.getGeometry(), 500.0);
			MasonGeometry closest = null;
			double lowestDistance = 501.0;

			for (final MasonGeometry node : nearestNodes) {
				final double distance = node.getGeometry().distance(buildingGeometry.getGeometry());

				if (distance < lowestDistance) {
					closest = node;
					lowestDistance = distance;
				}
			}
			building.node = closest != null ? PedSimCity.network.findNode(closest.getGeometry().getCoordinate()) : null;
			PedSimCity.buildingsMap.put(building.buildingID, building);
		}

		PedSimCity.network.getNodes().forEach((node) -> {
			ArrayList<MasonGeometry> nearestBuildings = PedSimCity.buildings
					.featuresWithinDistance(node.masonGeometry.geometry, 100);
			for (MasonGeometry building : nearestBuildings) {
				final int buildingID = building.getIntegerAttribute("buildingID");
				final String DMA = PedSimCity.buildingsMap.get(buildingID).DMA;
				node.DMA = DMA;
			}
		});
	}

	/**
	 * Gateways: Configures gateways between nodes.
	 */
	static private void prepareGateways() {

		// creating regions
		for (final NodeGraph node : PedSimCity.nodesMap.values()) {
			node.regionID = node.masonGeometry.getIntegerAttribute("district");
			PedSimCity.regionsMap.computeIfAbsent(node.regionID, region -> new Region());
		}

		for (final NodeGraph node : PedSimCity.nodesMap.values()) {
			final Integer gatewayID = node.masonGeometry.getIntegerAttribute("gateway");

			if (gatewayID == 0) {
				PedSimCity.startingNodes.add(node.masonGeometry);
				continue;
			}

			final Integer regionID = node.regionID;
			for (final EdgeGraph bridge : node.getEdges()) {
				final NodeGraph oppositeNode = (NodeGraph) bridge.getOppositeNode(node);
				final int possibleRegionID = oppositeNode.regionID;
				if (possibleRegionID == regionID)
					continue;

				final Gateway gateway = new Gateway();
				gateway.exit = node;
				gateway.edgeID = bridge.getID();
				gateway.gatewayID = new Pair<>(node, oppositeNode);
				gateway.regionTo = possibleRegionID;
				gateway.entry = oppositeNode;
				gateway.distance = bridge.getLength();
				gateway.entryAngle = Angles.angle(node, oppositeNode);
				PedSimCity.regionsMap.get(regionID).gateways.add(gateway);
				PedSimCity.gatewaysMap.put(new Pair<>(node, oppositeNode), gateway);
				node.gateway = true;
			}
			node.adjacentRegions = node.getAdjacentRegion();
		}
	}

	/**
	 * Creates the edgesMap (edgeID, edgeGraph mapping) .
	 */
	static private void createEdgesMap() {

		ArrayList<EdgeGraph> edges = PedSimCity.network.getEdges();
		for (EdgeGraph edge : edges) {
			int edgeID = edge.attributes.get("edgeID").getInteger();
			edge.setID(edgeID);
			PedSimCity.edgesMap.put(edgeID, edge);
		}
	}

	/**
	 * Centroids (Dual Graph): Assigns edgeID to centroids in the dual graph.
	 */
	static private void prepareDualGraph() {

		ArrayList<MasonGeometry> centroids = PedSimCity.centroids.getGeometries();
		for (final MasonGeometry centroidGeometry : centroids) {
			int edgeID = centroidGeometry.getIntegerAttribute("edgeID");
			NodeGraph centroid = PedSimCity.dualNetwork.findNode(centroidGeometry.geometry.getCoordinate());
			centroid.masonGeometry = centroidGeometry;
			centroid.setID(edgeID);
			centroid.primalEdge = PedSimCity.edgesMap.get(edgeID);
			PedSimCity.edgesMap.get(edgeID).dualNode = centroid;
			PedSimCity.centroidsMap.put(edgeID, centroid);
			centroid.setNeighbouringComponents();
		}

		ArrayList<EdgeGraph> dualEdges = PedSimCity.dualNetwork.getEdges();
		for (EdgeGraph edge : dualEdges)
			edge.deflectionDegrees = edge.attributes.get("deg").getDouble();
		PedSimCity.dualNetwork.generateGraphStructures();
	}

	/**
	 * Regions: Creates regions' subgraphs and store information.
	 */
	static private void prepareRegionsBarriers() {

		ArrayList<EdgeGraph> edges = PedSimCity.network.getEdges();

		for (EdgeGraph edge : edges) {
			BarrierIntegration.setEdgeGraphBarriers(edge);
			if (edge.fromNode.regionID == edge.toNode.regionID) {
				int regionID = edge.fromNode.regionID;
				edge.regionID = regionID;
				PedSimCity.regionsMap.get(regionID).edges.add(edge);
			} else
				edge.regionID = -1;
		}

		for (final Entry<Integer, Region> entry : PedSimCity.regionsMap.entrySet()) {
			int regionID = entry.getKey();
			Region region = entry.getValue();

			final ArrayList<EdgeGraph> edgesRegion = region.edges;
			final SubGraph primalGraph = new SubGraph(PedSimCity.network, edgesRegion);
			final VectorLayer regionNetwork = new VectorLayer();
			final ArrayList<EdgeGraph> dualEdgesRegion = new ArrayList<>();

			for (final EdgeGraph edge : edgesRegion) {
				regionNetwork.addGeometry(edge.masonGeometry);
				NodeGraph centroid = edge.dualNode;
				centroid.regionID = regionID;
				DirectedEdgeStar directedEdges = centroid.getOutEdges();
				for (final DirectedEdge directedEdge : directedEdges.getEdges())
					dualEdgesRegion.add((EdgeGraph) directedEdge.getEdge());
			}
			final SubGraph dualGraph = new SubGraph(PedSimCity.dualNetwork, dualEdgesRegion);
			primalGraph.generateSubGraphCentralityMap();
			primalGraph.setSubGraphSalientNodes(Parameters.salientNodesPercentile);
			region.regionID = regionID;
			region.primalGraph = primalGraph;
			region.dualGraph = dualGraph;
			region.regionNetwork = regionNetwork;
		}
		generateBarriersMap();
	}

	/**
	 * Generates a map of barriers based on provided data.
	 */
	private static void generateBarriersMap() {

		// Element 5 - Barriers: create barriers map
		ArrayList<MasonGeometry> geometries = PedSimCity.barriers.getGeometries();
		for (final MasonGeometry barrierGeometry : geometries) {
			final int barrierID = barrierGeometry.getIntegerAttribute("barrierID");
			final Barrier barrier = new Barrier();
			barrier.masonGeometry = barrierGeometry;
			barrier.type = barrierGeometry.getStringAttribute("type");
			final ArrayList<EdgeGraph> edgesAlong = new ArrayList<>();

			for (final EdgeGraph edge : PedSimCity.network.getEdges()) {
				ArrayList<Integer> edgeBarriers = edge.attributes.get("barriers").getArray();
				if (!edgeBarriers.isEmpty() && edgeBarriers.contains(barrierID))
					edgesAlong.add(edge);
			}

			barrier.edgesAlong = edgesAlong;
			barrier.type = barrierGeometry.getStringAttribute("type");
			PedSimCity.barriersMap.put(barrierID, barrier);
		}

	}

	/**
	 * Returns all the buildings enclosed between two nodes.
	 *
	 * @param originNode      The first node.
	 * @param destinationNode The second node.
	 * @return A list of buildings.
	 */
	public ArrayList<MasonGeometry> getBuildings(NodeGraph originNode, NodeGraph destinationNode) {
		Geometry smallestCircle = GraphUtils.enclosingCircleBetweenNodes(originNode, destinationNode);
		return PedSimCity.buildings.containedFeatures(smallestCircle);
	}

	/**
	 * Get buildings within a specified region.
	 *
	 * @param region The region for which buildings are to be retrieved.
	 * @return An ArrayList of MasonGeometry objects representing buildings within
	 *         the region.
	 */
	public ArrayList<MasonGeometry> getBuildingsWithinRegion(Region region) {
		VectorLayer regionNetwork = region.regionNetwork;
		Geometry convexHull = regionNetwork.getConvexHull();
		return PedSimCity.buildings.containedFeatures(convexHull);
	}
}