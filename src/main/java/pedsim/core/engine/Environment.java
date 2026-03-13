package pedsim.core.engine;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Optional;
import org.javatuples.Pair;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.planargraph.DirectedEdge;
import org.locationtech.jts.planargraph.DirectedEdgeStar;
import pedsim.core.cognition.cityimage.Barrier;
import pedsim.core.cognition.cityimage.Gateway;
import pedsim.core.cognition.cityimage.Region;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.cognition.metrics.BarrierIntegration;
import pedsim.core.utilities.StringEnum;
import pedsim.core.utilities.StringEnum.BarrierType;
import sim.field.geo.VectorLayer;
import sim.graph.Building;
import sim.graph.EdgeGraph;
import sim.graph.GraphUtils;
import sim.graph.NodeGraph;
import sim.graph.SubGraph;
import sim.util.geo.Angles;
import sim.util.geo.AttributeValue;
import sim.util.geo.MasonGeometry;

/**
 * The Environment class is responsible for preparing the simulation environment, including
 * junctions, buildings, barriers, and regions.
 */
public class Environment {

  /**
   * Prepares the simulation environment by initializing junctions, buildings, barriers, attributes,
   * dual graph, and regions (if barriers are present).
   */
  public static void prepare() {

    prepareGraph();
    if (!PedSimCity.buildings.getGeometries().isEmpty()) {
      prepareBuildings();
    }
    if (!PedSimCity.barriers.getGeometries().isEmpty()) {
      identifyGateways();
    }
    prepareDualGraph();

    if (!PedSimCity.barriers.getGeometries().isEmpty()) {
      integrateBarriers();
      prepareRegions();
    }

    SharedCognitiveMap.setCommunityCognitiveMap();
  }

  /**
   * Nodes: Assigns scores and attributes to nodes.
   */
  static private void prepareGraph() {

    List<MasonGeometry> geometries = PedSimCity.junctions.getGeometries();

    for (final MasonGeometry nodeGeometry : geometries) {
      // street junctions and betweenness centrality
      final NodeGraph node = PedSimCity.network.findNode(nodeGeometry.geometry.getCoordinate());
      node.setID(nodeGeometry.getIntegerAttribute("nodeID"));
      node.setMasonGeometry(nodeGeometry);
      setCentralityNode(nodeGeometry, node);
      PedSimCity.nodesMap.put(node.getID(), node);
    }
    // generate the centrality map of the graph
    PedSimCity.network.generateCentralityMap();
    createEdgesMap();
  }

  static private void setCentralityNode(MasonGeometry nodeGeometry, NodeGraph node) {

    double centrality = Double.MAX_VALUE;
    try {
      centrality = nodeGeometry.getDoubleAttribute("Bc_multi");
    } catch (final java.lang.NullPointerException e) {
      centrality = nodeGeometry.getDoubleAttribute("Bc_Rd");
    }
    node.setCentrality(centrality);
  }

  /**
   * Landmarks: Assign landmark scores to buildings.
   */
  static private void prepareBuildings() {

    List<MasonGeometry> geometries = PedSimCity.buildings.getGeometries();
    for (final MasonGeometry buildingGeometry : geometries) {
      final Building building = new Building();
      building.buildingID = buildingGeometry.getIntegerAttribute("buildingID");
      building.geometry = buildingGeometry;

      // land use
      Optional.ofNullable(buildingGeometry.getStringAttribute("land_use"))
          .ifPresent(lu -> building.landUse = lu);

      Optional.ofNullable(buildingGeometry.getAttributes().get("gScore_sc"))
          .ifPresent(v -> building.attributes.put("globalLandmarkness", v));

      // TODO switch to this
      // Optional.ofNullable(buildingGeometry.getAttributes().get("DMA"))
      // .ifPresent(v -> building.attributes.put("DMA", v));

      Optional.ofNullable(buildingGeometry.getStringAttribute("DMA"))
          .ifPresent(v -> building.dma = v);

      // landmarks
      Optional.ofNullable(buildingGeometry.getAttributes().get("gScore_sc"))
          .ifPresent(v -> building.attributes.put("globalLandmarkness", v));

      Optional.ofNullable(buildingGeometry.getAttributes().get("lScore_sc"))
          .ifPresent(v -> building.attributes.put("localLandmarkness", v));

      List<MasonGeometry> nearestNodes =
          PedSimCity.junctions.featuresWithinDistance(buildingGeometry.getGeometry(), 500.0);
      MasonGeometry closest = null;
      double lowestDistance = 501.0;

      for (final MasonGeometry node : nearestNodes) {
        final double distance = node.getGeometry().distance(buildingGeometry.getGeometry());

        if (distance < lowestDistance) {
          closest = node;
          lowestDistance = distance;
        }
      }
      building.node =
          closest != null ? PedSimCity.network.findNode(closest.getGeometry().getCoordinate())
              : null;
      PedSimCity.buildingsMap.put(building.buildingID, building);
    }

    PedSimCity.network.getNodes().forEach((node) -> {
      List<MasonGeometry> nearestBuildings =
          PedSimCity.buildings.featuresWithinDistance(node.getMasonGeometry().geometry, 100);
      for (MasonGeometry building : nearestBuildings) {
        final int buildingID = building.getIntegerAttribute("buildingID");
        node.dma = PedSimCity.buildingsMap.get(buildingID).dma;

        // TODO switch to this
        // node.attributes.put("DMA",
        // PedSimCity.buildingsMap.get(buildingID).attributes.get("DMA"));
      }
    });
  }

  /**
   * Gateways: Configures gateways between nodes.
   */
  static private void identifyGateways() {

    // creating regions
    for (NodeGraph node : PedSimCity.nodesMap.values()) {
      node.setRegionID(node.getMasonGeometry().getIntegerAttribute("district"));
      PedSimCity.regionsMap.computeIfAbsent(node.getRegionID(), region -> new Region());
    }

    int gatewayID = 1;
    for (NodeGraph node : PedSimCity.nodesMap.values()) {

      if (node.getMasonGeometry().getIntegerAttribute("gateway") == 0) {
        PedSimCity.startingNodes.add(node.getMasonGeometry());
        continue;
      }

      int regionID = node.getRegionID();
      for (EdgeGraph bridge : node.getEdges()) {
        NodeGraph oppositeNode = (NodeGraph) bridge.getOppositeNode(node);
        int possibleRegionID = oppositeNode.getRegionID();
        if (possibleRegionID == regionID) {
          continue;
        }

        Gateway gateway = new Gateway();
        gateway.exit = node;
        gateway.edgeID = bridge.getID();
        gateway.gatewayID = gatewayID;
        gateway.regionTo = possibleRegionID;
        gateway.entry = oppositeNode;
        gateway.nodesPair = new Pair<>(node, oppositeNode);

        gateway.distance = bridge.getLength();
        gateway.entryAngle = Angles.angle(node, oppositeNode);
        PedSimCity.regionsMap.get(regionID).gateways.add(gateway);
        PedSimCity.gatewaysMap.put(gatewayID, gateway);
        node.gateway = true;
        gatewayID += 1;
      }
      node.adjacentRegions = node.getAdjacentRegion();
    }
  }

  /**
   * Creates the edgesMap (edgeID, edgeGraph mapping) .
   */
  static private void createEdgesMap() {

    List<EdgeGraph> edges = PedSimCity.network.getEdges();
    for (EdgeGraph edge : edges) {
      int edgeID = edge.attributes.get("edgeID").getInteger();

      Optional.ofNullable(edge.attributes.get("lit")).map(AttributeValue::getInteger) // Integer
          .map(i -> new AttributeValue(i != 0)) // AttributeValue(boolean)
          .ifPresent(v -> edge.attributes.put("lit", v));

      edge.attributes.put("roadType", edge.attributes.get("highway"));
      edge.setID(edgeID);
      PedSimCity.edgesMap.put(edgeID, edge);
    }
  }

  /**
   * Centroids (Dual Graph): Assigns edgeID to centroids in the dual graph.
   */
  static private void prepareDualGraph() {

    List<MasonGeometry> centroids = PedSimCity.centroids.getGeometries();
    for (final MasonGeometry centroidGeometry : centroids) {
      int edgeID = centroidGeometry.getIntegerAttribute("edgeID");
      NodeGraph centroid =
          PedSimCity.dualNetwork.findNode(centroidGeometry.geometry.getCoordinate());
      centroid.setID(edgeID);
      centroid.setPrimalEdge(PedSimCity.edgesMap.get(edgeID));

      PedSimCity.edgesMap.get(edgeID).setDualNode(centroid);
      PedSimCity.centroidsMap.put(edgeID, centroid);
    }

    List<EdgeGraph> dualEdges = PedSimCity.dualNetwork.getEdges();
    for (EdgeGraph edge : dualEdges) {
      edge.setDeflectionAngle(edge.attributes.get("deg").getDouble());
    }
  }

  /**
   * Regions: Creates regions' subgraphs and store information.
   */
  static private void integrateBarriers() {

    List<EdgeGraph> edges = PedSimCity.network.getEdges();
    for (EdgeGraph edge : edges) {
      BarrierIntegration.setEdgeGraphBarriers(edge);
    }
    generateBarriersMap();
  }

  /**
   * Generates a map of barriers based on provided data.
   */
  private static void generateBarriersMap() {

    // Element 5 - Barriers: create barriers map
    List<MasonGeometry> geometries = PedSimCity.barriers.getGeometries();
    for (MasonGeometry barrierGeometry : geometries) {
      int barrierID = barrierGeometry.getIntegerAttribute("barrierID");
      Barrier barrier = new Barrier();
      barrier.masonGeometry = barrierGeometry;

      // Define a mapping from string attributes to BARRIER enums
      Map<String, StringEnum.BarrierType> barrierTypeMap =
          Map.of("park", BarrierType.PARK, "water", BarrierType.WATER, "road", BarrierType.ROAD,
              "railway", BarrierType.RAILWAY, "secondary_road", BarrierType.SECONDARY_ROAD);

      // Assign the correct BARRIERTYPE based on the string attribute
      barrier.type = barrierTypeMap.get(barrierGeometry.getStringAttribute("type"));

      List<EdgeGraph> edgesAlong = new ArrayList<>();

      for (EdgeGraph edge : PedSimCity.network.getEdges()) {
        List<Integer> edgeBarriers = edge.attributes.get("barriers").getArray();
        if (!edgeBarriers.isEmpty() && edgeBarriers.contains(barrierID)) {
          edgesAlong.add(edge);
        }
      }

      barrier.edgesAlong = edgesAlong;
      PedSimCity.barriersMap.put(barrierID, barrier);
    }

  }

  private static void prepareRegions() {

    List<EdgeGraph> edges = PedSimCity.network.getEdges();

    for (EdgeGraph edge : edges) {
      if (edge.getFromNode().getRegionID() == edge.getToNode().getRegionID()) {
        int regionID = edge.getFromNode().getRegionID();
        edge.setRegionID(regionID);
        PedSimCity.regionsMap.get(regionID).edges.add(edge);
        PedSimCity.regionsMap.get(regionID).nodes.add(edge.getFromNode());
        PedSimCity.regionsMap.get(regionID).nodes.add(edge.getToNode());
      } else {
        // gateway edge
        edge.setRegionID(-1);
      }
    }

    for (Entry<Integer, Region> entry : PedSimCity.regionsMap.entrySet()) {
      int regionID = entry.getKey();
      Region region = entry.getValue();

      List<EdgeGraph> edgesRegion = region.edges;
      SubGraph primalGraph = new SubGraph(edgesRegion);
      VectorLayer regionNetwork = new VectorLayer();
      List<EdgeGraph> dualEdgesRegion = new ArrayList<>();

      for (final EdgeGraph edge : edgesRegion) {
        regionNetwork.addGeometry(edge.getMasonGeometry());
        NodeGraph centroid = edge.getDualNode();
        centroid.setRegionID(regionID);
        DirectedEdgeStar directedEdges = centroid.getOutEdges();
        for (final DirectedEdge directedEdge : directedEdges.getEdges()) {
          dualEdgesRegion.add((EdgeGraph) directedEdge.getEdge());
        }
      }

      SubGraph dualGraph = new SubGraph(dualEdgesRegion);
      primalGraph.generateSubGraphCentralityMap();

      region.regionID = regionID;
      region.primalGraph = primalGraph;
      region.dualGraph = dualGraph;
      region.regionNetwork = regionNetwork;
    }
  }

  /**
   * Returns all the buildings enclosed between two nodes.
   *
   * @param originNode The first node.
   * @param destinationNode The second node.
   * @return A list of buildings.
   */
  public List<MasonGeometry> getBuildings(NodeGraph originNode, NodeGraph destinationNode) {
    Geometry smallestCircle = GraphUtils.smallestEnclosingGeometryBetweenNodes(
        new ArrayList<>(Arrays.asList(originNode, destinationNode)));
    return PedSimCity.buildings.containedFeatures(smallestCircle);
  }

  /**
   * Get buildings within a specified region.
   *
   * @param region The region for which buildings are to be retrieved.
   * @return An ArrayList of MasonGeometry objects representing buildings within the region.
   */
  public List<MasonGeometry> getBuildingsWithinRegion(Region region) {
    VectorLayer regionNetwork = region.regionNetwork;
    Geometry convexHull = regionNetwork.getConvexHull();
    return PedSimCity.buildings.containedFeatures(convexHull);
  }
}
