package pedsim.core.engine;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Optional;
import java.util.Set;
import org.javatuples.Pair;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.index.strtree.STRtree;
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
  private static final java.util.logging.Logger logger =
      pedsim.core.utilities.LoggerUtil.getLogger();

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

    if (!PedSimCity.censusZones.getGeometries().isEmpty()) {
      prepareCensusZones();
    }

    // Independent spatial joins for each optional dataset.
    // Each runs only when the layer was actually loaded.
    joinWeightLayerToNodes(
        PedSimCity.censusZonesVulnerability,
        "vulnerability_pct",
        PedSimCity.nodesVulnerabilityWeight,
        "censusData_vulnerability");

    joinWeightLayerToNodes(
        PedSimCity.nightPoiDensities,
        "poi_count",
        PedSimCity.nodesNightPoiWeight,
        "Night_POI_densities");

    joinWeightLayerToNodes(
        PedSimCity.workplacePoiDensities,
        "poi_count",
        PedSimCity.nodesWorkplacePoiWeight,
        "Workplace_POI_densities");

    // Join illuminated edges (mean_lux) onto the primal graph if the night dataset was loaded.
    if (!PedSimCity.illuminatedEdges.getGeometries().isEmpty()) {
      joinIlluminatedEdges();
    }
  }

  /**
   * Nodes: Assigns scores and attributes to nodes.
   */
  private static void prepareGraph() {

    List<MasonGeometry> geometries = PedSimCity.junctions.getGeometries();

    for (final MasonGeometry nodeGeometry : geometries) {
      // street junctions and betweenness centrality
      final NodeGraph node = PedSimCity.network.findNode(nodeGeometry.geometry.getCoordinate());
      node.dma = "";
      node.setID(nodeGeometry.getIntegerAttribute("nodeID"));
      node.setMasonGeometry(nodeGeometry);
      setCentralityNode(nodeGeometry, node);
      PedSimCity.nodesMap.put(node.getID(), node);
    }
    // generate the centrality map of the graph
    PedSimCity.network.generateCentralityMap();
    createEdgesMap();
  }

  private static void setCentralityNode(MasonGeometry nodeGeometry, NodeGraph node) {

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
  private static void prepareBuildings() {

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
          closest != null
              ? PedSimCity.network.findNode(closest.getGeometry().getCoordinate())
              : null;
      PedSimCity.buildingsMap.put(building.buildingID, building);
    }

    PedSimCity.network.getNodes().forEach(node -> node.dma = "");
    PedSimCity.network
        .getNodes()
        .forEach(
            (node) -> {
              List<MasonGeometry> nearestBuildings =
                  PedSimCity.buildings.featuresWithinDistance(
                      node.getMasonGeometry().geometry, 100);
              for (MasonGeometry building : nearestBuildings) {
                final int buildingID = building.getIntegerAttribute("buildingID");
                String dmaValue = PedSimCity.buildingsMap.get(buildingID).dma;
                if (dmaValue != null) {
                  node.attributes.put("DMA", new AttributeValue(dmaValue));
                  node.dma = dmaValue;
                }
              }
            });
  }

  /**
   * Gateways: Configures gateways between nodes.
   */
  private static void identifyGateways() {

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
  private static void createEdgesMap() {

    List<EdgeGraph> edges = PedSimCity.network.getEdges();
    for (EdgeGraph edge : edges) {
      int edgeID = edge.attributes.get("edgeID").getInteger();

      // Robust conversion of "lit" attribute.
      // If it's an Integer (1/0), convert to Boolean.
      // If it's already a Boolean, keep it.
      AttributeValue litAttr = edge.attributes.get("lit");
      if (litAttr != null) {
        Object val = litAttr.getValue();
        if (val instanceof Integer i) {
          edge.attributes.put("lit", new AttributeValue(i != 0));
        } else if (!(val instanceof Boolean)) {
          // Fallback or ignore if it's something else
          edge.attributes.put("lit", new AttributeValue(false));
        }
      }

      edge.attributes.put("roadType", edge.attributes.get("highway"));
      edge.setID(edgeID);
      PedSimCity.edgesMap.put(edgeID, edge);
    }
  }

  /**
   * Joins mean_lux from the illuminated edges dataset onto the primal graph edges by edgeID.
   * Only edges present in both datasets receive a mean_lux attribute.
   * Edges not present in the illuminated dataset are left without the attribute (default 0.0 in AgentMovement).
   */
  private static void joinIlluminatedEdges() {
    List<MasonGeometry> illuminatedGeoms = PedSimCity.illuminatedEdges.getGeometries();
    int joined = 0;
    int missing = 0;
    for (MasonGeometry geom : illuminatedGeoms) {
      AttributeValue edgeIDAttr = geom.getAttributes().get("edgeID");
      AttributeValue meanLuxAttr = geom.getAttributes().get("mean_lux");
      if (edgeIDAttr == null || meanLuxAttr == null) {
        missing++;
        continue;
      }
      int edgeID = edgeIDAttr.getInteger();
      EdgeGraph edge = PedSimCity.edgesMap.get(edgeID);
      if (edge == null) {
        missing++;
        continue;
      }
      double meanLux;
      try {
        meanLux = meanLuxAttr.getDouble();
      } catch (Exception e) {
        missing++;
        continue;
      }
      edge.attributes.put("mean_lux", new AttributeValue(meanLux));
      joined++;
    }
    logger.info(
        "Illuminated edges join: "
            + joined
            + " edges received mean_lux, "
            + missing
            + " skipped (no match or null).");
  }

  /**
   * Centroids (Dual Graph): Assigns edgeID to centroids in the dual graph.
   */
  private static void prepareDualGraph() {

    List<MasonGeometry> centroids = PedSimCity.centroids.getGeometries();
    int missingPrimalEdgeCount = 0;

    for (final MasonGeometry centroidGeometry : centroids) {
      int edgeID = centroidGeometry.getIntegerAttribute("edgeID");
      NodeGraph centroid =
          PedSimCity.dualNetwork.findNode(centroidGeometry.geometry.getCoordinate());
      centroid.setID(edgeID);

      EdgeGraph primalEdge = PedSimCity.edgesMap.get(edgeID);
      if (primalEdge != null) {
        centroid.setPrimalEdge(primalEdge);
        primalEdge.setDualNode(centroid);
      } else {
        missingPrimalEdgeCount++;
      }

      PedSimCity.centroidsMap.put(edgeID, centroid);
    }

    if (missingPrimalEdgeCount > 0) {
      logger.warning(
          "Found "
              + missingPrimalEdgeCount
              + " centroids with no corresponding primal edge in edgesMap.");
    }

    List<EdgeGraph> dualEdges = PedSimCity.dualNetwork.getEdges();
    for (EdgeGraph edge : dualEdges) {
      edge.setDeflectionAngle(edge.attributes.get("deg").getDouble());
    }
  }

  /**
   * Regions: Creates regions' subgraphs and store information.
   */
  private static void integrateBarriers() {

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

    // Build reverse index: barrierID → edges that reference it (O(edges) total).
    Map<Integer, List<EdgeGraph>> barrierToEdges = new HashMap<>();
    for (EdgeGraph edge : PedSimCity.network.getEdges()) {
      List<Integer> edgeBarriers = edge.attributes.get("barriers").getArray();
      for (Integer bid : edgeBarriers) {
        barrierToEdges.computeIfAbsent(bid, k -> new ArrayList<>()).add(edge);
      }
    }

    // Element 5 - Barriers: create barriers map
    List<MasonGeometry> geometries = PedSimCity.barriers.getGeometries();
    Map<String, StringEnum.BarrierType> barrierTypeMap =
        Map.of(
            "park",
            BarrierType.PARK,
            "water",
            BarrierType.WATER,
            "road",
            BarrierType.ROAD,
            "railway",
            BarrierType.RAILWAY,
            "secondary_road",
            BarrierType.SECONDARY_ROAD);

    for (MasonGeometry barrierGeometry : geometries) {
      int barrierID = barrierGeometry.getIntegerAttribute("barrierID");
      Barrier barrier = new Barrier();
      barrier.masonGeometry = barrierGeometry;

      String typeAttr = barrierGeometry.getStringAttribute("type");
      barrier.type = typeAttr == null ? null : barrierTypeMap.get(typeAttr);

      barrier.edgesAlong = barrierToEdges.getOrDefault(barrierID, new ArrayList<>());
      PedSimCity.barriersMap.put(barrierID, barrier);
    }
  }

  private static void prepareRegions() {

    List<EdgeGraph> edges = PedSimCity.network.getEdges();

    for (EdgeGraph edge : edges) {
      if (edge.getFromNode().getRegionID() == edge.getToNode().getRegionID()) {
        int regionID = edge.getFromNode().getRegionID();
        edge.setRegionID(regionID);
        Region region = PedSimCity.regionsMap.get(regionID);
        if (region != null) {
          region.edges.add(edge);
          region.nodes.add(edge.getFromNode());
          region.nodes.add(edge.getToNode());
        } else {
          logger.warning(
              "RegionID " + regionID + " not found in regionsMap for edge " + edge.getID());
        }
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
        if (centroid != null) {
          centroid.setRegionID(regionID);
          DirectedEdgeStar directedEdges = centroid.getOutEdges();
          for (final DirectedEdge directedEdge : directedEdges.getEdges()) {
            dualEdgesRegion.add((EdgeGraph) directedEdge.getEdge());
          }
        } else {
          // Log warning but don't crash
          // logger.warning("Edge " + edge.getID() + " has no dual node (centroid) in region " +
          // regionID);
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

  private static void prepareCensusZones() {

    final double MAX_PROXIMITY_M = 100.0;
    List<MasonGeometry> rawZones = PedSimCity.censusZones.getGeometries();
    List<NodeGraph> allNodes = SharedCognitiveMap.getCommunityPrimalNetwork().getNodes();

    // --- Pass 0: Fusion of Type 36 into Type 1 ---
    List<MasonGeometry> type1Zones = new ArrayList<>();
    List<MasonGeometry> type36Zones = new ArrayList<>();
    List<MasonGeometry> otherZones = new ArrayList<>();
    for (MasonGeometry z : rawZones) {
      int type = 0;
      try {
        type = z.getIntegerAttribute("censusZoneTypeID");
      } catch (Exception e) {
      }
      if (type == 1) type1Zones.add(z);
      else if (type == 36) type36Zones.add(z);
      else otherZones.add(z);
    }

    Map<MasonGeometry, MasonGeometry> fusionMap = new HashMap<>();
    if (!type1Zones.isEmpty()) {
      for (MasonGeometry z36 : type36Zones) {
        MasonGeometry parent = findClosestZone(z36, type1Zones);
        if (parent != null) {
          fusionMap.put(z36, parent);
          double w36 = 0, wP = 0;
          try {
            w36 = z36.getDoubleAttribute("residence_pct");
          } catch (Exception e) {
          }
          try {
            wP = parent.getDoubleAttribute("residence_pct");
          } catch (Exception e) {
          }
          parent.getAttributes().put("residence_pct", new AttributeValue(wP + w36));
        }
      }
    }

    // Resulting zones to track (Type 1 and Others)
    List<MasonGeometry> processedZones = new ArrayList<>(type1Zones);
    processedZones.addAll(otherZones);

    // Build spatial index of ALL zones (including Type 36) to capture nodes accurately
    STRtree index = new STRtree();
    for (MasonGeometry zone : rawZones) {
      if (zone.getGeometry() != null) {
        index.insert(zone.getGeometry().getEnvelopeInternal(), zone);
      }
    }
    index.build();
    PedSimCity.censusZonesSpatialIndex = index;

    // Register only result zones
    for (MasonGeometry zone : processedZones) {
      PedSimCity.censusZonesList.add(zone);
      PedSimCity.censusZonesNodesMap.put(zone, new ArrayList<>());
    }

    // --- Pass 1: spatial overlap - assign nodes that fall inside a zone ---
    Set<NodeGraph> assignedNodes = new HashSet<>();
    for (NodeGraph node : allNodes) {
      Geometry nodeGeom = node.getMasonGeometry().getGeometry();

      @SuppressWarnings("unchecked")
      List<MasonGeometry> candidates = index.query(nodeGeom.getEnvelopeInternal());

      for (MasonGeometry zone : candidates) {
        if (zone.getGeometry().contains(nodeGeom) || zone.getGeometry().distance(nodeGeom) < 1e-6) {

          // Fusion Logic: if node is in Type 36, redirect to Type 1 parent
          MasonGeometry targetZone = fusionMap.getOrDefault(zone, zone);

          if (PedSimCity.censusZonesNodesMap.containsKey(targetZone)) {
            PedSimCity.censusZonesNodesMap.get(targetZone).add(node);
            PedSimCity.nodesCensusZonesMap.put(node, targetZone);
            assignedNodes.add(node);
          }
          break;
        }
      }
    }

    // --- Pass 2: for processed zones still empty, find nearest unassigned node within 100m ---
    int fallbackZones = 0;
    int outOfBoundsZones = 0;
    for (MasonGeometry zone : processedZones) {
      if (!PedSimCity.censusZonesNodesMap.get(zone).isEmpty()) {
        continue; // already has nodes from overlap
      }
      NodeGraph nearest = nearestUnassignedNode(zone, allNodes, assignedNodes, MAX_PROXIMITY_M);
      if (nearest != null) {
        PedSimCity.censusZonesNodesMap.get(zone).add(nearest);
        PedSimCity.nodesCensusZonesMap.put(nearest, zone);
        assignedNodes.add(nearest);
        fallbackZones++;
      } else {
        outOfBoundsZones++; // no node within 100m – zone is outside city centre
      }
    }

    int matched = PedSimCity.nodesCensusZonesMap.size();
    java.util.logging.Logger.getLogger("pedsim.core.engine.Environment")
        .info(
            "censusData: spatial join done. Nodes matched to a zone: "
                + matched
                + " / "
                + allNodes.size()
                + " | Fallback zones: "
                + fallbackZones
                + " | Out-of-bounds zones: "
                + outOfBoundsZones);
  }

  /**
   * Generic helper that joins a weight layer to the road network with value splitting.
   *
   * <p>For each zone polygon in {@code layer}:
   * <ol>
   *   <li>Spatial overlap: collect all network nodes that fall inside the zone.</li>
   *   <li>Fallback: if a zone has no overlapping nodes, find the nearest <em>unassigned</em>
   *       node within {@code MAX_PROXIMITY_M} metres and assign the zone to it.</li>
   *   <li>Out-of-bounds: if still no node is found, the zone is skipped (assumed outside
   *       the city-centre network).</li>
   *   <li>The zone's attribute value is divided equally across all assigned nodes.</li>
   * </ol>
   * Nodes not assigned to any zone receive 0.0.
   * If the layer is null or empty the method returns immediately without error.
   *
   * @param layer       The spatial layer to join (may be null or empty).
   * @param attribute   The numeric attribute column to read from each zone polygon.
   * @param targetMap   The per-node output map to populate.
   * @param datasetName Human-readable name used only for log messages.
   */
  private static void joinWeightLayerToNodes(
      sim.field.geo.VectorLayer layer,
      String attribute,
      Map<NodeGraph, Double> targetMap,
      String datasetName) {

    if (layer == null || layer.getGeometries().isEmpty()) {
      return; // dataset not loaded – skip silently
    }

    final double MAX_PROXIMITY_M = 100.0;
    List<MasonGeometry> zones = layer.getGeometries();
    List<NodeGraph> allNodes = SharedCognitiveMap.getCommunityPrimalNetwork().getNodes();

    // --- Pass 0: Fusion (Weight-based) ---
    List<MasonGeometry> type1Zones = new ArrayList<>();
    List<MasonGeometry> type36Zones = new ArrayList<>();
    List<MasonGeometry> otherZones = new ArrayList<>();
    for (MasonGeometry z : zones) {
      int type = 0;
      try {
        type = z.getIntegerAttribute("censusZoneTypeID");
      } catch (Exception e) {
      }
      if (type == 1) type1Zones.add(z);
      else if (type == 36) type36Zones.add(z);
      else otherZones.add(z);
    }

    Map<MasonGeometry, MasonGeometry> fusionMap = new HashMap<>();
    if (!type1Zones.isEmpty()) {
      for (MasonGeometry z36 : type36Zones) {
        MasonGeometry parent = findClosestZone(z36, type1Zones);
        if (parent != null) {
          fusionMap.put(z36, parent);
          double w36 = 0, wP = 0;
          try {
            w36 = z36.getDoubleAttribute(attribute);
          } catch (Exception e) {
          }
          try {
            wP = parent.getDoubleAttribute(attribute);
          } catch (Exception e) {
          }
          parent.getAttributes().put(attribute, new AttributeValue(wP + w36));
        }
      }
    }

    List<MasonGeometry> processedZones = new ArrayList<>(type1Zones);
    processedZones.addAll(otherZones);

    // Build a temporary spatial index for ALL zones in this layer
    STRtree index = new STRtree();
    for (MasonGeometry zone : zones) {
      if (zone.getGeometry() != null) {
        index.insert(zone.getGeometry().getEnvelopeInternal(), zone);
      }
    }
    index.build();

    // --- Pass 1: spatial overlap ---
    Map<MasonGeometry, List<NodeGraph>> zoneNodesMap = new HashMap<>();
    for (MasonGeometry z : processedZones) zoneNodesMap.put(z, new ArrayList<>());

    Set<NodeGraph> assignedNodes = new HashSet<>();
    for (NodeGraph node : allNodes) {
      Geometry nodeGeom = node.getMasonGeometry().getGeometry();

      @SuppressWarnings("unchecked")
      List<MasonGeometry> candidates = index.query(nodeGeom.getEnvelopeInternal());

      for (MasonGeometry zone : candidates) {
        if (zone.getGeometry().contains(nodeGeom) || zone.getGeometry().distance(nodeGeom) < 1e-6) {

          MasonGeometry targetZone = fusionMap.getOrDefault(zone, zone);
          if (zoneNodesMap.containsKey(targetZone)) {
            zoneNodesMap.get(targetZone).add(node);
            assignedNodes.add(node);
          }
          break; // a node belongs to exactly one zone
        }
      }
    }

    // --- Pass 2: fallback for empty processed zones - nearest unassigned node within 100m ---
    int fallbackZones = 0;
    int outOfBoundsZones = 0;
    for (MasonGeometry zone : processedZones) {
      if (zoneNodesMap.get(zone) != null && !zoneNodesMap.get(zone).isEmpty()) {
        continue; // already has overlapping nodes
      }
      NodeGraph nearest = nearestUnassignedNode(zone, allNodes, assignedNodes, MAX_PROXIMITY_M);
      if (nearest != null) {
        zoneNodesMap.computeIfAbsent(zone, z -> new ArrayList<>()).add(nearest);
        assignedNodes.add(nearest);
        fallbackZones++;
      } else {
        outOfBoundsZones++; // no node within 100m – skip this zone
      }
    }

    // --- Pass 3: divide zone value by node count and write to targetMap ---
    int matched = 0;
    for (Map.Entry<MasonGeometry, List<NodeGraph>> entry : zoneNodesMap.entrySet()) {
      MasonGeometry zone = entry.getKey();
      List<NodeGraph> nodesInZone = entry.getValue();

      double zoneValue;
      try {
        zoneValue = zone.getDoubleAttribute(attribute);
      } catch (Exception e) {
        zoneValue = 0.0;
      }

      double perNodeValue = nodesInZone.isEmpty() ? 0.0 : zoneValue / nodesInZone.size();
      for (NodeGraph node : nodesInZone) {
        targetMap.put(node, perNodeValue);
        matched++;
      }
    }

    // Nodes not assigned to any zone get 0.0
    for (NodeGraph node : allNodes) {
      if (!assignedNodes.contains(node)) {
        targetMap.put(node, 0.0);
      }
    }

    java.util.logging.Logger.getLogger("pedsim.core.engine.Environment")
        .info(
            datasetName
                + ": spatial join done. Nodes matched to a zone: "
                + matched
                + " / "
                + allNodes.size()
                + " | Fallback zones: "
                + fallbackZones
                + " | Out-of-bounds zones: "
                + outOfBoundsZones);
  }

  /**
   * Finds the nearest node that has not yet been assigned to any zone,
   * within {@code maxDistanceMetres}. Returns null if no such node exists.
   */
  private static NodeGraph nearestUnassignedNode(
      MasonGeometry zone,
      List<NodeGraph> allNodes,
      Set<NodeGraph> assignedNodes,
      double maxDistanceMetres) {

    org.locationtech.jts.geom.Point centroid = zone.getGeometry().getCentroid();
    NodeGraph nearest = null;
    double nearestDist = Double.MAX_VALUE;

    for (NodeGraph node : allNodes) {
      if (assignedNodes.contains(node) || node.getMasonGeometry().getGeometry() == null) {
        continue; // already claimed or no geometry
      }
      double dist = centroid.distance(node.getMasonGeometry().getGeometry());
      if (dist < nearestDist) {
        nearestDist = dist;
        nearest = node;
      }
    }

    return (nearestDist <= maxDistanceMetres) ? nearest : null;
  }

  private static MasonGeometry findClosestZone(
      MasonGeometry target, List<MasonGeometry> candidates) {
    if (candidates.isEmpty()) return null;
    Geometry targetGeom = target.getGeometry();
    MasonGeometry closest = null;
    double minDist = Double.MAX_VALUE;
    for (MasonGeometry cand : candidates) {
      double dist = targetGeom.distance(cand.getGeometry());
      if (dist < minDist) {
        minDist = dist;
        closest = cand;
      }
    }
    return closest;
  }

  /**
   * Returns all the buildings enclosed between two nodes.
   *
   * @param originNode The first node.
   * @param destinationNode The second node.
   * @return A list of buildings.
   */
  public List<MasonGeometry> getBuildings(NodeGraph originNode, NodeGraph destinationNode) {
    Geometry smallestCircle =
        GraphUtils.smallestEnclosingGeometryBetweenNodes(
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
