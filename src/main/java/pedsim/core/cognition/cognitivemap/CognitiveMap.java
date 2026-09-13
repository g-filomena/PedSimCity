package pedsim.core.cognition.cognitivemap;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashSet;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Set;
import java.util.stream.Collectors;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.Polygon;
import pedsim.core.agents.Agent;
import pedsim.core.cognition.cityimage.Region;
import pedsim.core.cognition.network.NetworkBuilder;
import pedsim.core.engine.PedSimCity;
import pedsim.core.parameters.Pars;
import pedsim.core.parameters.RouteChoicePars;
import sim.graph.EdgeGraph;
import sim.graph.GraphUtils;
import sim.graph.Islands;
import sim.graph.NodeGraph;
import sim.routing.Astar;

/**
 * Represents an agent's cognitive map, which provides access to various map
 * attributes. In this version of PedSimCity, this is a simple structure
 * designed for further developments.
 */
public class CognitiveMap extends SharedCognitiveMap {

  /**
   * Mean of the spatial ability an agent is given, which is then spread half a point either way.
   * A trait every agent has, learning module or not, which is why it lives in core.
   */
  private static final double MEAN_SPATIAL_ABILITY = 0.75;

  Geometry activityBone;

  private List<Polygon> cognitiveCollage = new ArrayList<Polygon>();
  Geometry knownSpace = null;

  NetworkBuilder networkBuilder;
  protected Set<Integer> activityBoneNodes = new HashSet<>();
  protected Set<Integer> activityBoneEdges = new HashSet<>();
  protected Set<Integer> agentKnownNodes = new HashSet<>();
  protected Set<Integer> agentKnownEdges = new HashSet<>();

  protected Set<Integer> agentKnownRegions = new HashSet<>();
  protected Set<Integer> agentKnownBarriers = new HashSet<>();
  protected Set<Integer> agentKnownLocalLandmarks = new HashSet<>();

  protected Agent agent;
  GeometryFactory GEOMETRY_FACTORY = new GeometryFactory();
  public boolean formed = false;

  public double spatialAbility;
  public boolean individualised;

  /**
   * Constructs an AgentCognitiveMap.
   */
  public CognitiveMap(Agent agent) {

    this.agent = agent;
    spatialAbility =
        Math.min(
            1.0,
            Math.max(
                0.0,
                MEAN_SPATIAL_ABILITY
                    + (agent.getRandom().nextDouble() - 0.5) * 0.5));
  }

  public void formCognitiveMap() {

    buildActivityBone();
    fuseBoneWithCommunityNetwork();
    networkBuilder = new NetworkBuilder(this);
    networkBuilder.buildKnownNetwork();
    identifyKnownUrbanElements();
    individualised = true;
    formed = true;
  }

  private void buildActivityBone() {

    List<NodeGraph> knownNodes = agent.cognitiveAnchors();
    if (knownNodes.isEmpty()) {
      return;
    }

    Set<NodeGraph> activityBoneNodesTmp = new HashSet<>();
    Queue<NodeGraph> queue = new LinkedList<>();

    for (NodeGraph node : knownNodes) {
      activityBoneNodesTmp.add(node);
      queue.add(node);
      int region = node.getRegionID();
      agentKnownRegions.add(region);
      Region r = PedSimCity.regionsMap.get(region);
      if (r != null) activityBoneNodesTmp.addAll(r.nodes);
    }

    Map<NodeGraph, Double> distanceMap = new HashMap<>();
    for (NodeGraph anchor : knownNodes) {
      distanceMap.put(anchor, 0.0);
    }

    // Step 1: Collect nearby nodes with cumulative distance tracking
    while (!queue.isEmpty()) {
      NodeGraph currentNode = queue.poll();
      double currentDistance = distanceMap.get(currentNode);

      for (EdgeGraph edge : currentNode.getEdges()) {
        NodeGraph neighborNode = edge.getOtherNode(currentNode);
        double newDistance = currentDistance + edge.getLength(); // Cumulative distance

        if (!activityBoneNodesTmp.contains(neighborNode) && newDistance <= Pars.anchorRadius) {
          activityBoneNodesTmp.add(neighborNode);
          queue.add(neighborNode);
          distanceMap.put(neighborNode, newDistance); // Store cumulative distance
        }
      }
    }

    // Step 2: connectivity. The bone is not a set of islands: the streets between the anchors are
    // part of what someone knows, so a path is traced from the first anchor to each of the others.
    Astar astar = new Astar();
    NodeGraph origin = knownNodes.get(0);
    for (int i = 1; i < knownNodes.size(); i++) {
      List<NodeGraph> shortestPath =
          astar.astarRoute(
                  origin, knownNodes.get(i), SharedCognitiveMap.getCommunityPrimalNetwork(), null)
              .nodesSequence;
      if (!shortestPath.isEmpty()) {
        activityBoneNodesTmp.addAll(shortestPath);
      }
    }

    if (RouteChoicePars.cityCentreRegionsID.length > 0) {
      for (Integer regionID : RouteChoicePars.cityCentreRegionsID) {
        Region r = PedSimCity.regionsMap.get(regionID);
        if (r != null) activityBoneNodesTmp.addAll(r.nodes);
      }
    }

    for (NodeGraph node : activityBoneNodesTmp) {
      this.activityBoneNodes.add(node.getID());
      this.activityBoneEdges.addAll(GraphUtils.getEdgeIDs(node.getEdges()));
    }
  }

  /**
   * Builds the activity bone, which includes the agent's home and work nodes
   * along with edges in the known regions and from those nodes.
   *
   * <p>The cheap alternative to {@link #formCognitiveMap()}: home and work regions plus the edges
   * incident on those two nodes, and nothing else. No {@link NetworkBuilder}, no {@link Islands}
   * decomposition, no A* between the anchors — which is why the night module, the only caller,
   * does a day with thousands of agents in the time the full build takes for hundreds.
   *
   * <p>A null anchor is skipped rather than treated as an error: personas without a mandatory
   * activity (retirees, flex adults) are deliberately given no work node by
   * {@code ActivityPopulate.applyPersonaEmployment}, so their bone is built from home alone.
   *
   * <p><b>It fills {@code agentKnownEdges} and deliberately leaves {@code agentKnownNodes}
   * empty</b>, so {@link #getNodesInKnownNetwork()} returns nothing for an agent built this way.
   * That is safe only because this method leaves {@link #individualised} {@code false}: the flag
   * is what {@code Dijkstra.initialisePrimal} consults before confining its search to the known
   * subgraph, so such an agent routes over the full community network and the empty node set is
   * never asked for. The known edges are a <i>preference</i> signal here — what {@code
   * NightBehaviour} scores for comfort and what a vulnerable agent avoids — not a statement about
   * what is reachable.
   *
   * <p><b>The trap:</b> anything that turns {@code individualised} on for an agent whose bone was
   * built here, or any route through {@code Agent.defineRandomDestination()} — which filters its
   * candidate set by {@code getAgentKnownNodes()} — gets an empty set and silently degrades:
   * Dijkstra to an empty subgraph, destination choice to {@code recordDestinationFallback()} on
   * every trip. {@code NightAgent} avoids the second by overriding the method and passing {@code
   * null} for the restriction. If a simple-bone agent ever needs either, populate {@code
   * agentKnownNodes} here first — the nodes are already to hand.
   */
  public void buildSimpleActivityBone() {

    NodeGraph[] knownNodes = {agent.getHome(), agent.getWork()};

    for (NodeGraph node : knownNodes) {
      if (node == null) continue;
      int region = node.getRegionID();
      agentKnownRegions.add(region);
      // Declared per anchor, not outside the loop: accumulating across anchors would re-collect and
      // re-convert every earlier anchor's edges on each iteration.
      List<EdgeGraph> edges = new ArrayList<>();
      Region r = PedSimCity.regionsMap.get(region);
      if (r != null) edges.addAll(r.edges);
      edges.addAll(node.getEdges());
      agentKnownEdges.addAll(GraphUtils.getEdgeIDs(edges));
    }
  }

  private void fuseBoneWithCommunityNetwork() {

    for (Integer regionID : RouteChoicePars.cityCentreRegionsID) {
      agentKnownRegions.add(regionID);
      agentKnownNodes.addAll(GraphUtils.getNodeIDs(PedSimCity.regionsMap.get(regionID).nodes));
    }

    agentKnownNodes.addAll(activityBoneNodes);
    agentKnownNodes.addAll(GraphUtils.getNodeIDs(SharedCognitiveMap.getCommunityKnownNodes()));
    agentKnownEdges.addAll(activityBoneEdges);
    agentKnownEdges.addAll(GraphUtils.getEdgeIDs(SharedCognitiveMap.getCommunityKnownEdges()));
  }

  public void readjustCognitiveMap(List<Polygon> polygons) {

    agentKnownNodes.clear();
    agentKnownEdges.clear();
    cognitiveCollage = new ArrayList<Polygon>(polygons);
    // for (Pair<Polygon, List<NodeGraph>> pair : collage.keySet()) {
    for (Polygon polygon : cognitiveCollage) {
      // List<NodeGraph> nodesInKnownSpace = pair.getValue1();
      List<NodeGraph> nodesInKnownSpace =
          SharedCognitiveMap.getCommunityPrimalNetwork().getNodesWithinPolygon(polygon);
      agentKnownNodes.addAll(GraphUtils.getNodeIDs(nodesInKnownSpace));
      nodesInKnownSpace.forEach(
          node -> agentKnownEdges.addAll(GraphUtils.getEdgeIDs(node.getEdges())));
    }

    fuseBoneWithCommunityNetwork();
    identifyKnownUrbanElements();
    networkBuilder.buildKnownNetwork();
  }

  private void identifyKnownUrbanElements() {
    deriveOtherKnownRegions();
    findKnownBarriers();
  }

  public void deriveOtherKnownRegions() {

    Islands islands = new Islands(SharedCognitiveMap.getCommunityPrimalNetwork());

    // A set, not a list. This collected a region id per known *node*, so a region the agent knows
    // five hundred nodes of was queued five hundred times - and the loop below ran a full island
    // decomposition for every one of them. LinkedHashSet rather than HashSet so the order stays
    // fixed from run to run, which matters for a model that is meant to replay from its seed.
    Set<Integer> potentiallyKnownRegions = new LinkedHashSet<>();

    for (int nodeID : agentKnownNodes) {
      NodeGraph node = PedSimCity.nodesMap.get(nodeID);
      int regionID = node.getRegionID();
      if (agentKnownRegions.contains(regionID)) {
        continue;
      }
      potentiallyKnownRegions.add(regionID);
    }
    if (potentiallyKnownRegions.isEmpty()) {
      return;
    }

    // One pass over the known network, bucketed by region, instead of re-filtering the whole of it
    // once per region.
    Map<Integer, Set<EdgeGraph>> edgesByRegion = new HashMap<>();
    for (EdgeGraph edge : getEdgesInKnownNetwork()) {
      if (potentiallyKnownRegions.contains(edge.getRegionID())) {
        edgesByRegion.computeIfAbsent(edge.getRegionID(), unused -> new HashSet<>()).add(edge);
      }
    }

    for (int regionID : potentiallyKnownRegions) {
      Set<EdgeGraph> regionEdges = edgesByRegion.get(regionID);
      if (regionEdges == null || regionEdges.isEmpty()) {
        continue;
      }
      if (islands.findDisconnectedIslands(regionEdges).size() == 1) {
        agentKnownRegions.add(regionID);
      }
    }
  }

  // Methods to add and retrieve nodes, edges, landmarks, and regions
  public void findKnownLocalLandmarks(double localLandmarkThreshold) {

    agentKnownLocalLandmarks = new HashSet<>();

    List<NodeGraph> tmpNodes = GraphUtils.getNodesFromNodeIDs(agentKnownNodes, PedSimCity.nodesMap);
    // Collect local landmarks efficiently using streams
    tmpNodes.stream()
        .flatMap(node -> node.adjacentBuildings.stream())
        .filter(
            building -> {
              var attr = building.attributes.get("localLandmarkness");
              if (attr == null) {
                return false; // city without local-landmark scores: skip, don't NPE
              }
              Double lScore = attr.getDouble();
              return lScore != null && lScore > localLandmarkThreshold;
            })
        .map(building -> building.buildingID)
        .forEach(agentKnownLocalLandmarks::add);
  }

  // Methods to add and retrieve nodes, edges, landmarks, and regions
  private void findKnownBarriers() {

    agentKnownBarriers = new HashSet<>();
    List<EdgeGraph> tmpEdges = GraphUtils.getEdgesFromEdgeIDs(agentKnownEdges, PedSimCity.edgesMap);
    for (EdgeGraph edge : tmpEdges) {
      List<Integer> barrierIDs = edge.attributes.get("barriers").getArray();
      agentKnownBarriers.addAll(barrierIDs);
    }
    agentKnownBarriers.addAll(SharedCognitiveMap.communityKnownBarriers);
  }

  public Set<Integer> getAgentKnownNodes() {
    return new HashSet<>(agentKnownNodes);
  }

  public Set<Integer> getAgentKnownEdges() {
    return new HashSet<>(agentKnownEdges);
  }

  public Set<Integer> getAgentKnownRegions() {
    return agentKnownRegions;
  }

  /**
   * Gets the local landmarks from the cognitive map.
   *
   * @return The local landmarks.
   */
  @Override
  public Set<Integer> getLocalLandmarksIDs() {
    return agentKnownLocalLandmarks;
  }

  public Set<Integer> getAgentKnownBarriers() {
    return agentKnownBarriers;
  }

  public Set<NodeGraph> getNodesInKnownNetwork() {
    if (networkBuilder == null) {
      return new HashSet<>(GraphUtils.getNodesFromNodeIDs(agentKnownNodes, PedSimCity.nodesMap));
    }
    return new HashSet<>(networkBuilder.getNecessaryNodes());
  }

  public Set<EdgeGraph> getEdgesInKnownNetwork() {
    if (networkBuilder == null) {
      return new HashSet<>(GraphUtils.getEdgesFromEdgeIDs(agentKnownEdges, PedSimCity.edgesMap));
    }
    return new HashSet<>(networkBuilder.getNecessaryEdges());
  }

  public Set<Integer> getNodeIDsInKnownNetwork() {
    if (networkBuilder == null) return new HashSet<>(agentKnownNodes);
    return new HashSet<>(GraphUtils.getNodeIDs(networkBuilder.getNecessaryNodes()));
  }

  public Set<Integer> getEdgeIDsInKnownNetwork() {
    if (networkBuilder == null) return new HashSet<>(agentKnownEdges);
    return new HashSet<>(GraphUtils.getEdgeIDs(networkBuilder.getNecessaryEdges()));
  }

  public boolean isEdgeKnown(EdgeGraph edgeGraph) {
    return agentKnownEdges.contains(edgeGraph.getID());
  }

  public boolean isRegionKnown(Integer regionID) {
    return agentKnownRegions.contains(regionID);
  }

  public Set<NodeGraph> getNodesInKnownDualNetwork() {
    if (networkBuilder == null) return new HashSet<>();
    return new HashSet<>(networkBuilder.getNecessaryDualNodes());
  }

  public Set<EdgeGraph> getEdgesInKnownDualNetwork() {
    if (networkBuilder == null) return new HashSet<>();
    return new HashSet<>(networkBuilder.getNecessaryDualEdges());
  }

  // public void resetRegionMap() {
  // knownRegionsMap.clear();
  // }

  public boolean isInKnownNetwork(NodeGraph nodeGraph) {
    if (getNodesInKnownNetwork().contains(nodeGraph)) {
      return true;
    }
    return false;
  }

  public boolean isInKnownNetwork(EdgeGraph edgeGraph) {
    if (getEdgesInKnownNetwork().contains(edgeGraph)) {
      return true;
    }
    return false;
  }

  public double getWayfindingEasinessThreshold(boolean regionBased) {
    // TODO Auto-generated method stub
    return 0;
  }
  //
  // public double getLocalLandmarkThreshold() {
  // // TODO Auto-generated method stub
  // return 0;
  // }

  // /**
  // * Gets local landmarks for a specific region.
  // *
  // * @param region The region for which to get local landmarks.
  // * @return A list of local landmarks.
  // */
  // @Override
  // public List<MasonGeometry> getRegionLocalLandmarks(Region region) {
  // List<MasonGeometry> regionLocalLandmarks = new
  // ArrayList<>(region.localLandmarks);
  // regionLocalLandmarks.retainAll(knownLocalLandmarks.getGeometries());
  // return regionLocalLandmarks;
  // }

  // knownNodes.add(homeNode);
  // knownNodes.add(workNode);
}
