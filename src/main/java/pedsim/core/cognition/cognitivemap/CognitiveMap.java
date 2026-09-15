package pedsim.core.cognition.cognitivemap;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Set;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.Polygon;
import org.locationtech.jts.geom.prep.PreparedGeometry;
import org.locationtech.jts.geom.prep.PreparedGeometryFactory;
import org.locationtech.jts.index.strtree.STRtree;
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
            Math.max(0.0, MEAN_SPATIAL_ABILITY + (agent.getRandom().nextDouble() - 0.5) * 0.5));
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
    List<NodeGraph> nodesInKnownSpace = nodesWithinCollage(cognitiveCollage);
    agentKnownNodes.addAll(GraphUtils.getNodeIDs(nodesInKnownSpace));
    for (NodeGraph node : nodesInKnownSpace) {
      agentKnownEdges.addAll(GraphUtils.getEdgeIDs(node.getEdges()));
    }

    fuseBoneWithCommunityNetwork();
    identifyKnownUrbanElements();
    networkBuilder.buildKnownNetwork();
  }

  /**
   * The network's nodes that fall inside any polygon of the collage.
   *
   * <p>Same predicate as {@code Graph.getNodesWithinPolygon}, which is a plain
   * {@code Polygon.contains} on the node's geometry, and the same result: the loop this replaced
   * called that method once per collage polygon and unioned the answers into a set, so nothing
   * downstream ever saw the per-polygon grouping.
   *
   * <p>What it stops paying for is the shape of that loop. Each call scanned every node in the
   * city, and each test built a fresh JTS geometry graph for the polygon, so the cost was the
   * number of nodes times the number of polygons - and a collage is one polygon per connected blob
   * of remembered 5 m cells, so there are many. Here the polygons go into an STRtree once, prepared
   * (which indexes their edges instead of re-deriving them per test), and the nodes are walked
   * once. This was the whole remaining cost of the learning module's cognitive-map rebuild.
   */
  private static List<NodeGraph> nodesWithinCollage(List<Polygon> polygons) {
    List<NodeGraph> nodesInside = new ArrayList<>();
    if (polygons.isEmpty()) {
      return nodesInside;
    }
    STRtree index = new STRtree();
    for (Polygon polygon : polygons) {
      index.insert(polygon.getEnvelopeInternal(), PreparedGeometryFactory.prepare(polygon));
    }
    index.build();

    for (NodeGraph node : SharedCognitiveMap.getCommunityPrimalNetwork().getNodes()) {
      Geometry nodeGeometry = node.getMasonGeometry().getGeometry();
      @SuppressWarnings("unchecked")
      List<PreparedGeometry> candidates = index.query(nodeGeometry.getEnvelopeInternal());
      for (PreparedGeometry candidate : candidates) {
        if (candidate.contains(nodeGeometry)) {
          nodesInside.add(node);
          break;
        }
      }
    }
    return nodesInside;
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
        // LinkedHashSet: these buckets are handed to Islands.findDisconnectedIslands, and EdgeGraph
        // has no hashCode of its own, so a HashSet would order them by identity hash - stable
        // within
        // a JVM build and different across them.
        edgesByRegion
            .computeIfAbsent(edge.getRegionID(), unused -> new LinkedHashSet<>())
            .add(edge);
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

  /** The threshold the current {@code agentKnownLocalLandmarks} was collected at, or NaN. */
  private double localLandmarksThresholdUsed = Double.NaN;

  /**
   * Collects the local landmarks this agent can recognise: the buildings beside the nodes it knows
   * whose local-landmarkness clears {@code localLandmarkThreshold}.
   *
   * <p>A map that was never individualised has no known nodes, so it walks the community known
   * network instead - the set {@link #getNodesInKnownNetwork()} answers with, and the one an
   * individualised map has fused into its own bone. Collecting nothing there would leave
   * {@code Landmarkness.localLandmarknessNode}, which scores a candidate node by the best of its
   * adjacent landmarks <i>that are in this set</i>, returning 0.0 for every node of every route.
   *
   * <p>The community walk is done once per agent and reused: the threshold comes from the agent's
   * heuristics and does not change between trips, and this is called on every landmark-routed leg.
   *
   * @param localLandmarkThreshold the minimum local-landmarkness score to recognise
   */
  public void findKnownLocalLandmarks(double localLandmarkThreshold) {

    boolean community = agentKnownNodes.isEmpty() && !individualised;
    // Memoise the community walk only. An individualised map is re-derived as the agent learns -
    // the learning module rebuilds it outright - so its landmark set has to be recollected even at
    // an unchanged threshold; the community one cannot change between trips.
    if (community && localLandmarkThreshold == localLandmarksThresholdUsed) {
      return;
    }
    localLandmarksThresholdUsed = community ? localLandmarkThreshold : Double.NaN;
    agentKnownLocalLandmarks = new HashSet<>();

    List<NodeGraph> tmpNodes =
        community
            ? new ArrayList<>(SharedCognitiveMap.getCommunityKnownNodes())
            : GraphUtils.getNodesFromNodeIDs(agentKnownNodes, PedSimCity.nodesMap);
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

  /**
   * The regions this agent can navigate through.
   *
   * <p>An individualised map carries the regions the agent has derived for itself. A map that was
   * never individualised - a community-network agent - carries none, and knows the city's regions
   * instead: it routes on the community network, so every region on it is available to it.
   *
   * <p>The distinction is load-bearing. {@code RegionBasedNavigation} filters the city's regions
   * down to this set before looking for gateways, so an empty one makes
   * {@code isRegionalSequenceDouable()} false for every pair and region-based navigation silently
   * degrades to plain shortest path - indistinguishable, in the output, from region navigation that
   * ran and found nothing worth doing.
   *
   * @return the region IDs available to this agent
   */
  public Set<Integer> getAgentKnownRegions() {
    if (agentKnownRegions.isEmpty() && !individualised) {
      return new HashSet<>(PedSimCity.regionsMap.keySet());
    }
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

  /**
   * The barriers this agent perceives.
   *
   * <p>As with {@link #getAgentKnownRegions()}: an individualised map carries the barriers along the
   * streets the agent knows, plus the community's; a map that was never individualised carries the
   * community's alone, which is what {@code SharedCognitiveMap.communityKnownBarriers} means - the
   * water and road barriers everyone in the city knows about.
   *
   * <p>Both the cost adjustments in {@code Dijkstra.costPerceptionError} and the sub-goals in
   * {@code BarrierIntegration} test membership of this set, so an empty one leaves barrier-based
   * navigation with no effect on either cost or routing.
   *
   * @return the barrier IDs this agent perceives
   */
  public Set<Integer> getAgentKnownBarriers() {
    if (agentKnownBarriers.isEmpty() && !individualised) {
      // Every barrier, not {@code communityKnownBarriers}: that set holds water and road only, so
      // an agent restricted to it perceives no park and no railway, and the barrier type it is
      // configured to seek or avoid may be one it can never see. What narrows the candidates for
      // such an agent is its {@code AgentBarrierType}, which is the only filter that applied before
      // the known-barrier restriction existed.
      return new HashSet<>(PedSimCity.barriersMap.keySet());
    }
    return agentKnownBarriers;
  }

  /**
   * The nodes this agent can route over.
   *
   * <p>An individualised map answers with the network it built for itself, which
   * {@code fuseBoneWithCommunityNetwork} defines as <i>its own bone plus the community known
   * network</i>. A map that was never individualised has no bone, so it answers with the community
   * known network alone: the same formula with the personal half empty.
   *
   * <p>The community network is a <b>subset</b> of the city - primary and secondary roads, tertiary
   * when {@code includeTertiary}, the city-centre regions and the salient junctions. A cityImage or
   * empirical agent <i>routes</i> over the full network, because {@code restrictToKnownNetwork()} is
   * false for it, but it <i>plans</i> - picks gateways, barrier sub-goals, on-route marks - among
   * the places everyone in the city is taken to know.
   *
   * <p><b>This must never answer with an empty set.</b>
   * {@code RegionBasedNavigation.getKnownGateways} keeps only gateways whose entry and exit are both
   * in here, so an empty one leaves no gateway in any region and region-based navigation degrades to
   * the shortest path - with no error and nothing in the output to say the model did not run.
   *
   * @return the nodes available to this agent
   */
  public Set<NodeGraph> getNodesInKnownNetwork() {
    if (networkBuilder == null) {
      if (agentKnownNodes.isEmpty() && !individualised) {
        return SharedCognitiveMap.getCommunityKnownNodes();
      }
      return new HashSet<>(GraphUtils.getNodesFromNodeIDs(agentKnownNodes, PedSimCity.nodesMap));
    }
    return new HashSet<>(networkBuilder.getNecessaryNodes());
  }

  /**
   * The edges this agent can route over. See {@link #getNodesInKnownNetwork()} for why a map that
   * was never individualised answers with the whole city rather than with nothing.
   *
   * <p>The gate here is {@code BarrierBasedNavigation}, which keeps only the edges along a barrier
   * that are in this set before looking for a sub-goal. An empty set empties {@code edgesAlong} for
   * every barrier in turn, so no barrier yields a sub-goal and barrier navigation is left with the
   * cost adjustments in {@code Dijkstra.costPerceptionError} alone - a weaker model than the one the
   * scenario names. A community agent offers the barrier's stretches along community-known streets:
   * a sub-goal is somewhere the agent steers towards by name.
   *
   * <p>A night agent is the case this must not disturb: {@code buildSimpleActivityBone} leaves
   * {@code individualised} false but fills {@code agentKnownEdges}, so it is non-empty and keeps its
   * bone.
   *
   * @return the edges available to this agent
   */
  public Set<EdgeGraph> getEdgesInKnownNetwork() {
    if (networkBuilder == null) {
      if (agentKnownEdges.isEmpty() && !individualised) {
        return SharedCognitiveMap.getCommunityKnownEdges();
      }
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

  /**
   * Whether this agent can navigate within the given region.
   *
   * <p>Goes through {@link #getAgentKnownRegions()} rather than the field, so a community-network
   * agent answers for the city's regions like it does everywhere else. Reading the field directly
   * makes this false for such an agent whatever that accessor says, and the consequence is not
   * local: {@code Dijkstra.regionCondition()} gates the region subgraph on this call, so a leg whose
   * endpoints share a region is routed over the whole city instead of being confined to the region.
   * Confinement is what makes a region-based route differ from the shortest path, so without it
   * region-based navigation returns the shortest path and looks like a model that simply has no
   * effect.
   *
   * @param regionID the region to test
   * @return whether the agent can route within that region
   */
  public boolean isRegionKnown(Integer regionID) {
    return getAgentKnownRegions().contains(regionID);
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

  /**
   * How legible a space has to be before the agent stops looking for an on-route mark in it.
   *
   * <p>{@code LandmarkNavigation} inserts sub-goals while the space's wayfinding easiness is
   * <i>below</i> this, so <b>it must be positive</b>: easiness is never negative, and a threshold of
   * 0 ends the loop before its first iteration. Local-landmark navigation then produces no on-route
   * marks and every model built on it returns the plain minimisation route, leaving the
   * distant-landmark weight inside {@code Dijkstra} as the only landmark term in the model.
   *
   * <p>The learning module overrides this and derives the threshold from the agent's spatial
   * ability.
   *
   * @param regionBased whether the space being judged is a region leg rather than the whole trip
   * @return the easiness above which no further on-route mark is sought
   */
  public double getWayfindingEasinessThreshold(boolean regionBased) {
    return regionBased
        ? RouteChoicePars.wayfindingEasinessThresholdRegionsCommunity
        : RouteChoicePars.wayfindingEasinessThresholdCommunity;
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
