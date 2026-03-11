package pedsim.core.cognition.cognitivemap;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Set;
import java.util.concurrent.ThreadLocalRandom;
import java.util.stream.Collectors;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.Polygon;
import pedsim.core.agents.Agent;
import pedsim.core.cognition.network.NetworkBuilder;
import pedsim.core.engine.PedSimCity;
import pedsim.core.parameters.LearningPars;
import pedsim.core.parameters.Pars;
import pedsim.core.parameters.RouteChoicePars;
import sim.graph.EdgeGraph;
import sim.graph.GraphUtils;
import sim.graph.Islands;
import sim.graph.NodeGraph;
import sim.routing.Astar;

/**
 * Represents an agent's cognitive map, which provides access to various map attributes. In this
 * version of PedSimCity, this is a simple structure designed for further developments.
 */
public class CognitiveMap extends SharedCognitiveMap {

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
    spatialAbility = Math.min(1.0, Math.max(0.0,
        LearningPars.MEAN_MEMORY_ROUTES + (ThreadLocalRandom.current().nextDouble() - 0.5) * 0.5));
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

    NodeGraph[] knownNodes = {agent.getHome(), agent.getWork()};

    Set<NodeGraph> activityBoneNodesTmp = new HashSet<>();
    Queue<NodeGraph> queue = new LinkedList<>();

    for (NodeGraph node : knownNodes) {
      activityBoneNodesTmp.add(node);
      queue.add(node);
      int region = node.getRegionID();
      agentKnownRegions.add(region);
      activityBoneNodesTmp.addAll(PedSimCity.regionsMap.get(region).nodes);
    }

    Map<NodeGraph, Double> distanceMap = new HashMap<>();
    distanceMap.put(agent.getHome(), 0.0);
    distanceMap.put(agent.getWork(), 0.0);

    // Step 1: Collect nearby nodes with cumulative distance tracking
    while (!queue.isEmpty()) {
      NodeGraph currentNode = queue.poll();
      double currentDistance = distanceMap.get(currentNode);

      for (EdgeGraph edge : currentNode.getEdges()) {
        NodeGraph neighborNode = edge.getOtherNode(currentNode);
        double newDistance = currentDistance + edge.getLength(); // Cumulative distance

        if (!activityBoneNodesTmp.contains(neighborNode) && newDistance <= Pars.homeWorkRadius) {
          activityBoneNodesTmp.add(neighborNode);
          queue.add(neighborNode);
          distanceMap.put(neighborNode, newDistance); // Store cumulative distance
        }
      }
    }

    // Step 2: Ensure path connectivity (shortest path between home and work)
    Astar astar = new Astar();
    List<NodeGraph> shortestPath = astar.astarRoute(agent.getHome(), agent.getWork(),
        SharedCognitiveMap.getCommunityPrimalNetwork(), null).nodesSequence;
    if (!shortestPath.isEmpty()) {
      activityBoneNodesTmp.addAll(shortestPath);
    }

    if (RouteChoicePars.cityCentreRegionsID.length > 0) {
      for (Integer regionID : RouteChoicePars.cityCentreRegionsID) {
        activityBoneNodesTmp.addAll(PedSimCity.regionsMap.get(regionID).nodes);
      }
    }

    for (NodeGraph node : activityBoneNodesTmp) {
      this.activityBoneNodes.add(node.getID());
      this.activityBoneEdges.addAll(GraphUtils.getEdgeIDs(node.getEdges()));
    }
  }

  /**
   * Builds the activity bone, which includes the agent's home and work nodes along with edges in
   * the known regions and from those nodes.
   */
  public void buildSimpleActivityBone() {

    NodeGraph[] knownNodes = {agent.getHome(), agent.getWork()};
    List<EdgeGraph> edges = new ArrayList<>();

    for (NodeGraph node : knownNodes) {
      int region = node.getRegionID();
      agentKnownRegions.add(region);
      edges.addAll(PedSimCity.regionsMap.get(region).edges);
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
      nodesInKnownSpace
          .forEach(node -> agentKnownEdges.addAll(GraphUtils.getEdgeIDs(node.getEdges())));
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
    List<Integer> potentiallyKnownRegions = new ArrayList<>();

    for (int nodeID : agentKnownNodes) {
      NodeGraph node = PedSimCity.nodesMap.get(nodeID);
      int regionID = node.getRegionID();
      if (agentKnownRegions.contains(regionID)) {
        continue;
      }
      potentiallyKnownRegions.add(regionID);
    }

    for (int regionID : potentiallyKnownRegions) {
      Set<EdgeGraph> regionEdges = new HashSet<>(getEdgesInKnownNetwork().stream()
          .filter(edge -> edge.getRegionID() == regionID).collect(Collectors.toList()));

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
    tmpNodes.stream().flatMap(node -> node.adjacentBuildings.stream()).filter(building -> {
      Double lScore = building.attributes.get("localLandmarkness").getDouble();
      return lScore != null && lScore > localLandmarkThreshold;

    }).map(building -> building.buildingID).forEach(agentKnownLocalLandmarks::add);
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
    return new HashSet<>(networkBuilder.getNecessaryNodes());
  }

  public Set<EdgeGraph> getEdgesInKnownNetwork() {
    return new HashSet<>(networkBuilder.getNecessaryEdges());
  }

  public Set<Integer> getNodeIDsInKnownNetwork() {
    return new HashSet<>(GraphUtils.getNodeIDs(networkBuilder.getNecessaryNodes()));
  }

  public Set<Integer> getEdgeIDsInKnownNetwork() {
    return new HashSet<>(GraphUtils.getEdgeIDs(networkBuilder.getNecessaryEdges()));
  }

  public boolean isEdgeKnown(EdgeGraph edgeGraph) {
    return agentKnownEdges.contains(edgeGraph.getID());
  }

  public boolean isRegionKnown(Integer regionID) {
    return agentKnownRegions.contains(regionID);
  }

  public Set<NodeGraph> getNodesInKnownDualNetwork() {
    return new HashSet<>(networkBuilder.getNecessaryDualNodes());
  }

  public Set<EdgeGraph> getEdgesInKnownDualNetwork() {
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
