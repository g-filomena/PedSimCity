package pedsim.core.cognition.network;

import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;
import org.javatuples.Pair;
import org.locationtech.jts.planargraph.DirectedEdge;
import pedsim.core.cognition.cognitivemap.CognitiveMap;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.engine.PedSimCity;
import sim.graph.EdgeGraph;
import sim.graph.Graph;
import sim.graph.GraphUtils;
import sim.graph.Islands;
import sim.graph.NodeGraph;
import sim.routing.Astar;
import sim.routing.Route;

public class NetworkBuilder {

  private Set<EdgeGraph> necessaryEdges = new HashSet<>();
  private Set<NodeGraph> necessaryNodes = new HashSet<>();
  private Set<NodeGraph> necessaryDualNodes = new HashSet<>();
  private Set<EdgeGraph> necessaryDualEdges = new HashSet<>();

  // they include known nodes as well as nodes that are not known, but represented
  // in the CM
  CognitiveMap cognitiveMap;

  public NetworkBuilder(CognitiveMap cognitiveMap) {
    this.cognitiveMap = cognitiveMap;
  }

  public synchronized void buildKnownNetwork() {

    setNecessaryEdges(new HashSet<>(
        GraphUtils.getEdgesFromEdgeIDs(cognitiveMap.getAgentKnownEdges(), PedSimCity.edgesMap)));

    Graph graph = SharedCognitiveMap.getCommunityPrimalNetwork();
    Islands islands = new Islands(graph);
    if (islands.findDisconnectedIslands(getNecessaryEdges()).size() > 1) {
      setNecessaryEdges(islands.mergeConnectedIslands(getNecessaryEdges()));
    }
    setNecessaryNodes(GraphUtils.nodesFromEdges(getNecessaryEdges()));
    buildKNownDualNetwork();
  }

  private void buildKNownDualNetwork() {

    setNecessaryDualEdges(new HashSet<>());
    for (EdgeGraph edge : getNecessaryEdges()) {
      for (DirectedEdge directedEdge : edge.getDualNode().getOutEdges().getEdges()) {
        getNecessaryDualEdges().add((EdgeGraph) directedEdge.getEdge());
      }
    }

    Graph dualGraph = SharedCognitiveMap.getCommunityDualNetwork();
    Islands dualIslands = new Islands(dualGraph);
    if (dualIslands.findDisconnectedIslands(getNecessaryDualEdges()).size() > 1) {
      setNecessaryDualEdges(dualIslands.mergeConnectedIslands(getNecessaryDualEdges()));
    }
    setNecessaryDualNodes(GraphUtils.nodesFromEdges(getNecessaryDualEdges()));
    accommodateDualNetwork();
  }

  private void accommodateDualNetwork() {
    getNecessaryDualNodes().forEach(dualNode -> {
      EdgeGraph edge = dualNode.getPrimalEdge();
      getNecessaryEdges().add(edge);
      getNecessaryNodes().addAll(edge.getNodes());
    });
  }

  // known node always in community network
  public void addRouteToNetwork(NodeGraph knownNode, NodeGraph newNode) {

    EdgeGraph edgeBetween =
        SharedCognitiveMap.getCommunityPrimalNetwork().getEdgeBetween(knownNode, newNode);
    Set<EdgeGraph> newEdges = new HashSet<>();
    Route route = null;

    if (edgeBetween == null) {
      route = findMostKnownRoute(knownNode, newNode);
      newEdges.addAll(route.edgesSequence);
    } else {
      newEdges.add(edgeBetween);
    }

    Graph graph = SharedCognitiveMap.getCommunityPrimalNetwork();
    Islands islands = new Islands(graph);
    getNecessaryEdges().addAll(newEdges);
    getNecessaryEdges().addAll(newNode.getEdges());
    if (islands.findDisconnectedIslands(getNecessaryEdges()).size() > 1) {
      setNecessaryEdges(islands.mergeConnectedIslands(getNecessaryEdges()));
    }
    getNecessaryNodes().addAll(GraphUtils.nodesFromEdges(getNecessaryEdges()));
    // addRouteToDualNetwork(newEdges);
  }

  private void addRouteToDualNetwork(Set<EdgeGraph> newEdges) {

    for (EdgeGraph edge : newEdges) {
      for (DirectedEdge directedEdge : edge.getDualNode().getOutEdges().getEdges()) {
        getNecessaryDualEdges().add((EdgeGraph) directedEdge.getEdge());
      }
    }

    Graph dualGraph = SharedCognitiveMap.getCommunityDualNetwork();
    Islands dualIslands = new Islands(dualGraph);
    if (dualIslands.findDisconnectedIslands(getNecessaryDualEdges()).size() > 1) {
      setNecessaryDualEdges(dualIslands.mergeConnectedIslands(getNecessaryDualEdges()));
    }

    setNecessaryDualNodes(GraphUtils.nodesFromEdges(getNecessaryDualEdges()));
  }

  private Route findMostKnownRoute(NodeGraph originNode, NodeGraph destinationNode) {
    Pair<NodeGraph, NodeGraph> nodesPair = new Pair<>(originNode, destinationNode);
    Pair<NodeGraph, NodeGraph> reversePair = new Pair<>(destinationNode, originNode);

    // 1. Try to fetch from cache
    Route cached = getRouteFromNetworks(nodesPair, reversePair);
    if (cached != null)
      return cached;

    // 2. Build initial "avoid all" set
    List<Set<EdgeGraph>> edgeCategories = Arrays.asList(SharedCognitiveMap.tertiaryEdges,
        SharedCognitiveMap.neighbourhoodEdges, SharedCognitiveMap.unknownEdges);

    Set<Integer> edgesToAvoid =
        edgeCategories.stream().flatMap(cat -> GraphUtils.getEdgeIDs(cat).stream())
            .collect(Collectors.toCollection(LinkedHashSet::new));

    Graph communityNetwork = SharedCognitiveMap.getCommunityPrimalNetwork();
    Astar aStar = new Astar();

    // 3. Progressive relaxation loop
    for (int attempt = 0; attempt <= edgeCategories.size(); attempt++) {
      if (attempt > 0) {
        // allow one more category at each retry
        edgesToAvoid.removeAll(GraphUtils.getEdgeIDs(edgeCategories.get(attempt - 1)));
      }

      Route astarRoute =
          aStar.astarRoute(originNode, destinationNode, communityNetwork, edgesToAvoid);
      if (astarRoute != null) {
        cacheRoute(nodesPair, astarRoute, attempt == edgeCategories.size());
        return astarRoute;
      }
    }
    return null;
  }

  private Route getRouteFromNetworks(Pair<NodeGraph, NodeGraph> nodesPair,
      Pair<NodeGraph, NodeGraph> reversePair) {
    // Try normal routes first, then forced routes
    return SharedCognitiveMap.routesSubNetwork.getOrDefault(nodesPair,
        SharedCognitiveMap.routesSubNetwork.getOrDefault(reversePair,
            SharedCognitiveMap.forcedRoutesSubNetwork.getOrDefault(nodesPair,
                SharedCognitiveMap.forcedRoutesSubNetwork.get(reversePair))));
  }

  /**
   * Store route in appropriate cache.
   */
  private void cacheRoute(Pair<NodeGraph, NodeGraph> nodesPair, Route route, boolean forced) {
    Map<Pair<NodeGraph, NodeGraph>, Route> targetMap =
        forced ? SharedCognitiveMap.forcedRoutesSubNetwork : SharedCognitiveMap.routesSubNetwork;
    targetMap.put(nodesPair, route);
  }

  /**
   * @return the necessaryNodes
   */
  public Set<NodeGraph> getNecessaryNodes() {
    return necessaryNodes;
  }

  /**
   * @param necessaryNodes the necessaryNodes to set
   */
  public void setNecessaryNodes(Set<NodeGraph> necessaryNodes) {
    this.necessaryNodes = necessaryNodes;
  }

  /**
   * @return the necessaryEdges
   */
  public Set<EdgeGraph> getNecessaryEdges() {
    return necessaryEdges;
  }

  /**
   * @param necessaryEdges the necessaryEdges to set
   */
  public void setNecessaryEdges(Set<EdgeGraph> necessaryEdges) {
    this.necessaryEdges = necessaryEdges;
  }

  /**
   * @return the necessaryDualNodes
   */
  public Set<NodeGraph> getNecessaryDualNodes() {
    return necessaryDualNodes;
  }

  /**
   * @param necessaryDualNodes the necessaryDualNodes to set
   */
  public void setNecessaryDualNodes(Set<NodeGraph> necessaryDualNodes) {
    this.necessaryDualNodes = necessaryDualNodes;
  }

  /**
   * @return the necessaryDualEdges
   */
  public Set<EdgeGraph> getNecessaryDualEdges() {
    return necessaryDualEdges;
  }

  /**
   * @param necessaryDualEdges the necessaryDualEdges to set
   */
  public void setNecessaryDualEdges(Set<EdgeGraph> necessaryDualEdges) {
    this.necessaryDualEdges = necessaryDualEdges;
  }
}
