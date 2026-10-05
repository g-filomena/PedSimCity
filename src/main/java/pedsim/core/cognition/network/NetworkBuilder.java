package pedsim.core.cognition.network;

import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.atomic.LongAdder;
import java.util.stream.Collectors;
import org.javatuples.Pair;
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

  /** Known networks built, and their streets before and after adding the visible side streets. */
  private static final LongAdder networksBuilt = new LongAdder();

  private static final LongAdder streetsWalked = new LongAdder();
  private static final LongAdder streetsWithSideStreets = new LongAdder();

  // they include known nodes as well as nodes that are not known, but represented
  // in the CM
  CognitiveMap cognitiveMap;

  public NetworkBuilder(CognitiveMap cognitiveMap) {
    this.cognitiveMap = cognitiveMap;
  }

  public synchronized void buildKnownNetwork() {

    // LinkedHashSet, for reproducibility rather than taste. Islands.findDisconnectedIslands and
    // mergeConnectedIslands walk this set to decide which islands to join and through which edges,
    // so its order changes the agent's known network and every route planned on it. Insertion order
    // is edge-id order, which is the same everywhere; a HashSet would order by EdgeGraph's hashCode
    // instead - deterministic since GeoMason-light 2.2.1 gave it one, but arbitrary, and dependent
    // on that version rather than on anything stated here.
    setNecessaryEdges(
        new LinkedHashSet<>(
            GraphUtils.getEdgesFromEdgeIDs(
                cognitiveMap.getAgentKnownEdges(), PedSimCity.edgesMap)));

    Graph graph = SharedCognitiveMap.getCommunityPrimalNetwork();
    Islands islands = new Islands(graph);
    if (islands.findDisconnectedIslands(getNecessaryEdges()).size() > 1) {
      setNecessaryEdges(islands.mergeConnectedIslands(getNecessaryEdges()));
    }
    addVisibleSideStreets();
    setNecessaryNodes(GraphUtils.nodesFromEdges(getNecessaryEdges()));
  }

  /**
   * Adds every street leaving a junction of a known street: whoever walks a street sees where each
   * side street begins.
   */
  private void addVisibleSideStreets() {
    Set<EdgeGraph> known = getNecessaryEdges();
    Set<EdgeGraph> withSideStreets = new LinkedHashSet<>(known);
    for (EdgeGraph edge : known) {
      withSideStreets.addAll(edge.getFromNode().getEdges());
      withSideStreets.addAll(edge.getToNode().getEdges());
    }
    networksBuilt.increment();
    streetsWalked.add(known.size());
    streetsWithSideStreets.add(withSideStreets.size());
    setNecessaryEdges(withSideStreets);
  }

  /**
   * How much the visible side streets widen known networks, over every one built in this run.
   *
   * @return e.g. {@code "173 known networks, side streets x1.53"}, or an empty string if none.
   */
  public static String sideStreetSummary() {
    long walked = streetsWalked.sum();
    if (walked == 0) {
      return "";
    }
    return String.format(
        "%d known networks, side streets x%.2f",
        networksBuilt.sum(), (double) streetsWithSideStreets.sum() / walked);
  }

  // known node always in community network
  public void addRouteToNetwork(NodeGraph knownNode, NodeGraph newNode) {

    // every street joining the two, where more than one does
    List<EdgeGraph> edgesBetween =
        SharedCognitiveMap.getCommunityPrimalNetwork().getEdgesBetween(knownNode, newNode);
    Set<EdgeGraph> newEdges = new HashSet<>();
    Route route = null;

    if (edgesBetween.isEmpty()) {
      route = findMostKnownRoute(knownNode, newNode);
      newEdges.addAll(route.edgesSequence);
    } else {
      newEdges.addAll(edgesBetween);
    }

    Graph graph = SharedCognitiveMap.getCommunityPrimalNetwork();
    Islands islands = new Islands(graph);
    getNecessaryEdges().addAll(newEdges);
    getNecessaryEdges().addAll(newNode.getEdges());
    if (islands.findDisconnectedIslands(getNecessaryEdges()).size() > 1) {
      setNecessaryEdges(islands.mergeConnectedIslands(getNecessaryEdges()));
    }
    getNecessaryNodes().addAll(GraphUtils.nodesFromEdges(getNecessaryEdges()));
  }

  private Route findMostKnownRoute(NodeGraph originNode, NodeGraph destinationNode) {
    Pair<NodeGraph, NodeGraph> nodesPair = new Pair<>(originNode, destinationNode);
    Pair<NodeGraph, NodeGraph> reversePair = new Pair<>(destinationNode, originNode);

    // 1. Try to fetch from cache
    Route cached = getRouteFromNetworks(nodesPair, reversePair);
    if (cached != null) return cached;

    // 2. Build initial "avoid all" set
    List<Set<EdgeGraph>> edgeCategories =
        Arrays.asList(
            SharedCognitiveMap.tertiaryEdges,
            SharedCognitiveMap.neighbourhoodEdges,
            SharedCognitiveMap.unknownEdges);

    Set<Integer> edgesToAvoid =
        edgeCategories.stream()
            .flatMap(cat -> GraphUtils.getEdgeIDs(cat).stream())
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

  private Route getRouteFromNetworks(
      Pair<NodeGraph, NodeGraph> nodesPair, Pair<NodeGraph, NodeGraph> reversePair) {
    // Try normal routes first, then forced routes
    return SharedCognitiveMap.routesSubNetwork.getOrDefault(
        nodesPair,
        SharedCognitiveMap.routesSubNetwork.getOrDefault(
            reversePair,
            SharedCognitiveMap.forcedRoutesSubNetwork.getOrDefault(
                nodesPair, SharedCognitiveMap.forcedRoutesSubNetwork.get(reversePair))));
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
}
