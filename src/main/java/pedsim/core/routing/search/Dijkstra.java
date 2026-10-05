package pedsim.core.routing.search;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;
import org.locationtech.jts.planargraph.DirectedEdge;
import pedsim.core.agents.Agent;
import pedsim.core.agents.AgentProperties;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.cognition.metrics.Landmarkness;
import pedsim.core.engine.PedSimCity;
import pedsim.core.parameters.RouteChoicePars;
import sim.graph.EdgeGraph;
import sim.graph.Graph;
import sim.graph.GraphUtils;
import sim.graph.NodeGraph;
import sim.graph.SubGraph;
import sim.routing.NodeWrapper;
import sim.routing.Route;

/**
 * The Dijkstra class provides functionality for performing Dijkstra's algorithm
 * and related calculations for route planning in the pedestrian simulation.
 */
public abstract class Dijkstra {

  protected NodeGraph originNode;
  protected NodeGraph destinationNode;
  protected NodeGraph finalDestinationNode;
  protected Set<NodeGraph> visitedNodes;
  protected PriorityQueue<Entry> unvisitedNodes;

  /**
   * The street segments the search must not use, in either direction.
   *
   * <p><b>Avoidance is direction-insensitive, and that is a modelling fact rather than an
   * implementation detail.</b> Callers hand in {@code DirectedEdge}s - JTS half-edges, which carry a
   * direction - and {@link #collectEdgesToAvoid} keeps only {@code directedEdge.getEdge()}, the
   * undirected {@code EdgeGraph} beneath. Ask to avoid A→B and B→A goes with it.
   *
   * <p>This is intended. The sequence routers build the set from {@code completeSequence}, the
   * traversals as actually taken, and each sub-route advances towards the destination. Having walked
   * A→B on the way there, the agent does not then walk B→A, so forbidding the reverse costs nothing
   * it would have used. Collapsing to the undirected segment is stricter than the
   * {@code Set<DirectedEdge>} signature suggests, and the signature is the misleading half.
   *
   * <p>There is deliberately no directed counterpart to this field: direction-specific avoidance has
   * to be built rather than switched on, and a second similarly-named field beside this one is what
   * let {@code subGraphInitialisation} test the wrong one. {@code PathFinder} has a field of that
   * name, which is the caller's, built from a route sequence and handed in.
   */
  protected Set<EdgeGraph> edgesToAvoid = new HashSet<>();

  protected Map<NodeGraph, NodeWrapper> nodeWrappersMap = new HashMap<>();
  protected AgentProperties properties;
  protected double tentativeCost;

  protected Graph agentNetwork;
  protected Agent agent;
  protected Route route = new Route();

  protected Set<EdgeGraph> knownEdges = new HashSet<>();
  protected Set<NodeGraph> knownNodes = new HashSet<>();
  protected SubGraph subGraph = null;

  /**
   * Set to search the whole network, ignoring what the agent knows.
   *
   * <p>An individualised agent that cannot reach its destination through the streets it knows does
   * not stay at home; it walks streets it has never walked. The escalation is the caller's to make
   * - see {@code RoadDistancePathFinder} and {@code AngularChangePathFinder} - and is counted on
   * the day trace, because a route found this way is one the agent could not have planned from its
   * own knowledge.
   *
   * <p>What is not represented: the agent plans against the length of a route through streets it
   * has never seen, so that length should carry a far larger error than a route through known
   * streets does. The model applies the same (small) perception error to both. Sizing that error
   * needs a source.
   */
  protected boolean ignoreKnownNetwork = false;

  /** Searches the whole network on this run, whatever the agent knows. */
  public void ignoreKnownNetwork() {
    ignoreKnownNetwork = true;
  }

  /**
   * Whether this search is confined to the agent's known network.
   *
   * <p>Only individualised cognitive maps carry one: {@code knownNodes} and friends are populated
   * only then, so testing them unconditionally would filter out every neighbour for a community-map
   * agent.
   */
  protected boolean restrictToKnownNetwork() {
    return !ignoreKnownNetwork
        && agent != null
        && agent.getCognitiveMap() != null
        && agent.getCognitiveMap().individualised;
  }

  protected static final double MAX_DEFLECTION_ANGLE = 180.00;
  protected static final double MIN_DEFLECTION_ANGLE = 0;

  /**
   * Immutable priority-queue entry pairing a node with the cost snapshot it was enqueued with.
   *
   * <p>This enables proper lazy deletion: instead of ordering the queue with a comparator that
   * reads the live, mutable {@code gx} (which can break the heap invariant after a relaxation),
   * each entry carries a frozen cost. Entries whose snapshot is no longer the node's best cost,
   * or whose node is already finalised, are simply discarded on poll.
   */
  protected static final class Entry {
    final NodeGraph node;
    final double cost;

    Entry(NodeGraph node, double cost) {
      this.node = node;
      this.cost = cost;
    }
  }

  /**
   * Creates a fresh priority queue ordered by each entry's frozen cost snapshot.
   */
  protected void initialiseQueue() {
    unvisitedNodes = new PriorityQueue<>(Comparator.comparingDouble(entry -> entry.cost));
  }

  /**
   * Polls the next node that still needs to be expanded, discarding stale entries.
   *
   * <p>An entry is stale if a cheaper relaxation has since lowered the node's best cost
   * ({@code entry.cost > getBest(node)}) or if the node has already been finalised. Each returned
   * node is therefore expanded exactly once, with the cost it was actually finalised at.
   *
   * @return the next node to expand, or {@code null} if the queue is exhausted
   */
  protected NodeGraph pollFreshNode() {
    while (!unvisitedNodes.isEmpty()) {
      Entry entry = unvisitedNodes.poll();
      if (entry.cost > getBest(entry.node) || !visitedNodes.add(entry.node)) {
        continue;
      }
      return entry.node;
    }
    return null;
  }

  protected void initialise(
      NodeGraph originNode,
      NodeGraph destinationNode,
      NodeGraph finalDestinationNode,
      Agent agent) {

    nodeWrappersMap.clear();
    this.agentNetwork = SharedCognitiveMap.getCommunityPrimalNetwork();
    this.agent = agent;
    this.properties = agent.getProperties();
    this.originNode = originNode;
    this.destinationNode = destinationNode;
    this.finalDestinationNode = finalDestinationNode;
  }

  /**
   * Initialises the Dijkstra algorithm for route calculation in a primal graph.
   *
   * @param segmentsToAvoid A set of directed edges to avoid during route
   */
  protected void initialisePrimal(Set<DirectedEdge> segmentsToAvoid) {

    initialiseKnownNetwork();
    if (segmentsToAvoid != null && !segmentsToAvoid.isEmpty()) {
      collectEdgesToAvoid(segmentsToAvoid);
    }
    subGraphInitialisation();
  }

  /**
   * Loads the primal known-network sets that {@link #isNodeKnown} and {@link #isEdgeKnown} read.
   *
   * <p>Separate from {@link #initialisePrimal} because a primal search can begin without one: the
   * three-argument {@code dijkstraAlgorithm} deliberately skips the region subgraph and the
   * avoid-set, and must still load this. <b>An individualised agent whose search never loads it
   * meets {@code restrictToKnownNetwork()} true with {@code knownEdges} empty, which rejects every
   * neighbour and yields no route at all</b> - a harder failure than the unrestricted search it is
   * meant to fall back to. Every entry point therefore calls it.
   */
  protected void initialiseKnownNetwork() {
    if (restrictToKnownNetwork()) {
      knownNodes = agent.getCognitiveMap().getNodesInKnownNetwork();
      knownEdges = agent.getCognitiveMap().getEdgesInKnownNetwork();
    }
  }

  /**
   * Confines the search to the region's subgraph when origin and destination are in one region the
   * agent knows, mapping the edges to avoid onto the subgraph's own edges.
   */
  protected void subGraphInitialisation() {
    if (regionCondition()) {
      subGraph = PedSimCity.regionsMap.get(originNode.getRegionID()).primalGraph;
      // The search tests subgraph edges, and EdgeGraph has no value equality: a parent edge never
      // matches its own child.
      edgesToAvoid =
          (!edgesToAvoid.isEmpty())
              ? new HashSet<>(subGraph.getChildEdges(new ArrayList<>(edgesToAvoid)))
              : new HashSet<>();
      originNode = subGraph.findNode(originNode.getCoordinate());
      destinationNode = subGraph.findNode(destinationNode.getCoordinate());
      agentNetwork = subGraph;
    }
  }

  /**
   * Records the undirected edges behind a set of directed ones, as the set the search consults.
   *
   * @param directedEdges The directed edges to avoid.
   */
  protected void collectEdgesToAvoid(Set<DirectedEdge> directedEdges) {
    for (DirectedEdge directedEdge : directedEdges) {
      edgesToAvoid.add((EdgeGraph) directedEdge.getEdge());
    }
  }

  /**
   * Computes the cost perception error based on the role of barriers.
   *
   * <p>Performance: this runs once per neighbour of every expanded node, so it draws exactly one
   * random value (a severing-barrier draw wins over a natural-barrier one, matching the original
   * overwrite order) and tests barrier membership in place instead of copying the edge's barrier
   * lists and intersecting them with retainAll.
   *
   * @param commonEdge The street whose cost is perceived.
   * @return The computed cost perception error.
   */
  protected double costPerceptionError(EdgeGraph commonEdge) {

    if (!properties.shouldOnlyUseMinimization()) {
      Set<Integer> knownBarriers = agent.getCognitiveMap().getAgentKnownBarriers();

      if (properties.isAversionSeveringBarriers()
          && anyBarrierKnown(commonEdge, "negativeBarriers", knownBarriers)) {
        return drawFromDistribution(
            properties.getSeveringBarriersMean(), properties.getSeveringBarriersSD(), "right");
      }
      if (properties.isPreferenceNaturalBarriers()
          && anyBarrierKnown(commonEdge, "positiveBarriers", knownBarriers)) {
        return drawFromDistribution(
            properties.getNaturalBarriersMean(), properties.getNaturalBarriersSD(), "left");
      }
    }
    return drawFromDistribution(1.0, RouteChoicePars.perceptionErrorSD, null);
  }

  /**
   * Whether any of the edge's barriers under the given attribute is known to the agent. Reads the
   * stored list in place; no copies, no mutation.
   */
  private static boolean anyBarrierKnown(
      EdgeGraph edge, String attribute, Set<Integer> knownBarriers) {
    List<Integer> barriers = edge.attributes.get(attribute).getArray();
    for (Integer barrier : barriers) {
      if (knownBarriers.contains(barrier)) {
        return true;
      }
    }
    return false;
  }

  /**
   * Truncated-Gaussian draw with the same semantics as GeoMason's
   * {@code Utilities.fromDistribution}, but sourced from the job's own seeded MASON RNG. The
   * library helper funnels every draw through one shared static generator, which
   * both serialises parallel jobs on a single atomic seed and escapes per-job seeding.
   */
  protected double drawFromDistribution(double mean, double sd, String direction) {
    double value = agent.getRandom().nextGaussian() * sd + mean;
    if (("left".equals(direction) && value > mean) || ("right".equals(direction) && value < mean)) {
      value = mean;
    }
    return value <= 0 ? mean : value;
  }

  /**
   * Computes the tentative cost for a given currentNode and targetNode with the
   * specified edgeCost.
   *
   * @param currentNode The current node.
   * @param targetNode  The target node.
   * @param edgeCost    The cost of the edge between the current and target nodes.
   */
  protected void computeTentativeCost(
      NodeGraph currentNode, NodeGraph targetNode, double edgeCost) {
    if (landmarkCondition(targetNode)) {
      double globalLandmarkness =
          Landmarkness.globalLandmarknessNode(targetNode, finalDestinationNode);
      double nodeLandmarkness =
          1.0 - globalLandmarkness * agent.getHeuristics().getGlobalLandmarkWeight(false);
      double nodeCost = edgeCost * nodeLandmarkness;
      tentativeCost = getBest(currentNode) + nodeCost;
    } else {
      tentativeCost = getBest(currentNode) + edgeCost;
    }
  }

  /**
   * Checks if the tentative cost is the best for the currentNode and targetNode
   * with the specified outEdge.
   *
   * @param currentNode The current node.
   * @param targetNode  The target node.
   * @param outEdge     The directed edge from the current node to the target
   *                    node.
   */
  protected void isBest(NodeGraph currentNode, NodeGraph targetNode, DirectedEdge outEdge) {
    if (getBest(targetNode) > tentativeCost) {
      NodeWrapper nodeWrapper = nodeWrappersMap.computeIfAbsent(targetNode, NodeWrapper::new);
      nodeWrapper.nodeFrom = currentNode;
      nodeWrapper.directedEdgeFrom = outEdge;
      nodeWrapper.gx = tentativeCost;
      unvisitedNodes.add(new Entry(targetNode, tentativeCost));
    }
  }

  /**
   * Retrieves the best value for the specified targetNode from the
   * nodeWrappersMap.
   *
   * @param targetNode The target node.
   * @return The best value for the target node.
   */
  protected double getBest(NodeGraph targetNode) {
    NodeWrapper nodeWrapper = nodeWrappersMap.get(targetNode);
    return nodeWrapper != null ? nodeWrapper.gx : Double.MAX_VALUE;
  }

  /**
   * Checks if the landmark condition is met for the target node and the agent
   * properties.
   *
   * @param targetNode The node to check for the landmark condition.
   * @return True if the landmark condition is met; otherwise, false.
   */
  protected boolean landmarkCondition(NodeGraph targetNode) {
    return (!properties.shouldOnlyUseMinimization()
        && properties.isUsingDistantLandmarks()
        && GraphUtils.nodesDistance(targetNode, finalDestinationNode)
            > RouteChoicePars.threshold3dVisibility);
  }

  /**
   * Checks if the region-based navigation condition is met and the agent
   * properties.
   *
   * @return True if the region-based navigation condition is met; otherwise,
   *         false.
   */
  protected boolean regionCondition() {
    return properties.isRegionBasedNavigation()
        && originNode.getRegionID() == destinationNode.getRegionID()
        && agent.getCognitiveMap().isRegionKnown(originNode.getRegionID());
  }

  protected boolean isRegionKnown(int regionID) {
    return agent.getCognitiveMap().isRegionKnown(regionID)
        || SharedCognitiveMap.isRegionKnownByCommunity(regionID);
  }

  /**
   * Whether the agent knows this node, mapping it back to the parent graph first when the search is
   * running inside a region subgraph.
   *
   * <p>`SubGraph` gives a child node the parent's `nodeID`,
   * coordinate and attributes, so the two look identical in a debugger, but they are distinct
   * objects and `NodeGraph` has no value equality - so `knownNodes.contains(childNode)` is false for
   * every node the agent knows perfectly well.
   */
  protected boolean isNodeKnown(NodeGraph node) {
    if (knownNodes == null) {
      return false;
    }
    return knownNodes.contains(subGraph == null ? node : subGraph.getParentNode(node));
  }

  protected boolean isEdgeKnown(EdgeGraph commonEdge) {
    if (knownEdges == null) return false;
    if ((subGraph == null && !knownEdges.contains(commonEdge))
        || (subGraph != null && !knownEdges.contains(subGraph.getParentEdge(commonEdge)))) {
      return false;
    }
    return true;
  }

  protected DirectedEdge retrieveFromPrimalParentGraph(NodeGraph step) {
    NodeWrapper wrapper = nodeWrappersMap.get(step);
    NodeGraph nodeTo = subGraph.getParentNode(step);
    NodeGraph nodeFrom = subGraph.getParentNode(wrapper.nodeFrom);
    // The parent of the edge the search took. Looking the pair up in the primal network instead
    // answers the shortest of any parallel streets joining it, which need not be the one walked.
    EdgeGraph parentEdge = subGraph.getParentEdge((EdgeGraph) wrapper.directedEdgeFrom.getEdge());
    if (parentEdge == null) {
      return SharedCognitiveMap.getCommunityPrimalNetwork()
          .getDirectedEdgeBetween(nodeFrom, nodeTo);
    }
    DirectedEdge forward = parentEdge.getDirEdge(0);
    return forward.getFromNode().equals(nodeFrom) ? forward : parentEdge.getDirEdge(1);
  }
}
