package pedsim.core.routing.pathfinding;

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
import sim.routing.RoutingUtils;
import sim.util.geo.Utilities;

/**
 * The Dijkstra class provides functionality for performing Dijkstra's algorithm
 * and related calculations for route planning in the pedestrian simulation.
 */
public abstract class Dijkstra {

  protected NodeGraph originNode;
  protected NodeGraph destinationNode;
  protected NodeGraph previousJunction;
  protected NodeGraph finalDestinationNode;
  protected Set<NodeGraph> visitedNodes;
  protected PriorityQueue<Entry> unvisitedNodes;

  protected Set<NodeGraph> centroidsToAvoid = new HashSet<>();
  protected Set<DirectedEdge> directedEdgesToAvoid = new HashSet<>();
  protected Set<EdgeGraph> edgesToAvoid = new HashSet<>();
  protected Map<NodeGraph, NodeWrapper> nodeWrappersMap = new HashMap<>();
  protected AgentProperties properties;
  protected double tentativeCost;

  protected Graph agentNetwork;
  protected Graph agentDualNetwork;
  protected Agent agent;
  protected Route route = new Route();

  protected Set<EdgeGraph> knownEdges = new HashSet<>();
  protected Set<EdgeGraph> knownDualEdges = new HashSet<>();
  protected Set<NodeGraph> knownNodes = new HashSet<>();
  protected Set<NodeGraph> knownDualNodes = new HashSet<>();
  protected SubGraph subGraph = null;

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

    if (agent.getCognitiveMap().individualised) {
      knownNodes = agent.getCognitiveMap().getNodesInKnownNetwork();
      knownEdges = agent.getCognitiveMap().getEdgesInKnownNetwork();
    }
    if (segmentsToAvoid != null && !segmentsToAvoid.isEmpty()) {
      getEdgesToAvoid(segmentsToAvoid);
    }
    subGraphInitialisation();
  }

  /**
   * Initialises the Dijkstra algorithm for route calculation in a dual graph.
   *
   * @param centroidsToAvoid A set of centroids to avoid during route calculation.
   * @param previousJunction The previous junction node in the dual graph.
   */
  protected void initialiseDual(Set<NodeGraph> centroidsToAvoid, NodeGraph previousJunction) {

    if (agent.getCognitiveMap().individualised) {
      knownDualEdges = new HashSet<>(agent.getCognitiveMap().getEdgesInKnownDualNetwork());
      knownDualNodes = new HashSet<>(agent.getCognitiveMap().getNodesInKnownDualNetwork());
    }
    if (centroidsToAvoid != null && !centroidsToAvoid.isEmpty()) {
      this.centroidsToAvoid = new HashSet<>(centroidsToAvoid);
    }
    this.previousJunction = previousJunction;
    this.agentDualNetwork = SharedCognitiveMap.getCommunityDualNetwork();
    subGraphInitialisationDual();
  }

  /**
   * Initialises the subgraph for primal graph route calculation either between
   * the origin and the destination nodes or at the region level. Adjusts the set
   * of centroids to avoid if necessary.
   */
  protected void subGraphInitialisation() {
    if (regionCondition()) {
      subGraph = PedSimCity.regionsMap.get(originNode.getRegionID()).primalGraph;
      edgesToAvoid =
          (directedEdgesToAvoid.isEmpty())
              ? new HashSet<>(subGraph.getChildEdges(new ArrayList<>(edgesToAvoid)))
              : new HashSet<>();
      originNode = subGraph.findNode(originNode.getCoordinate());
      destinationNode = subGraph.findNode(destinationNode.getCoordinate());
      agentNetwork = subGraph;
    }
  }

  /**
   * Initialises the subgraph for dual graph route calculation either between the
   * origin and the destination nodes or at the region level. Adjusts the set of
   * centroids to avoid if necessary.
   */
  protected void subGraphInitialisationDual() {
    if (regionCondition()) {
      subGraph = PedSimCity.regionsMap.get(originNode.getRegionID()).dualGraph;
      centroidsToAvoid =
          (!centroidsToAvoid.isEmpty())
              ? new HashSet<>(subGraph.getChildNodes(new ArrayList<>(centroidsToAvoid)))
              : new HashSet<>();
      originNode = subGraph.findNode(originNode.getCoordinate());
      destinationNode = subGraph.findNode(destinationNode.getCoordinate());
      agentDualNetwork = subGraph;
    }
  }

  /**
   * Extracts edges to avoid from a set of directed edges.
   *
   * @param directedEdgesToAvoid A set of directed edges to avoid.
   */
  protected void getEdgesToAvoid(Set<DirectedEdge> directedEdgesToAvoid) {
    this.directedEdgesToAvoid = new HashSet<>(directedEdgesToAvoid);
    for (DirectedEdge edge : this.directedEdgesToAvoid) {
      edgesToAvoid.add((EdgeGraph) edge.getEdge());
    }
  }

  /**
   * Computes the cost perception error based on the role of barriers.
   *
   * @param targetNode The target node for cost calculation.
   * @param commonEdge The common edge used in cost calculation.
   * @param dual       Indicates whether it is a dual graph.
   * @return The computed cost perception error.
   */
  protected double costPerceptionError(NodeGraph targetNode, EdgeGraph commonEdge, boolean dual) {

    double error = Utilities.fromDistribution(1.0, 0.10, null);
    Set<Integer> knownBarriers = agent.getCognitiveMap().getAgentKnownBarriers();

    if (positiveBarrierEffect()) {
      // Defensive copy: getArray() returns the stored list by reference, so retainAll must
      // not be applied to it directly or it would permanently corrupt the edge's barrier data.
      List<Integer> pBarriers =
          new ArrayList<>(
              dual
                  ? targetNode.getPrimalEdge().attributes.get("positiveBarriers").getArray()
                  : commonEdge.attributes.get("positiveBarriers").getArray());
      pBarriers.retainAll(knownBarriers);
      if (!pBarriers.isEmpty()) {
        error =
            Utilities.fromDistribution(
                properties.getNaturalBarriersMean(), properties.getNaturalBarriersSD(), "left");
      }
    }
    if (negativeBarrierEffect()) {
      List<Integer> nBarriers =
          new ArrayList<>(
              dual
                  ? targetNode.getPrimalEdge().attributes.get("negativeBarriers").getArray()
                  : commonEdge.attributes.get("negativeBarriers").getArray());
      nBarriers.retainAll(knownBarriers);
      if (!nBarriers.isEmpty()) {
        error =
            Utilities.fromDistribution(
                properties.getSeveringBarriersMean(), properties.getSeveringBarriersSD(), "right");
      }
    }

    return error;
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
   * Computes the tentative cost for a dual currentNode and targetNode with the
   * specified turnCost.
   *
   * @param currentNode The current node.
   * @param targetNode  The target node.
   * @param turnCost    The cost associated with turning from the current to the
   *                    target node.
   */
  protected void computeTentativeCostDual(
      NodeGraph currentNode, NodeGraph targetNode, double turnCost) {
    if (turnCost > MAX_DEFLECTION_ANGLE) {
      turnCost = MAX_DEFLECTION_ANGLE;
    }

    if (turnCost < MIN_DEFLECTION_ANGLE) {
      turnCost = MIN_DEFLECTION_ANGLE;
    }

    if (landmarkCondition(targetNode)) {
      double globalLandmarkness =
          Landmarkness.globalLandmarknessDualNode(currentNode, targetNode, finalDestinationNode);
      double nodeLandmarkness =
          1.0 - globalLandmarkness * agent.getHeuristics().getGlobalLandmarkWeight(true);
      double nodeCost = nodeLandmarkness * turnCost;
      tentativeCost = getBest(currentNode) + nodeCost;
    } else {
      tentativeCost = getBest(currentNode) + turnCost;
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
   * Checks if the tentative cost is the best for the currentNode and targetNode
   * with the specified outEdge in a dual context.
   *
   * @param currentNode The current node.
   * @param targetNode  The target node.
   * @param outEdge     The directed edge from the current node to the target
   *                    node.
   */
  protected void isBestDual(NodeGraph currentNode, NodeGraph targetNode, DirectedEdge outEdge) {
    if (getBest(targetNode) > tentativeCost) {
      NodeWrapper nodeWrapper = nodeWrappersMap.computeIfAbsent(targetNode, NodeWrapper::new);
      nodeWrapper.nodeFrom = currentNode;
      nodeWrapper.directedEdgeFrom = outEdge;
      nodeWrapper.commonPrimalJunction = RoutingUtils.getPrimalJunction(currentNode, targetNode);
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
   * Determines whether there is a positive barrier effect based on the provided
   * list of barriers.
   *
   * @param pBarriers The list of positive barriers to check.
   * @return True if there is a positive barrier effect; otherwise, false.
   */
  private boolean positiveBarrierEffect() {
    return (!properties.shouldOnlyUseMinimization() && properties.isPreferenceNaturalBarriers());
  }

  /**
   * Determines whether there is a negative barrier effect based on the provided
   * list of barriers and the agent properties.
   *
   * @param nBarriers The list of negative barriers to check.
   * @return True if there is a negative barrier effect; otherwise, false.
   */
  private boolean negativeBarrierEffect() {
    return (!properties.shouldOnlyUseMinimization() && properties.isAversionSeveringBarriers());
  }

  /**
   * Checks if the landmark condition is met for the target node and the agent
   * properties.
   *
   * @param targetNode The node to check for the landmark condition.
   * @return True if the landmark condition is met; otherwise, false.
   */
  private boolean landmarkCondition(NodeGraph targetNode) {
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

  protected boolean isEdgeKnown(EdgeGraph commonEdge) {
    if (knownEdges == null) return false;
    if ((subGraph == null && !knownEdges.contains(commonEdge))
        || (subGraph != null && !knownEdges.contains(subGraph.getParentEdge(commonEdge)))) {
      return false;
    }
    return true;
  }

  protected boolean isDualEdgeKnown(EdgeGraph commonEdge) {
    if (knownDualEdges == null) return false;
    if ((subGraph == null && !knownDualEdges.contains(commonEdge))
        || (subGraph != null && !knownDualEdges.contains(subGraph.getParentEdge(commonEdge)))) {
      return false;
    }
    return true;
  }

  protected DirectedEdge retrieveFromPrimalParentGraph(NodeGraph step) {
    NodeGraph nodeTo = subGraph.getParentNode(step);
    NodeGraph nodeFrom = subGraph.getParentNode(nodeWrappersMap.get(step).nodeFrom);
    // retrieving from Primal Network (no SubGraph)
    return SharedCognitiveMap.getCommunityPrimalNetwork().getDirectedEdgeBetween(nodeFrom, nodeTo);
  }
}
