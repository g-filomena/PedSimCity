package pedsim.core.routing.pathfinding;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;
import org.locationtech.jts.planargraph.DirectedEdge;
import pedsim.core.agents.Agent;
import sim.graph.EdgeGraph;
import sim.graph.NodeGraph;
import sim.routing.NodeWrapper;
import sim.routing.RoutingUtils;

/**
 * The class allows computing the least cumulative angular change route by employing the Dijkstra
 * shortest-path algorithm on a dual graph representation of the street network.
 *
 * It furthermore supports combined navigation strategies based on landmark and urban subdivisions
 * (regions, barriers).
 **/
public class DijkstraAngularChange extends Dijkstra {

  /**
   * Performs the Dijkstra's algorithm to find the least cumulative angular change path from the
   * origin node to the destination node.
   *
   * @param originNode The starting node for the path.
   * @param destinationNode The destination node to reach.
   * @param finalDestinationNode The final destination node (primal graph) for the path, if
   *        different.
   * @param centroidsToAvoid A set of centroids (nodes representing segments) to avoid during the
   *        path calculation.
   * @param agent The agent for which the route is computed.
   *
   * @return An ArrayList of DirectedEdges representing the path.
   */
  public ArrayList<DirectedEdge> dijkstraAlgorithm(
      NodeGraph originNode,
      NodeGraph destinationNode,
      NodeGraph finalDestinationNode,
      Set<NodeGraph> centroidsToAvoid,
      NodeGraph previousJunction,
      Agent agent) {

    initialise(originNode, destinationNode, finalDestinationNode, agent);
    initialiseDual(centroidsToAvoid, previousJunction);
    runDijkstra();
    return reconstructSequence();
  }

  /**
   * Runs the Dijkstra algorithm to find the shortest path.
   *
   * <p>Uses the shared lazy-deletion queue (see {@link Dijkstra.Entry} and
   * {@link #pollFreshNode()}): each dual node is expanded exactly once at its finalised cost.
   */
  private void runDijkstra() {

    visitedNodes = new HashSet<>();
    initialiseQueue();

    // NodeWrapper = container for the metainformation about a Node
    NodeWrapper nodeWrapper = new NodeWrapper(this.originNode);
    nodeWrapper.gx = 0.0;
    if (previousJunction != null) {
      nodeWrapper.commonPrimalJunction = previousJunction;
    }
    nodeWrappersMap.put(this.originNode, nodeWrapper);

    // centroids to avoid are pre-marked as visited so they are never expanded
    if (this.centroidsToAvoid != null) {
      for (NodeGraph centroid : this.centroidsToAvoid) {
        visitedNodes.add(centroid);
      }
    }

    unvisitedNodes.add(new Entry(this.originNode, 0.0));

    NodeGraph currentNode;
    while ((currentNode = pollFreshNode()) != null) {
      // The destination's cost is final once it is polled; expanding the rest of the
      // network cannot change the reconstructed route.
      if (currentNode.equals(destinationNode)) {
        break;
      }
      findLeastAngularChange(currentNode);
    }
  }

  /**
   * Finds the least cumulative angular deviations for adjacent nodes of the given current node in
   * the dual graph.
   *
   *
   * @param currentNode The current node in the dual graph for which to find adjacent nodes.
   */
  private void findLeastAngularChange(NodeGraph currentNode) {

    NodeGraph currentJunction = nodeWrappersMap.get(currentNode).commonPrimalJunction;

    for (DirectedEdge outEdge : currentNode.getOutDirectedEdges()) {
      NodeGraph targetNode = (NodeGraph) outEdge.getToNode();
      if (visitedNodes.contains(targetNode)) {
        continue;
      }

      // Check if the current and the possible next centroid share in the primal graph
      // the same junction as the current with its previous centroid
      // --> if yes move on. This essentially means that the in the primal graph you
      // would go back to an
      // already traversed node; but the dual graph wouldn't know.
      NodeGraph primalJunction = RoutingUtils.getPrimalJunction(targetNode, currentNode);
      if (primalJunction != null && primalJunction.equals(currentJunction)) {
        continue;
      }

      EdgeGraph commonEdge = (EdgeGraph) outEdge.getEdge();
      // Known-network filtering only applies to individualised cognitive maps (mirrors the
      // primal variant): knownDualEdges is only populated when the map is individualised, so
      // testing it unconditionally would filter out every edge for community-map agents.
      if (agent.getCognitiveMap().individualised && !isDualEdgeKnown(commonEdge)) {
        continue;
      }

      // compute errors in perception of road coasts with stochastic variables
      double error = costPerceptionError(targetNode, commonEdge, true);
      double edgeCost = commonEdge.getDeflectionAngle() * error;
      computeTentativeCostDual(currentNode, targetNode, edgeCost);
      // the shared primal junction is symmetric, so the value resolved above is reused
      isBestDual(currentNode, targetNode, outEdge, primalJunction);
    }
  }

  /**
   * Reconstructs the sequence of directed edges composing the path.
   *
   * <p>Performance: each predecessor wrapper is fetched once per step, and edges are appended then
   * reversed once, avoiding the O(n^2) cost of repeated head insertions on an {@link ArrayList}. A
   * broken predecessor chain ends the walk; the returned order is origin to destination.
   *
   * @return An ArrayList of DirectedEdges representing the path sequence.
   */
  private ArrayList<DirectedEdge> reconstructSequence() {
    ArrayList<DirectedEdge> directedEdgesSequence = new ArrayList<>();

    // check that the route has been formulated properly
    if (nodeWrappersMap.get(destinationNode) == null || nodeWrappersMap.size() <= 1) {
      return directedEdgesSequence;
    }

    NodeGraph step = destinationNode;
    while (true) {
      NodeWrapper wrapper = nodeWrappersMap.get(step);
      if (wrapper == null || wrapper.nodeFrom == null) {
        break;
      }
      // the primal edge refers in any case to the parent primal graph
      directedEdgesSequence.add(step.getPrimalEdge().getDirEdge(0));
      step = wrapper.nodeFrom;

      if (step.equals(originNode)) {
        directedEdgesSequence.add(step.getPrimalEdge().getDirEdge(0));
        break;
      }
    }
    Collections.reverse(directedEdgesSequence);
    return directedEdgesSequence;
  }
}
