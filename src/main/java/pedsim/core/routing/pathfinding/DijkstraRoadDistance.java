package pedsim.core.routing.pathfinding;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.locationtech.jts.planargraph.DirectedEdge;
import pedsim.core.agents.Agent;
import sim.graph.EdgeGraph;
import sim.graph.NodeGraph;
import sim.routing.NodeWrapper;

/**
 * The class allows computing the road distance shortest route by employing the Dijkstra
 * shortest-path algorithm on a primal graph representation of the street network.
 *
 * It furthermore supports combined navigation strategies based on landmark and urban subdivisions
 * (regions, barriers).
 **/
public class DijkstraRoadDistance extends Dijkstra {

  /**
   * Performs the Dijkstra's algorithm to find the shortest path from the origin node to the
   * destination node.
   *
   * This method calculates the shortest path in the network graph from the specified origin node to
   * the destination node while considering optional segments to avoid and agent properties.
   *
   * @param originNode The starting node for the path.
   * @param destinationNode The destination node to reach.
   * @param finalDestinationNode The final destination node (primal graph) for the path.
   * @param directedEdgesToAvoid A set of directed edges (segments) to avoid during the path calculation.
   * @param agent The agent for which the route is computed.
   *
   * @return An ArrayList of DirectedEdges representing the shortest path from the origin to the
   *         destination.
   */
  public List<DirectedEdge> dijkstraAlgorithm(
      NodeGraph originNode,
      NodeGraph destinationNode,
      NodeGraph finalDestinationNode,
      Set<DirectedEdge> directedEdgesToAvoid,
      Agent agent) {

    initialise(originNode, destinationNode, finalDestinationNode, agent);
    initialisePrimal(directedEdgesToAvoid);
    runDijkstra();

    return reconstructSequence();
  }

  public List<DirectedEdge> dijkstraAlgorithm(
      NodeGraph originNode, NodeGraph destinationNode, Agent agent) {

    initialise(originNode, destinationNode, destinationNode, agent);
    runDijkstra();
    return reconstructSequence();
  }

  /**
   * Runs the Dijkstra algorithm to find the shortest path.
   *
   * <p>Uses lazy deletion: each node is enqueued with a frozen cost snapshot (see
   * {@link Dijkstra.Entry}) and {@link #pollFreshNode()} discards stale entries, so every node is
   * expanded exactly once at its finalised cost. This removes the redundant re-expansions caused
   * by the previous live-comparator queue.
   */
  protected void runDijkstra() {

    visitedNodes = new HashSet<>();
    initialiseQueue();

    // NodeWrapper = container for the metainformation about a Node
    NodeWrapper nodeWrapper = new NodeWrapper(originNode);
    nodeWrapper.gx = 0.0;
    nodeWrappersMap.put(originNode, nodeWrapper);
    unvisitedNodes.add(new Entry(originNode, 0.0));

    NodeGraph currentNode;
    while ((currentNode = pollFreshNode()) != null) {
      // The destination's cost is final once it is polled; expanding the rest of the
      // network cannot change the reconstructed route.
      if (currentNode.equals(destinationNode)) {
        break;
      }
      findMinDistances(currentNode);
    }
  }

  /**
   * Finds the minimum distances for adjacent nodes of the given current node in the primal graph.
   *
   * <p>Performance: neighbours are reached by iterating the node's own outgoing directed edges,
   * which yields the target node, the undirected edge and the directed edge in one object — no
   * {@code getEdgeBetween}/{@code getDirectedEdgeBetween} map lookups (each of which allocates a
   * key pair) are needed.
   *
   * @param currentNode The current node in the primal graph for which to find adjacent nodes.
   */
  protected void findMinDistances(NodeGraph currentNode) {

    for (DirectedEdge outEdge : currentNode.getOutDirectedEdges()) {

      NodeGraph targetNode = (NodeGraph) outEdge.getToNode();
      if (visitedNodes.contains(targetNode)) {
        continue;
      }

      EdgeGraph commonEdge = (EdgeGraph) outEdge.getEdge();
      if (agent.getCognitiveMap().individualised && !isEdgeKnown(commonEdge)) {
        continue;
      }
      if (edgesToAvoid.contains(commonEdge)) {
        continue;
      }

      double error = costPerceptionError(targetNode, commonEdge, false);
      double edgeCost = commonEdge.getLength() * error;
      computeTentativeCost(currentNode, targetNode, edgeCost);

      if (getBest(targetNode) > tentativeCost) {
        isBest(currentNode, targetNode, outEdge);
      }
    }
  }

  /**
   * Reconstructs the sequence of directed edges composing the path.
   *
   * <p>Performance: each predecessor wrapper is fetched once per step (instead of repeatedly), and
   * edges are appended then reversed once, avoiding the O(n^2) cost of repeated head insertions on
   * an {@link ArrayList}. The returned order (origin to destination) is unchanged.
   *
   * @return An ArrayList of DirectedEdges representing the path sequence.
   */
  protected List<DirectedEdge> reconstructSequence() {
    List<DirectedEdge> directedEdgesSequence = new ArrayList<>();
    NodeGraph step = destinationNode;

    // Check that the route has been formulated properly (otherwise: no route).
    if (nodeWrappersMap.get(destinationNode) != null && nodeWrappersMap.size() > 1) {
      while (true) {
        NodeWrapper wrapper = nodeWrappersMap.get(step);
        if (wrapper.nodeFrom == null) {
          break;
        }
        DirectedEdge directedEdge =
            (subGraph != null) ? retrieveFromPrimalParentGraph(step) : wrapper.directedEdgeFrom;
        directedEdgesSequence.add(directedEdge);
        step = wrapper.nodeFrom;
      }
      Collections.reverse(directedEdgesSequence);
    }

    return directedEdgesSequence;
  }
}
