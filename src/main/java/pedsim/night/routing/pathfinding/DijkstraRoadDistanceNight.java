package pedsim.night.routing.pathfinding;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.locationtech.jts.planargraph.DirectedEdge;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.routing.pathfinding.DijkstraRoadDistance;
import sim.graph.EdgeGraph;
import sim.graph.NodeGraph;

/**
 * Road-distance shortest path for night-time routing.
 *
 * <p>Resolution proceeds as a graded fallback so that constraints are relaxed only as far as
 * necessary:
 * <ol>
 *   <li><b>First attempt</b>: avoid parks/water <em>and</em> edges in unknown regions.
 *   <li><b>Second attempt</b>: relax the unknown-region constraint but keep avoiding parks/water.
 *   <li><b>Final fallback</b>: an unconstrained {@link DijkstraRoadDistance} shortest path.
 * </ol>
 *
 * <p>Both constrained attempts apply only to vulnerable agents; for non-vulnerable agents the
 * filter is a no-op and night avoidance is left to situated navigation.
 *
 * ## Possible to do - add agents filter out zero light edges but add look up table so when they
 * arrive at a node and the start of an edge is dark they reroute
 */
public class DijkstraRoadDistanceNight extends DijkstraRoadDistance {

  public Set<NodeGraph> disregardedNodes = new HashSet<>();
  protected boolean secondAttempt;

  /**
   * Finds the minimum distances for adjacent nodes in the primal graph.
   *
   * <p>The neighbour filter is applied on every attempt; the degree of relaxation is governed by
   * {@link #secondAttempt} via {@link #shouldAvoidEdgeAtNight(EdgeGraph, boolean)}.
   *
   * <p>Performance: neighbours are reached by iterating the node's own outgoing directed edges,
   * which yields the target node, the undirected edge and the directed edge in one object — no
   * {@code getEdgeBetween}/{@code getDirectedEdgeBetween} map lookups are needed. The cost noise
   * comes from the job's own seeded RNG (see {@code Dijkstra.drawFromDistribution}) instead of the
   * shared static Random in GeoMason.
   *
   * @param currentNode the current node in the primal graph
   */
  @Override
  protected void findMinDistances(NodeGraph currentNode) {
    boolean anyValidNeighbour = false;

    for (DirectedEdge outEdge : currentNode.getOutDirectedEdges()) {
      NodeGraph targetNode = (NodeGraph) outEdge.getToNode();
      EdgeGraph commonEdge = (EdgeGraph) outEdge.getEdge();
      if (!canMoveToNodeAtNight(targetNode, commonEdge)) {
        continue;
      }
      anyValidNeighbour = true;

      double error = drawFromDistribution(1.0, 0.10, null);
      double edgeCost = commonEdge.getLength() * error;
      computeTentativeCost(currentNode, targetNode, edgeCost);
      isBest(currentNode, targetNode, outEdge);
    }

    if (!anyValidNeighbour) {
      disregardedNodes.add(currentNode);
    }
  }

  /**
   * Determines whether an edge should be avoided at night.
   *
   * <p>The {@code secondAttempt} term is tested first so that, on the relaxed attempt, the
   * region-knowledge lookup is short-circuited away entirely.
   *
   * @param edge the edge to evaluate
   * @param secondAttempt if true, relaxes the unknown-region avoidance criterion (parks/water are
   *     still avoided)
   * @return true if the edge should be avoided at night
   */
  protected boolean shouldAvoidEdgeAtNight(EdgeGraph edge, boolean secondAttempt) {
    if (edge.getNodes().contains(destinationNode)) {
      return false;
    }
    return SharedCognitiveMap.getEdgesWithinParksOrAlongWater().contains(edge)
        || (!secondAttempt && !isRegionKnown(edge.getRegionID()));
  }

  /**
   * @param targetNode the candidate neighbour
   * @param edge the already-resolved edge between the current node and {@code targetNode}
   * @return true if the agent may move onto {@code targetNode} at night
   */
  private boolean canMoveToNodeAtNight(NodeGraph targetNode, EdgeGraph edge) {
    return (!agent.isVulnerableBoolean() || !shouldAvoidEdgeAtNight(edge, secondAttempt))
        && !disregardedNodes.contains(targetNode);
  }

  /**
   * Reconstructs the sequence of directed edges composing the path.
   *
   * <p>Performance: each predecessor wrapper is fetched once per step (instead of three map
   * lookups), and edges are appended then reversed once, avoiding the O(n^2) cost of repeated
   * head insertions on an {@link ArrayList}.
   *
   * @return the reconstructed directed-edge sequence
   */
  @Override
  protected List<DirectedEdge> reconstructSequence() {
    List<DirectedEdge> directedEdgesSequence = new ArrayList<>();
    NodeGraph step = destinationNode;

    if (nodeWrappersMap.get(destinationNode) != null && nodeWrappersMap.size() > 1) {
      while (true) {
        var wrapper = nodeWrappersMap.get(step);
        if (wrapper.nodeFrom == null) {
          break;
        }
        directedEdgesSequence.add(wrapper.directedEdgeFrom);
        step = wrapper.nodeFrom;
      }
      Collections.reverse(directedEdgesSequence);
    }

    if (directedEdgesSequence.isEmpty()) {
      if (!secondAttempt) {
        secondAttempt = true;
        // The relaxed attempt must not inherit dead-ends pruned under the stricter constraints.
        disregardedNodes.clear();
        directedEdgesSequence = dijkstraAlgorithm(originNode, destinationNode, agent);
      }
      if (directedEdgesSequence.isEmpty()) {
        directedEdgesSequence =
            new DijkstraRoadDistance().dijkstraAlgorithm(originNode, destinationNode, agent);
      }
    }
    return directedEdgesSequence;
  }
}
