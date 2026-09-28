package pedsim.night.routing.search;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.locationtech.jts.planargraph.DirectedEdge;
import pedsim.core.routing.search.DijkstraRoadDistance;
import pedsim.night.agents.NightAgent;
import pedsim.night.engine.PedSimCityNight;
import pedsim.night.routing.NightRouteCost;
import sim.graph.EdgeGraph;
import sim.graph.NodeGraph;

/**
 * Least-cost path for night-time routing: road distance, with each edge's length raised by {@link
 * NightRouteCost} while it is dark.
 *
 * <p>Nothing is forbidden. Darkness, and parks or water after dark, make a street more expensive,
 * so a route goes round one only when the way round is worth it, and how far is worth it is the
 * cost function's to say. The same function prices the re-plan an agent makes on the way ({@code
 * NightAgentMovement}), so the plan and the reaction agree.
 *
 * <p>In daylight, and for a caller that is not a {@link NightAgent}, the cost is plain road distance
 * with perception error. The error is {@code costPerceptionError}'s, so {@code
 * --perceptionErrorSD=0} pins it here as everywhere.
 */
public class DijkstraRoadDistanceNight extends DijkstraRoadDistance {

  /**
   * Relaxes the edges out of a node at their night cost.
   *
   * <p>Neighbours are reached through the node's own outgoing directed edges, which gives the target
   * node, the edge and the directed edge in one object, with no map lookups. The parent's skips -
   * settled nodes, the known-network restriction and the avoid-set - are kept, so this cannot mean
   * something different from its parent on a call path where one of them applies.
   *
   * @param currentNode the current node in the primal graph
   */
  @Override
  protected void findMinDistances(NodeGraph currentNode) {
    for (DirectedEdge outEdge : currentNode.getOutDirectedEdges()) {
      NodeGraph targetNode = (NodeGraph) outEdge.getToNode();
      EdgeGraph commonEdge = (EdgeGraph) outEdge.getEdge();
      if (visitedNodes.contains(targetNode)
          || (restrictToKnownNetwork() && !isEdgeKnown(commonEdge))
          || edgesToAvoid.contains(commonEdge)) {
        continue;
      }
      double error = costPerceptionError(targetNode, commonEdge, false);
      double edgeCost = commonEdge.getLength() * error * nightFactor(commonEdge);
      computeTentativeCost(currentNode, targetNode, edgeCost);
      isBest(currentNode, targetNode, outEdge);
    }
  }

  /** {@link NightRouteCost#plannedFactor} while it is dark for a night agent, otherwise 1.0. */
  private double nightFactor(EdgeGraph edge) {
    if (agent instanceof NightAgent nightAgent
        && nightAgent.getState() instanceof PedSimCityNight night
        && night.isDark) {
      return NightRouteCost.plannedFactor(nightAgent, edge);
    }
    return 1.0;
  }

  /**
   * Reconstructs the sequence of directed edges composing the path, appending and reversing once
   * rather than inserting at the head. An empty result means the destination is not reachable from
   * the origin at all, which pricing cannot cause.
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
    return directedEdgesSequence;
  }
}
