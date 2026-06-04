package pedsim.night.routing.pathfinding;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import org.locationtech.jts.planargraph.DirectedEdge;

import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.routing.pathfinding.DijkstraRoadDistance;
import sim.graph.EdgeGraph;
import sim.graph.NodeGraph;
import sim.util.geo.Utilities;

/**
 * Road-distance shortest path for night-time routing.
 */
public class DijkstraRoadDistanceNight extends DijkstraRoadDistance {

  public Set<NodeGraph> disregardedNodes = new HashSet<>();

  protected boolean secondAttempt;

  /**
   * Finds the minimum distances for adjacent nodes in the primal graph.
   *
   * @param currentNode the current node in the primal graph
   */
  @Override
  protected void findMinDistances(NodeGraph currentNode) {
    List<NodeGraph> adjacentNodes = currentNode.getAdjacentNodes();
    List<NodeGraph> validNeighbors;

    if (!secondAttempt) {
      validNeighbors =
          adjacentNodes.stream()
              .filter(targetNode -> canMoveToNodeAtNight(currentNode, targetNode))
              .collect(Collectors.toList());

      if (validNeighbors.isEmpty()) {
        disregardedNodes.add(currentNode);
        return;
      }
    } else {
      validNeighbors = adjacentNodes;
    }

    for (NodeGraph targetNode : validNeighbors) {
      EdgeGraph commonEdge = agentNetwork.getEdgeBetween(currentNode, targetNode);
      DirectedEdge outEdge = agentNetwork.getDirectedEdgeBetween(currentNode, targetNode);

      tentativeCost = 0.0;
      double error = Utilities.fromDistribution(1.0, 0.10, null);
      double edgeCost = commonEdge.getLength() * error;

      computeTentativeCost(currentNode, targetNode, edgeCost);
      isBest(currentNode, targetNode, outEdge);
    }
  }

  /**
   * Determines whether an edge should be avoided at night.
   *
   * @param edge the edge to evaluate
   * @param secondAttempt if true, relaxes the unknown-region avoidance criterion
   * @return true if the edge should be avoided at night
   */
  protected boolean shouldAvoidEdgeAtNight(EdgeGraph edge, boolean secondAttempt) {
    if (edge.getNodes().contains(destinationNode)) {
      return false;
    }

    boolean isRegionKnown = isRegionKnown(edge.getRegionID());

    return SharedCognitiveMap.getEdgesWithinParksOrAlongWater().contains(edge)
        || (!isRegionKnown && !secondAttempt);
  }

  private boolean canMoveToNodeAtNight(NodeGraph currentNode, NodeGraph targetNode) {
    EdgeGraph edge = agentNetwork.getEdgeBetween(currentNode, targetNode);

    return (!agent.isVulnerableBoolean() || !shouldAvoidEdgeAtNight(edge, secondAttempt))
        && !disregardedNodes.contains(targetNode);
  }

  /**
   * Reconstructs the sequence of directed edges composing the path.
   *
   * @return the reconstructed directed-edge sequence
   */
  @Override
  protected List<DirectedEdge> reconstructSequence() {
    List<DirectedEdge> directedEdgesSequence = new ArrayList<>();
    NodeGraph step = destinationNode;

    if (nodeWrappersMap.get(destinationNode) == null || nodeWrappersMap.size() <= 1) {
      directedEdgesSequence.clear();
    } else {
      while (nodeWrappersMap.get(step).nodeFrom != null) {
        DirectedEdge directedEdge = nodeWrappersMap.get(step).directedEdgeFrom;
        step = nodeWrappersMap.get(step).nodeFrom;
        directedEdgesSequence.add(0, directedEdge);
      }
    }

    if (directedEdgesSequence.isEmpty()) {
      if (!secondAttempt) {
        secondAttempt = true;
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
