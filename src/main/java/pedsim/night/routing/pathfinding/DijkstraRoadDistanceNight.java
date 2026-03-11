package pedsim.night.routing.pathfinding;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import org.locationtech.jts.planargraph.DirectedEdge;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.routing.pathfinding.DijkstraRoadDistance;
import pedsim.core.utilities.StringEnum.Vulnerable;
import sim.graph.EdgeGraph;
import sim.graph.NodeGraph;
import sim.util.geo.Utilities;

/**
 * The class allows computing the road distance shortest route by employing the Dijkstra
 * shortest-path algorithm on a primal graph representation of the street network.
 *
 * It furthermore supports combined navigation strategies based on landmark and urban subdivisions
 * (regions, barriers).
 **/
public class DijkstraRoadDistanceNight extends DijkstraRoadDistance {

  public Set<NodeGraph> disregardedNodes = new HashSet<>();
  protected boolean secondAttempt;

  /**
   * Finds the minimum distances for adjacent nodes of the given current node in the primal graph.
   *
   * @param currentNode The current node in the primal graph for which to find adjacent nodes.
   */
  @Override
  protected void findMinDistances(NodeGraph currentNode) {

    List<NodeGraph> adjacentNodes = currentNode.getAdjacentNodes();
    List<NodeGraph> validNeighbors;

    if (!secondAttempt) {
      validNeighbors =
          adjacentNodes.stream().filter(targetNode -> canMoveToNodeAtNight(currentNode, targetNode))
              .collect(Collectors.toList());

      if (validNeighbors.isEmpty()) {
        // Flag currentNode as "dead-end"
        disregardedNodes.add(currentNode);
        return;
      }
    } else
      validNeighbors = adjacentNodes;

    for (

    NodeGraph targetNode : validNeighbors) {
      EdgeGraph commonEdge = agentNetwork.getEdgeBetween(currentNode, targetNode);
      DirectedEdge outEdge = agentNetwork.getDirectedEdgeBetween(currentNode, targetNode);
      tentativeCost = 0.0;

      // TODO - CHECK: at night basic error - no barrier effect;
      double error = Utilities.fromDistribution(1.0, 0.10, null);
      double edgeCost = commonEdge.getLength() * error;
      computeTentativeCost(currentNode, targetNode, edgeCost);
      isBest(currentNode, targetNode, outEdge);
    }
  }

  /**
   * Determines whether a pedestrian agent should avoid a specific edge during the night, based on
   * the edge's characteristics and the agent's cognitive map.
   * 
   * This method checks if the edge belongs to a region that is either unknown to the agent (i.e.,
   * not in its cognitive map or the community's cognitive map) or is located within a park or near
   * water. If either condition is true, the method returns true, indicating that the agent should
   * avoid this edge at night. If the edge leads directly to the agent's destination or if the
   * region is known (to the agent or the community), the agent may proceed, and the method returns
   * false.
   * 
   * The second attempt parameter relaxes the avoidance criteria. If the first attempt fails to find
   * a suitable edge, setting `secondAttempt` to true allows the agent to consider edges that it
   * would otherwise avoid, such as edges in unknown regions or near parks/water.
   *
   * @param edge The edge to evaluate for avoidance.
   * @param secondAttempt If true, the method applies a more lenient check on the edge
   *        characteristics (e.g., it may allow edges in unknown regions or near parks/water). *
   * @return true if the agent should avoid the edge at night; false otherwise.
   */
  protected boolean shouldAvoidEdgeAtNight(EdgeGraph edge, boolean secondAttempt) {

    // Avoid if the edge leads to the destination
    if (edge.getNodes().contains(destinationNode))
      return false;
    boolean isRegionKnown = isRegionKnown(edge.getRegionID());

    // If the edge is in a park/water or if the region is unknown and it's not the second attempt,
    // avoid it
    if (SharedCognitiveMap.getEdgesWithinParksOrAlongWater().contains(edge)
        || (!isRegionKnown && !secondAttempt))
      return true;

    return false;
  }

  private boolean canMoveToNodeAtNight(NodeGraph currentNode, NodeGraph targetNode) {

    EdgeGraph edge = agentNetwork.getEdgeBetween(currentNode, targetNode);
    return (agent.getAgentScenario() == Vulnerable.NON_VULNERABLE
        || !shouldAvoidEdgeAtNight(edge, secondAttempt)) && !disregardedNodes.contains(targetNode);
  }

  /**
   * Reconstructs the sequence of directed edges composing the path.
   *
   * @return An ArrayList of DirectedEdges representing the path sequence.
   */
  @Override
  protected List<DirectedEdge> reconstructSequence() {
    List<DirectedEdge> directedEdgesSequence = new ArrayList<>();
    NodeGraph step = destinationNode;

    // Check that the route has been formulated properly
    // No route
    if (nodeWrappersMap.get(destinationNode) == null || nodeWrappersMap.size() <= 1)
      directedEdgesSequence.clear();
    else
      while (nodeWrappersMap.get(step).nodeFrom != null) {
        DirectedEdge directedEdge;
        directedEdge = nodeWrappersMap.get(step).directedEdgeFrom;
        step = nodeWrappersMap.get(step).nodeFrom;
        directedEdgesSequence.add(0, directedEdge);
      }

    // TODO SHOULD NOT HAPPEN
    // If the sequence is empty, attempt the second approach (dijkstraAlgorithm)
    if (directedEdgesSequence.isEmpty()) {
      secondAttempt = true;
      directedEdgesSequence = dijkstraAlgorithm(originNode, destinationNode, agent);
      // If the second attempt also fails, use the fallback method the class with no avoidance;
      if (directedEdgesSequence.isEmpty())
        directedEdgesSequence =
            new DijkstraRoadDistance().dijkstraAlgorithm(originNode, destinationNode, agent);
    }
    return directedEdgesSequence;
  }
}
