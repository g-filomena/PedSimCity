package pedsim.core.routing.elements;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import pedsim.core.agents.Agent;
import pedsim.core.engine.PedSimCity;
import pedsim.core.parameters.RouteChoicePars;
import sim.graph.GraphUtils;
import sim.graph.NodeGraph;

/**
 * Series of functions that support landmark-based navigation, landmarkness computation,
 * identification of on-route marks and wayfinding easiness of a certain space.
 */
public class GlobalLandmarkNavigation extends LandmarkNavigation {

  public GlobalLandmarkNavigation(NodeGraph originNode, NodeGraph destinationNode, Agent agent) {

    this.originNode = originNode;
    this.destinationNode = destinationNode;
    this.agent = agent;
    idntifyAgentLocalLandmarks();
  }

  /**
   * Generates a sequence of intermediate nodes (on-route marks) between the origin and destination
   * nodes on the basis of local landmarkness.
   *
   * @return An ArrayList of on-route marks, including the origin and destination nodes.
   */
  public List<NodeGraph> onRouteMarks() {

    sequence = new ArrayList<>();
    findSalientJunctions(originNode);

    if (salientNodes.isEmpty())
      return sequence;

    // compute wayfinding easiness and the resulting research space
    double wayfindingEasiness = complexity.wayfindingEasiness(originNode, destinationNode, agent);
    double searchDistance =
        GraphUtils.nodesDistance(originNode, destinationNode) * wayfindingEasiness;
    currentNode = originNode;

    // while the wayfindingEasiness is lower than the threshold the agent looks for
    // intermediate-points.
    while (wayfindingEasiness < agent.getCognitiveMap().getWayfindingEasinessThreshold(false)) {
      NodeGraph bestNode = findOnRouteMark(salientNodes, searchDistance);
      if (bestNode == null || bestNode.equals(currentNode)) {
        break;
      }
      getOnRouteMarks().add(bestNode);
      findSalientJunctions(bestNode);
      if (salientNodes.isEmpty()) {
        return sequence;
      }

      wayfindingEasiness = complexity.wayfindingEasiness(bestNode, destinationNode, agent);
      searchDistance = GraphUtils.nodesDistance(bestNode, destinationNode) * wayfindingEasiness;
      currentNode = bestNode;
      bestNode = null;
    }
    sequence = new ArrayList<>(getOnRouteMarks());
    sequence.add(0, originNode);
    sequence.add(destinationNode);
    return sequence;
  }

  /**
   * Identifies salient junctions within the network space between a given node and the destination
   * node. This method finds salient nodes based on their salience within the specified network
   * space.
   *
   * @param node The node from which to identify salient junctions, in the space with the
   *        destination node.
   */
  public void findSalientJunctions(NodeGraph node) {

    double percentile = RouteChoicePars.salientNodesPercentile;
    salientNodes =
        new HashMap<>(network.getSalientNodesWithinSpace(node, destinationNode, percentile));

    salientNodes.keySet().retainAll(GraphUtils
        .getNodesFromNodeIDs(agent.getCognitiveMap().getAgentKnownNodes(), PedSimCity.nodesMap));

    // If no salient junctions are found, the tolerance increases till the 0.50
    // percentile;
    // if still no salient junctions are found, the agent continues without
    // landmarks
    while (salientNodes.isEmpty()) {
      percentile -= 0.05;
      if (percentile < 0.50) {
        sequence.add(0, originNode);
        sequence.add(destinationNode);
        break;
      }
      salientNodes =
          new HashMap<>(network.getSalientNodesWithinSpace(node, destinationNode, percentile));
    }
  }

  /**
   * Finds the most salient on-route mark between the current node and the destination node, amongst
   * the salient nodes (junctions).
   *
   * @param currentNode The current node in the navigation.
   * @param salientNodes A map of salient junctions and their centrality scores along the route.
   * @param searchDistance The search distance limit from the currentNode for evaluating potential
   *        nodes.
   * @return The selected node that serves as an on-route mark, or null if none is found.
   */
  private NodeGraph findOnRouteMark(Map<NodeGraph, Double> salientNodes, Double searchDistance) {

    List<NodeGraph> junctions = new ArrayList<>(salientNodes.keySet());
    List<NodeGraph> sortedJunctions =
        junctions.stream().filter(candidateNode -> checkCriteria(candidateNode, searchDistance))
            .sorted(Comparator.comparingDouble(
                candidateNode -> calculateScore(candidateNode, destinationNode, false)))
            .collect(Collectors.toList());

    return sortedJunctions.isEmpty() ? null : sortedJunctions.get(sortedJunctions.size() - 1);
  }
  //
  // List<Double> centralities = new ArrayList<>(salientNodes.values());
  // double maxCentrality = Collections.max(centralities);
  // double minCentrality = Collections.min(centralities);
  //

}
