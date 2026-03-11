package pedsim.core.routing.elements;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import pedsim.core.agents.Agent;
import pedsim.core.cognition.cityimage.Region;
import pedsim.core.cognition.metrics.Complexity;
import pedsim.core.engine.PedSimCity;
import pedsim.core.parameters.RouteChoicePars;
import sim.graph.GraphUtils;
import sim.graph.NodeGraph;

/**
 * Series of functions that support landmark-based navigation, landmarkness computation,
 * identification of on-route marks and wayfinding easiness of a certain space.
 */
public class RegionLandmarkNavigation extends LandmarkNavigation {

  private Complexity complexity = new Complexity();

  /**
   * Initialises a new instance of the LandmarkNavigation class with the specified origin node,
   * destination node, and agent.
   *
   * @param originNode The starting node for navigation.
   * @param destinationNode The target destination node.
   * @param agent The agent associated with the navigation.
   */
  public RegionLandmarkNavigation(NodeGraph originNode, NodeGraph destinationNode, Agent agent) {

    this.originNode = originNode;
    this.destinationNode = destinationNode;
    this.agent = agent;
    idntifyAgentLocalLandmarks();
  }

  /**
   * Generates a sequence of intermediate nodes (on-route marks) between the origin and destination
   * nodes on the basis of local landmarkness while passing through region gateways
   * (sequenceGateways).
   *
   * @param sequenceGateways An ArrayList of gateways (nodes at the boundary between regions) that
   *        need to be traversed on the route.
   * @return An ArrayList of region-based on-route marks, including the origin and destination
   *         nodes.
   */
  public List<NodeGraph> regionOnRouteMarks(List<NodeGraph> sequenceGateways) {

    sequence = new ArrayList<>();
    currentNode = originNode;

    for (NodeGraph exitGateway : sequenceGateways) {
      if (exitGateway.equals(originNode) || currentNode.equals(destinationNode))
        continue;

      sequence.add(currentNode);
      if (currentNode.getRegionID() != exitGateway.getRegionID()) {
        currentNode = exitGateway;
        continue;
      }
      inRegionSequence = new ArrayList<>();
      // works also for nodeBasedNavigation only:
      inRegionSequence = onRouteMarksInRegion(exitGateway);
      getOnRouteMarks().addAll(inRegionSequence);
      currentNode = exitGateway;
    }
    sequence.add(destinationNode);
    return sequence;
  }

  /**
   * Finds within-region on-route marks from the current node to the passed exit gateway node,
   * within the current node's region.
   *
   * @param exitGateway The exit gateway node that marks the boundary of the region.
   * @return An ArrayList of in-region on-route marks within the same region, including the current
   *         node and exit gateway.
   */
  public List<NodeGraph> onRouteMarksInRegion(NodeGraph exitGateway) {

    Region region = PedSimCity.regionsMap.get(currentNode.getRegionID());
    findRegionSalientJunctions(region);
    if (salientNodes.isEmpty())
      return inRegionSequence;

    // compute wayfinding complexity and the resulting easinesss
    double wayfindingEasiness = complexity.wayfindingEasinessRegion(currentNode, exitGateway,
        originNode, destinationNode, agent);
    double searchDistance = GraphUtils.nodesDistance(currentNode, exitGateway) * wayfindingEasiness;

    // while the wayfindingEasiness is lower than the threshold the agent looks for
    // intermediate-points.
    while (wayfindingEasiness < agent.getCognitiveMap().getWayfindingEasinessThreshold(true)) {
      NodeGraph bestNode = findOnRouteMarkRegion(exitGateway, salientNodes, searchDistance);
      if (bestNode == null || bestNode.equals(exitGateway) || bestNode.equals(destinationNode))
        break;

      inRegionSequence.add(bestNode);
      findRegionSalientJunctions(region);
      if (salientNodes.isEmpty())
        return inRegionSequence;

      wayfindingEasiness = complexity.wayfindingEasinessRegion(bestNode, originNode,
          destinationNode, exitGateway, agent);
      searchDistance = GraphUtils.nodesDistance(bestNode, exitGateway) * wayfindingEasiness;
      currentNode = bestNode;
      bestNode = null;
    }
    return inRegionSequence;
  }

  /**
   * Identifies salient junctions within a specific region's graph based on their centrality in the
   * graph.
   *
   * @param region The region for which to identify salient junctions.
   */
  private void findRegionSalientJunctions(Region region) {

    double percentile = RouteChoicePars.salientNodesPercentile;
    salientNodes = new HashMap<>(region.primalGraph.getSalientNodes(percentile));

    // If no salient junctions are found, the tolerance increases till the 0.50
    // percentile;
    // still no salient junctions are found, the agent continues without landmarks

    while (salientNodes.isEmpty()) {
      percentile -= 0.05;
      if (percentile < 0.50)
        break;

      salientNodes = new HashMap<>(region.primalGraph.getSubGraphSalientNodes(percentile));
    }
  }

  /**
   * Finds the most salient on-route mark between the current node and the exit gateway, amongst the
   * salientNodes within a specific region.
   *
   * @param currentNode The current node in the region.
   * @param exitGateway The exit gateway node from the region.
   * @param salientNodes A map of salient junctions and their centrality scores within the region.
   * @param searchDistance The search distance limit for evaluating potential nodes.
   * @return The selected node that serves as an on-route mark, or null if none is found.
   */
  private NodeGraph findOnRouteMarkRegion(NodeGraph exitGateway,
      Map<NodeGraph, Double> salientNodes, Double searchDistance) {

    List<NodeGraph> junctions = new ArrayList<>(salientNodes.keySet());
    double currentDistance = GraphUtils.nodesDistance(currentNode, exitGateway);

    // Optimised sorting using Comparator and Collections.sort
    List<NodeGraph> sortedJunctions = junctions.stream().filter(
        candidateNode -> checkCriteria(candidateNode, exitGateway, searchDistance, currentDistance))
        .sorted(Comparator
            .comparingDouble(candidateNode -> calculateScore(candidateNode, exitGateway, true)))
        .collect(Collectors.toList());
    return sortedJunctions.isEmpty() ? null : sortedJunctions.get(sortedJunctions.size() - 1);
  }
}
