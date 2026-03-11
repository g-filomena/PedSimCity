package pedsim.core.routing.elements;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import pedsim.core.agents.Agent;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.cognition.metrics.Complexity;
import pedsim.core.cognition.metrics.Landmarkness;
import sim.graph.Graph;
import sim.graph.GraphUtils;
import sim.graph.NodeGraph;

/**
 * Series of functions that support landmark-based navigation, landmarkness computation,
 * identification of on-route marks and wayfinding easiness of a certain space.
 */
public class LandmarkNavigation {

  protected Agent agent;
  protected NodeGraph originNode;
  protected NodeGraph currentNode;
  protected NodeGraph destinationNode;
  protected Graph network = SharedCognitiveMap.getCommunityPrimalNetwork();
  protected List<NodeGraph> sequence = new ArrayList<>();
  protected List<NodeGraph> inRegionSequence = new ArrayList<>();
  protected List<NodeGraph> onRouteMarks = new ArrayList<>();
  protected Map<NodeGraph, Double> salientNodes = new HashMap<>();
  protected Complexity complexity = new Complexity();
  private Landmarkness landmarkness = new Landmarkness();

  private static final double SCORE_WEIGHT = 0.60;
  private static final double DISTANCE_GAIN_WEIGHT = 0.40;
  private static final double SCORE_WEIGHT_REGION = 0.50;
  private static final double DISTANCE_GAIN_WEIGHT_REGION = 0.50;

  protected void idntifyAgentLocalLandmarks() {

    double localLandmarkThreshold = agent.getHeuristics().getLocallLandmarksThreshold();
    agent.getCognitiveMap().findKnownLocalLandmarks(localLandmarkThreshold);
  }

  /**
   * Checks the criteria for selecting a candidate node based on various conditions.
   *
   * @param candidateNode The node being evaluated.
   * @param searchDistance The search distance limit for evaluating potential nodes.
   * @return {@code true} if the candidate node meets all criteria, {@code false} otherwise.
   */
  protected boolean checkCriteria(NodeGraph candidateNode, double searchDistance) {

    return !sequence.contains(candidateNode) && !candidateNode.equals(originNode)
        && network.getEdgeBetween(candidateNode, currentNode) == null
        && network.getEdgeBetween(candidateNode, originNode) == null
        && GraphUtils.nodesDistance(currentNode, candidateNode) <= searchDistance;
  }

  /**
   * Checks the criteria for selecting a candidate node based on various conditions.
   *
   * @param candidateNode The node being evaluated.
   * @param exitGateway The exit gateway node within the region.
   * @param searchDistance The search distance limit for evaluating potential nodes.
   * @param currentDistance The current distance to the exit gateway.
   * @return {@code true} if the candidate node meets all criteria, {@code false} otherwise.
   */
  protected boolean checkCriteria(NodeGraph candidateNode, NodeGraph exitGateway,
      double searchDistance, double currentDistance) {

    return !inRegionSequence.contains(candidateNode) && !candidateNode.equals(currentNode)
        && network.getEdgeBetween(candidateNode, currentNode) == null
        && GraphUtils.nodesDistance(currentNode, candidateNode) <= searchDistance
        && GraphUtils.nodesDistance(candidateNode, exitGateway) <= currentDistance
        && !sequence.contains(candidateNode);
  }

  /**
   * Calculates the score for a candidate node based on centrality and distance gain metrics.
   * 
   * @param candidateNode The node being evaluated.
   * @return The calculated score for the candidate node, considering centrality and distance gain.
   */
  protected double calculateScore(NodeGraph candidateNode, NodeGraph destinationNode,
      boolean regionBased) {

    double score = landmarkness.localLandmarknessNode(agent, candidateNode);
    double currentDistance = GraphUtils.nodesDistance(currentNode, destinationNode);
    double distanceGain =
        (currentDistance - GraphUtils.nodesDistance(candidateNode, destinationNode))
            / currentDistance;
    if (regionBased)
      return score * SCORE_WEIGHT_REGION + distanceGain * DISTANCE_GAIN_WEIGHT_REGION;
    return score * SCORE_WEIGHT + distanceGain * DISTANCE_GAIN_WEIGHT;
  }

  // /**
  // * * Calculates the score for a candidate node based on centrality and distance gain metrics. *
  // * * @param candidateNode The node being evaluated. * @param minCentrality The minimum
  // centrality
  // * value in the network. * @param maxCentrality The maximum centrality value in the network.
  // * * @return The calculated score for the candidate node, considering centrality and distance
  // * gain.
  // */
  // protected double calculateScore(NodeGraph candidateNode, double minCentrality,
  // double maxCentrality) {
  // NodeScorer nodeScorer = new NodeScorer(agent, currentNode);
  // double score = agent.getProperties().usingLocalLandmarks
  // ? nodeScorer.localLandmarknessNode(agent, candidateNode)
  // : (candidateNode.getCentrality() - minCentrality) / (maxCentrality - minCentrality);
  // double currentDistance = GraphUtils.nodesDistance(currentNode, destinationNode);
  // double distanceGain =
  // (currentDistance - GraphUtils.nodesDistance(candidateNode, destinationNode))
  // / currentDistance;
  // return score * 0.60 + distanceGain * 0.40;
  // }
  //
  //
  //
  // /**
  // * Calculates the score for a candidate node based on centrality/landmarkness and distance gain.
  // *
  // * @param candidateNode node being evaluated
  // * @param target immediate target (destination or exit gateway)
  // * @param currentDistance distance from currentNode to target
  // * @param minCentrality min centrality value in candidate set
  // * @param maxCentrality max centrality value in candidate set
  // * @param centralityWeight weight for centrality/landmarkness
  // * @param gainWeight weight for distance gain
  // * @return weighted score
  // */
  // protected double calculateScore(NodeGraph candidateNode, NodeGraph target, double
  // currentDistance,
  // double minCentrality, double maxCentrality, double salienceWeight,
  // double distanceGainWeight) {
  //
  //
  // double score = agent.getProperties().usingLocalLandmarks
  // ? nodeScorer.localLandmarknessNode(agent, candidateNode)
  // : (maxCentrality == minCentrality ? 0.0
  // : (candidateNode.getCentrality() - minCentrality) / (maxCentrality - minCentrality));
  //
  // double distanceGain =
  // (currentDistance - GraphUtils.nodesDistance(candidateNode, target)) / currentDistance;
  //
  // return score * centralityWeight + distanceGain * distanceGainWeight;
  // }
  //
  //
  //
  // /**
  // * Calculates the score for a candidate node based on centrality and gain metrics within a
  // region.
  // *
  // * @param candidateNode The candidate node for which the score is calculated.
  // * @param exitGateway The exit gateway node within the region.
  // * @param currentDistance The current distance to the exit gateway.
  // * @param minCentrality The minimum centrality value in the network.
  // * @param maxCentrality The maximum centrality value in the network.
  // * @return The calculated score for the candidate node, considering centrality and gain metrics.
  // */



  // /**
  // * Computes the local landmarkness score for a given node based on local landmarks in its
  // * proximity and the landmarks known by the agent.
  // *
  // * @param node The node for which to calculate local landmarkness.
  // * @return The computed local landmarkness score for the node.
  // */
  // private double localLandmarkness(NodeGraph candidateNode) {
  // List<Building> nodeLocalLandmarks = new ArrayList<>(candidateNode.adjacentBuildings);
  // Set<Integer> landmarksIDs = agent.getCognitiveMap().getLocalLandmarksIDs();
  //
  // for (Building landmark : candidateNode.adjacentBuildings) {
  // if (!landmarksIDs.contains(landmark.buildingID)) {
  // nodeLocalLandmarks.remove(landmark);
  // }
  // }
  //
  // if (nodeLocalLandmarks.isEmpty()) {
  // return 0.0;
  // }
  // List<Double> localScores = new ArrayList<>();
  // for (Building landmark : nodeLocalLandmarks) {
  // localScores.add(landmark.attributes.get("localLandmarkness").getDouble());
  // }
  // return Collections.max(localScores);
  // }
  //
  // /**
  // * Computes the global landmarkness score for a target node based on the global landmarks in its
  // * proximity and their relationship with the destination node.
  // *
  // * @param targetNode The target node being examined.
  // * @param destinationNode The final destination node.
  // * @return The computed global landmarkness score for the target node.
  // */
  // public static double globalLandmarknessNode(NodeGraph targetNode, NodeGraph destinationNode) {
  //
  // // get the distant landmarks
  // List<Building> distantLandmarks = new ArrayList<>(targetNode.visibleBuildings3d);
  //
  // if (distantLandmarks.isEmpty()) {
  // return 0.0;
  // }
  //
  // // get the anchors of the destination
  // ArrayList<Building> anchors =
  // new ArrayList<>(LandmarkIntegration.getAnchors(destinationNode).getArray());
  // double nodeGlobalScore = 0.0;
  // double targetDistance = GraphUtils.nodesDistance(targetNode, destinationNode);
  // for (Building landmark : distantLandmarks) {
  // if (!anchors.isEmpty() && !anchors.contains(landmark)) {
  // continue;
  // }
  //
  // double score = landmark.attributes.get("globalLandmarkness").getDouble();
  // List<Double> distances =
  // new ArrayList<>(LandmarkIntegration.getDistances(destinationNode).getArray());
  // double distanceLandmark = distances.get(anchors.indexOf(landmark));
  // double distanceFactor = Math.min(targetDistance / distanceLandmark, 1.0);
  // score *= distanceFactor;
  //
  // if (anchors.isEmpty()) {
  // score *= 0.90;
  // }
  // nodeGlobalScore = Math.max(nodeGlobalScore, score);
  // }
  // return nodeGlobalScore;
  // }
  //
  // /**
  // * Computes the global landmarkness for a target dual node based on the global landmarks in its
  // * proximity and their relationship with the destination node.
  // *
  // * @param centroid The current centroid node.
  // * @param targetCentroid The target centroid node.
  // * @param destinationNode The destination node.
  // * @return The computed global landmarkness score for the dual node.
  // */
  // public static double globalLandmarknessDualNode(NodeGraph centroid, NodeGraph targetCentroid,
  // NodeGraph destinationNode) {
  //
  // // current real segment: identifying the node
  // DirectedEdge streetSegment = targetCentroid.getPrimalEdge().getDirEdge(0);
  // NodeGraph targetNode = (NodeGraph) streetSegment.getToNode(); // targetNode
  // NodeGraph primalJunction = RoutingUtils.getPrimalJunction(centroid, targetCentroid);
  // if (primalJunction != null && primalJunction.equals(targetNode)) {
  // targetNode = (NodeGraph) streetSegment.getFromNode();
  // }
  //
  // return globalLandmarknessNode(targetNode, destinationNode);
  // }

  /**
   * @return the onRouteMarks
   */
  public List<NodeGraph> getOnRouteMarks() {
    return onRouteMarks;
  }

  /**
   * @param onRouteMarks the onRouteMarks to set
   */
  public void setOnRouteMarks(List<NodeGraph> onRouteMarks) {
    this.onRouteMarks = onRouteMarks;
  }
}
