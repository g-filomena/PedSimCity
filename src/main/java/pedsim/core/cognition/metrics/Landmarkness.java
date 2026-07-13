package pedsim.core.cognition.metrics;

import java.util.Collections;
import java.util.List;
import java.util.Set;
import org.locationtech.jts.planargraph.DirectedEdge;
import pedsim.core.agents.Agent;
import sim.graph.Building;
import sim.graph.GraphUtils;
import sim.graph.NodeGraph;
import sim.routing.RoutingUtils;
import sim.util.geo.AttributeValue;

/**
 * Utility for computing local and global landmarkness scores, scoring candidate nodes, and
 * selecting the best candidate.
 */
public class Landmarkness {

  /**
   * Local landmarkness: max score of known landmarks adjacent to candidate node.
   */
  public double localLandmarknessNode(Agent agent, NodeGraph candidateNode) {
    Set<Integer> landmarksIDs = agent.getCognitiveMap().getLocalLandmarksIDs();

    double bestScore = 0.0;
    for (Building landmark : candidateNode.adjacentBuildings) {
      if (landmarksIDs.contains(landmark.buildingID)) {
        bestScore =
            Math.max(bestScore, landmark.attributes.get("localLandmarkness").getDouble());
      }
    }
    return bestScore;
  }

  /**
   * Global landmarkness of a target node relative to destination.
   *
   * <p>Performance: this runs for every candidate neighbour during landmark-aware routing, so the
   * anchor and distance lists are read in place (they are never mutated here) and resolved once
   * per call instead of being copied per visible landmark. A destination without anchoring data is
   * treated as having no anchors (the discounted-score path) rather than failing.
   */
  public static double globalLandmarknessNode(NodeGraph targetNode, NodeGraph destinationNode) {
    List<Building> distantLandmarks = targetNode.visibleBuildings3d;
    if (distantLandmarks.isEmpty()) {
      return 0.0;
    }

    AttributeValue anchorsAttribute = LandmarkIntegration.getAnchors(destinationNode);
    List<Building> anchors;
    if (anchorsAttribute == null) {
      anchors = Collections.emptyList();
    } else {
      anchors = anchorsAttribute.getArray();
    }

    double nodeGlobalScore = 0.0;

    if (anchors.isEmpty()) {
      // No anchoring information at the destination: best raw score, discounted.
      for (Building landmark : distantLandmarks) {
        double score = landmark.attributes.get("globalLandmarkness").getDouble() * 0.90;
        nodeGlobalScore = Math.max(nodeGlobalScore, score);
      }
      return nodeGlobalScore;
    }

    List<Double> distances = LandmarkIntegration.getDistances(destinationNode).getArray();
    double targetDistance = GraphUtils.nodesDistance(targetNode, destinationNode);

    for (Building landmark : distantLandmarks) {
      int anchorIndex = anchors.indexOf(landmark);
      if (anchorIndex == -1) {
        continue;
      }
      double score = landmark.attributes.get("globalLandmarkness").getDouble();
      double distanceLandmark = distances.get(anchorIndex);
      score *= Math.min(targetDistance / distanceLandmark, 1.0);
      nodeGlobalScore = Math.max(nodeGlobalScore, score);
    }
    return nodeGlobalScore;
  }

  /**
   * Global landmarkness for dual-node centroids.
   */
  public static double globalLandmarknessDualNode(
      NodeGraph centroid, NodeGraph targetCentroid, NodeGraph destinationNode) {
    DirectedEdge streetSegment = targetCentroid.getPrimalEdge().getDirEdge(0);
    NodeGraph targetNode = (NodeGraph) streetSegment.getToNode();
    NodeGraph primalJunction = RoutingUtils.getPrimalJunction(centroid, targetCentroid);

    if (primalJunction != null && primalJunction.equals(targetNode))
      targetNode = (NodeGraph) streetSegment.getFromNode();

    return globalLandmarknessNode(targetNode, destinationNode);
  }
}
