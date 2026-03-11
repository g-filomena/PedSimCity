package pedsim.core.cognition.metrics;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Set;
import org.locationtech.jts.planargraph.DirectedEdge;
import pedsim.core.agents.Agent;
import sim.graph.Building;
import sim.graph.GraphUtils;
import sim.graph.NodeGraph;
import sim.routing.RoutingUtils;

/**
 * Utility for computing local and global landmarkness scores, scoring candidate nodes, and
 * selecting the best candidate.
 */
public class Landmarkness {

  /**
   * Local landmarkness: max score of known landmarks adjacent to candidate node.
   */
  public double localLandmarknessNode(Agent agent, NodeGraph candidateNode) {
    List<Building> nodeLocalLandmarks = new ArrayList<>(candidateNode.adjacentBuildings);
    Set<Integer> landmarksIDs = agent.getCognitiveMap().getLocalLandmarksIDs();

    nodeLocalLandmarks.removeIf(lm -> !landmarksIDs.contains(lm.buildingID));
    if (nodeLocalLandmarks.isEmpty())
      return 0.0;

    List<Double> scores = new ArrayList<>();
    for (Building lm : nodeLocalLandmarks)
      scores.add(lm.attributes.get("localLandmarkness").getDouble());

    return Collections.max(scores);
  }

  /**
   * Global landmarkness of a target node relative to destination.
   */
  public static double globalLandmarknessNode(NodeGraph targetNode, NodeGraph destinationNode) {
    List<Building> distantLandmarks = new ArrayList<>(targetNode.visibleBuildings3d);
    if (distantLandmarks.isEmpty())
      return 0.0;

    ArrayList<Building> anchors =
        new ArrayList<>(LandmarkIntegration.getAnchors(destinationNode).getArray());
    double nodeGlobalScore = 0.0;
    double targetDistance = GraphUtils.nodesDistance(targetNode, destinationNode);

    for (Building lm : distantLandmarks) {
      if (!anchors.isEmpty() && !anchors.contains(lm))
        continue;

      double score = lm.attributes.get("globalLandmarkness").getDouble();
      List<Double> distances =
          new ArrayList<>(LandmarkIntegration.getDistances(destinationNode).getArray());
      double distanceLandmark = distances.get(anchors.indexOf(lm));
      double distanceFactor = Math.min(targetDistance / distanceLandmark, 1.0);
      score *= distanceFactor;

      if (anchors.isEmpty())
        score *= 0.90;
      nodeGlobalScore = Math.max(nodeGlobalScore, score);
    }
    return nodeGlobalScore;
  }

  /**
   * Global landmarkness for dual-node centroids.
   */
  public static double globalLandmarknessDualNode(NodeGraph centroid, NodeGraph targetCentroid,
      NodeGraph destinationNode) {
    DirectedEdge streetSegment = targetCentroid.getPrimalEdge().getDirEdge(0);
    NodeGraph targetNode = (NodeGraph) streetSegment.getToNode();
    NodeGraph primalJunction = RoutingUtils.getPrimalJunction(centroid, targetCentroid);

    if (primalJunction != null && primalJunction.equals(targetNode))
      targetNode = (NodeGraph) streetSegment.getFromNode();

    return globalLandmarknessNode(targetNode, destinationNode);
  }
}
