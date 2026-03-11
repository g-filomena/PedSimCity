package pedsim.core.cognition.metrics;

import java.util.ArrayList;
import java.util.List;
import java.util.PriorityQueue;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.engine.PedSimCity;
import pedsim.core.parameters.RouteChoicePars;
import sim.field.geo.VectorLayer;
import sim.graph.Building;
import sim.graph.NodeGraph;
import sim.util.geo.AttributeValue;
import sim.util.geo.MasonGeometry;

/**
 * Manages the integration of landmarks into a graph.
 */
public class LandmarkIntegration {

  /**
   * It assigns to each node in the graph a list of local landmarks.
   *
   */
  public static void setLocalLandmarksAtJunctions() {

    List<NodeGraph> nodes = SharedCognitiveMap.getCommunityPrimalNetwork().getNodes();
    nodes.forEach((node) -> {
      List<MasonGeometry> containedLandmarks =
          SharedCognitiveMap.getLocalLandmarks().featuresWithinDistance(
              node.getMasonGeometry().geometry, RouteChoicePars.distanceNodeLandmark);
      for (MasonGeometry masonGeometry : containedLandmarks) {
        node.adjacentBuildings.add(PedSimCity.buildingsMap.get((int) masonGeometry.getUserData()));
      }
    });
  }

  /**
   * It assigns to each node in the graph a list of distant landmarks and their corresponding global
   * landmarkness values.
   *
   */
  public static void assignAnchoringLandmarksToNodes() {

    List<NodeGraph> nodes = SharedCognitiveMap.getCommunityPrimalNetwork().getNodes();
    nodes.forEach((node) -> {
      List<Building> anchors = new ArrayList<>(); //
      List<Double> distances = new ArrayList<>();

      List<MasonGeometry> potentialAnchors =
          SharedCognitiveMap.getGlobalLandmarks().featuresWithinDistance(
              node.getMasonGeometry().geometry, RouteChoicePars.distanceAnchors);

      if (!potentialAnchors.isEmpty()) {
        PriorityQueue<Double> topScores = new PriorityQueue<>(); // Min-heap for top scores

        // Find the top nrAnchors gScores efficiently
        for (MasonGeometry masonGeometry : potentialAnchors) {
          double gScore = masonGeometry.getDoubleAttribute("gScore_sc");
          if (topScores.size() < RouteChoicePars.nrAnchors) {
            topScores.add(gScore); // Fill up the heap first
          } else if (gScore > topScores.peek()) {
            topScores.poll(); // Remove smallest
            topScores.add(gScore);
          }
        }

        double minimum_gScore = topScores.poll();

        for (MasonGeometry anchor : potentialAnchors) {
          if (anchor.getDoubleAttribute("gScore_sc") < minimum_gScore) {
            continue;
          }
          int buildingID = (int) anchor.getUserData();
          anchors.add(PedSimCity.buildingsMap.get(buildingID));
          distances.add(anchor.geometry.distance(node.getMasonGeometry().geometry));
        }

        node.attributes.put("anchors", new AttributeValue(anchors));
        node.attributes.put("distances", new AttributeValue(distances));
      }
    });

  }

  public static void assignVisibileGlobalLandmakrsToNodes(VectorLayer sightLines) {

    List<MasonGeometry> sightLinesGeometries = sightLines.getGeometries();

    for (MasonGeometry sightLine : sightLinesGeometries) {
      Building building = PedSimCity.buildingsMap.get(sightLine.getIntegerAttribute("buildingID"));
      NodeGraph node = PedSimCity.nodesMap.get(sightLine.getIntegerAttribute("nodeID"));
      if (node != null) {
        node.visibleBuildings3d.add(building);
      }
    }
  }

  /**
   * Retrieves the attribute value of the "anchors" for the given NodeGraph.
   *
   * @param node The NodeGraph from which to retrieve the attribute value.
   * @return The attribute value associated with "anchors".
   */
  public static AttributeValue getAnchors(NodeGraph node) {
    return node.attributes.get("anchors");
  }

  /**
   * Retrieves the attribute value of the "distances" for the given NodeGraph.
   *
   * @param node The NodeGraph from which to retrieve the attribute value.
   * @return The attribute value associated with "distances".
   */
  public static AttributeValue getDistances(NodeGraph node) {
    return node.attributes.get("distances");
  }
}
