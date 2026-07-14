package pedsim.activity.engine;

import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.logging.Logger;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.index.strtree.STRtree;
import pedsim.activity.agents.ActivityPurpose;
import pedsim.activity.parameters.ActivityPars;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.engine.PedSimCity;
import pedsim.core.utilities.LoggerUtil;
import sim.graph.NodeGraph;
import sim.util.geo.MasonGeometry;

/**
 * Scans the buildings layer and the optional POI layer for OSM-like use tags ({@code amenity},
 * {@code shop}, {@code leisure}, {@code office}, {@code use}, …), classifies each tagged feature
 * into {@link ActivityPurpose}s, and accumulates a per-node attraction weight per purpose (each
 * feature attracts to its nearest network node within {@link ActivityPars#poiClaimRadius}).
 *
 * <p>The two layers weigh differently: a POI is a <em>venue</em> and counts 1.0, while a building
 * is land-use <em>fabric</em> and counts its footprint area in units of
 * {@link ActivityPars#buildingAreaPerAttractionUnit} (min 1.0) — so an office block attracts
 * proportionally more WORK trips than a single office venue.
 *
 * <p>The resulting maps drive purpose-aware destination choice in {@code ActivityAgent} and
 * work-node assignment in {@code ActivityPopulate}. Cities without tags simply produce empty maps:
 * destination choice degrades to uniform and work assignment to the DMA / distance fallbacks.
 */
public final class PoiClassifier {

  private static final Logger logger = LoggerUtil.getLogger();

  /** Radii (m) tried in order when snapping a POI to its nearest network node. */
  private static final double[] SNAP_RADII = {50.0, 100.0};

  private PoiClassifier() {}

  /**
   * Builds {@code PedSimCityActivity.nodesPurposeWeight} from the tagged buildings and POIs.
   * Idempotent per run: clears any previous content first.
   */
  public static void buildPurposeWeights() {
    Map<ActivityPurpose, Map<NodeGraph, Double>> weights =
        PedSimCityActivity.nodesPurposeWeight;
    weights.clear();

    List<NodeGraph> nodes = SharedCognitiveMap.getCommunityPrimalNetwork().getNodes();
    if (nodes.isEmpty()) {
      return;
    }
    STRtree nodeIndex = ActivityEnvironment.buildNodeIndex(nodes);

    int classified = 0;
    classified += classifyLayer(PedSimCity.buildings.getGeometries(), nodeIndex, weights, true);
    classified +=
        classifyLayer(PedSimCityActivity.poisLayer.getGeometries(), nodeIndex, weights, false);

    if (classified == 0) {
      logger.info("poiClassifier: no OSM-like use tags found; purpose weights unavailable "
          + "(uniform destination choice; work nodes from DMA / distance fallbacks).");
      return;
    }

    Map<ActivityPurpose, Integer> counts = new EnumMap<>(ActivityPurpose.class);
    weights.forEach((purpose, map) -> counts.put(purpose, map.size()));
    logger.info(
        "poiClassifier: classified " + classified + " tagged features; nodes per purpose: "
            + counts);
  }

  private static int classifyLayer(
      List<MasonGeometry> features,
      STRtree nodeIndex,
      Map<ActivityPurpose, Map<NodeGraph, Double>> weights,
      boolean areaWeighted) {

    int classified = 0;
    for (MasonGeometry feature : features) {
      Geometry geometry = feature.getGeometry();
      if (geometry == null) {
        continue;
      }
      // All recognised uses: scalar tags plus every label of the land_uses list, so a
      // mixed-use building weighs in on each purpose it hosts.
      Set<ActivityPurpose> purposes = ActivityPurpose.classifyAll(feature.getAttributes());
      if (purposes.isEmpty()) {
        continue;
      }
      NodeGraph node = nearestNode(geometry, nodeIndex);
      if (node == null) {
        continue;
      }
      double weight = areaWeighted ? attractionUnits(geometry) : 1.0;
      for (ActivityPurpose purpose : purposes) {
        weights
            .computeIfAbsent(purpose, p -> new java.util.HashMap<>())
            .merge(node, weight, Double::sum);
      }
      classified++;
    }
    return classified;
  }

  /**
   * A building's attraction in venue units: footprint area (m², from the projected geometry) over
   * {@link ActivityPars#buildingAreaPerAttractionUnit}, never below one venue.
   */
  private static double attractionUnits(Geometry geometry) {
    double area = geometry.getArea();
    if (area <= 0.0) {
      return 1.0;
    }
    return Math.max(1.0, area / ActivityPars.buildingAreaPerAttractionUnit);
  }

  /** Nearest network node within the claim radius, trying small envelopes first. */
  private static NodeGraph nearestNode(Geometry geometry, STRtree nodeIndex) {
    for (double radius : SNAP_RADII) {
      NodeGraph node = nearestWithin(geometry, nodeIndex, radius);
      if (node != null) {
        return node;
      }
    }
    return nearestWithin(geometry, nodeIndex, ActivityPars.poiClaimRadius);
  }

  private static NodeGraph nearestWithin(Geometry geometry, STRtree nodeIndex, double radius) {
    Envelope envelope = new Envelope(geometry.getEnvelopeInternal());
    envelope.expandBy(radius);

    @SuppressWarnings("unchecked")
    List<NodeGraph> candidates = nodeIndex.query(envelope);

    NodeGraph best = null;
    double bestDistance = radius;
    for (NodeGraph candidate : candidates) {
      Geometry nodeGeometry = candidate.getMasonGeometry().getGeometry();
      if (nodeGeometry == null) {
        continue;
      }
      double distance = geometry.distance(nodeGeometry);
      if (distance <= bestDistance) {
        bestDistance = distance;
        best = candidate;
      }
    }
    return best;
  }
}
