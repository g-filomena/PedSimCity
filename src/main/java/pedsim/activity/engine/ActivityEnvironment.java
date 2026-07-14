package pedsim.activity.engine;

import java.util.ArrayList;
import java.util.List;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.index.strtree.STRtree;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.engine.Environment;
import pedsim.core.parameters.Pars;
import sim.graph.NodeGraph;
import sim.util.geo.AttributeValue;
import sim.util.geo.MasonGeometry;

/**
 * Environment preparation for activity-based modules. Extends the core {@link Environment} (graph,
 * buildings, barriers, regions) by building the {@link CensusZone} model from the census layer:
 * per-zone residence weights (population structure) and the network nodes associated with each
 * zone by proximity. Destination attraction is no census matter — it comes from the OSM-tag
 * purpose weights built by {@link PoiClassifier}.
 *
 * <p>Nodes are assigned to zones by proximity rather than strict containment, because graph nodes
 * sit on the street network (inside road zones) while population/POIs are attributed to the
 * surrounding parcels. Each zone claims the network nodes within the smallest {@link #CLAIM_RADII}
 * radius that yields at least one node; a node may be claimed by several zones (so residents of a
 * parcel land on the bordering street nodes). No type-based fusion is needed.
 *
 * <p>The same census layer also carries module-specific columns (e.g. {@code vulnerability_pct} for
 * the night module); modules read those from each zone's geometry via {@link #zoneValue}.
 */
public class ActivityEnvironment extends Environment {

  /** Radii (m) tried in order when claiming nodes for a zone; grows until ≥1 node is found. */
  private static final double[] CLAIM_RADII = {50.0, 100.0, 200.0, 400.0};

  /**
   * Runs the core infrastructure preparation, then builds the census-zone model, the
   * purpose-attraction weights from OSM-like use tags (independent of census availability), and
   * loads the transit stops.
   */
  public static void prepare() {
    Environment.prepare();
    if (!PedSimCityActivity.censusLayer.isEmpty()) {
      buildCensusZones();
    }
    PoiClassifier.buildPurposeWeights();
    pedsim.transit.TransitLoader.loadStops(null);
  }


  private static void buildCensusZones() {
    List<NodeGraph> allNodes = SharedCognitiveMap.getCommunityPrimalNetwork().getNodes();
    STRtree nodeIndex = buildNodeIndex(allNodes);

    int withNodes = 0;
    double totalResidents = 0.0;
    for (MasonGeometry geom : PedSimCityActivity.censusLayer.getGeometries()) {
      if (geom.getGeometry() == null) continue;

      CensusZone zone = new CensusZone(geom);
      zone.residence = zoneValue(geom, "residence_pct");
      zone.retireeShare = zoneValueOrNaN(geom, "retiree_pct");
      zone.studentShare = zoneValueOrNaN(geom, "student_pct");
      totalResidents += zoneValue(geom, "residents");

      zone.nodes.addAll(claimNodes(geom.getGeometry(), nodeIndex));
      if (!zone.nodes.isEmpty()) withNodes++;

      PedSimCityActivity.censusZones.add(zone);
    }

    // When the census carries absolute resident counts (P1), the sampling fraction applies to the
    // real headcount: override the population and recompute the agent count. Absent the column
    // (totalResidents == 0), the user-supplied population is kept.
    if (totalResidents > 0.0) {
      if (Pars.population == 1500000) {
        if (Pars.cityName != null && Pars.cityName.contains("Torino")) {
          // Hardcode full municipal census population of Torino (846,567) even when running on Torino_simplified graph
          Pars.population = 846567;
        } else {
          // Census carries absolute counts: apply the sampling fraction to the real headcount.
          Pars.population = (int) Math.round(totalResidents);
        }
      }
      Pars.recomputeAgentCount();
      logger.info(
          "Population set from census resident total (P1): "
              + Pars.population
              + " -> numAgents="
              + Pars.numAgents);
    }

    logger.info(
        "censusData: built "
            + PedSimCityActivity.censusZones.size()
            + " zones ("
            + withNodes
            + " with ≥1 node) over "
            + allNodes.size()
            + " network nodes.");
  }

  /** Builds an STRtree over the network nodes for fast proximity queries. */
  protected static STRtree buildNodeIndex(List<NodeGraph> nodes) {
    STRtree index = new STRtree();
    for (NodeGraph node : nodes) {
      Geometry g = node.getMasonGeometry().getGeometry();
      if (g != null) index.insert(g.getEnvelopeInternal(), node);
    }
    index.build();
    return index;
  }

  /**
   * Returns the network nodes associated with a zone: those within the smallest {@link #CLAIM_RADII}
   * radius (metres from the polygon) that yields at least one node.
   */
  protected static List<NodeGraph> claimNodes(Geometry zoneGeom, STRtree nodeIndex) {
    for (double radius : CLAIM_RADII) {
      Envelope env = new Envelope(zoneGeom.getEnvelopeInternal());
      env.expandBy(radius);

      @SuppressWarnings("unchecked")
      List<NodeGraph> candidates = nodeIndex.query(env);

      List<NodeGraph> claimed = new ArrayList<>();
      for (NodeGraph node : candidates) {
        Geometry g = node.getMasonGeometry().getGeometry();
        if (g != null && zoneGeom.distance(g) <= radius) claimed.add(node);
      }
      if (!claimed.isEmpty()) return claimed;
    }
    return new ArrayList<>();
  }

  /** Reads a numeric column from a census-zone geometry, returning 0.0 if absent or non-numeric. */
  protected static double zoneValue(MasonGeometry geom, String key) {
    AttributeValue value = geom.getAttributes().get(key);
    if (value == null) return 0.0;
    try {
      return value.getDouble();
    } catch (Exception e) {
      try {
        return Double.parseDouble(value.toString().trim());
      } catch (Exception e2) {
        try {
          return value.getInteger();
        } catch (Exception e3) {
          return 0.0;
        }
      }
    }
  }

  /**
   * Like {@link #zoneValue}, but returns {@code NaN} when the column is absent or non-numeric —
   * for optional columns where "not provided" must stay distinguishable from a genuine zero.
   */
  protected static double zoneValueOrNaN(MasonGeometry geom, String key) {
    AttributeValue value = geom.getAttributes().get(key);
    if (value == null) return Double.NaN;
    try {
      return value.getDouble();
    } catch (Exception e) {
      return Double.NaN;
    }
  }
}
