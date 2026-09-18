package pedsim.night.engine;

import java.util.List;
import pedsim.activity.engine.ActivityEnvironment;
import pedsim.activity.engine.CensusZone;
import pedsim.activity.engine.PedSimCityActivity;
import sim.graph.EdgeGraph;
import sim.graph.NodeGraph;
import sim.util.geo.AttributeValue;
import sim.util.geo.MasonGeometry;

/**
 * Environment preparation for the night module. Extends the activity-based
 * {@link ActivityEnvironment} (graph, census, workplace + night POI) with the night perception/
 * safety joins: per-zone vulnerability rate and illuminated-edge {@code mean_lux}/{@code min_lux}.
 */
public class NightEnvironment extends ActivityEnvironment {

  /** Runs the activity preparation, then the night-specific (vulnerability + lighting) joins. */
  public static void prepare() {

    ActivityEnvironment.prepare();

    deriveVulnerability();

    // Join illuminated edges (mean_lux, min_lux) onto the primal graph if the night dataset was
    // loaded.
    if (!PedSimCityNight.illuminatedEdges.isEmpty()) {
      joinIlluminatedEdges();
    }
  }

  /**
   * Reads the {@code vulnerability_pct} column carried by the unified census layer and broadcasts it
   * to each zone's nodes. Vulnerability is an intensive rate, so it is broadcast unchanged (not
   * split like the POI counts); when several zones claim a node the highest rate wins.
   */
  private static void deriveVulnerability() {
    for (CensusZone zone : PedSimCityActivity.censusZones) {
      if (zone.nodes.isEmpty()) continue;
      double vulnerability = zoneValue(zone.geometry, "vulnerability_pct");
      if (vulnerability == 0.0) continue;
      for (NodeGraph node : zone.nodes) {
        PedSimCityNight.nodesVulnerabilityWeight.merge(node, vulnerability, Double::max);
      }
    }
  }

  /**
   * Joins mean_lux and min_lux from the illuminated edges dataset onto the primal graph edges by
   * edgeID. An edge is only considered joined when the dataset carries edgeID and mean_lux for it
   * and a matching graph edge exists; min_lux is joined alongside whenever the dataset record
   * carries it too.
   *
   * <p>min_lux is written by the same per-edge aggregation as mean_lux ({@code
   * 03_street_lights.py}: {@code edge_stats = points.groupby(...).agg(min_lux=..., mean_lux=...)},
   * both {@code fillna}'d), so wherever mean_lux is present in the dataset, min_lux normally is
   * too. Before this fix only mean_lux was joined, so {@link NightLighting#isLit}'s min_lux check
   * read null on every edge and its {@code minLuxAttr == null} fallback passed the gate
   * unconditionally regardless of how dark the edge's darkest sampled point actually was
   * (register finding C1). Edges not present in the illuminated dataset are left without either
   * attribute.
   */
  private static void joinIlluminatedEdges() {
    List<MasonGeometry> illuminatedGeoms = PedSimCityNight.illuminatedEdges.getGeometries();
    int joined = 0;
    int missing = 0;

    for (MasonGeometry geom : illuminatedGeoms) {
      AttributeValue edgeIDAttr = geom.getAttributes().get("edgeID");
      AttributeValue meanLuxAttr = geom.getAttributes().get("mean_lux");
      if (edgeIDAttr == null || meanLuxAttr == null) {
        missing++;
        continue;
      }

      EdgeGraph edge = pedsim.core.engine.PedSimCity.edgesMap.get(edgeIDAttr.getInteger());
      if (edge == null) {
        missing++;
        continue;
      }

      edge.attributes.put("mean_lux", new AttributeValue(meanLuxAttr.getDouble()));
      AttributeValue minLuxAttr = geom.getAttributes().get("min_lux");
      if (minLuxAttr != null) {
        edge.attributes.put("min_lux", new AttributeValue(minLuxAttr.getDouble()));
      }
      joined++;
    }

    int graphEdges = pedsim.core.engine.PedSimCity.edgesMap.size();
    logger.info(
        "mean_lux set on "
            + joined
            + " / "
            + graphEdges
            + " graph edges ("
            + illuminatedGeoms.size()
            + " illuminated records, "
            + missing
            + " with no matching graph edge).");
  }
}
