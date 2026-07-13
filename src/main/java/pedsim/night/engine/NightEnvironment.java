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
 * safety joins: per-zone vulnerability rate and illuminated-edge {@code mean_lux}.
 */
public class NightEnvironment extends ActivityEnvironment {

  /** Runs the activity preparation, then the night-specific (vulnerability + lighting) joins. */
  public static void prepare() {

    ActivityEnvironment.prepare();

    deriveVulnerability();

    // Join illuminated edges (mean_lux) onto the primal graph if the night dataset was loaded.
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
     * Joins mean_lux from the illuminated edges dataset onto the primal graph edges by edgeID.
     * Only edges present in both datasets receive a mean_lux attribute. Edges not present in the
     * illuminated dataset are left without the attribute.
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
            joined++;
        }

        int graphEdges = pedsim.core.engine.PedSimCity.edgesMap.size();
        logger.info(
            "mean_lux set on " + joined + " / " + graphEdges + " graph edges ("
                + illuminatedGeoms.size() + " illuminated records, " + missing
                + " with no matching graph edge).");
    }
}
