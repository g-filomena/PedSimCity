package pedsim.night.engine;

import java.util.List;
import pedsim.activity.engine.ActivityEnvironment;
import sim.graph.EdgeGraph;
import sim.util.geo.AttributeValue;
import sim.util.geo.MasonGeometry;

/**
 * Environment preparation for the night module. Extends the activity-based
 * {@link ActivityEnvironment} (graph, census, workplace + night POI) with the night lighting join:
 * illuminated-edge {@code mean_lux} and {@code min_lux}.
 */
public class NightEnvironment extends ActivityEnvironment {

  /** Runs the activity preparation, then the night-specific lighting join. */
  public static void prepare() {

    ActivityEnvironment.prepare();

    // Join illuminated edges onto the primal graph if the night dataset was loaded.
    if (!PedSimCityNight.illuminatedEdges.isEmpty()) {
      joinIlluminatedEdges();
    }
  }

  /**
   * Joins mean_lux and min_lux from the illuminated edges dataset onto the primal graph edges by
   * edgeID. Only edges present in both datasets receive them; edges not present in the illuminated
   * dataset are left without the attributes.
   *
   * <p>Both are joined because {@link NightLighting#isLit} tests both, and its min_lux test is
   * written to pass when the attribute is absent - the degradation a city with no lighting pipeline
   * needs. An edge carrying mean_lux but not min_lux therefore takes that pass silently, so the
   * count of each is logged.
   */
  private static void joinIlluminatedEdges() {
    List<MasonGeometry> illuminatedGeoms = PedSimCityNight.illuminatedEdges.getGeometries();
    int joined = 0;
    int joinedMin = 0;
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

      AttributeValue minLuxAttr = geom.getAttributes().get("min_lux");
      if (minLuxAttr != null) {
        edge.attributes.put("min_lux", new AttributeValue(minLuxAttr.getDouble()));
        joinedMin++;
      }
    }

    int graphEdges = pedsim.core.engine.PedSimCity.edgesMap.size();
    logger.info(
        "mean_lux set on "
            + joined
            + " / "
            + graphEdges
            + " graph edges, min_lux on "
            + joinedMin
            + " ("
            + illuminatedGeoms.size()
            + " illuminated records, "
            + missing
            + " with no matching graph edge).");
    if (joined > 0 && joinedMin == 0) {
      logger.warning(
          "No min_lux on any edge: the dark-spot half of the lighting gate cannot fire, and every "
              + "edge passes it. Check that the illuminated layer carries a min_lux column.");
    }
  }
}
