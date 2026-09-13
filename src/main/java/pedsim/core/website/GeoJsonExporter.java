package pedsim.core.website;

import java.util.LinkedHashMap;
import java.util.Map;
import sim.field.geo.VectorLayer;
import sim.io.geo.GeoJSONExporter;
import sim.util.geo.MasonGeometry;

/**
 * Converts the road layer into a GeoJSON FeatureCollection for the dashboards.
 *
 * <p>The document itself is written by {@link GeoJSONExporter}; this class only decides what goes
 * in each feature's {@code properties}. Keep it that way: assembling the JSON here would duplicate
 * that class's escaping, number formatting and geometry handling.
 */
public final class GeoJsonExporter {

  private GeoJsonExporter() {}

  /**
   * Exports all geometries in the supplied road layer as a GeoJSON FeatureCollection string.
   * Properties are empty — used by the live Streamlit/browser dashboard.
   */
  public static String exportRoads(VectorLayer roads) {
    if (roads == null) {
      return "{\"type\":\"FeatureCollection\",\"features\":[]}";
    }
    return GeoJSONExporter.toFeatureCollection(roads, false);
  }

  /**
   * Exports roads with their cumulative pedestrian {@code volume}, {@code edgeID} and
   * {@code mean_lux} embedded in each feature's {@code properties} object. Used by
   * {@link pedsim.core.website.HtmlExporter} to colour streets by traffic intensity in the
   * self-contained HTML dashboard.
   *
   * @param roads The road VectorLayer from {@code PedSimCity.roads}.
   * @param volumesMap Map of edgeID → (scenario → count). All scenarios are summed per edge.
   * @return A GeoJSON FeatureCollection string with {@code edgeID}, {@code volume} and
   *     {@code mean_lux} properties.
   */
  public static String exportRoadsWithVolumes(
      VectorLayer roads, Map<Integer, Map<String, Integer>> volumesMap) {

    if (roads == null || roads.isEmpty()) {
      return "{\"type\":\"FeatureCollection\",\"features\":[]}";
    }

    // Features are written in layer order, so a positional fallback id stays stable across the
    // pass, as it did when this walked the list itself.
    int[] autoId = {0};

    return GeoJSONExporter.toFeatureCollection(
        roads, road -> roadProperties(road, volumesMap, autoId[0]++));
  }

  private static Map<String, Object> roadProperties(
      MasonGeometry road, Map<Integer, Map<String, Integer>> volumesMap, int positionalId) {

    Integer edgeID = road.getIntegerAttribute("edgeID");
    int edgeId = edgeID != null ? edgeID : positionalId;

    int totalVolume = 0;
    if (volumesMap != null) {
      Map<String, Integer> edgeVolumes = volumesMap.get(edgeId);
      if (edgeVolumes != null) {
        totalVolume = edgeVolumes.values().stream().mapToInt(Integer::intValue).sum();
      }
    }

    double meanLux = 0.0;
    if (road.hasAttribute("mean_lux")) {
      Double lux = road.getDoubleAttribute("mean_lux");
      if (lux != null) {
        meanLux = lux;
      }
    }

    Map<String, Object> properties = new LinkedHashMap<>();
    properties.put("edgeID", edgeId);
    properties.put("volume", totalVolume);
    properties.put("mean_lux", meanLux);
    return properties;
  }
}
