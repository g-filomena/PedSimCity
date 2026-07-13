package pedsim.core.website;

import java.util.Map;
import org.locationtech.jts.geom.Geometry;
import sim.field.geo.VectorLayer;
import sim.io.geo.GeoJSONExporter;
import sim.util.geo.MasonGeometry;

/**
 * Utility class that converts a VectorLayer of road geometries into a
 * GeoJSON FeatureCollection string.
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
   * Exports roads with their cumulative pedestrian {@code volume} and {@code edgeID} embedded
   * in each feature's {@code properties} object. Used by {@link pedsim.core.website.HtmlExporter}
   * to colour streets by traffic intensity in the self-contained HTML dashboard.
   *
   * @param roads      The road VectorLayer from {@code PedSimCity.roads}.
   * @param volumesMap Map of edgeID → (scenario → count). All scenarios are summed per edge.
   * @return A GeoJSON FeatureCollection string with {@code edgeID} and {@code volume} properties.
   */
  public static String exportRoadsWithVolumes(
      VectorLayer roads, Map<Integer, Map<String, Integer>> volumesMap) {

    if (roads == null || roads.isEmpty()) {
      return "{\"type\":\"FeatureCollection\",\"features\":[]}";
    }

    StringBuilder sb = new StringBuilder();
    sb.append("{\"type\":\"FeatureCollection\",\"features\":[");

    boolean first = true;
    int autoId = 0;

    for (Object obj : roads.getGeometries()) {
      if (!(obj instanceof MasonGeometry mg)) continue;

      Geometry geom = mg.getGeometry();
      if (geom == null) continue;

      // Retrieve the edgeID attribute; fall back to a sequential counter if absent
      int edgeId;
      try {
        edgeId = mg.getIntegerAttribute("edgeID");
      } catch (Exception e) {
        edgeId = autoId;
      }
      autoId++;

      // Sum all scenario volumes for this edge into one total
      int totalVolume = 0;
      if (volumesMap != null) {
        Map<String, Integer> edgeVols = volumesMap.get(edgeId);
        if (edgeVols != null) {
          totalVolume = edgeVols.values().stream().mapToInt(Integer::intValue).sum();
        }
      }

      double meanLux = 0.0;
      if (mg.hasAttribute("mean_lux")) {
        meanLux = mg.getDoubleAttribute("mean_lux");
      }

      if (!first) sb.append(',');
      first = false;

      sb.append("{\"type\":\"Feature\",\"geometry\":");
      sb.append(geomToGeoJson(geom));
      sb.append(",\"properties\":{\"edgeID\":")
          .append(edgeId)
          .append(",\"volume\":")
          .append(totalVolume)
          .append(",\"mean_lux\":")
          .append(meanLux)
          .append("}}");
    }

    sb.append("]}");
    return sb.toString();
  }

  // -------------------------------------------------------------------------
  // Internal geometry → GeoJSON helpers
  // -------------------------------------------------------------------------

  private static String geomToGeoJson(Geometry geom) {
    return switch (geom.getGeometryType()) {
      case "LineString" -> {
        StringBuilder sb = new StringBuilder("{\"type\":\"LineString\",\"coordinates\":[");
        var coords = geom.getCoordinates();
        for (int i = 0; i < coords.length; i++) {
          if (i > 0) sb.append(',');
          sb.append('[').append(coords[i].x).append(',').append(coords[i].y).append(']');
        }
        sb.append("]}");
        yield sb.toString();
      }
      case "MultiLineString" -> {
        StringBuilder sb = new StringBuilder("{\"type\":\"MultiLineString\",\"coordinates\":[");
        for (int n = 0; n < geom.getNumGeometries(); n++) {
          if (n > 0) sb.append(',');
          sb.append('[');
          var coords = geom.getGeometryN(n).getCoordinates();
          for (int i = 0; i < coords.length; i++) {
            if (i > 0) sb.append(',');
            sb.append('[').append(coords[i].x).append(',').append(coords[i].y).append(']');
          }
          sb.append(']');
        }
        sb.append("]}");
        yield sb.toString();
      }
      default -> "{\"type\":\"GeometryCollection\",\"geometries\":[]}";
    };
  }
}
