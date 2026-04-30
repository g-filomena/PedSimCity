package pedsim.core.website;

import org.locationtech.jts.geom.Geometry;
import sim.field.geo.VectorLayer;
import sim.util.geo.MasonGeometry;

/**
 * Utility class that converts a VectorLayer of road geometries into a
 * GeoJSON FeatureCollection string.
 */
public final class GeoJsonExporter {

  private GeoJsonExporter() {}

  /**
   * Exports all geometries in the supplied road layer as a GeoJSON FeatureCollection string.
   */
  public static String exportRoads(VectorLayer roads) {
    if (roads == null || roads.getGeometries().isEmpty()) {
      return "{\"type\":\"FeatureCollection\",\"features\":[]}";
    }

    StringBuilder sb = new StringBuilder();
    sb.append("{\"type\":\"FeatureCollection\",\"features\":[");

    boolean first = true;
    for (Object obj : roads.getGeometries()) {
      if (!(obj instanceof MasonGeometry mg))
        continue;

      Geometry geom = mg.getGeometry();
      if (geom == null)
        continue;

      if (!first)
        sb.append(',');
      first = false;

      sb.append("{\"type\":\"Feature\",\"geometry\":");
      sb.append(geomToGeoJson(geom));
      sb.append(",\"properties\":{}}");
    }

    sb.append("]}");
    return sb.toString();
  }

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
