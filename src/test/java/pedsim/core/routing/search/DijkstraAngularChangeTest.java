package pedsim.core.routing.search;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Stream;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.locationtech.jts.geom.Coordinate;
import pedsim.core.routing.Deflection;
import sim.field.geo.VectorLayer;
import sim.util.geo.MasonGeometry;

/** The deflection the angular search prices a turn with. */
class DijkstraAngularChangeTest {

  private static final Coordinate A = new Coordinate(0, 0);
  private static final Coordinate J = new Coordinate(10, 0);

  @Test
  void straight_on_is_zero_and_straight_back_is_180() {
    assertEquals(0.0, Deflection.degrees(A, J, new Coordinate(25, 0)), 1e-12);
    assertEquals(180.0, Deflection.degrees(A, J, new Coordinate(3, 0)), 1e-12);
  }

  @Test
  void a_right_angle_either_way_is_90() {
    assertEquals(90.0, Deflection.degrees(A, J, new Coordinate(10, 7)), 1e-12);
    assertEquals(90.0, Deflection.degrees(A, J, new Coordinate(10, -7)), 1e-12);
  }

  @Test
  void a_leg_without_length_deflects_nothing() {
    assertEquals(0.0, Deflection.degrees(J, J, new Coordinate(10, 7)), 0.0);
  }

  private static final Path RESOURCES = Path.of("src", "main", "resources");

  static Stream<String> citiesWithDualGraph() throws Exception {
    try (Stream<Path> folders = Files.list(RESOURCES)) {
      List<String> cities = new ArrayList<>();
      folders
          .filter(Files::isDirectory)
          .map(folder -> folder.getFileName().toString())
          .filter(
              city ->
                  Files.exists(layer(city, "_edgesDual")) && Files.exists(layer(city, "_nodes")))
          .sorted()
          .forEach(cities::add);
      return cities.stream();
    }
  }

  /**
   * On every dual link a city ships, the deflection computed from the three junctions equals the
   * {@code deg} cityImage wrote: the turns are priced as cityImage prices them.
   */
  @Tag("slow")
  @ParameterizedTest(name = "{0}")
  @MethodSource("citiesWithDualGraph")
  void the_deflection_is_cityImages_deg(String city) throws Exception {
    Map<Integer, Coordinate> junctions = new HashMap<>();
    for (MasonGeometry node : read(city, "_nodes").getGeometries()) {
      junctions.put(node.getIntegerAttribute("nodeID"), node.geometry.getCoordinate());
    }
    Map<Integer, int[]> segments = new HashMap<>();
    for (MasonGeometry edge : read(city, "_edges").getGeometries()) {
      segments.put(
          edge.getIntegerAttribute("edgeID"),
          new int[] {edge.getIntegerAttribute("u"), edge.getIntegerAttribute("v")});
    }

    int links = 0;
    double worst = 0.0;
    String worstLink = "";
    for (MasonGeometry link : read(city, "_edgesDual").getGeometries()) {
      int[] a = segments.get(link.getIntegerAttribute("u"));
      int[] b = segments.get(link.getIntegerAttribute("v"));
      int junction = a[0] == b[0] || a[0] == b[1] ? a[0] : a[1];
      assertTrue(
          junction == b[0] || junction == b[1],
          city + ": a dual link between streets that do not meet");
      int farA = a[0] == junction ? a[1] : a[0];
      int farB = b[0] == junction ? b[1] : b[0];
      double deflection =
          Deflection.degrees(junctions.get(farA), junctions.get(junction), junctions.get(farB));
      double difference = Math.abs(deflection - link.getDoubleAttribute("deg"));
      if (difference > worst) {
        worst = difference;
        worstLink = link.getIntegerAttribute("u") + "-" + link.getIntegerAttribute("v");
      }
      links++;
    }
    assertTrue(links > 0, city + ": no dual links");
    // Paris, an older build, differs by 1.2e-6 degrees; the pipeline-built cities by less than
    // 1e-6.
    assertTrue(
        worst < 1e-5, city + ": deflection differs from deg by " + worst + " on " + worstLink);
  }

  private static Path layer(String city, String suffix) {
    return RESOURCES.resolve(city).resolve(city + suffix + ".gpkg");
  }

  private static VectorLayer read(String city, String suffix) throws Exception {
    VectorLayer layer = new VectorLayer();
    VectorLayer.readGPKG(layer(city, suffix).toUri().toURL(), layer);
    return layer;
  }
}
