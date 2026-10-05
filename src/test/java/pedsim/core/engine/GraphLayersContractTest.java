package pedsim.core.engine;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Stream;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import sim.field.geo.VectorLayer;
import sim.graph.EdgeGraph;
import sim.graph.Graph;
import sim.graph.NodeGraph;
import sim.util.geo.MasonGeometry;

/**
 * Every bundled city's graph layers, loaded the way {@link Import#readGraphs()} loads them, build
 * the graphs the layers describe.
 *
 * <p>GeoMason-light builds a graph from the segments' end coordinates, not from their {@code u} /
 * {@code v}: an end creates or finds the node at exactly that point, and a junction's attributes
 * reach a node only when the junction sits exactly there. So a layer can be read without an error
 * and still give a different graph: an end a hair off its junction adds a bare node (which {@link
 * Environment} then reads a null {@code nodeID} from), and two junctions on one point become one
 * node.
 *
 * <p>Each city is checked against its own layers: the graph has one node per junction and one edge
 * per segment, every node carries its junction's id, every edge joins the nodes its {@code u} and
 * {@code v} name. A city folder whose files do
 * not carry its own name (such as {@code Torino_centre}) is not a city {@code --cityName} can load
 * and is skipped.
 *
 * <p>Tagged {@code slow}: it reads every city. Run it with {@code mvn test -Pslow-tests
 * -Pall-modules}.
 */
@Tag("slow")
class GraphLayersContractTest {

  private static final Path RESOURCES = Path.of("src", "main", "resources");

  static Stream<String> cities() throws IOException {
    try (Stream<Path> folders = Files.list(RESOURCES)) {
      List<String> cities = new ArrayList<>();
      folders
          .filter(Files::isDirectory)
          .map(folder -> folder.getFileName().toString())
          .filter(
              city -> Files.exists(layer(city, "_nodes")) && Files.exists(layer(city, "_edges")))
          .sorted()
          .forEach(cities::add);
      return cities.stream();
    }
  }

  @Test
  void at_least_one_city_is_bundled() throws IOException {
    assertFalse(cities().toList().isEmpty(), "no city with _nodes and _edges layers found");
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("cities")
  void the_primal_graph_is_the_one_the_layers_describe(String city) throws Exception {
    VectorLayer junctions = read(city, "_nodes");
    VectorLayer segments = read(city, "_edges");
    Graph network = new Graph();
    network.fromStreetJunctionsSegments(junctions, segments);

    assertGraphIsTheLayers(city + " primal", network, junctions, segments, "nodeID");
  }

  /**
   * The graph built from {@code junctions} and {@code segments} has one node per junction carrying
   * that junction's {@code idField}, one edge per segment, and each edge joins the nodes its
   * {@code u} and {@code v} name.
   */
  private static void assertGraphIsTheLayers(
      String label, Graph graph, VectorLayer junctions, VectorLayer segments, String idField) {
    int junctionCount = junctions.getGeometries().size();
    int segmentCount = segments.getGeometries().size();

    // Nodes first: two junctions on one point is the usual cause, and it also drops the segments
    // between them (both ends land on the merged node), which the edge count would report instead.
    Set<Integer> ids = new HashSet<>();
    int bare = 0;
    for (NodeGraph node : graph.getNodes()) {
      Integer id = idOf(node.getMasonGeometry(), idField);
      if (id == null) {
        bare++;
      } else {
        ids.add(id);
      }
    }
    assertEquals(
        0,
        bare,
        label + ": nodes at a segment end where no junction sits exactly (no " + idField + ")");
    assertEquals(
        junctionCount,
        graph.getNodes().size(),
        label + ": junctions on one point merge, and junctions no segment reaches are left out");
    assertEquals(junctionCount, ids.size(), label + ": each junction's " + idField + " once");
    assertEquals(
        segmentCount,
        graph.getEdges().size(),
        label + ": segments that are not one LineString with two distinct ends are dropped");

    int wrong = 0;
    List<String> examples = new ArrayList<>();
    for (EdgeGraph edge : graph.getEdges()) {
      Integer from = idOf(edge.getFromNode().getMasonGeometry(), idField);
      Integer to = idOf(edge.getToNode().getMasonGeometry(), idField);
      Integer u = idOf(edge.getMasonGeometry(), "u");
      Integer v = idOf(edge.getMasonGeometry(), "v");
      boolean joins =
          from != null
              && to != null
              && ((from.equals(u) && to.equals(v)) || (from.equals(v) && to.equals(u)));
      if (!joins) {
        wrong++;
        if (examples.size() < 5) {
          examples.add(u + "-" + v + " built as " + from + "-" + to);
        }
      }
    }
    assertEquals(
        0, wrong, label + ": edges joining other nodes than their u and v, e.g. " + examples);
  }

  private static Integer idOf(MasonGeometry geometry, String field) {
    if (geometry == null || !geometry.hasAttribute(field)) {
      return null;
    }
    return geometry.getIntegerAttribute(field);
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
