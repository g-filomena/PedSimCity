package pedsim.core.routing.search;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import sim.field.geo.VectorLayer;
import sim.graph.EdgeGraph;
import sim.graph.Graph;
import sim.graph.NodeGraph;
import sim.util.geo.MasonGeometry;

/**
 * The angular search never steps between two streets joining the same pair of junctions: a straight
 * road and a crescent bowing north of it, with a street continuing west of the pair.
 */
class DijkstraAngularChangeTest {

  private static final GeometryFactory FACTORY = new GeometryFactory();

  private static MasonGeometry street(double... xy) {
    Coordinate[] coordinates = new Coordinate[xy.length / 2];
    for (int i = 0; i < coordinates.length; i++) {
      coordinates[i] = new Coordinate(xy[2 * i], xy[2 * i + 1]);
    }
    return new MasonGeometry(FACTORY.createLineString(coordinates));
  }

  private static NodeGraph centroid(EdgeGraph edge) {
    NodeGraph centroid = new NodeGraph(edge.getLine().getCentroid().getCoordinate());
    centroid.setPrimalEdge(edge);
    return centroid;
  }

  private final Graph graph = new Graph();
  private final EdgeGraph crescent;
  private final EdgeGraph road;
  private final EdgeGraph west;

  DijkstraAngularChangeTest() {
    List<MasonGeometry> streets =
        new ArrayList<>(
            Arrays.asList(
                street(0, 0, 50, 100, 100, 0), street(100, 0, 0, 0), street(-100, 0, 0, 0)));
    graph.fromStreetJunctionsSegments(new VectorLayer(), new VectorLayer(streets));
    NodeGraph a = graph.findNode(new Coordinate(0, 0));
    NodeGraph b = graph.findNode(new Coordinate(100, 0));
    NodeGraph westEnd = graph.findNode(new Coordinate(-100, 0));
    List<EdgeGraph> pair = graph.getEdgesBetween(a, b);
    assertEquals(2, pair.size());
    road = pair.get(0).getLength() < pair.get(1).getLength() ? pair.get(0) : pair.get(1);
    crescent = road == pair.get(0) ? pair.get(1) : pair.get(0);
    west = graph.getEdgeBetween(westEnd, a);
  }

  @Test
  @DisplayName(
      "a crescent and the road it bows away from are parallel, whichever way each is drawn")
  void streetsSharingBothJunctionsAreParallel() {
    // the road is drawn east to west and the crescent west to east
    assertTrue(DijkstraAngularChange.areParallel(centroid(crescent), centroid(road)));
    assertTrue(DijkstraAngularChange.areParallel(centroid(road), centroid(crescent)));
  }

  @Test
  @DisplayName("streets sharing one junction are not parallel, nor is a street with itself")
  void oneSharedJunctionIsNotParallel() {
    assertFalse(DijkstraAngularChange.areParallel(centroid(west), centroid(road)));
    assertFalse(DijkstraAngularChange.areParallel(centroid(west), centroid(crescent)));
    assertFalse(DijkstraAngularChange.areParallel(centroid(road), centroid(road)));
  }
}
