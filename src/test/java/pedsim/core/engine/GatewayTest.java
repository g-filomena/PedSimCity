package pedsim.core.engine;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import pedsim.core.cognition.elements.Gateway;
import sim.field.geo.VectorLayer;
import sim.graph.EdgeGraph;
import sim.graph.Graph;
import sim.graph.NodeGraph;
import sim.util.geo.MasonGeometry;

/**
 * Gateways between two regions whose only crossing is a pair of junctions joined by two streets:
 * a straight road and, read first, a crescent bowing north of it.
 */
class GatewayTest {

  private static final GeometryFactory FACTORY = new GeometryFactory();

  private static MasonGeometry street(double... xy) {
    Coordinate[] coordinates = new Coordinate[xy.length / 2];
    for (int i = 0; i < coordinates.length; i++) {
      coordinates[i] = new Coordinate(xy[2 * i], xy[2 * i + 1]);
    }
    return new MasonGeometry(FACTORY.createLineString(coordinates));
  }

  private static Graph crescentBesideARoad() {
    List<MasonGeometry> streets =
        new ArrayList<>(
            Arrays.asList(
                street(0, 0, 50, 100, 100, 0),
                street(0, 0, 100, 0),
                street(-100, 0, 0, 0),
                street(100, 0, 200, 0)));
    Graph graph = new Graph();
    graph.fromStreetJunctionsSegments(new VectorLayer(), new VectorLayer(streets));
    int edgeID = 0;
    for (EdgeGraph edge : graph.getEdges()) {
      edge.setID(edgeID++);
    }
    for (NodeGraph node : graph.getNodes()) {
      node.setRegionID(node.getCoordinate().x <= 0 ? 1 : 2);
    }
    return graph;
  }

  @Test
  @DisplayName("a pair joined by two streets is one crossing, one gateway")
  void parallelStreetsMakeOneGateway() {
    Graph graph = crescentBesideARoad();
    NodeGraph exit = graph.findNode(new Coordinate(0, 0));

    long crossings =
        exit.getAdjacentNodes().stream()
            .filter(node -> node.getRegionID() != exit.getRegionID())
            .count();

    assertEquals(3, exit.getEdges().size());
    assertEquals(1, crossings);
  }

  @Test
  @DisplayName("the gateway follows the shortest street, and its angle that street's shape")
  void gatewayTakesTheShortestStreet() {
    Graph graph = crescentBesideARoad();
    NodeGraph exit = graph.findNode(new Coordinate(0, 0));
    NodeGraph entry = graph.findNode(new Coordinate(100, 0));

    Gateway gateway = Environment.buildGateway(exit, entry);

    EdgeGraph road = graph.getEdgeBetween(exit, entry);
    assertEquals(100.0, road.getLength(), 1e-9);
    assertEquals(road.getID(), gateway.edgeID);
    assertEquals(100.0, gateway.distance, 1e-9);
    assertSame(entry, gateway.entry);
    assertEquals(2, gateway.regionTo);
    assertEquals(90.0, gateway.entryAngle, 1e-9); // due east along the straight road
  }

  @Test
  @DisplayName("with only the crescent between them, the angle bows towards it")
  void entryAngleFollowsTheStreetShape() {
    List<MasonGeometry> streets =
        new ArrayList<>(
            Arrays.asList(
                street(0, 0, 50, 100, 100, 0), street(-100, 0, 0, 0), street(100, 0, 200, 0)));
    Graph graph = new Graph();
    graph.fromStreetJunctionsSegments(new VectorLayer(), new VectorLayer(streets));
    NodeGraph exit = graph.findNode(new Coordinate(0, 0));
    NodeGraph entry = graph.findNode(new Coordinate(100, 0));

    Gateway gateway = Environment.buildGateway(exit, entry);

    // halfway along the crescent is its apex (50, 100): north-north-east, not due east
    assertEquals(Math.toDegrees(Math.atan2(50, 100)), gateway.entryAngle, 1e-9);
  }
}
