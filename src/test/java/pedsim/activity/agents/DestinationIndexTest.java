package pedsim.activity.agents;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import ec.util.MersenneTwisterFast;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import sim.field.geo.VectorLayer;
import sim.graph.Graph;
import sim.graph.NodeGraph;
import sim.graph.NodesLookup;
import sim.util.geo.MasonGeometry;

/**
 * Pins that a layout lists the same nodes as the distance lookup it stands in for, in the same
 * order: {@link DestinationChoice} draws from the list by position, so the order is part of which
 * destination a seed gives.
 */
public class DestinationIndexTest {

  private final MersenneTwisterFast random = new MersenneTwisterFast(11L);
  private Graph network;

  /** Streets between scattered junctions, so that no two distances are alike. */
  @BeforeEach
  void buildNetwork() {
    GeometryFactory factory = new GeometryFactory();
    List<MasonGeometry> streets = new ArrayList<>();
    Coordinate previous = new Coordinate(1_394_000, 4_990_000);
    for (int i = 0; i < 600; i++) {
      Coordinate next =
          new Coordinate(
              1_392_000 + 4_000 * random.nextDouble(), 4_988_000 + 4_000 * random.nextDouble());
      streets.add(new MasonGeometry(factory.createLineString(new Coordinate[] {previous, next})));
      previous = next;
    }
    network = new Graph();
    network.fromStreetJunctionsSegments(new VectorLayer(), new VectorLayer(streets));
  }

  @Test
  void everyNodeWithinAnyRadiusComesInLookupOrder() {
    List<NodeGraph> all = DestinationIndex.inLookupOrder(network);
    assertEquals(network.getNodes().size(), all.size());
    DestinationIndex.Layout layout = new DestinationIndex.Layout(all, network, null);

    for (int i = 0; i < 200; i++) {
      NodeGraph origin = network.getNodes().get(random.nextInt(all.size()));
      double radius = 3_500 * random.nextDouble();

      assertEquals(
          NodesLookup.getNodesBetweenDistanceInterval(network, origin, 0.0, radius),
          layout.within(origin, radius));
    }
  }

  @Test
  void aPurposeLayoutIsTheLookupNarrowedToItsOpportunities() {
    List<NodeGraph> all = DestinationIndex.inLookupOrder(network);
    Map<NodeGraph, Double> attraction = new LinkedHashMap<>();
    for (NodeGraph node : network.getNodes()) {
      if (random.nextDouble() < 0.2) {
        attraction.put(node, random.nextDouble() < 0.1 ? 0.0 : 1.0);
      }
    }
    List<NodeGraph> places = new ArrayList<>();
    for (NodeGraph node : all) {
      if (attraction.getOrDefault(node, 0.0) > 0.0) {
        places.add(node);
      }
    }
    DestinationIndex.Layout layout = new DestinationIndex.Layout(places, attraction, null);

    int compared = 0;
    for (int i = 0; i < 200; i++) {
      NodeGraph origin = network.getNodes().get(random.nextInt(all.size()));
      double radius = 500 + 3_000 * random.nextDouble();
      List<NodeGraph> found = layout.within(origin, radius);
      if (found.isEmpty()) {
        continue;
      }
      compared++;
      assertEquals(
          DestinationChoice.opportunities(
              NodesLookup.getNodesBetweenDistanceInterval(network, origin, 0.0, radius),
              attraction),
          found);
    }
    assertFalse(compared < 100, "only " + compared + " origins had a place in reach");
  }

  @Test
  void theOriginIsNotAPlaceToGo() {
    List<NodeGraph> all = DestinationIndex.inLookupOrder(network);
    DestinationIndex.Layout layout = new DestinationIndex.Layout(all, network, null);
    NodeGraph origin = all.get(5);

    assertEquals(List.of(), layout.within(origin, 0.0));
    assertFalse(layout.within(origin, 1_000.0).contains(origin));
  }
}
