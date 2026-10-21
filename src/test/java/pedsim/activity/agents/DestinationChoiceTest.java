package pedsim.activity.agents;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertSame;

import ec.util.MersenneTwisterFast;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.locationtech.jts.geom.Coordinate;
import sim.graph.NodeGraph;

/**
 * Pins what {@link DestinationChoice} chooses among: the nodes holding something for the purpose,
 * each drawing in proportion to what it holds. A node with nothing there is not a destination,
 * however many of them surround the agent.
 */
public class DestinationChoiceTest {

  private static NodeGraph node(double x, double y) {
    return new NodeGraph(new Coordinate(x, y));
  }

  @Test
  void anEmptyNodeIsNeverChosenWhileANodeWithSomethingIsInReach() {
    NodeGraph origin = node(0, 0);
    List<NodeGraph> candidates = new ArrayList<>();
    for (int i = 0; i < 200; i++) {
      candidates.add(node(10, i));
    }
    NodeGraph shop = node(500, 0);
    candidates.add(shop);
    Map<NodeGraph, Double> attraction = new LinkedHashMap<>();
    attraction.put(shop, 1.0);

    MersenneTwisterFast random = new MersenneTwisterFast(1L);
    for (int i = 0; i < 100; i++) {
      assertSame(shop, DestinationChoice.choose(candidates, origin, attraction, null, random));
    }
  }

  @Test
  void equidistantNodesDrawInProportionToTheirAttraction() {
    NodeGraph origin = node(0, 0);
    NodeGraph small = node(300, 0);
    NodeGraph large = node(-300, 0);
    Map<NodeGraph, Double> attraction = new LinkedHashMap<>();
    attraction.put(small, 1.0);
    attraction.put(large, 9.0);

    MersenneTwisterFast random = new MersenneTwisterFast(2L);
    int draws = 20_000;
    int toLarge = 0;
    for (int i = 0; i < draws; i++) {
      if (DestinationChoice.choose(List.of(small, large), origin, attraction, null, random)
          == large) {
        toLarge++;
      }
    }
    assertEquals(0.9, (double) toLarge / draws, 0.01);
  }

  @Test
  void withNothingForThePurposeInReachEveryNodeStaysACandidate() {
    NodeGraph origin = node(0, 0);
    List<NodeGraph> candidates = List.of(node(100, 0), node(0, 100));
    Map<NodeGraph, Double> attraction = new LinkedHashMap<>();
    attraction.put(node(5000, 0), 3.0);

    assertNotNull(
        DestinationChoice.choose(
            candidates, origin, attraction, null, new MersenneTwisterFast(3L)));
  }
}
