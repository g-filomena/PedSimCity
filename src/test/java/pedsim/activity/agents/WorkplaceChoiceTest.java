package pedsim.activity.agents;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ec.util.MersenneTwisterFast;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.locationtech.jts.geom.Coordinate;
import pedsim.core.parameters.RouteChoicePars;
import sim.graph.GraphUtils;
import sim.graph.NodeGraph;

/**
 * Pins the workplace draw against the definition it implements, written out over the map: the node
 * a seed gives depends on the order of the map, on every weight and on the single number drawn.
 */
public class WorkplaceChoiceTest {

  private final MersenneTwisterFast random = new MersenneTwisterFast(23L);
  private final Map<NodeGraph, Double> attraction = new LinkedHashMap<>();
  private final List<NodeGraph> nodes = new ArrayList<>();

  @BeforeEach
  void scatterJobs() {
    for (int i = 0; i < 400; i++) {
      NodeGraph node =
          node(1_392_000 + 4_000 * random.nextDouble(), 4_988_000 + 4_000 * random.nextDouble());
      nodes.add(node);
      attraction.put(node, random.nextDouble() < 0.1 ? 0.0 : 50 * random.nextDouble());
    }
  }

  @AfterEach
  void restore() {
    RouteChoicePars.useGravityModel = true;
    WorkplaceChoice.clear();
  }

  @Test
  void theDrawIsTheOneTheWeightsDefine() {
    double[][] settings = {{1.0, 0.0}, {1.5, 0.0}, {2.0, 540.0}, {0.0, 0.0}, {1.0, 900.0}};
    for (boolean gravity : new boolean[] {true, false}) {
      RouteChoicePars.useGravityModel = gravity;
      for (double[] setting : settings) {
        Set<NodeGraph> drawn = new HashSet<>();
        for (int i = 0; i < 500; i++) {
          NodeGraph home = nodes.get(random.nextInt(nodes.size()));
          long seed = random.nextLong();
          MersenneTwisterFast one = new MersenneTwisterFast(seed);
          MersenneTwisterFast other = new MersenneTwisterFast(seed);

          NodeGraph workplace = WorkplaceChoice.draw(home, attraction, setting[0], setting[1], one);

          assertSame(byDefinition(home, setting[0], setting[1], other), workplace);
          assertEquals(other.nextLong(), one.nextLong(), "not one number per draw");
          assertTrue(GraphUtils.nodesDistance(home, workplace) >= setting[1]);
          drawn.add(workplace);
        }
        assertTrue(drawn.size() > 100, "only " + drawn.size() + " different workplaces");
      }
    }
  }

  @Test
  void anotherMapIsDrawnFromItsOwnNodes() {
    NodeGraph elsewhere = node(1_400_000, 4_995_000);
    WorkplaceChoice.draw(nodes.get(0), attraction, 1.0, 0.0, random);

    assertSame(
        elsewhere, WorkplaceChoice.draw(nodes.get(0), Map.of(elsewhere, 1.0), 1.0, 0.0, random));
  }

  @Test
  void nothingToDrawFromIsNull() {
    NodeGraph home = nodes.get(0);
    assertNull(WorkplaceChoice.draw(home, null, 1.0, 0.0, random));
    assertNull(WorkplaceChoice.draw(home, Map.of(), 1.0, 0.0, random));
    assertNull(WorkplaceChoice.draw(null, attraction, 1.0, 0.0, random));
    assertNull(WorkplaceChoice.draw(home, attraction, 1.0, 1e6, random));
    assertNull(WorkplaceChoice.draw(home, Map.of(nodes.get(1), 0.0), 1.0, 0.0, random));
  }

  private NodeGraph byDefinition(
      NodeGraph home, double beta, double floor, MersenneTwisterFast generator) {
    List<NodeGraph> candidates = new ArrayList<>();
    List<Double> weights = new ArrayList<>();
    double total = 0.0;
    for (Map.Entry<NodeGraph, Double> entry : attraction.entrySet()) {
      double distance = GraphUtils.nodesDistance(home, entry.getKey());
      if (distance < floor) {
        continue;
      }
      double weight = entry.getValue();
      if (RouteChoicePars.useGravityModel) {
        weight /= Math.pow(Math.max(10.0, distance), beta);
      }
      candidates.add(entry.getKey());
      weights.add(weight);
      total += weight;
    }
    double r = generator.nextDouble() * total;
    double cumulative = 0.0;
    for (int i = 0; i < candidates.size(); i++) {
      cumulative += weights.get(i);
      if (r <= cumulative) {
        return candidates.get(i);
      }
    }
    return candidates.get(candidates.size() - 1);
  }

  private static NodeGraph node(double x, double y) {
    return new NodeGraph(new Coordinate(x, y));
  }
}
