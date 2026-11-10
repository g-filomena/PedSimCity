package pedsim.learning.habit;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.linearref.LengthIndexedLine;
import pedsim.core.engine.Engine;
import pedsim.learning.agents.LearningAgent;
import pedsim.learning.cognitivemap.VividnessGrid;
import pedsim.learning.parameters.LearningPars;
import sim.graph.EdgeGraph;
import sim.graph.NodeGraph;
import sim.routing.Route;

/**
 * The cost factor a learner puts on each street, by {@link LearningPars#habitMode}.
 *
 * <p>{@code REMEMBERED}: {@code 1 - habitGain * v}, {@code v} the mean over the street of {@code
 * w / (1 + w)}, {@code w} the strength in walks of the vividness-grid cell at each sample point.
 *
 * <p>{@code REWARDED}: {@code 1 - rewardGain * q}, {@code q} the street's value. After each trip
 * the agent compares the route's efficiency (straight line over walked length) with the efficiency
 * of the first route it walked between the same two nodes. The relative gain moves the value of
 * every street on the route towards it by {@code rewardRate}. The value is kept in [-0.5, 0.5] and
 * the factor in [0.5, 1.5].
 */
public class RouteHabit {

  private static final double MAX_VALUE = 0.5;

  private final LearningAgent agent;
  private final Map<Integer, Double> value = new HashMap<>();
  private final Map<Long, Double> firstEfficiency = new HashMap<>();
  private final Map<Integer, Double> familiarity = new HashMap<>();
  private long familiarityDay = -1;

  public RouteHabit(LearningAgent agent) {
    this.agent = agent;
  }

  /** The multiplier for a street's cost. */
  public double factor(EdgeGraph edge) {
    switch (LearningPars.habitMode) {
      case REMEMBERED:
        return 1.0 - LearningPars.habitGain * familiarity(edge);
      case REWARDED:
        Double q = value.get(edge.getID());
        return q == null ? 1.0 : Math.max(0.5, Math.min(1.5, 1.0 - LearningPars.rewardGain * q));
      default:
        return 1.0;
    }
  }

  /** Learns from a walked route; called once the agent has reached its destination. */
  public void walked(Route route, NodeGraph origin, NodeGraph destination) {
    familiarity.clear();
    if (LearningPars.habitMode != HabitMode.REWARDED || route.edgesSequence == null) {
      return;
    }
    double efficiency = efficiency(route, origin, destination);
    if (Double.isNaN(efficiency)) {
      return;
    }
    long od = ((long) origin.getID() << 32) | (destination.getID() & 0xffffffffL);
    Double first = firstEfficiency.putIfAbsent(od, efficiency);
    if (first == null) {
      return;
    }
    double gain = efficiency / first - 1.0;
    Set<Integer> seen = new HashSet<>();
    for (EdgeGraph edge : route.edgesSequence) {
      if (seen.add(edge.getID())) {
        double q = value.getOrDefault(edge.getID(), 0.0);
        q += LearningPars.rewardRate * (gain - q);
        value.put(edge.getID(), Math.max(-MAX_VALUE, Math.min(MAX_VALUE, q)));
      }
    }
  }

  /** Straight line over walked length, at most 1; NaN for a trip without either. */
  public static double efficiency(Route route, NodeGraph origin, NodeGraph destination) {
    double length = route.getLineString() == null ? 0.0 : route.getLineString().getLength();
    double straight = origin.getCoordinate().distance(destination.getCoordinate());
    return length > 0.0 && straight > 0.0 ? Math.min(1.0, straight / length) : Double.NaN;
  }

  private double familiarity(EdgeGraph edge) {
    VividnessGrid grid = agent.learning.getVividnessGrid();
    if (grid == null) {
      return 0.0;
    }
    long day = Engine.getDays(agent.getState().schedule.getSteps());
    if (day != familiarityDay) {
      familiarity.clear();
      familiarityDay = day;
    }
    Double known = familiarity.get(edge.getID());
    if (known != null) {
      return known;
    }
    LineString line = edge.getLine();
    LengthIndexedLine indexed = new LengthIndexedLine(line);
    double length = line.getLength();
    int points = Math.max(1, (int) Math.ceil(length / LearningPars.cellSize));
    double sum = 0.0;
    for (int i = 0; i < points; i++) {
      Coordinate c = indexed.extractPoint((i + 0.5) * length / points);
      double w = grid.getValueAt(c.x, c.y);
      sum += w / (1.0 + w);
    }
    double v = sum / points;
    familiarity.put(edge.getID(), v);
    return v;
  }
}
