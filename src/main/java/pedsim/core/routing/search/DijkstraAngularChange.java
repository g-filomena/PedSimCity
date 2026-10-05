package pedsim.core.routing.search;

import java.util.ArrayList;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.planargraph.DirectedEdge;
import pedsim.core.agents.Agent;
import pedsim.core.cognition.metrics.Landmarkness;
import pedsim.core.routing.Deflection;
import sim.graph.EdgeGraph;
import sim.graph.NodeGraph;

/**
 * Least cumulative angular change route, searched on the primal graph.
 *
 * <p>A search state is a street walked in one direction, so the junction it reaches is known.
 * Moving from one street to the next costs the deflection between them at that junction, the angle
 * between the two streets' chords ({@link Deflection}). The first street from the origin costs
 * nothing, unless the leg continues a walk, in which case the turn from the street it arrived by is
 * paid. A street is never walked straight back along.
 *
 * <p>The route is the cheapest over every street leaving the origin and every street reaching the
 * destination; among routes of equal angle, the shorter.
 */
public class DijkstraAngularChange extends Dijkstra {

  /**
   * A street walked in one direction, with the angle and length accumulated to its far end and the
   * state it was reached from. Ordered by angle, then length, then creation.
   */
  private static final class State implements Comparable<State> {
    final DirectedEdge edge;
    final double angle;
    final double length;
    final long order;
    final State previous;
    boolean settled;

    State(DirectedEdge edge, double angle, double length, long order, State previous) {
      this.edge = edge;
      this.angle = angle;
      this.length = length;
      this.order = order;
      this.previous = previous;
    }

    @Override
    public int compareTo(State other) {
      int byAngle = Double.compare(angle, other.angle);
      if (byAngle != 0) {
        return byAngle;
      }
      int byLength = Double.compare(length, other.length);
      return byLength != 0 ? byLength : Long.compare(order, other.order);
    }
  }

  /** The best state found for each directed street; a state replaced here is stale in the queue. */
  private final Map<DirectedEdge, State> best = new IdentityHashMap<>(1 << 12);

  private final PriorityQueue<State> queue = new PriorityQueue<>(1 << 10);
  private long order = 0;
  private boolean confined;

  /**
   * Computes the route.
   *
   * @param originNode The origin node.
   * @param destinationNode The destination node.
   * @param finalDestinationNode The destination of the whole trip, for landmark weighting.
   * @param arrivalEdge The street the walk reached the origin by, or null at the start of a trip.
   * @param directedEdgesToAvoid Streets the route must not use, in either direction; may be null.
   * @param agent The agent.
   * @return The directed edges of the route, origin to destination; empty when there is none.
   */
  public List<DirectedEdge> dijkstraAlgorithm(
      NodeGraph originNode,
      NodeGraph destinationNode,
      NodeGraph finalDestinationNode,
      DirectedEdge arrivalEdge,
      Set<DirectedEdge> directedEdgesToAvoid,
      Agent agent) {

    initialise(originNode, destinationNode, finalDestinationNode, agent);
    initialisePrimal(directedEdgesToAvoid);
    confined = restrictToKnownNetwork();
    State last = search(arrivalEdge);
    return last == null ? new ArrayList<>() : reconstructSequence(last);
  }

  private State search(DirectedEdge arrivalEdge) {
    Coordinate arrivalFrom = arrivalEdge == null ? null : arrivalEdge.getFromNode().getCoordinate();
    EdgeGraph arrivalStreet = arrivalEdge == null ? null : (EdgeGraph) arrivalEdge.getEdge();
    for (DirectedEdge out : originNode.getOutDirectedEdges()) {
      if (!usable(out) || parentEdge(out) == arrivalStreet) {
        continue;
      }
      double turn = arrivalFrom == null ? 0.0 : turnCost(arrivalFrom, out);
      relax(null, best.get(out), out, turn, length(out));
    }

    State state;
    while ((state = poll()) != null) {
      DirectedEdge edge = state.edge;
      if (edge.getToNode().equals(destinationNode)) {
        return state;
      }
      Coordinate from = edge.getFromNode().getCoordinate();
      for (DirectedEdge out : ((NodeGraph) edge.getToNode()).getOutDirectedEdges()) {
        if (out.getEdge() == edge.getEdge()) {
          continue;
        }
        State current = best.get(out);
        if ((current != null && current.settled) || !usable(out)) {
          continue;
        }
        relax(state, current, out, state.angle + turnCost(from, out), state.length + length(out));
      }
    }
    return null;
  }

  /** Records {@code to} reached from {@code from} if that beats {@code current}, its best so far. */
  private void relax(State from, State current, DirectedEdge to, double angle, double length) {
    if (current != null
        && (current.angle < angle || (current.angle == angle && current.length <= length))) {
      return;
    }
    State state = new State(to, angle, length, order++, from);
    best.put(to, state);
    queue.add(state);
  }

  private State poll() {
    State state;
    while ((state = queue.poll()) != null) {
      if (!state.settled && best.get(state.edge) == state) {
        state.settled = true;
        return state;
      }
    }
    return null;
  }

  /** Whether the search may walk this street: known to the agent if confined, and not avoided. */
  private boolean usable(DirectedEdge directedEdge) {
    EdgeGraph edge = (EdgeGraph) directedEdge.getEdge();
    if (confined && !isEdgeKnown(edge)) {
      return false;
    }
    return !edgesToAvoid.contains(edge);
  }

  /**
   * The cost of turning into {@code out}, coming from {@code from}: the deflection, with the
   * agent's perception error, and discounted by the global landmarkness of the street's far end
   * when distant landmarks guide the agent.
   */
  private double turnCost(Coordinate from, DirectedEdge out) {
    NodeGraph target = (NodeGraph) out.getToNode();
    double angle =
        Deflection.degrees(from, out.getFromNode().getCoordinate(), target.getCoordinate())
            * costPerceptionError((EdgeGraph) out.getEdge());
    angle = Math.max(MIN_DEFLECTION_ANGLE, Math.min(MAX_DEFLECTION_ANGLE, angle));
    if (landmarkCondition(target)) {
      double globalLandmarkness = Landmarkness.globalLandmarknessNode(target, finalDestinationNode);
      angle *= 1.0 - globalLandmarkness * agent.getHeuristics().getGlobalLandmarkWeight(true);
    }
    return angle;
  }

  private static double length(DirectedEdge directedEdge) {
    return ((EdgeGraph) directedEdge.getEdge()).getLength();
  }

  /** The street in the whole network: a region subgraph's edge is mapped back to its parent. */
  private EdgeGraph parentEdge(DirectedEdge directedEdge) {
    EdgeGraph edge = (EdgeGraph) directedEdge.getEdge();
    return subGraph == null ? edge : subGraph.getParentEdge(edge);
  }

  private List<DirectedEdge> reconstructSequence(State last) {
    List<DirectedEdge> sequence = new ArrayList<>();
    for (State step = last; step != null; step = step.previous) {
      sequence.add(subGraph == null ? step.edge : toParent(step.edge));
    }
    Collections.reverse(sequence);
    return sequence;
  }

  /** The parent network's directed edge for a subgraph one, by the street and its direction. */
  private DirectedEdge toParent(DirectedEdge directedEdge) {
    EdgeGraph parent = parentEdge(directedEdge);
    DirectedEdge forward = parent.getDirEdge(0);
    return forward
            .getFromNode()
            .getCoordinate()
            .equals2D(directedEdge.getFromNode().getCoordinate())
        ? forward
        : parent.getDirEdge(1);
  }
}
