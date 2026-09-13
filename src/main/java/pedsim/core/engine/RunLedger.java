package pedsim.core.engine;

import java.util.concurrent.atomic.DoubleAdder;
import java.util.concurrent.atomic.LongAdder;

/**
 * What a day of simulation measured about itself.
 *
 * <p>Measurement only, and it must stay that way. Every quantity here is recorded so that a run can
 * be checked, and none of it is read back by the mechanism that produced it: the release is charged
 * an estimate hours before the routes that answer it exist, so the ledger always lags, and an
 * allocation that grows when it sees less walking than it charged for is a positive feedback loop -
 * it reached 21x on a one-day run when it was tried. Where a measured gap is real, it is closed at
 * its source, not by feeding it back here.
 *
 * <p>Separate from {@code PedSimCity} because it is not simulation state: it is what an observer
 * wrote down while watching. Keeping the two apart is what stops a measurement being read as though
 * it were an input.
 *
 * <p>Adders rather than counters because agents step concurrently when {@code Pars.parallel} is set.
 */
public class RunLedger {

  /**
   * Metres of route planned today, summed over every leg of every agent.
   *
   * <p>Recorded the moment a route is laid out, which is when its length is first known exactly.
   */
  private final DoubleAdder plannedRouteMeters = new DoubleAdder();

  /**
   * Metres actually walked on the legs that finished today.
   *
   * <p>Separate from the planned ledger because a trip chain still under way at the day boundary has
   * its last leg planned in full and walked in part. Counting only the planned figure biases any
   * planned-against-charged comparison in the same direction as the charge itself.
   */
  private final DoubleAdder walkedRouteMeters = new DoubleAdder();

  /**
   * Lengths offered to the two ledgers above that could not be counted: zero, negative, NaN.
   *
   * <p>A ledger of metres should not accumulate a NaN, so these are rejected - but a ledger that
   * discards input has to say so, or a run whose routes all measure zero reports having planned and
   * walked nothing with no indication why. The count is printed alongside the totals.
   */
  private final LongAdder unusableRouteLengths = new LongAdder();

  /**
   * Times a destination search had to widen its distance band, and times it gave up and took any
   * node in the city.
   *
   * <p>Both are counted because they happen inside the lookup call, so a run could not otherwise say
   * whether a leg came from the band it asked for or from one three times wider, and the fallback
   * replaces the band with the whole network. Since leg length is the open question, these two
   * counts are the difference between an answer and a guess.
   */
  private final LongAdder destinationWidenings = new LongAdder();

  private final LongAdder destinationFallbacks = new LongAdder();

  /**
   * Legs that asked for an angular-change route and got the shortest path instead, because no
   * angular path survived. A route-choice model quietly substituting another one is exactly the
   * kind of thing that has to be counted: at a low rate it is a handful of awkward pairs, and at a
   * high one the simplest-path results are partly shortest-path results.
   */
  private final LongAdder angularFallbacks = new LongAdder();

  /** Angular routes whose dual-graph search returned nothing at all. */
  private final LongAdder angularNoDualPath = new LongAdder();

  /** Angular routes that had a dual path until {@code cleanDualPath} trimmed it to nothing. */
  private final LongAdder angularTrimmedAway = new LongAdder();

  /**
   * Records an angular-change route that fell back to shortest path.
   *
   * @param trimmed whether the dual search found a path and the cleaning step removed all of it,
   *     as opposed to finding no path in the first place. The two say different things: the first
   *     is a defect in the trimming, the second is the dual graph being less connected than the
   *     primal one.
   */
  public void recordAngularFallback(boolean trimmed) {
    angularFallbacks.increment();
    if (trimmed) {
      angularTrimmedAway.increment();
    } else {
      angularNoDualPath.increment();
    }
  }

  /** Angular routes lost because the dual search found no path. */
  public long angularNoDualPath() {
    return angularNoDualPath.sum();
  }

  /** Angular routes lost because the cleaning step trimmed the path away. */
  public long angularTrimmedAway() {
    return angularTrimmedAway.sum();
  }

  /** Angular routes attempted, so the fallbacks above can be read as a rate. */
  private final LongAdder angularAttempts = new LongAdder();

  /** Records an attempt to build an angular-change route. */
  public void recordAngularAttempt() {
    angularAttempts.increment();
  }

  /** Angular-change routes attempted so far today. */
  public long angularAttempts() {
    return angularAttempts.sum();
  }

  /**
   * Failed angular routes whose dual entry or exit node was not in the agent's known dual network.
   *
   * <p>{@code getDualNode} picks an edge centroid beside the origin without consulting what the
   * agent knows, while the search is confined to the agent's known dual subgraph. When the two
   * disagree there is no path by construction, whoever is asking and however well connected the
   * city is.
   */
  private final LongAdder angularEndpointUnknown = new LongAdder();

  /** Records a failed angular route whose dual endpoints were outside the agent's known network. */
  public void recordAngularEndpointUnknown() {
    angularEndpointUnknown.increment();
  }

  /** Failed angular routes with a dual endpoint the agent did not know. */
  public long angularEndpointUnknown() {
    return angularEndpointUnknown.sum();
  }

  /** Angular-change routes replaced by shortest path so far today. */
  public long angularFallbacks() {
    return angularFallbacks.sum();
  }

  /**
   * Routes an individualised agent could not build on the streets it knows, and which were found
   * by searching the whole network instead.
   *
   * <p>Kept separate from {@link #angularFallbacks()} because the two are different substitutions:
   * a fallback swaps the route-choice model, while this keeps the model and widens the graph. Both
   * are ways the model answers a question the agent's own knowledge could not, so both are counted
   * rather than absorbed.
   *
   * <p>The agent then plans against a route through streets it has never walked. Its length should
   * carry a much larger error than a known route's; the model does not yet represent that.
   */
  private final LongAdder fullNetworkEscalations = new LongAdder();

  /** Of those, the ones that were angular-change searches rather than road-distance ones. */
  private final LongAdder fullNetworkEscalationsAngular = new LongAdder();

  /**
   * Records a route found only by searching beyond the agent's known network.
   *
   * @param angular whether the search was an angular-change one.
   */
  public void recordFullNetworkEscalation(boolean angular) {
    fullNetworkEscalations.increment();
    if (angular) {
      fullNetworkEscalationsAngular.increment();
    }
  }

  /** Routes found only by searching beyond the agent's known network. */
  public long fullNetworkEscalations() {
    return fullNetworkEscalations.sum();
  }

  /** Of those, the angular-change ones. */
  public long fullNetworkEscalationsAngular() {
    return fullNetworkEscalationsAngular.sum();
  }

  /**
   * Records a leg whose route has just been planned.
   *
   * @param meters the routed length, which is what will actually be walked
   */
  public void recordPlannedRoute(double meters) {
    if (meters > 0.0 && Double.isFinite(meters)) {
      plannedRouteMeters.add(meters);
    } else {
      unusableRouteLengths.increment();
    }
  }

  /** Records a leg that has just finished, with the length actually covered. */
  public void recordWalkedRoute(double meters) {
    if (meters > 0.0 && Double.isFinite(meters)) {
      walkedRouteMeters.add(meters);
    } else {
      unusableRouteLengths.increment();
    }
  }

  /** Records that a destination search widened its band the given number of times. */
  public void recordDestinationWidening(int widenings) {
    if (widenings > 0) {
      destinationWidenings.add(widenings);
    }
  }

  /** Records a destination search that exhausted its band and took any node instead. */
  public void recordDestinationFallback() {
    destinationFallbacks.increment();
  }

  /** Metres of route planned so far today. */
  public double plannedRouteMeters() {
    return plannedRouteMeters.sum();
  }

  /** Metres walked on legs completed so far today. */
  public double walkedRouteMeters() {
    return walkedRouteMeters.sum();
  }

  /** Route lengths the ledgers had to throw away today, because they were not usable numbers. */
  public long unusableRouteLengths() {
    return unusableRouteLengths.sum();
  }

  /** Total band widenings so far today. */
  public long destinationWidenings() {
    return destinationWidenings.sum();
  }

  /** Total band fallbacks so far today. */
  public long destinationFallbacks() {
    return destinationFallbacks.sum();
  }

  /** Clears the day's ledgers and counters. */
  public void reset() {
    plannedRouteMeters.reset();
    walkedRouteMeters.reset();
    unusableRouteLengths.reset();
    destinationWidenings.reset();
    destinationFallbacks.reset();
    angularFallbacks.reset();
    angularNoDualPath.reset();
    angularTrimmedAway.reset();
    angularAttempts.reset();
    angularEndpointUnknown.reset();
    fullNetworkEscalations.reset();
    fullNetworkEscalationsAngular.reset();
  }
}
