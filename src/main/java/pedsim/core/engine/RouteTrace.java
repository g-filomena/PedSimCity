package pedsim.core.engine;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.io.Writer;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.concurrent.atomic.DoubleAdder;
import java.util.concurrent.atomic.LongAdder;
import java.util.logging.Logger;
import pedsim.core.agents.Agent;
import sim.graph.NodeGraph;
import sim.routing.Route;

/**
 * What a run measured about itself: the totals and counters a day is summarised by, and an opt-in
 * record of every leg that produced them.
 *
 * <p>Both come from one place, {@link pedsim.core.agents.Agent#initialiseRoute}, which is the seam every
 * planner crosses. They are one class because they fail together: an agent that assigns its route
 * field directly instead of going through that seam leaves the totals reading zero and the per-leg
 * record empty, with nothing to say which of the two is wrong.
 *
 * <p>The two halves answer different questions. The counters answer <i>how much</i> - metres
 * planned and walked, band widenings, destination fallbacks, route-choice models that silently
 * served a shortest path - and are what a day's summary line prints. The per-leg record answers
 * <i>which route this model took for this OD pair</i>, which no total can, and is what makes two
 * route-choice models comparable. It is written only when {@code -Dpedsim.trace=<file>} names a
 * file, carries no timestamps, and is therefore byte-comparable between runs and between machines.
 *
 * <p><b>Measurement only, and it must stay that way.</b> Nothing here is read back by the mechanism
 * that produced it: the release is charged an estimate hours before the routes that answer it
 * exist, so the measurement always lags, and an allocation that grows when it sees less walking
 * than it charged for is a positive feedback loop - it reached 21x on a one-day run when it was
 * tried. Where a measured gap is real, it is closed at its source.
 *
 * <p>Separate from {@code PedSimCity} because it is not simulation state: it is what an observer
 * wrote down while watching. Keeping the two apart is what stops a measurement being read as though
 * it were an input.
 *
 * <p>Adders rather than counters because agents step concurrently when {@code Pars.parallel} is set;
 * writes to the per-leg file are synchronised for the same reason.
 */
public class RouteTrace {

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

  /**
   * Legs that asked for a distant-landmark route and got the shortest path instead, because the
   * landmark search returned no path. Counted for the same reason as {@code angularFallbacks}: a
   * landmark model silently serving shortest paths is indistinguishable, in the output, from a
   * landmark model that works.
   */
  private final LongAdder landmarkFallbacks = new LongAdder();

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

  /** Records a distant-landmark leg served as shortest path because no landmark path was found. */
  public void recordLandmarkFallback() {
    landmarkFallbacks.increment();
  }

  /** Legs that asked for a distant-landmark route and were served the shortest path. */
  public long landmarkFallbacks() {
    return landmarkFallbacks.sum();
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
   * Records a leg whose route has just been planned: counted here, and written to the per-leg file
   * when one is open.
   *
   * @param agent the agent that planned it
   * @param route the route it planned
   */
  public void recordPlannedRoute(Agent agent, Route route) {
    if (route == null) {
      return;
    }
    recordPlannedRoute(route.getLength());
    writeLeg(agent, route);
  }

  /**
   * Records the length of a planned leg, without the identity the per-leg file needs.
   *
   * @param meters the routed length, which is what will actually be walked
   */
  public void recordPlannedRoute(double meters) {
    if (meters > 0.0 && Double.isFinite(meters)) {
      plannedRouteMeters.add(meters);
      legsPlanned.increment();
      runPlannedRouteMeters.add(meters);
      runLegsPlanned.increment();
    } else {
      unusableRouteLengths.increment();
    }
  }

  /** Records a leg that has just finished, with the length actually covered. */
  public void recordWalkedRoute(double meters) {
    if (meters > 0.0 && Double.isFinite(meters)) {
      walkedRouteMeters.add(meters);
      runWalkedRouteMeters.add(meters);
    } else {
      unusableRouteLengths.increment();
    }
  }

  /**
   * Records how much of a finished leg was repetition: the edges it traversed against the distinct
   * edges among them.
   *
   * <p>A leg that walks its route once reports a ratio of 1.0. The two totals are kept separately
   * rather than as a running mean because the day's figure is the ratio of the sums, which a mean of
   * per-leg ratios does not give. {@code worstLegRevisit} is kept alongside because the distribution
   * is what matters: a day whose legs are almost all 1.0 and whose worst is 90 is a day with a
   * handful of agents walking in circles, and a mean near 1.3 hides that entirely.
   *
   * @param traversals edges the leg walked, counting repeats
   * @param distinctEdges distinct edges among them
   */
  public void recordWalkedEdges(int traversals, int distinctEdges) {
    if (traversals <= 0 || distinctEdges <= 0) {
      return;
    }
    edgeTraversals.add(traversals);
    distinctEdgesWalked.add(distinctEdges);
    worstLegRevisit.accumulate((double) traversals / distinctEdges);
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

  /** Metres of route planned over the whole job, across every day. */
  public double runPlannedRouteMeters() {
    return runPlannedRouteMeters.sum();
  }

  /** Metres walked over the whole job, across every day. */
  public double runWalkedRouteMeters() {
    return runWalkedRouteMeters.sum();
  }

  /** Legs planned over the whole job, across every day. */
  public long runLegsPlanned() {
    return runLegsPlanned.sum();
  }

  /** Legs planned so far today. */
  public long legsPlanned() {
    return legsPlanned.sum();
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

  /**
   * How many times over the day's finished legs walked their own streets: edges traversed divided by
   * distinct edges among them. 1.0 means every leg walked its route once.
   */
  public double revisitFactor() {
    long distinct = distinctEdgesWalked.sum();
    return distinct > 0 ? (double) edgeTraversals.sum() / distinct : 1.0;
  }

  /** The highest revisit factor any single leg reached today. */
  public double worstLegRevisit() {
    return worstLegRevisit.get();
  }

  /** Total band widenings so far today. */
  public long destinationWidenings() {
    return destinationWidenings.sum();
  }

  /** Total band fallbacks so far today. */
  public long destinationFallbacks() {
    return destinationFallbacks.sum();
  }

  // Job totals: not cleared by the daily reset below, so a multi-day run reports the run.
  private final java.util.concurrent.atomic.DoubleAdder runPlannedRouteMeters =
      new java.util.concurrent.atomic.DoubleAdder();
  private final java.util.concurrent.atomic.DoubleAdder runWalkedRouteMeters =
      new java.util.concurrent.atomic.DoubleAdder();
  private final java.util.concurrent.atomic.LongAdder runLegsPlanned =
      new java.util.concurrent.atomic.LongAdder();

  private final java.util.concurrent.atomic.LongAdder legsPlanned =
      new java.util.concurrent.atomic.LongAdder();

  private final LongAdder edgeTraversals = new LongAdder();
  private final LongAdder distinctEdgesWalked = new LongAdder();
  private final java.util.concurrent.atomic.DoubleAccumulator worstLegRevisit =
      new java.util.concurrent.atomic.DoubleAccumulator(Math::max, 1.0);

  /** Clears the day's ledgers and counters. */
  public void reset() {
    plannedRouteMeters.reset();
    legsPlanned.reset();
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
    edgeTraversals.reset();
    distinctEdgesWalked.reset();
    worstLegRevisit.reset();
  }

  // ---------------------------------------------------------------------------------------------
  // The per-leg record: one line per planned leg, when -Dpedsim.trace names a file.
  // ---------------------------------------------------------------------------------------------

  private static final Logger LOGGER = Logger.getLogger(RouteTrace.class.getName());
  private static final String TRACE_PROPERTY = "pedsim.trace";

  private static Writer writer;
  private static boolean writingLegs;

  /**
   * Opens the per-leg file if {@code -Dpedsim.trace} named one. Safe to call more than once; one
   * file serves every job of the run, which is why it is static where the counters are not.
   */
  public static synchronized void openLegFile() {
    if (writer != null || writingLegs) {
      return;
    }
    String target = System.getProperty(TRACE_PROPERTY);
    if (target == null || target.isBlank()) {
      return;
    }
    try {
      Path path = Path.of(target);
      if (path.getParent() != null) {
        Files.createDirectories(path.getParent());
      }
      writer =
          Files.newBufferedWriter(
              path,
              StandardCharsets.UTF_8,
              StandardOpenOption.CREATE,
              StandardOpenOption.TRUNCATE_EXISTING,
              StandardOpenOption.WRITE);
      writer.write("scenario,agentID,trip,origin,destination,nodes,edges,length\n");
      writingLegs = true;
      LOGGER.info("route trace enabled: " + path.toAbsolutePath());
    } catch (IOException e) {
      throw new UncheckedIOException("could not open the route trace at " + target, e);
    }
  }

  /** Whether a per-leg file is open. */
  public static boolean writingLegs() {
    return writingLegs;
  }

  private static synchronized void writeLeg(Agent agent, Route route) {
    if (!writingLegs || agent == null) {
      return;
    }
    Object scenario = agent.getAgentScenario();
    int nodes = route.nodesSequence == null ? 0 : route.nodesSequence.size();
    int edges = route.directedEdgesSequence == null ? 0 : route.directedEdgesSequence.size();
    try {
      writer.write(
          String.format(
              "%s,%d,%d,%s,%s,%d,%d,%.3f%n",
              scenario == null ? "DEFAULT" : scenario.toString(),
              agent.agentID,
              agent.getTripsDone(),
              endpoint(route, true),
              endpoint(route, false),
              nodes,
              edges,
              route.getLength()));
    } catch (IOException e) {
      throw new UncheckedIOException("could not write to the route trace", e);
    }
  }

  private static String endpoint(Route route, boolean first) {
    if (route.nodesSequence == null || route.nodesSequence.isEmpty()) {
      return "NA";
    }
    NodeGraph node = first ? route.nodesSequence.get(0) : route.nodesSequence.getLast();
    return node == null ? "NA" : String.valueOf(node.getID());
  }

  /** Flushes and closes the per-leg file, if one is open. */
  public static synchronized void closeLegFile() {
    if (writer == null) {
      return;
    }
    try {
      writer.flush();
      writer.close();
    } catch (IOException e) {
      LOGGER.warning("could not close the route trace: " + e.getMessage());
    } finally {
      writer = null;
      writingLegs = false;
    }
  }
}
