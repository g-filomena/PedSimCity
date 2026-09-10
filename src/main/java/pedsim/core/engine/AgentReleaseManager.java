package pedsim.core.engine;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.logging.Logger;
import pedsim.core.agents.Agent;
import pedsim.core.parameters.RouteChoicePars;
import pedsim.core.parameters.TimePars;
import pedsim.core.parameters.TripDistanceBands;
import pedsim.core.utilities.LoggerUtil;

/**
 * The AgentReleaseManager class handles the release of agents for the
 * pedestrian simulation, distributing the total expected walking distance for
 * agents during a given time period.
 */
public class AgentReleaseManager implements AutoCloseable {

  protected static final Logger logger = LoggerUtil.getLogger();
  protected LocalDateTime currentTime;
  /** Seeded from the model's seed and the day, so a release schedule is repeatable. */
  protected Random random;

  protected PedSimCity state;
  protected double metersToWalkCurrentDay;
  protected double expectedMetersWalkedSoFarToday;
  protected double metersWalkedSoFarToday;
  private final int dayNumber;
  private String logFilePath;
  private PrintWriter logWriter = null;

  /**
   * Running balance of metres carried between release events. Sizing a release by
   * {@code budget / mean} discarded the fractional remainder every time and, once the walk-share
   * filter reshaped the draws, spent meters the count had not been derived from. Carrying the
   * balance instead makes the day's total exact by construction, whatever the trip distribution
   * and filter do to the realised mean.
   */
  private double residualMeters = 0.0;

  /** Metres actually committed by the last release, for the log. */
  private double lastSpentMeters = 0.0;

  /**
   * Constructor for AgentReleaseManager.
   *
   * @param state                  the PedSimCity instance representing the
   *                               simulation state.
   * @param metersToWalkCurrentDay the current expected walking distance for the
   *                               day (in meters).
   * @param dayNumber              the current simulated day number.
   */
  public AgentReleaseManager(PedSimCity state, Double metersToWalkCurrentDay, int dayNumber) {
    this.state = state;
    this.metersToWalkCurrentDay = metersToWalkCurrentDay;
    this.dayNumber = dayNumber;
    random = new Random(state.seed() * 7919L + dayNumber);
    resetMetersWalkedSoFar();
    state.resetPlannedRouteMeters();
    expectedMetersWalkedSoFarToday = 0.0;
    metersWalkedSoFarToday = 0.0;
    initLogFile();
  }

  /**
   * Releases agents to start walking based on the calculated walking distances
   * for the day.
   *
   * @param steps the current simulation step count.
   */
  public void releaseAgents(double steps) {

    currentTime = TimePars.getTime(steps);

    // Module hook: a module may take over this release event entirely (e.g. paired releases for
    // an A/B experiment); the standard meters-based release is then skipped.
    int overrideReleased = state.releaseAgentsOverride(steps, dayNumber);
    if (overrideReleased >= 0) {
      if (overrideReleased > 0) {
        double meters = RouteChoicePars.avgTripDistance * overrideReleased;
        logRelease(steps, meters, meters, overrideReleased);
      }
      return;
    }

    metersWalkedSoFarToday = computeMetersWalkedSoFar();
    double metersToAllocate =
        metersToWalkCurrentDay
            * state.departureShare(currentTime)
            * state.releaseBudgetMultiplier(currentTime);

    int agentsReleased = 0;
    lastSpentMeters = 0.0;
    if (metersToAllocate > 0) {
      agentsReleased = releaseAgentsMeters(metersToAllocate);
    }

    logRelease(steps, metersToAllocate, lastSpentMeters, agentsReleased);

    if (currentTime.getMinute() == 0) {
      logWalkingAgents();
    }

    expectedMetersWalkedSoFarToday += metersToAllocate;
  }

  /**
   * Releases agents to spend the allocated metres, drawing them one at a time and subtracting each
   * sampled trip from the budget until it is exhausted. Sizing the release this way rather than as
   * {@code metres / avgTripDistance} keeps the budget honoured whatever the trip-distance
   * distribution is, and drops the {@code Math.max(1, ...)} floor the count-first version needed:
   * that floor released one agent at every event regardless of the diurnal curve, injecting a
   * constant 72 agents a day into the hours where the curve asks for almost none.
   *
   * @param metersToAllocate the metres allocated to this release event.
   * @return the number of agents released.
   */
  private int releaseAgentsMeters(double metersToAllocate) {

    double budget = metersToAllocate + residualMeters;

    List<Agent> candidates = new ArrayList<>(state.agentsAtHome);
    if (candidates.isEmpty()) {
      lastSpentMeters = 0.0;
      residualMeters =
          capResidual(budget, RouteChoicePars.maxTripDistance * state.expectedTourLegs(null));
      return 0;
    }
    candidates.sort(Comparator.comparingDouble(Agent::getTotalMetersWalked));

    int hour = currentTime != null ? currentTime.getHour() : 0;
    TripDistanceBands.Band band = TripDistanceBands.bandFor(hour);

    Set<Agent> released = new HashSet<>();
    int attempts = 0;
    int maxAttempts = Math.max(100, candidates.size() * 20);

    while (budget > 0 && released.size() < candidates.size() && attempts < maxAttempts) {
      attempts++;

      // Weighted towards agents that have walked least, then gated by the module's persona x hour
      // affinity.
      int weightedIndex = (int) (Math.pow(random.nextDouble(), 1.5) * candidates.size());
      Agent candidate = candidates.get(weightedIndex);
      if (released.contains(candidate)) {
        continue;
      }
      double weight = state.releaseCandidateWeight(candidate, hour);
      if (weight < 1.0 && random.nextDouble() >= weight) {
        continue;
      }

      double meters = sampleTripMeters(band);
      candidate.setDistanceNextDestination(meters);
      released.add(candidate);
      // The sampled distance drives where this agent goes, so it stays the leg length. The budget
      // is charged for the whole tour the release sets in motion: the agent walks out, chains
      // through its agenda and walks home, and every one of those metres is walked on the network
      // that metersPerDayPerPerson is meant to account for.
      budget -= meters * state.expectedTourLegs(candidate);
    }

    lastSpentMeters = metersToAllocate + residualMeters - budget;
    // One tour's worth, for the same reason: the residual exists so an event too poor to afford
    // the next release hands its metres to the following one, and the unit being afforded is a
    // tour.
    residualMeters =
        capResidual(budget, TripDistanceBands.max(band) * state.expectedTourLegs(null));

    for (Agent agent : released) {
      agent.startWalkingAlone();
    }

    return released.size();
  }

  /**
   * Bounds the running balance. An overshoot is negative and at most one trip by construction, so it
   * carries in full and the next event pays it back. Unspent metres carry only up to one trip's
   * worth: when the persona gate starves a release there is no deferred walking demand to represent,
   * and an uncapped balance would accumulate through the quiet hours and discharge as a spike when
   * the gate reopens.
   *
   * @param balance the metres left over from this release event.
   * @param oneTrip the band's maximum trip distance.
   */
  private double capResidual(double balance, double oneTrip) {
    return balance > oneTrip ? oneTrip : balance;
  }

  /**
   * Logs the current walking agent statistics, including the number of agents
   * walking, expected versus walked kilometres.
   */
  private void logWalkingAgents() {
    logger.info(
        String.format(
            "TIME: %02d:%02d | Agents walking: %d | Expected Km walked till this time: %.1f vs KM"
                + " Walked today: %.1f",
            currentTime.getHour(),
            currentTime.getMinute(),
            state.agentsWalking.size(),
            expectedMetersWalkedSoFarToday / 1000,
            metersWalkedSoFarToday / 1000));
  }

  // private int determineNrAgentsToRelease(int expectedPedestrians, Set<Agent>
  // agentsWalking) {
  //
  // double timeStepWeight = computeTimeStepWeight(); // Adjusted based on the
  // time of day
  // // Ensure the result is non-negative
  // return Math.max((int) (expectedPedestrians / timeStepWeight) -
  // agentsWalking.size(), 0);
  // }
  //
  // private int calculateActivePedestrians() {
  // if (isPeakHours())
  // return (int) (TimePars.peakPercentage * Pars.numAgents);
  // else if (isOffPeakHours())
  // return (int) (TimePars.offPeakPercentage * Pars.numAgents);
  // else
  // return (int) (TimePars.nightPercentage * Pars.numAgents);
  // }

  /**
   * Draws a trip distance for the band, resampling a bounded number of times while the module's
   * {@link PedSimCity#tripAcceptanceProbability} filter (e.g. a walk-share logit) rejects the draw; the
   * last draw stands if the filter keeps rejecting. The budget is spent against whatever comes out,
   * so a filter that biases the realised mean no longer desynchronises it from the release size.
   */
  private double sampleTripMeters(TripDistanceBands.Band band) {
    double metersToWalk = TripDistanceBands.sample(band, random.nextDouble());
    for (int attempt = 0; attempt < 20; attempt++) {
      if (random.nextDouble() < state.tripAcceptanceProbability(metersToWalk)) {
        return metersToWalk;
      }
      metersToWalk = TripDistanceBands.sample(band, random.nextDouble());
    }
    return metersToWalk;
  }

  /**
   * Computes the total meters walked by all agents in the simulation up to the
   * current time.
   *
   * @return the total meters walked by all agents.
   */
  private double computeMetersWalkedSoFar() {
    return state.agentsList.stream().mapToDouble(Agent::getMetersWalkedDay).sum();
  }

  /**
   * Resets the metersWalkedDay attribute for all agents in the simulation to
   * zero.
   */
  private void resetMetersWalkedSoFar() {
    state.agentsList.forEach(agent -> agent.metersWalkedDay = 0.0);
  }

  private void initLogFile() {
    try {
      File dir = new File("outputs");
      if (!dir.exists() && !dir.mkdirs()) {
        logger.warning("Could not create outputs directory for agent release log.");
        return;
      }

      logFilePath = "outputs/agent_release_day_" + dayNumber + ".csv";

      logWriter = new PrintWriter(new BufferedWriter(new FileWriter(logFilePath, false)));
      logWriter.println("step,datetime,meters_to_allocate,meters_spent,agents_released");
      logWriter.flush();

      if (logWriter.checkError()) {
        logger.warning("Could not write agent release log header.");
      }

    } catch (IOException e) {
      logger.warning("Could not initialise agent release log file: " + e.getMessage());
    }
  }

  private void logRelease(
      double step, double metersToAllocate, double metersAdjusted, int agentsReleased) {
    if (logWriter == null) {
      return;
    }

    logWriter.printf(
        "%f,%s,%.4f,%.4f,%d%n",
        step, currentTime, metersToAllocate, metersAdjusted, agentsReleased);

    logWriter.flush();

    if (logWriter.checkError()) {
      logger.warning("Could not write agent release log entry.");
    }
  }

  @Override
  public void close() {
    logger.info(
        String.format(
            "Day %d: planned %.0f m, walked %.0f m on completed legs, %d band widenings, "
                + "%d fallbacks to any node",
            dayNumber,
            state.plannedRouteMeters(),
            state.walkedRouteMeters(),
            state.destinationWidenings(),
            state.destinationFallbacks()));
    if (logWriter != null) {
      logWriter.flush();
      logWriter.close();
      logWriter = null;
    }
  }
}
