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
import pedsim.core.utilities.LoggerUtil;
import sim.util.geo.Utilities;

/**
 * The AgentReleaseManager class handles the release of agents for the
 * pedestrian simulation, distributing the total expected walking distance for
 * agents during a given time period.
 */
public class AgentReleaseManager implements AutoCloseable {

  protected static final Logger logger = LoggerUtil.getLogger();
  protected LocalDateTime currentTime;
  protected Random random = new Random();

  protected PedSimCity state;
  protected double metersToWalkCurrentDay;
  protected double expectedMetersWalkedSoFarToday;
  protected double metersWalkedSoFarToday;
  private final int dayNumber;
  private String logFilePath;
  private PrintWriter logWriter = null;

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
    resetMetersWalkedSoFar();
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
            * TimePars.computeTimeStepShare(currentTime)
            * state.releaseBudgetMultiplier(currentTime);

    int agentsReleased = 0;
    if (metersToAllocate > 0) {
      agentsReleased = releaseAgentsMeters(metersToAllocate);
    }

    logRelease(steps, metersToAllocate, metersToAllocate, agentsReleased);

    if (currentTime.getMinute() == 0) {
      logWalkingAgents();
    }

    expectedMetersWalkedSoFarToday += metersToAllocate;
  }

  /**
   * Releases a set of agents to walk a specific distance, based on the kilometers
   * to allocate. The number of agents to release is calculated based on the
   * expected distance and the average trip distance. After selecting the agents,
   * the distance is allocated to them and their activities are updated.
   *
   * @param metersToAllocate the total meters to be allocated for the selected
   *                         agents to walk.
   * @return the number of agents released.
   */
  private int releaseAgentsMeters(double metersToAllocate) {

    int agentsExpectedToWalk =
        Math.max(1, (int) (metersToAllocate / RouteChoicePars.avgTripDistance));

    Set<Agent> agentsAtHome = new HashSet<>(state.agentsAtHome);
    Set<Agent> agentsToRelease = selectRandomAgents(agentsAtHome, agentsExpectedToWalk);

    allocateMetersAcrossAgents(agentsToRelease);

    for (Agent agent : agentsToRelease) {
      agent.startWalkingAlone();
    }

    return agentsToRelease.size();
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
   * Allocates the specified walking distance across a set of agents using
   * parallel processing. Each agent gets a random variability applied to the
   * allocated distance, ensuring they stay within defined minimum and maximum
   * limits.
   *
   * @param agentSet the set of agents to which the distance will be allocated.
   */
  private void allocateMetersAcrossAgents(Set<Agent> agentSet) {

    agentSet.parallelStream()
        .forEach(agent -> agent.setDistanceNextDestination(sampleTripMeters()));
  }

  /**
   * Draws a trip distance around the average, clamped to the allowed interval. Draws rejected by
   * the module's {@link PedSimCity#acceptTripDistance} filter (e.g. a walk-share filter) are
   * resampled a bounded number of times; the last draw stands if the filter keeps rejecting.
   */
  private double sampleTripMeters() {
    double metersToWalk = RouteChoicePars.avgTripDistance;
    for (int attempt = 0; attempt < 20; attempt++) {
      double variabilityFactor = Utilities.fromDistribution(1.00, 0.30, null);
      metersToWalk = RouteChoicePars.avgTripDistance * variabilityFactor;

      if (metersToWalk < RouteChoicePars.minTripDistance) {
        metersToWalk = RouteChoicePars.minTripDistance;
      } else if (metersToWalk > RouteChoicePars.maxTripDistance) {
        metersToWalk = RouteChoicePars.maxTripDistance;
      }

      if (state.acceptTripDistance(metersToWalk)) {
        return metersToWalk;
      }
    }
    return metersToWalk;
  }

  /**
   * Selects a specified number of agents randomly, with a weighted probability towards agents that
   * have walked less distance, each pick additionally gated by the module's
   * {@link PedSimCity#releaseCandidateWeight} (e.g. persona × hour affinity). When the gate starves
   * the selection, the remainder is filled ignoring it so the release budget is still honoured.
   *
   * @param homeAgents the set of home agents to select from.
   * @param nrAgents   the number of agents to select.
   * @return a set of randomly selected agents.
   */
  private Set<Agent> selectRandomAgents(Set<Agent> homeAgents, int nrAgents) {

    if (nrAgents >= homeAgents.size()) {
      return homeAgents;
    }

    List<Agent> agents = new ArrayList<>(homeAgents);
    agents.sort(Comparator.comparingDouble(Agent::getTotalMetersWalked));

    int hour = currentTime != null ? currentTime.getHour() : 0;
    int target = Math.min(nrAgents, agents.size());
    Set<Agent> selectedAgents = new HashSet<>();

    int attempts = 0;
    int maxAttempts = Math.max(100, agents.size() * 20);
    while (selectedAgents.size() < target && attempts < maxAttempts) {
      attempts++;
      int weightedIndex = (int) (Math.pow(random.nextDouble(), 1.5) * agents.size());
      Agent candidate = agents.get(weightedIndex);
      double weight = state.releaseCandidateWeight(candidate, hour);
      if (weight < 1.0 && random.nextDouble() >= weight) {
        continue;
      }
      selectedAgents.add(candidate);
    }
    while (selectedAgents.size() < target) {
      int weightedIndex = (int) (Math.pow(random.nextDouble(), 1.5) * agents.size());
      selectedAgents.add(agents.get(weightedIndex));
    }

    return selectedAgents;
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
      logWriter.println("step,datetime,meters_to_allocate,meters_adjusted,agents_released");
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
    if (logWriter != null) {
      logWriter.flush();
      logWriter.close();
      logWriter = null;
    }
  }
}
