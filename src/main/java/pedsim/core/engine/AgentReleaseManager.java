package pedsim.core.engine;

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
import java.util.stream.Collectors;

import pedsim.core.agents.Agent;
import pedsim.core.parameters.RouteChoicePars;
import pedsim.core.parameters.TimePars;
import pedsim.core.utilities.LoggerUtil;
import sim.util.geo.Utilities;

/**
 * The AgentReleaseManager class handles the release of agents for the pedestrian simulation,
 * distributing the total expected walking distance for agents during a given time period.
 */
public class AgentReleaseManager {

  protected static final Logger logger = LoggerUtil.getLogger();
  protected LocalDateTime currentTime;
  protected Random random = new Random();

  protected PedSimCity state;
  protected double metersToWalkCurrentDay;
  protected double expectedMetersWalkedSoFarToday;
  protected double metersWalkedSoFarToday;
  private final int dayNumber;
  private String logFilePath;

     /**
   * Constructor for AgentReleaseManager.
   * 
   * @param state the PedSimCity instance representing the simulation state.
   * @param metersToWalkCurrentDay the current expected walking distance for the day (in meters).
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
   * Releases agents to start walking based on the calculated walking distances for the day.
   *
   * @param steps the current simulation step count.
   */
  public void releaseAgents(double steps) {

    currentTime = TimePars.getTime(steps);
    metersWalkedSoFarToday = computeMetersWalkedSoFar();
    double metersToAllocate = (metersToWalkCurrentDay * TimePars.computeTimeStepShare(currentTime));
    double metersAdjusted =
        (metersToAllocate + (expectedMetersWalkedSoFarToday - metersWalkedSoFarToday)) * 0.5;

    int agentsReleased = 0;
    if (metersAdjusted > 0) {
      agentsReleased = releaseAgentsMeters(metersAdjusted);
    }
    logRelease(steps, metersToAllocate, metersAdjusted, agentsReleased);
    if (currentTime.getMinute() == 0) { // Log walking agents every full hour
      logWalkingAgents();
    }
    expectedMetersWalkedSoFarToday += metersToAllocate;
  }

  /**
   * Releases a set of agents to walk a specific distance, based on the kilometers to allocate. The
   * number of agents to release is calculated based on the expected distance and the average trip
   * distance. After selecting the agents, the distance is allocated to them and their activities
   * are updated.
   *
   * @param kmToAllocate the total kilometres to be allocated for the selected agents to walk.
   */
  private int releaseAgentsMeters(double metersToAllocate) {

    int agentsExpectedToWalk =
        Math.max(1, (int) (metersToAllocate / RouteChoicePars.avgTripDistance));

    Set<Agent> agentsAtHome = new HashSet<>(state.agentsAtHome);// least one
    // agent
    Set<Agent> agentsToRelease = selectRandomAgents(agentsAtHome, agentsExpectedToWalk);
    allocateMetersAcrossAgents(agentsToRelease); // Allocate km accordingly

    for (Agent agent : agentsToRelease) {
      agent.startWalkingAlone();
    }
    return agentsToRelease.size();
  }

  /**
   * Logs the current walking agent statistics, including the number of agents walking, expected
   * versus walked Kilometres, and whether it is night or not.
   */
  private void logWalkingAgents() {
    logger.info(String.format(
        "TIME: %02d:%02d | Agents walking: %d | Expected Km walked till this time: %.1f vs KM Walked today: %.1f",
        currentTime.getHour(), currentTime.getMinute(), state.agentsWalking.size(),
        expectedMetersWalkedSoFarToday / 1000, metersWalkedSoFarToday / 1000));
  }

  // private int determineNrAgentsToRelease(int expectedPedestrians, Set<Agent> agentsWalking) {
  //
  // double timeStepWeight = computeTimeStepWeight(); // Adjusted based on the time of day
  // // Ensure the result is non-negative
  // return Math.max((int) (expectedPedestrians / timeStepWeight) - agentsWalking.size(), 0);
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
   * Allocates the specified walking distance across a set of agents using parallel processing. Each
   * agent gets a random variability applied to the allocated distance, ensuring they stay within
   * defined minimum and maximum limits.
   *
   * @param agentSet the set of agents to which the distance will be allocated.
   * @param kmToAllocate the total kilometers to be allocated to agents.
   */
  private void allocateMetersAcrossAgents(Set<Agent> agentSet) {

    // Apply randomisation for variability (+/- 30%) using parallelStream
    agentSet.parallelStream().forEach(agent -> {
      double variabilityFactor = Utilities.fromDistribution(1.00, 0.30, null); // Variability (+/-
                                                                               // 30%)
      double metersToWalk = RouteChoicePars.avgTripDistance * variabilityFactor;

      // Ensure kmToWalk is within the defined boundaries
      if (metersToWalk < RouteChoicePars.minTripDistance) {
        metersToWalk = RouteChoicePars.minTripDistance;
      } else if (metersToWalk > RouteChoicePars.maxTripDistance) {
        metersToWalk = RouteChoicePars.maxTripDistance;
      }

      // This guides the selection of the destination for the agent, divided by two as
      // the agent will go back home
      agent.setDistanceNextDestination(metersToWalk);
    });
  }

  /**
   * Selects a specified number of agents randomly, with a weighted probability towards agents that
   * have walked less distance. The selection is done using parallel streams to improve performance.
   *
   * @param homeAgents the set of home agents to select from.
   * @param nrAgents the number of agents to select.
   * @return a set of randomly selected agents.
   */
  private Set<Agent> selectRandomAgents(Set<Agent> homeAgents, int nrAgents) {

    if (nrAgents >= homeAgents.size()) {
      return homeAgents;
    }

    // Sort agents by metres walked ascending (least-walked first), then apply weighted selection
    List<Agent> agents = new ArrayList<>(homeAgents);
    agents.sort(Comparator.comparingDouble(Agent::getTotalMetersWalked));

    Set<Agent> selectedAgents = new HashSet<>();
    while (selectedAgents.size() < Math.min(nrAgents, agents.size())) {
      // Weighted random: lower index (less walked) more likely to be chosen
      int weightedIndex = (int) (Math.pow(random.nextDouble(), 1.5) * agents.size());
      selectedAgents.add(agents.get(weightedIndex));
    }

    return selectedAgents;
  }

  /**
   * Computes the total kilometres walked by all agents in the simulation up to the current time.
   *
   * @return the total kilometres walked by all agents.
   */
  private double computeMetersWalkedSoFar() {
    return state.agentsList.stream().mapToDouble(Agent::getMetersWalkedDay).sum();
  }

  /**
   * Resets the kmWalkedDay attribute for all agents in the simulation to zero.
   */
  private void resetMetersWalkedSoFar() {
    state.agentsList.forEach(agent -> agent.metersWalkedDay = 0.0);
  }

  private void initLogFile() {
    try {
      File dir = new File("output");
      if (!dir.exists()) {
        dir.mkdirs();
      }
      logFilePath = "output/agent_release_day_" + dayNumber + ".csv";
      try (PrintWriter out = new PrintWriter(new FileWriter(logFilePath, false))) {
        out.println("step,datetime,meters_to_allocate,meters_adjusted,agents_released");
      }
    } catch (IOException e) {
      logger.warning("Could not initialise agent release log file: " + e.getMessage());
    }
  }

  private void logRelease(double step, double metersToAllocate, double metersAdjusted,
      int agentsReleased) {
    if (logFilePath == null) {
      return;
    }
    try (PrintWriter out = new PrintWriter(new FileWriter(logFilePath, true))) {
      out.printf("%f,%s,%.4f,%.4f,%d%n", step, currentTime, metersToAllocate, metersAdjusted,
          agentsReleased);
    } catch (IOException e) {
      logger.warning("Could not write agent release log entry: " + e.getMessage());
    }
  }

}
