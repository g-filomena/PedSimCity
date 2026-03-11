package pedsim.core.engine;

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

  /**
   * Constructor for AgentReleaseManager.
   * 
   * @param state the PedSimCity instance representing the simulation state.
   * @param kmCurrentDay the current expected walking distance for the day (in meters).
   */
  public AgentReleaseManager(PedSimCity state, Double metersToWalkCurrentDay) {
    this.state = state;
    this.metersToWalkCurrentDay = metersToWalkCurrentDay;
    resetMetersWalkedSoFar();
    expectedMetersWalkedSoFarToday = 0.0;
    metersWalkedSoFarToday = 0.0;
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
        (metersToAllocate + (expectedMetersWalkedSoFarToday - metersWalkedSoFarToday)) * 0.5; // to
                                                                                              // account
                                                                                              // for
    // return home
    if (metersAdjusted > 0) {
      releaseAgentsMeters(metersAdjusted);
    }
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
  private void releaseAgentsMeters(double metersToAllocate) {

    int agentsExpectedToWalk =
        Math.max(1, (int) (metersToAllocate / RouteChoicePars.avgTripDistance));

    Set<Agent> agentsAtHome = new HashSet<>(state.agentsAtHome);// least one
    // agent
    Set<Agent> agentsToRelease = selectRandomAgents(agentsAtHome, agentsExpectedToWalk);
    allocateMetersAcrossAgents(agentsToRelease); // Allocate km accordingly

    for (Agent agent : agentsToRelease) {
      agent.startWalkingAlone();
    }
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

    List<Agent> agents = new ArrayList<>(homeAgents);
    // Sort agents by kmWalked in ascending order (less walked first) using parallel
    // sort
    agents.parallelStream().sorted(Comparator.comparingDouble(Agent::getTotalMetersWalked))
        .collect(Collectors.toList());

    Set<Agent> selectedAgents = agents.parallelStream().limit(nrAgents) // Select only the first
                                                                        // 'nrAgents' after
                                                                        // sorting
        .map(agent -> {
          // Weighted selection: lower km-Walked has a higher probability
          int weightedIndex = (int) (Math.pow(random.nextDouble(), 1.5) * agents.size());
          return agents.get(weightedIndex);
        }).collect(Collectors.toSet());

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

}
