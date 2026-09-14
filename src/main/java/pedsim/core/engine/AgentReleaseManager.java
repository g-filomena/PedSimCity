package pedsim.core.engine;

import ec.util.MersenneTwisterFast;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.logging.Logger;
import pedsim.core.agents.Agent;
import pedsim.core.parameters.Pars;
import pedsim.core.parameters.TimePars;
import pedsim.core.utilities.LoggerUtil;

/**
 * Sends agents out to walk: the departures a day's travel demand says are due now.
 *
 * <p>There is one release mechanism, for every module: a count of departures. Each module states
 * how much travel its population makes as a number of departures per person per day, spread over the
 * day by {@link TravelDemand#departureShare}, and the metres fall out of where those people choose
 * to go. No distance is consulted here, and no budget is spent.
 */
public class AgentReleaseManager implements AutoCloseable {

  protected static final Logger logger = LoggerUtil.getLogger();
  protected LocalDateTime currentTime;
  /** Seeded from the model's seed and the day, so a release schedule is repeatable. */
  protected MersenneTwisterFast random;

  protected PedSimCity state;

  /**
   * Who goes out, when, and by what mode. Resolved once: the release manager is built per day, and
   * the demand object caches a departure profile that is itself rebuilt per day.
   */
  protected final TravelDemand demand;

  /** Measurement only: what the population has walked so far today, for the log. */
  protected double metersWalkedSoFarToday;
  private final int dayNumber;
  private String logFilePath;
  private PrintWriter logWriter = null;

  /**
   * Constructor for AgentReleaseManager.
   *
   * @param state     the PedSimCity instance representing the simulation state.
   * @param dayNumber the current simulated day number.
   */
  public AgentReleaseManager(PedSimCity state, int dayNumber) {
    this.state = state;
    this.demand = state.travelDemand();
    this.dayNumber = dayNumber;
    random = new MersenneTwisterFast(state.seed() * 7919L + dayNumber);
    resetMetersWalkedSoFar();
    state.ledger().reset();
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
    // an A/B experiment); the standard release is then skipped.
    int overrideReleased = demand.releaseAgentsOverride(steps, dayNumber);
    if (overrideReleased >= 0) {
      if (overrideReleased > 0) {
        logRelease(steps, overrideReleased);
      }
      return;
    }

    metersWalkedSoFarToday = computeMetersWalkedSoFar();

    // Scheduled departures first. People with somewhere they have to be are not a matter of
    // chance, and the unscheduled count below has already had their travel subtracted.
    int agentsReleased = releaseScheduled();
    double departuresPerPerson = demand.unscheduledDeparturesPerPerson(currentTime);
    if (departuresPerPerson > 0.0) {
      agentsReleased += releaseAgentsByCount(departuresPerPerson);
    }

    logRelease(steps, agentsReleased);

    if (currentTime.getMinute() == 0) {
      logWalkingAgents();
    }
  }

  /**
   * Sends out the agents whose scheduled departure falls in this release event.
   *
   * <p>No weighting, no budget, no filter: these agents are at home and it is time to go. Having a
   * job means going to it, so a commute is generated rather than drawn; what is left to chance is
   * the discretionary travel around it.
   *
   * @return the number of agents released
   */
  private int releaseScheduled() {
    int released = 0;
    for (Agent agent : demand.scheduledDepartures(currentTime)) {
      agent.startWalkingAlone();
      released++;
    }
    return released;
  }

  /** Fractional part of a release event's agent count, carried to the next event. */
  private double residualAgents = 0.0;

  /**
   * Releases the agents the day's activity pattern says should set off now.
   *
   * <p>The count is the population's daily departures spread over the day by the departure share.
   * Fractions are carried rather than rounded away: a share that asks for 0.4 agents at every one
   * of seventy-two events is asking for twenty-nine agents over the day, not zero.
   *
   * <p>Nothing here consults a distance. How far these people walk is settled where they choose
   * where to go, and the day's metres are an outcome to be compared against
   * {@code metersPerDayPerPerson} rather than a budget arranged to match it.
   *
   * @param departuresPerPerson unscheduled departures one person makes on an average day
   * @return the number of agents released
   */
  private int releaseAgentsByCount(double departuresPerPerson) {
    double wanted =
        Pars.numAgents
                * departuresPerPerson
                * demand.departureShare(currentTime)
                * demand.releaseBudgetMultiplier(currentTime)
            + residualAgents;
    int toRelease = (int) Math.floor(wanted);
    residualAgents = wanted - toRelease;
    if (toRelease <= 0) {
      return 0;
    }

    // Ordered by agent ID before anything draws from it. state.agentsAtHome is a
    // ConcurrentHashMap key set and Agent overrides no hashCode, so it iterates in identity-hash
    // order - which HotSpot derives from a per-JVM generator. Copying it into a list and then
    // indexing that list by position meant the same seed drew the same index into a differently
    // ordered list, and released a different agent, on a different machine. The draw below is
    // uniform over the list, so imposing an order changes no distribution; it only fixes which
    // agent sits at each index.
    List<Agent> candidates = new ArrayList<>(state.agentsAtHome);
    if (candidates.isEmpty()) {
      return 0;
    }
    candidates.sort(Comparator.comparingInt(candidate -> candidate.agentID));

    int hour = currentTime != null ? currentTime.getHour() : 0;
    // Insertion-ordered, so the order agents are started - and therefore the order they enter the
    // schedule and plan their routes - follows the draw order rather than identity-hash order.
    Set<Agent> released = new LinkedHashSet<>();
    int attempts = 0;
    int maxAttempts = Math.max(100, candidates.size() * 20);

    while (released.size() < toRelease
        && released.size() < candidates.size()
        && attempts < maxAttempts) {
      attempts++;
      // Uniform among the agents at home, which leaves the per-agent trip count binomial - the
      // no-information baseline. Real walking is concentrated on fewer people than that, but nothing
      // measured says by how much; a propensity, once the data support one, belongs in
      // releaseCandidateWeight below rather than in a bias applied here.
      Agent candidate = candidates.get(random.nextInt(candidates.size()));
      if (released.contains(candidate)) {
        continue;
      }
      double weight = demand.releaseCandidateWeight(candidate, hour);
      if (weight < 1.0 && random.nextDouble() >= weight) {
        continue;
      }
      released.add(candidate);
    }

    for (Agent agent : released) {
      agent.startWalkingAlone();
    }
    return released.size();
  }

  /**
   * Logs how many agents are walking and how far the population has walked today.
   *
   * <p>Reported rather than compared: the day's metres are whatever the day's destinations turn out
   * to be, so there is no hourly expectation to print them against.
   */
  private void logWalkingAgents() {
    logger.info(
        String.format(
            "TIME: %02d:%02d | Agents walking: %d | KM walked today: %.1f",
            currentTime.getHour(),
            currentTime.getMinute(),
            state.agentsWalking.size(),
            metersWalkedSoFarToday / 1000));
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
      logWriter.println("step,datetime,agents_released");
      logWriter.flush();

      if (logWriter.checkError()) {
        logger.warning("Could not write agent release log header.");
      }

    } catch (IOException e) {
      logger.warning("Could not initialise agent release log file: " + e.getMessage());
    }
  }

  private void logRelease(double step, int agentsReleased) {
    if (logWriter == null) {
      return;
    }

    logWriter.printf("%f,%s,%d%n", step, currentTime, agentsReleased);

    logWriter.flush();

    if (logWriter.checkError()) {
      logger.warning("Could not write agent release log entry.");
    }
  }

  @Override
  public void close() {
    logger.info(
        String.format(
            "Day %d: planned %.0f m, walked %.0f m on completed legs "
                + "(%d route lengths unusable), %d band widenings, "
                + "%d fallbacks to any node, %d/%d angular routes served as shortest path "
                + "(%d no dual path, %d trimmed away, %d with an unknown dual endpoint); "
                + "%d routes found only beyond the agent's known network (%d angular); "
                + "%d known networks left in pieces",
            dayNumber,
            state.ledger().plannedRouteMeters(),
            state.ledger().walkedRouteMeters(),
            state.ledger().unusableRouteLengths(),
            state.ledger().destinationWidenings(),
            state.ledger().destinationFallbacks(),
            state.ledger().angularFallbacks(),
            state.ledger().angularAttempts(),
            state.ledger().angularNoDualPath(),
            state.ledger().angularTrimmedAway(),
            state.ledger().angularEndpointUnknown(),
            state.ledger().fullNetworkEscalations(),
            state.ledger().fullNetworkEscalationsAngular(),
            sim.graph.Islands.incompleteMerges()));
    if (logWriter != null) {
      logWriter.flush();
      logWriter.close();
      logWriter = null;
    }
  }
}
