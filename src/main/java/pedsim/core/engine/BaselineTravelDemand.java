package pedsim.core.engine;

import java.time.LocalDateTime;
import pedsim.core.agents.Agent;
import pedsim.core.parameters.Pars;
import pedsim.core.parameters.TimePars;

/**
 * Core's travel demand: a tuned diurnal curve, a flat departure rate, and everyone walks.
 *
 * <p>Core models no reason for anyone to leave home. It therefore has nothing to say about who
 * departs and when beyond a rate and a curve, and no mode split, so every journey is walked.
 * Modules that model why people travel replace these; see {@code ActivityTravelDemand}.
 *
 * <p>This used to fall back on a metres budget instead, which made core the only part of the model
 * with a trip-distance parameter and gave {@code metersPerDayPerPerson} two jobs at once - a budget
 * here, a statistic to check a finished run against everywhere else. Core now releases the way
 * every module does.
 */
public class BaselineTravelDemand implements TravelDemand {

  protected final PedSimCity state;

  public BaselineTravelDemand(PedSimCity state) {
    this.state = state;
  }

  @Override
  public double departureShare(LocalDateTime time) {
    return TimePars.computeTimeStepShare(time);
  }

  /** Core models no reason anyone has to be anywhere, so nothing is scheduled. */
  @Override
  public java.util.List<Agent> scheduledDepartures(LocalDateTime time) {
    return java.util.Collections.emptyList();
  }

  /**
   * A flat rate: core knows nothing about who these people are, so it cannot say that one departs
   * more often than another.
   */
  @Override
  public double unscheduledDeparturesPerPerson(LocalDateTime time) {
    return Pars.departuresPerPersonPerDay;
  }

  @Override
  public double releaseBudgetMultiplier(LocalDateTime time) {
    return 1.0;
  }

  @Override
  public double releaseCandidateWeight(Agent agent, int hour) {
    return 1.0;
  }

  @Override
  public int releaseAgentsOverride(double steps, int dayNumber) {
    return -1;
  }

  @Override
  public double walkProbability(double meters) {
    return 1.0;
  }

  /** Core has no mode split, so it walks every commute whoever is making it. */
  @Override
  public double commuteWalkProbability(double meters, boolean student) {
    return walkProbability(meters);
  }
}
