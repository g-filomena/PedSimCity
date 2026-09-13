package pedsim.core.engine;

import java.time.LocalDateTime;
import pedsim.core.agents.Agent;

/**
 * Who goes out, when, and by what mode.
 *
 * <p>These questions were answered by overriding methods on {@link PedSimCity}, which is the
 * simulation state: the graphs, the layers, the agent sets, the schedule. Travel demand is not
 * state, it is policy, and keeping the two in one class is how the departure profile came to read
 * {@code Pars.metersPerDayPerPerson} - a calibration anchor was in scope because everything was.
 *
 * <p>{@link AgentReleaseManager} now asks a {@code TravelDemand} rather than the state, so what the
 * release manager may consult is exactly this list, and a module supplies its demand by writing an
 * implementation instead of by reaching into a 480-line base class. Core's answers are in
 * {@link BaselineTravelDemand}; the activity tier's are in {@code ActivityTravelDemand}.
 *
 * <p>Implementations are consulted from agent threads and must be thread-safe.
 */
public interface TravelDemand {

  /**
   * Share of the day's releases belonging to the event at this moment.
   *
   * <p>The shares across a day's release events must sum to 1.0, or the day is over- or
   * under-released.
   *
   * @param time the moment of the release event
   * @return the share, in {@code [0, 1]}
   */
  double departureShare(LocalDateTime time);

  /**
   * Agents that must set out at this moment, whatever chance says.
   *
   * <p>Travel divides in two: departures already decided, and departures drawn. Going to work is
   * not a draw: an agent with a job, who walks to it, goes on every working day, and the only open
   * question is when. Everything else is a draw. Merging the two into one lottery is what forced a
   * commute <i>share</i> to be computed and handed to the departure profile - the model had to be
   * told how often chance would produce the commutes it was certain to have.
   *
   * <p>Returned agents are released unconditionally: no persona affinity, no budget, no filter.
   * They are at home and it is time to go.
   *
   * @param time the moment of the release event
   * @return the agents departing now, possibly empty, never null
   */
  java.util.List<Agent> scheduledDepartures(LocalDateTime time);

  /**
   * Departures one person makes by chance on an average day.
   *
   * <p>What is left of the day's travel once the scheduled departures have taken theirs. A count of
   * departures is the quantity travel surveys measure; the metres then fall out of where people
   * choose to go. This was {@code unscheduledChainsPerPerson}, but a trip chain is an activity
   * programme's idea - out, through an agenda, home - and core has no agenda. What the release
   * manager needs to know is how often somebody sets off; what that departure then sets in motion
   * belongs to whichever module implements this, behind the interface.
   *
   * <p>It sat beside a {@code usesCountBasedRelease()} flag that chose between this and a metres
   * budget. The budget is gone, so there is nothing to choose.
   *
   * @param time the moment of the release event
   * @return unscheduled departures per person per day
   */
  double unscheduledDeparturesPerPerson(LocalDateTime time);

  /**
   * Multiplier on the size of the release event at this moment - weather, for instance.
   *
   * @param time the moment of the release event
   * @return the multiplier, 1.0 for no change
   */
  double releaseBudgetMultiplier(LocalDateTime time);

  /**
   * Acceptance probability in {@code [0, 1]} that this agent is released at this hour - a persona
   * affinity, say, or a measured propensity to walk when one exists.
   *
   * @param agent the candidate
   * @param hour the clock hour
   * @return the acceptance probability
   */
  double releaseCandidateWeight(Agent agent, int hour);

  /**
   * Hook for taking over a single release event entirely, e.g. the night module's paired A/B
   * releases.
   *
   * @return the number of agents released ({@code >= 0}) when the event was handled here, or
   *     {@code -1} to let the standard release run
   */
  int releaseAgentsOverride(double steps, int dayNumber);

  /**
   * Probability that a journey of this length is made on foot rather than some other way.
   *
   * <p>Mode choice, asked where a journey already has a length - a commute between a known home and
   * a known workplace, for instance. Three neighbours of this method went with the metres budget on
   * 13 Sep 2026: {@code expectedTripChainLegs}, which charged the budget for a whole trip chain;
   * {@code measuredLegMetres}, for the case where a module knew its mean leg and did not want a
   * length drawn; and {@code tripAcceptanceProbability}, a rejection filter on drawn lengths. The
   * last of those looked like this one and was not - it asked whether a <i>length</i> was plausible,
   * where this asks how a <i>journey</i> is made, and the two being adjacent is how disabling a
   * legacy filter once silently walked every commute in the model.
   */
  double walkProbability(double meters);

  /**
   * Probability that a commute of this length is walked, for a commuter of this kind.
   *
   * <p>Split by kind because the observed shares are: ISTAT puts 38.0% of intra-Turin study
   * commutes on foot against 16.3% of work commutes, and no single curve produces both from the
   * same distances.
   *
   * @param meters the commute length, in walked metres
   * @param student whether the journey is to a place of study rather than to work
   */
  double commuteWalkProbability(double meters, boolean student);
}
