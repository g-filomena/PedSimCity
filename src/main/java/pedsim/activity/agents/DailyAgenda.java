package pedsim.activity.agents;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Random;
import pedsim.activity.parameters.ActivityPars;

/**
 * The queue of discretionary activities an agent intends to carry out on the current tour. Built
 * when the agent is released; consumed one purpose at a time as the tour chains from stop to stop
 * (home → [work] → activity → activity → home).
 *
 * <p>The agenda holds only discretionary purposes — the mandatory work/study leg is decided by
 * {@code ActivityAgent.shouldGoToWork()} — so a worker's agenda naturally becomes the post-work
 * chain (commute, then shopping on the way home), while a non-worker's agenda is the whole tour.
 *
 * <p>Purposes are validated against their opening window when *popped*, not when built: an activity
 * sampled at release time may have closed by the time the agent gets to it (e.g. errands planned
 * before a 9-hour work day); closed purposes are silently dropped.
 */
public class DailyAgenda {

  private final Deque<ActivityPurpose> upcoming = new ArrayDeque<>();

  private DailyAgenda() {}

  /**
   * Builds the tour agenda for an agent released now.
   *
   * @param persona the agent's persona (null → single discretionary stop, no chaining)
   * @param hourOfDay the release hour (0–24)
   * @param expectingWorkLeg whether the first leg of this tour will be the mandatory work/study
   *        trip — the agenda then only holds optional post-work activities
   * @param random the agent's RNG
   * @param rainy whether the current day is rainy — rain thins the optional chained stops
   *        ({@link ActivityPars#rainDiscretionaryMultiplier}) while commutes still happen
   */
  public static DailyAgenda build(
      Persona persona, double hourOfDay, boolean expectingWorkLeg, Random random, boolean rainy) {
    DailyAgenda agenda = new DailyAgenda();
    if (persona == null) {
      return agenda; // destination chosen per trip, no chaining
    }

    double chainFactor = rainy ? ActivityPars.rainDiscretionaryMultiplier : 1.0;

    if (expectingWorkLeg) {
      if (random.nextDouble() < ActivityPars.postWorkActivityProbability * chainFactor) {
        // Sampled for the late afternoon, when the post-work leg will actually start.
        agenda.upcoming.add(persona.sampleDiscretionaryPurpose(17.5, random));
        if (random.nextDouble() < ActivityPars.secondPostWorkActivityProbability * chainFactor) {
          agenda.upcoming.add(persona.sampleDiscretionaryPurpose(19.0, random));
        }
      }
      return agenda;
    }

    agenda.upcoming.add(persona.sampleDiscretionaryPurpose(hourOfDay, random));
    if (random.nextDouble() < ActivityPars.secondActivityProbability * chainFactor) {
      // The second stop happens roughly one activity later.
      agenda.upcoming.add(persona.sampleDiscretionaryPurpose(hourOfDay + 1.5, random));
    }
    return agenda;
  }

  /**
   * Expected number of discretionary stops {@link #build} would put on this agenda.
   *
   * <p>Deliberately kept beside {@code build}: the release manager charges the metres budget for a
   * whole tour rather than a single leg, and it can only do that if it can predict the tour's size
   * before the agenda exists. Any change to {@code build} has to be mirrored here, which is why the
   * two sit together.
   *
   * @param persona the agent's persona (null means no chaining, so no stops)
   * @param expectingWorkLeg whether the tour opens with the mandatory work/study trip
   * @param rainy whether rain is thinning the optional stops
   * @return the expected count, a real number rather than a draw
   */
  public static double expectedStops(Persona persona, boolean expectingWorkLeg, boolean rainy) {
    if (persona == null) {
      return 0.0;
    }
    double chainFactor = rainy ? ActivityPars.rainDiscretionaryMultiplier : 1.0;
    if (expectingWorkLeg) {
      double first = ActivityPars.postWorkActivityProbability * chainFactor;
      return first * (1.0 + ActivityPars.secondPostWorkActivityProbability * chainFactor);
    }
    return 1.0 + ActivityPars.secondActivityProbability * chainFactor;
  }

  /**
   * Expected number of walked legs the tour will produce.
   *
   * <p>A tour is home, then optionally work, then the stops, then home again, so the legs are the
   * stops themselves plus the leg home, plus the commute leg when there is one. An agent with no
   * persona still walks out and back, which is two.
   *
   * @param persona the agent's persona
   * @param expectingWorkLeg whether the tour opens with the mandatory work/study trip
   * @param rainy whether rain is thinning the optional stops
   * @return the expected leg count, never below the two of a plain out-and-back
   */
  public static double expectedLegs(Persona persona, boolean expectingWorkLeg, boolean rainy) {
    if (persona == null) {
      return 2.0;
    }
    double legs = expectedStops(persona, expectingWorkLeg, rainy) + 1.0;
    if (expectingWorkLeg) {
      legs += 1.0;
    }
    return Math.max(2.0, legs);
  }

  /**
   * Removes and returns the next activity that is open at the given hour; drops the ones that have
   * closed in the meantime. Returns {@code null} when the agenda is exhausted (→ go home).
   */
  public ActivityPurpose pollOpenActivity(double hourOfDay) {
    while (!upcoming.isEmpty()) {
      ActivityPurpose next = upcoming.poll();
      if (next.isOpenAt(hourOfDay)) {
        return next;
      }
    }
    return null;
  }

  public boolean isEmpty() {
    return upcoming.isEmpty();
  }
}
