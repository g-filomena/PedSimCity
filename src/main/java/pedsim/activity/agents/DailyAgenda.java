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
