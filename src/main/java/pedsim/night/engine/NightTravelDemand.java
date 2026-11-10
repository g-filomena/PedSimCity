package pedsim.night.engine;

import java.time.LocalDate;
import java.time.LocalDateTime;
import pedsim.activity.engine.ActivityTravelDemand;
import pedsim.activity.engine.Daylight;
import pedsim.core.agents.Agent;
import pedsim.core.parameters.TimePars;
import pedsim.core.utilities.LoggerUtil;
import pedsim.night.agents.NightAgent;
import pedsim.night.parameters.NightPars;

/** Night travel demand: the activity tier's, plus the light A/B experiment's paired releases. */
public class NightTravelDemand extends ActivityTravelDemand {

  private final PedSimCityNight night;

  /**
   * How many pairs have been sent out today, which is the index of the next one. Counted rather
   * than derived from the step, because pairs are released into darkness only and the dark events
   * are not the first {@code n} events of the day.
   */
  private int pairsReleased = 0;

  /** Pairs sent out at each dark release event; 0 until the first event sizes it. */
  private int pairsPerEvent = 0;

  /** The twins by agent ID, built when the first pair is released. */
  private java.util.Map<Integer, NightAgent> twins;

  private boolean announcedCapacity = false;

  public NightTravelDemand(PedSimCityNight state) {
    super(state);
    this.night = state;
  }

  /**
   * With light A/B testing enabled, day 1 releases vulnerable/non-vulnerable twin pairs (agents
   * {@code 2i} and {@code 2i+1} for the {@code i}-th pair) at the <b>dark</b> release events
   * instead of the standard release: {@code ceil(abTestPairs / dark events)} pairs at each, until
   * {@code abTestPairs} have gone.
   *
   * <p><b>Pairs depart into darkness only</b>, where the lighting manipulation can act; the
   * number of dark release events the date allows is counted and logged at the start of the day.
   */
  @Override
  public int releaseAgentsOverride(double steps, int dayNumber) {
    if (!NightPars.enableLightABTesting || dayNumber != 1) {
      return -1;
    }

    announceCapacityOnce(dayNumber);

    // A pair released into daylight is a pair the manipulated variable cannot reach.
    if (!night.isDark) {
      return 0;
    }

    int requested = Math.max(1, NightPars.abTestPairs);
    if (pairsPerEvent == 0) {
      pairsPerEvent = pairsPerEvent(requested, darkReleaseEventsToday(dayNumber));
    }
    if (twins == null) {
      twins = new java.util.HashMap<>();
      for (Agent agent : night.agentsList) {
        if (agent instanceof NightAgent nightAgent) {
          twins.put(nightAgent.agentID, nightAgent);
        }
      }
    }

    int released = 0;
    for (int k = 0; k < pairsPerEvent && pairsReleased < requested; k++) {
      released += releasePair(pairsReleased, steps);
    }
    return 2 * released;
  }

  /**
   * Sends out the twins {@code 2 * pairIndex} and {@code 2 * pairIndex + 1}, and moves on to the
   * next pair whether or not this one exists.
   *
   * @return 1 if the pair departed, 0 if it was never built
   */
  private int releasePair(int pairIndex, double steps) {
    NightAgent vulnAgent = twins.get(pairIndex * 2);
    NightAgent normalAgent = twins.get(pairIndex * 2 + 1);
    if (vulnAgent == null || normalAgent == null) {
      // Advance past the gap rather than retrying it: the index is a counter, not a function of
      // the step, so a pair that was never built would otherwise consume every remaining dark event
      // of the day and nothing would depart at all.
      pairsReleased++;
      LoggerUtil.getLogger()
          .warning("A/B Testing: no twin pair at index " + pairIndex + "; skipping it.");
      return 0;
    }

    // The pair must differ in vulnerability and in nothing else. What makes that true is the
    // shared destination: NightAgent.defineRandomDestination copies its twin's, whichever of the
    // two chooses first. A shared trip *length* was handed to both here as well, drawn from a band
    // of metres, and it was the last trip-distance parameter left in the activity tier - which is
    // the tier that is not supposed to have one, night included. It bought nothing the shared
    // destination does not already buy.
    vulnAgent.startWalkingAlone();
    normalAgent.startWalkingAlone();
    pairsReleased++;

    LoggerUtil.getLogger()
        .fine(
            "A/B Testing: Released pair "
                + pairIndex
                + " (Agent "
                + vulnAgent.agentID
                + " & "
                + normalAgent.agentID
                + ") at step "
                + steps);
    return 1;
  }

  /**
   * States, once, how many pairs this date can actually send out after dark, so that an experiment
   * sized beyond its own night is visible in the log rather than in a shortfall nobody counted.
   */
  private void announceCapacityOnce(int dayNumber) {
    if (announcedCapacity) {
      return;
    }
    announcedCapacity = true;
    int darkEvents = darkReleaseEventsToday(dayNumber);
    int requested = Math.max(1, NightPars.abTestPairs);
    String message =
        String.format(
            "A/B light experiment: %d requested pairs, %d dark release events on %s"
                + " (%d-minute cadence), %d pairs at each.",
            requested,
            darkEvents,
            dateOf(dayNumber),
            TimePars.releaseAgentsEveryMinutes,
            pairsPerEvent(requested, darkEvents));
    if (darkEvents == 0) {
      LoggerUtil.getLogger()
          .warning(message + " There is no dark release event on this date: no pair departs.");
    } else {
      LoggerUtil.getLogger().info(message);
    }
  }

  /**
   * How many of the day's release events fall in darkness, by the same seasonal rule the engine
   * uses to set {@code isDark}. A count of opportunities, not of departures: it is what bounds the
   * experiment on this date.
   */
  private int darkReleaseEventsToday(int dayNumber) {
    LocalDate date = dateOf(dayNumber);
    int eventsPerDay = (24 * 60) / Math.max(1, TimePars.releaseAgentsEveryMinutes);
    int dark = 0;
    for (int event = 0; event < eventsPerDay; event++) {
      LocalDateTime time =
          date.atStartOfDay().plusMinutes((long) event * TimePars.releaseAgentsEveryMinutes);
      if (Daylight.isDark(time)) {
        dark++;
      }
    }
    return dark;
  }

  /**
   * The pairs sent out at each dark release event: {@code requested} spread over the events, rounded
   * up so that the events together can hold all of them. With no dark event the whole experiment
   * goes at the first one that comes.
   */
  static int pairsPerEvent(int requested, int darkEvents) {
    return (int) Math.ceil(requested / (double) Math.max(1, darkEvents));
  }

  private static LocalDate dateOf(int dayNumber) {
    return TimePars.SIMULATION_START_DATE.plusDays(dayNumber - 1L);
  }
}
