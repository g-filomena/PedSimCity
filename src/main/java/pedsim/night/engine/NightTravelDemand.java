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
   * How many dark release events have happened today, which is the index of the next pair to send
   * out. Counted rather than derived from the step, because pairs are released into darkness only
   * and the dark events are not the first {@code n} events of the day.
   */
  private int pairsReleased = 0;

  private boolean announcedCapacity = false;

  public NightTravelDemand(PedSimCityNight state) {
    super(state);
    this.night = state;
  }

  /**
   * With light A/B testing enabled, day 1 releases one vulnerable/non-vulnerable twin pair per
   * <b>dark</b> release event (agents {@code 2i} and {@code 2i+1} for the {@code i}-th such event)
   * instead of the standard release.
   *
   * <p><b>Pairs depart into darkness only.</b> One pair per release event across the whole day left
   * the majority of them setting off in daylight, where the lighting manipulation does nothing:
   * with {@code releaseAgentsEveryMinutes = 20} a day holds 72 release events and, on a Turin June
   * day, only about 31 of those are dark. The experiment's size was therefore not
   * {@code abTestPairs} but whatever fraction of it happened to fall after sunset, and it moved
   * with the date without anything saying so. Skipping the light events makes {@code abTestPairs}
   * mean what it says, up to the number of dark events the date allows - which
   * {@link #darkReleaseEventsToday} counts and logs at the start of the day, so an experiment too
   * big for its night is reported rather than silently truncated.
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

    int pairIndex = pairsReleased;
    if (pairIndex >= Math.max(1, NightPars.abTestPairs)) {
      return 0;
    }

    NightAgent vulnAgent = null;
    NightAgent normalAgent = null;
    for (Agent agent : night.agentsList) {
      if (agent instanceof NightAgent nightAgent) {
        if (nightAgent.agentID == pairIndex * 2) {
          vulnAgent = nightAgent;
        } else if (nightAgent.agentID == pairIndex * 2 + 1) {
          normalAgent = nightAgent;
        }
      }
    }
    if (vulnAgent == null || normalAgent == null) {
      // Advance past the gap rather than retrying it: the index is a counter now, not a function of
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
    return 2;
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
                + " (%d-minute cadence) - %d pairs will depart after dark.",
            requested,
            darkEvents,
            dateOf(dayNumber),
            TimePars.releaseAgentsEveryMinutes,
            Math.min(requested, darkEvents));
    if (requested > darkEvents) {
      LoggerUtil.getLogger()
          .warning(message + " The remaining " + (requested - darkEvents) + " stay at home.");
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

  private static LocalDate dateOf(int dayNumber) {
    return TimePars.SIMULATION_START_DATE.plusDays(dayNumber - 1L);
  }
}
