package pedsim.activity.agents;

import ec.util.MersenneTwisterFast;
import pedsim.activity.parameters.ActivityPars;

/**
 * A tourist's programme for one day, in two outings from the accommodation. By day, sights until
 * it is time to go back, with lunch at the first stop after the lunch hour; in the evening, dinner,
 * then for some a bar or a show. Each time is drawn uniformly in its {@link ActivityPars} window.
 */
public final class TouristDay {

  private final int leaveMinute;
  private final int lunchMinute;
  private final int backMinute;
  private final int eveningMinute;
  private final boolean nightlife;

  private int outings = 0;
  private boolean lunched = false;
  private boolean dined = false;
  private boolean wentOut = false;

  private TouristDay(
      int leaveMinute, int lunchMinute, int backMinute, int eveningMinute, boolean nightlife) {
    this.leaveMinute = leaveMinute;
    this.lunchMinute = lunchMinute;
    this.backMinute = backMinute;
    this.eveningMinute = eveningMinute;
    this.nightlife = nightlife;
  }

  /** Draws a day. */
  public static TouristDay draw(MersenneTwisterFast random) {
    return new TouristDay(
        within(
            ActivityPars.touristLeaveEarliestMinute, ActivityPars.touristLeaveLatestMinute, random),
        within(
            ActivityPars.touristLunchEarliestMinute, ActivityPars.touristLunchLatestMinute, random),
        within(
            ActivityPars.touristBackEarliestMinute, ActivityPars.touristBackLatestMinute, random),
        within(
            ActivityPars.touristEveningEarliestMinute,
            ActivityPars.touristEveningLatestMinute,
            random),
        random.nextDouble() < ActivityPars.touristNightlifeShare);
  }

  private static int within(int from, int to, MersenneTwisterFast random) {
    return to > from ? from + random.nextInt(to - from + 1) : from;
  }

  /** Minute of the day the day's outing starts. */
  public int leaveMinute() {
    return leaveMinute;
  }

  /** Minute of the day the evening's outing starts; -1 once it has. */
  public int eveningMinute() {
    return outings > 1 ? -1 : eveningMinute;
  }

  /** Sets off on the next outing: the day's first, then the evening's. */
  public void startOuting() {
    outings++;
  }

  /**
   * What to do next, at this minute of the day, or null to go back to the accommodation.
   *
   * @param minute minute of the day
   * @return the next stop's purpose, or null
   */
  public ActivityPurpose next(int minute) {
    if (outings <= 1) {
      if (minute >= backMinute) {
        return null;
      }
      if (!lunched && minute >= lunchMinute) {
        lunched = true;
        return ActivityPurpose.DINING;
      }
      return ActivityPurpose.SIGHTSEEING.isOpenAt(minute / 60.0)
          ? ActivityPurpose.SIGHTSEEING
          : null;
    }
    if (!dined) {
      dined = true;
      return ActivityPurpose.DINING;
    }
    if (nightlife && !wentOut) {
      wentOut = true;
      return ActivityPurpose.NIGHTLIFE;
    }
    return null;
  }
}
