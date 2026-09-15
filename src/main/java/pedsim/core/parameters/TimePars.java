package pedsim.core.parameters;

import java.time.DayOfWeek;
import java.time.LocalDate;
import java.time.LocalDateTime;
import java.time.LocalTime;

public class TimePars {

  // in seconds. One step = 5 minutes (finer than the historical 20 so that trips span several
  // steps and intra-hour flow dynamics are resolved; moveRate is derived from this in Pars).
  public static double STEP_DURATION = 300; // seconds
  public static int releaseAgentsEveryMinutes = 20;

  /**
   * Calendar date of simulation step 0 (00:00). Gives the simulation real dates so that day-of-week
   * (weekday/weekend release curves, persona work days) and day-of-year (seasonal daylight) are
   * defined. Default: a Monday in early June.
   */
  public static LocalDate SIMULATION_START_DATE = LocalDate.of(2026, 6, 1);

  public static double MINUTE_TO_STEPS;
  public static double releaseAgentsEverySteps;

  // Calculate the total simulation time in seconds for a certain number of days
  public static double simulationDurationInSteps;

  // Gaussian Distribution parameters for pedestrian activity
  // The total volume across all distributions should roughly sum to 1.0 (100%)
  public static double morningPeakVolume = 0.25;
  public static double morningPeakTime = 8.50; // 8:30 AM
  public static double morningPeakSpread = 1.20; // standard deviation in hours

  public static double lunchPeakVolume = 0.10;
  public static double lunchPeakTime = 13.00; // 1:00 PM
  public static double lunchPeakSpread = 0.80;

  public static double eveningPeakVolume = 0.35;
  public static double eveningPeakTime = 17.50; // 5:30 PM
  public static double eveningPeakSpread = 1.30;

  public static double nightPeakVolume = 0.10;
  public static double nightPeakTime = 20.00; // 8:00 PM
  public static double nightLeftSpread = 1.00; // steep curve in the evening
  public static double nightRightSpread = 1.50; //

  public static double backgroundVolume = 0.20; // Base volume uniformly spread over 24 hours

  // Weekend release curve: no commute peak, later and flatter morning, larger night share.
  // Each set sums to ~1.0 like the weekday one.
  public static double weekendMorningPeakVolume = 0.10;
  public static double weekendMorningPeakTime = 10.50;
  public static double weekendMorningPeakSpread = 1.60;
  public static double weekendLunchPeakVolume = 0.12;
  public static double weekendLunchPeakTime = 13.50;
  public static double weekendEveningPeakVolume = 0.28;
  public static double weekendEveningPeakTime = 16.50;
  public static double weekendEveningPeakSpread = 2.00;
  public static double weekendNightPeakVolume = 0.18;
  public static double weekendNightPeakTime = 21.50;
  public static double weekendBackgroundVolume = 0.32;

  // Friday keeps the weekday commute shape but shifts volume into the night (going out).
  public static double fridayNightPeakVolume = 0.15;
  public static double fridayBackgroundVolume = 0.15;

  /**
   * Defines the simulation mode and sets simulation parameters based on the
   * selected mode. Called at the beginning of the simulation to configure
   * simulation settings.
   */
  public static void setTemporalPars() {
    MINUTE_TO_STEPS = 60 / STEP_DURATION;
    releaseAgentsEverySteps = releaseAgentsEveryMinutes * MINUTE_TO_STEPS;
    simulationDurationInSteps = Pars.durationDays * 24 * 60 * MINUTE_TO_STEPS; // Days to steps
  }

  private static double splitGaussian(
      double x, double mean, double leftStdDev, double rightStdDev) {
    double stdDev = (x < mean) ? leftStdDev : rightStdDev;
    // The normalization factor for split normal is sqrt(2/pi) / (left + right)
    double normalization = Math.sqrt(2.0 / Math.PI) / (leftStdDev + rightStdDev);
    return normalization * Math.exp(-0.5 * Math.pow((x - mean) / stdDev, 2));
  }

  private static double wrappedSplitGaussian(
      double x, double mean, double leftStdDev, double rightStdDev) {
    // Evaluate at x-24, x, x+24 to handle wrapping around midnight
    return splitGaussian(x - 24, mean, leftStdDev, rightStdDev)
        + splitGaussian(x, mean, leftStdDev, rightStdDev)
        + splitGaussian(x + 24, mean, leftStdDev, rightStdDev);
  }

  /**
   * Share of the daily walking budget released at this release event, following a
   * day-of-week-specific diurnal curve: the weekday curve has the commute peaks, Friday shifts
   * volume into the night, and the weekend has no morning commute, a later flatter morning, and a
   * larger night share.
   *
   * <p>The curve is a density over the day integrating to 1.0, so the share is its area over the
   * interval between release events — {@link #releaseAgentsEveryMinutes}, NOT {@link
   * #STEP_DURATION}. Integrating over the wrong one scales the whole day by their ratio without any
   * error to show for it: the shares stop summing to 1.0 and that fraction of the day's departures
   * is never released.
   */
  public static double computeTimeStepShare(LocalDateTime currentTime) {
    LocalTime localTime = currentTime.toLocalTime();
    double timeInHours =
        localTime.getHour() + localTime.getMinute() / 60.0 + localTime.getSecond() / 3600.0;
    double releaseIntervalHours = releaseAgentsEveryMinutes / 60.0;

    DayOfWeek day = currentTime.getDayOfWeek();
    boolean weekend = day == DayOfWeek.SATURDAY || day == DayOfWeek.SUNDAY;
    boolean friday = day == DayOfWeek.FRIDAY;

    double share = 0.0;
    if (weekend) {
      share +=
          weekendMorningPeakVolume
              * wrappedSplitGaussian(
                  timeInHours,
                  weekendMorningPeakTime,
                  weekendMorningPeakSpread,
                  weekendMorningPeakSpread);
      share +=
          weekendEveningPeakVolume
              * wrappedSplitGaussian(
                  timeInHours,
                  weekendEveningPeakTime,
                  weekendEveningPeakSpread,
                  weekendEveningPeakSpread);
      share +=
          weekendLunchPeakVolume
              * wrappedSplitGaussian(
                  timeInHours, weekendLunchPeakTime, lunchPeakSpread, lunchPeakSpread);
      share +=
          weekendNightPeakVolume
              * wrappedSplitGaussian(
                  timeInHours, weekendNightPeakTime, nightLeftSpread, nightRightSpread);
      share += weekendBackgroundVolume / 24.0;
    } else {
      double effectiveNightVolume = friday ? fridayNightPeakVolume : nightPeakVolume;
      double effectiveBackground = friday ? fridayBackgroundVolume : backgroundVolume;
      share +=
          morningPeakVolume
              * wrappedSplitGaussian(
                  timeInHours, morningPeakTime, morningPeakSpread, morningPeakSpread);
      share +=
          eveningPeakVolume
              * wrappedSplitGaussian(
                  timeInHours, eveningPeakTime, eveningPeakSpread, eveningPeakSpread);
      share +=
          lunchPeakVolume
              * wrappedSplitGaussian(timeInHours, lunchPeakTime, lunchPeakSpread, lunchPeakSpread);
      share +=
          effectiveNightVolume
              * wrappedSplitGaussian(timeInHours, nightPeakTime, nightLeftSpread, nightRightSpread);
      share += effectiveBackground / 24.0;
    }

    // Density x the interval between release events = the area under the curve this release
    // accounts for; summed over the day's releases this returns 1.0.
    return share * releaseIntervalHours;
  }

  public static LocalDateTime getTime(double totalSteps) {
    long totalMinutes =
        (long) (totalSteps * (TimePars.STEP_DURATION / 60)); // Convert steps to minutes based on
    // the stepTimeUnit
    long totalDays = totalMinutes / (24 * 60); // Calculate total days
    long remainingMinutes =
        totalMinutes % (24 * 60); // Calculate remaining minutes in the current day

    long hours = remainingMinutes / 60; // Convert remaining minutes to hours
    long minutes = remainingMinutes % 60; // Calculate remaining minutes

    // Date and time anchored at the configured start date (step 0 = start date, 00:00), so
    // day-of-week and day-of-year are meaningful.
    return SIMULATION_START_DATE
        .atStartOfDay()
        .plusDays(totalDays)
        .plusHours(hours)
        .plusMinutes(minutes);
  }

  /** Whether the given simulation time falls on a Saturday or Sunday. */
  public static boolean isWeekend(LocalDateTime time) {
    DayOfWeek day = time.getDayOfWeek();
    return day == DayOfWeek.SATURDAY || day == DayOfWeek.SUNDAY;
  }
}
