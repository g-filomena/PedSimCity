package pedsim.core.parameters;

import java.time.LocalDateTime;
import java.time.LocalTime;

public class TimePars {

  // in seconds. One step = 20 minutes,
  public static double STEP_DURATION = 1200; // seconds
  public static int releaseAgentsEveryMinutes = 20;
  public static double MINUTE_TO_STEPS;
  public static double releaseAgentsEverySteps;
  // public static static double hoursInSteps = 60 * minuteInSteps;

  // Calculate the total simulation time in seconds for a certain number of days
  public static int numberOfDays = 7;
  public static double simulationDurationInSteps;

  // Define peak hours (adjust these as needed)
  public static final LocalTime morningPeakStart = LocalTime.of(7, 0);
  public static final LocalTime morningPeakEnd = LocalTime.of(9, 0);
  public static final LocalTime eveningPeakStart = LocalTime.of(17, 0);
  public static final LocalTime eveningPeakEnd = LocalTime.of(19, 0);
  public static final LocalTime nightStart = LocalTime.of(0, 30);
  public static final LocalTime nightEnd = LocalTime.of(5, 30);

  // Assume percentage of the population walking during peak hours
  public static final double peakPercentage = 0.35; // 35%
  public static final double offPeakPercentage = 0.15; // 15%
  public static final double nightPercentage = 0.01; // 1%

  public static int stepsInPeakHours;
  public static int stepsInOffPeak;
  public static int stepsAtNight;

  /**
   * Defines the simulation mode and sets simulation parameters based on the selected mode. Called
   * at the beginning of the simulation to configure simulation settings.
   */
  public static void setTemporalPars() {

    MINUTE_TO_STEPS = 60 / STEP_DURATION;
    releaseAgentsEverySteps = releaseAgentsEveryMinutes * MINUTE_TO_STEPS;
    simulationDurationInSteps = numberOfDays * 24 * 60 * MINUTE_TO_STEPS; // Days to seconds
    getStepsInReleaseTimes();

  }

  /**
   * Calculates the number of steps in a given time range based on the step interval. Assumes a step
   * occurs every 20 minutes.
   */

  private static void getStepsInReleaseTimes() {

    stepsInPeakHours = getStepsBetween(morningPeakStart, morningPeakEnd)
        + getStepsBetween(eveningPeakStart, eveningPeakEnd);
    stepsInOffPeak = getStepsBetween(nightEnd, morningPeakStart)
        + getStepsBetween(morningPeakEnd, eveningPeakStart)
        + getStepsBetween(eveningPeakEnd, nightStart);
    stepsAtNight = getStepsBetween(nightStart, nightEnd);
  }

  private static int getStepsBetween(LocalTime start, LocalTime end) {
    int startMinutes = start.getHour() * 60 + start.getMinute();
    int endMinutes = end.getHour() * 60 + end.getMinute();

    if (endMinutes < startMinutes) {
      // Case where the range crosses midnight
      return ((24 * 60 - startMinutes) + endMinutes) / releaseAgentsEveryMinutes;
    }
    return (endMinutes - startMinutes) / 20;
  }

  public static double computeTimeStepShare(LocalDateTime currentTime) {
    if (isPeakHours(currentTime)) {
      return peakPercentage / stepsInPeakHours;
    }
    if (isOffPeakHours(currentTime)) {
      return offPeakPercentage / stepsInOffPeak;
    }
    return nightPercentage / stepsAtNight;
  }

  public static boolean isPeakHours(LocalDateTime currentTime) {
    LocalTime currentTimeOnly = currentTime.toLocalTime();
    return (currentTimeOnly.isAfter(morningPeakStart) && currentTimeOnly.isBefore(morningPeakEnd))
        || (currentTimeOnly.isAfter(eveningPeakStart) && currentTimeOnly.isBefore(eveningPeakEnd));
  }

  public static boolean isOffPeakHours(LocalDateTime currentTime) {
    LocalTime currentTimeOnly = currentTime.toLocalTime();
    return (currentTimeOnly.isAfter(nightEnd) && currentTimeOnly.isBefore(morningPeakStart))
        || (currentTimeOnly.isAfter(morningPeakEnd)
            && currentTimeOnly.isBefore(TimePars.eveningPeakStart))
        || currentTimeOnly.isAfter(eveningPeakEnd) || currentTimeOnly.isBefore(nightStart);
  }

  public static LocalDateTime getTime(double totalSteps) {
    long totalMinutes = (long) (totalSteps * (TimePars.STEP_DURATION / 60)); // Convert steps to
                                                                             // minutes based on
                                                                             // the stepTimeUnit
    long totalDays = totalMinutes / (24 * 60); // Calculate total days
    long remainingMinutes = totalMinutes % (24 * 60); // Calculate remaining minutes in the current
                                                      // day

    long hours = remainingMinutes / 60; // Convert remaining minutes to hours
    long minutes = remainingMinutes % 60; // Calculate remaining minutes

    // Return date and time starting from day 0 at 00:00
    return LocalDateTime.of(1970, 1, 1, 0, 0).plusDays(totalDays).plusHours(hours)
        .plusMinutes(minutes);
  }
}
