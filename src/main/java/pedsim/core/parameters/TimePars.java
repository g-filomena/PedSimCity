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
  public static double simulationDurationInSteps;

  // Gaussian Distribution parameters for pedestrian activity
  // The total volume across all distributions should roughly sum to 1.0 (100%)
  public static double morningPeakVolume = 0.2;
  public static double morningPeakTime = 8.5; // 8:30 AM
  public static double morningPeakSpread = 1.0; // standard deviation in hours

  public static double eveningPeakVolume = 0.2;
  public static double eveningPeakTime = 17.5; // 5:30 PM
  public static double eveningPeakSpread = 1.0;

  public static double lunchPeakVolume = 0.05;
  public static double lunchPeakTime = 13.0; // 1:00 PM
  public static double lunchPeakSpread = 0.5;

  public static double nightPeakVolume = 0.3;
  public static double nightPeakTime = 20.0; // 8:00 PM
  public static double nightLeftSpread = 1.0; // steep curve in the evening
  public static double nightRightSpread = 4.0; // long tail into the morning

  public static double backgroundVolume = 0.25; // Base volume uniformly spread over 24 hours

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

  public static double computeTimeStepShare(LocalDateTime currentTime) {
    LocalTime localTime = currentTime.toLocalTime();
    double timeInHours =
        localTime.getHour() + localTime.getMinute() / 60.0 + localTime.getSecond() / 3600.0;
    double stepHours = STEP_DURATION / 3600.0;

    double share = 0.0;
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
        nightPeakVolume
            * wrappedSplitGaussian(timeInHours, nightPeakTime, nightLeftSpread, nightRightSpread);

    // Add background noise across all steps evenly
    share += backgroundVolume / 24.0;

    // Multiply probability density by the step duration to get the area/share for
    // this step
    return share * stepHours;
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

    // Return date and time starting from day 0 at 00:00
    return LocalDateTime.of(1970, 1, 1, 0, 0)
        .plusDays(totalDays)
        .plusHours(hours)
        .plusMinutes(minutes);
  }
}
