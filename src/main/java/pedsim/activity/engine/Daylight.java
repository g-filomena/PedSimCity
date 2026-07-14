package pedsim.activity.engine;

import java.time.LocalDateTime;
import pedsim.activity.parameters.ActivityPars;

/**
 * Seasonal daylight model: computes sunrise/sunset for the simulation date and latitude so that
 * behavioural darkness ({@code isDark}) follows the seasons instead of a fixed 20:00–06:00 window
 * (in Liverpool sunset ranges from ~16:00 in December to ~21:45 in June).
 *
 * <p>Uses the standard solar-declination approximation (Cooper's equation) with the hour-angle
 * sunrise equation; solar time is treated as local clock time (no longitude / equation-of-time
 * correction — errors of a few tens of minutes are irrelevant at this behavioural resolution). A
 * civil-twilight buffer keeps it "light" for a configurable margin around sunrise/sunset.
 *
 * <p>Only behavioural darkness uses this model; the exporter's day/night volume aggregation keeps
 * the fixed {@code TimePars} window so outputs stay comparable across runs and seasons.
 */
public final class Daylight {

  private Daylight() {}

  /** Whether it is behaviourally dark at the given simulation date-time. */
  public static boolean isDark(LocalDateTime time) {
    double[] sunriseSunset = sunriseSunsetHours(time.getDayOfYear(), ActivityPars.latitudeDegrees);
    double hour = time.getHour() + time.getMinute() / 60.0;
    double buffer = ActivityPars.twilightBufferMinutes / 60.0;
    return hour < sunriseSunset[0] - buffer || hour > sunriseSunset[1] + buffer;
  }

  /**
   * Sunrise and sunset as fractional hours of day for the given day-of-year and latitude.
   * Clamped for polar day/night: returns {12, 12} (no daylight) or {0, 24} (no night).
   */
  static double[] sunriseSunsetHours(int dayOfYear, double latitudeDegrees) {
    double declination =
        Math.toRadians(23.44) * Math.sin(2.0 * Math.PI * (284.0 + dayOfYear) / 365.0);
    double latitude = Math.toRadians(latitudeDegrees);

    double cosHourAngle = -Math.tan(latitude) * Math.tan(declination);
    cosHourAngle = Math.max(-1.0, Math.min(1.0, cosHourAngle));
    double halfDaylightHours = Math.toDegrees(Math.acos(cosHourAngle)) / 15.0;

    return new double[] {12.0 - halfDaylightHours, 12.0 + halfDaylightHours};
  }
}
