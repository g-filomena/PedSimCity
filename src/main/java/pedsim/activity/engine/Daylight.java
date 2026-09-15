package pedsim.activity.engine;

import java.time.DateTimeException;
import java.time.LocalDateTime;
import java.time.ZoneId;
import pedsim.activity.parameters.ActivityPars;
import pedsim.core.parameters.Pars;

/**
 * Whether it is dark. One definition, for every module and for the exports.
 *
 * <p><b>Darkness comes from where the city is and what day it is.</b> Sunrise and sunset are
 * computed for the simulation date at {@link Pars#cityLatitude} / {@link Pars#cityLongitude}, which
 * {@link pedsim.core.engine.CityLocation} measures from the street network at startup, so a
 * December evening is dark hours before a June one. {@link ActivityPars#twilightBufferMinutes}
 * keeps it light for a margin either side.
 *
 * <p><b>With no position, a fixed window.</b> A city whose network declares no usable CRS falls
 * back to {@link ActivityPars#dayStartHour} / {@link ActivityPars#nightStartHour} - 06:00 to 20:00
 * unless a run sets them - which is a stated assumption rather than a season. The alternative, a
 * default latitude, is worse: it computes a precise sunset for the wrong place and says nothing
 * about having done so. {@link #describeRegime()} names which of the two is in force, and the
 * activity environment logs it at startup.
 *
 * <p><b>Clock time, not solar time.</b> Sunset at a city's eastern edge of its time zone is not
 * sunset at its western edge, and half of Europe adds an hour in summer, so the model converts:
 * longitude against the zone's standard meridian, plus the equation of time. The zone comes from
 * {@link ActivityPars#timeZoneId}, and a city that does not state one is assumed to keep the
 * standard time of its nearest 15-degree meridian all year - which is an hour out wherever summer
 * time is observed. Turin with {@code Europe/Rome} lands within a few minutes of the published
 * sunset in both June and December; without it, an hour early all summer.
 *
 * <p><b>This is the only place that decides.</b> {@code ActivityEngine} recomputes {@code
 * PedSimCityActivity.isDark} from it each step and the exporter splits its volumes by it, so a
 * run's behaviour and its outputs cannot disagree. The night module inherits that field and never
 * recomputes darkness; core holds only the {@code DarknessModel} seam, and has no answer of its own
 * to give - it knows where the city is, but it has no clock and no date.
 *
 * <p>The solar geometry is the standard set: Cooper's declination, the hour-angle sunrise equation
 * taken at the conventional -0.833 degree altitude (refraction and the sun's own radius), and the
 * equation of time.
 */
public final class Daylight {

  /**
   * Sun altitude at the moment of sunrise and sunset: below the horizon by the sum of atmospheric
   * refraction and the sun's apparent radius, which is the convention every published sunrise table
   * uses.
   */
  private static final double SUNRISE_ALTITUDE_DEGREES = -0.833;

  private Daylight() {}

  /** Whether it is behaviourally dark at the given simulation date-time. */
  public static boolean isDark(LocalDateTime time) {
    double hour = time.getHour() + time.getMinute() / 60.0;
    if (!hasPosition()) {
      return hour < ActivityPars.dayStartHour || hour >= ActivityPars.nightStartHour;
    }
    double[] sunriseSunset = sunriseSunsetHours(time);
    double buffer = ActivityPars.twilightBufferMinutes / 60.0;
    return hour < sunriseSunset[0] - buffer || hour > sunriseSunset[1] + buffer;
  }

  /** Whether the city's position is known, and so whether darkness follows the sun. */
  public static boolean hasPosition() {
    return !Double.isNaN(Pars.cityLatitude) && !Double.isNaN(Pars.cityLongitude);
  }

  /** One line naming which of the two regimes decides darkness, for the startup log. */
  public static String describeRegime() {
    if (!hasPosition()) {
      return String.format(
          "darkness: fixed window %04.1f-%04.1f (the city's position is unknown, so no season)",
          ActivityPars.dayStartHour, ActivityPars.nightStartHour);
    }
    String zone =
        ActivityPars.timeZoneId.isBlank()
            ? "standard time of the nearest meridian (no summer time)"
            : ActivityPars.timeZoneId;
    return String.format(
        "darkness: sunrise/sunset at %.4f, %.4f degrees, %s, %.0f min twilight buffer",
        Pars.cityLatitude, Pars.cityLongitude, zone, ActivityPars.twilightBufferMinutes);
  }

  /**
   * Sunrise and sunset as fractional hours of the local clock, for the city's position on the given
   * date. Clamped for polar day and night: returns {12, 12} (no daylight) or {0, 24} (no night).
   */
  static double[] sunriseSunsetHours(LocalDateTime time) {
    int dayOfYear = time.getDayOfYear();
    double declination =
        Math.toRadians(23.44) * Math.sin(2.0 * Math.PI * (284.0 + dayOfYear) / 365.0);
    double latitude = Math.toRadians(Pars.cityLatitude);

    double cosHourAngle =
        (Math.sin(Math.toRadians(SUNRISE_ALTITUDE_DEGREES))
                - Math.sin(latitude) * Math.sin(declination))
            / (Math.cos(latitude) * Math.cos(declination));
    if (cosHourAngle <= -1.0) {
      return new double[] {0.0, 24.0};
    }
    if (cosHourAngle >= 1.0) {
      return new double[] {12.0, 12.0};
    }
    double halfDaylightHours = Math.toDegrees(Math.acos(cosHourAngle)) / 15.0;

    double solarNoon = 12.0 - timeCorrectionMinutes(time) / 60.0;
    return new double[] {solarNoon - halfDaylightHours, solarNoon + halfDaylightHours};
  }

  /**
   * How far local clock time runs ahead of local solar time, in minutes: four minutes per degree
   * of longitude away from the time zone's standard meridian, plus the equation of time.
   */
  private static double timeCorrectionMinutes(LocalDateTime time) {
    double standardMeridian = 15.0 * utcOffsetHours(time);
    return 4.0 * (Pars.cityLongitude - standardMeridian) + equationOfTimeMinutes(time);
  }

  /**
   * The city's offset from UTC on the given date, summer time included when a zone is named.
   * Without one, the offset of the nearest 15-degree meridian, which is standard time all year.
   */
  private static double utcOffsetHours(LocalDateTime time) {
    if (ActivityPars.timeZoneId.isBlank()) {
      return Math.round(Pars.cityLongitude / 15.0);
    }
    try {
      return ZoneId.of(ActivityPars.timeZoneId).getRules().getOffset(time).getTotalSeconds()
          / 3600.0;
    } catch (DateTimeException e) {
      return Math.round(Pars.cityLongitude / 15.0);
    }
  }

  /**
   * The equation of time in minutes: the sun's own departure from clock regularity, from the
   * earth's elliptical orbit and axial tilt. About a quarter of an hour either way over the year.
   */
  private static double equationOfTimeMinutes(LocalDateTime time) {
    double b = 2.0 * Math.PI * (time.getDayOfYear() - 81) / 364.0;
    return 9.87 * Math.sin(2.0 * b) - 7.53 * Math.cos(b) - 1.5 * Math.sin(b);
  }
}
