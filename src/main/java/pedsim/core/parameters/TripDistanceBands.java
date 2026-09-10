package pedsim.core.parameters;

import org.apache.commons.math3.special.Erf;

/**
 * Release-time trip-distance bands.
 *
 * <p>A band is defined by its {@code [min, max]} metres; the mean is derived from the band, not set
 * alongside it. Distances are drawn from a lognormal truncated to the band, by inverse CDF:
 * right-skewed like observed trip-length distributions, and — unlike the previous
 * Gaussian-times-mean with a hard clamp — with no probability mass piled on the band edges. That
 * clamp put the two tails beyond ±1.67σ onto the bounds themselves, so ~9.6% of all trips came out
 * at exactly 900 m or exactly 2700 m.
 *
 * <p>Bands are resolved by clock hour. The day/evening and pre-dawn/day boundaries reuse
 * {@link TimePars#NIGHT_START_HOUR} and {@link TimePars#DAY_START_HOUR} so they stay consistent with
 * the day/night definition the exporters aggregate on; {@link #PREDAWN_START_HOUR} is the only new
 * boundary.
 *
 * <p>All three bands default to the same range, so introducing them changes no band on its own. The
 * per-band values are a calibration input: the expectation is that evening (nightlife, social) trips
 * run longer and pre-dawn (sparse, shift-work) trips shorter than the daytime mix.
 */
public class TripDistanceBands {

  /** Time-of-day band a released trip's distance is drawn from. */
  public enum Band {
    /** Daytime: commuting and errands. */
    DAY,
    /** Evening and early night, across midnight. */
    EVENING,
    /** Small hours, before the day band opens. */
    PREDAWN
  }

  /** Clock hour at which the evening band gives way to the pre-dawn band. */
  public static int PREDAWN_START_HOUR = 2;

  public static double dayMin = 900;
  public static double dayMax = 2700;
  public static double eveningMin = 900;
  public static double eveningMax = 2700;
  public static double predawnMin = 900;
  public static double predawnMax = 2700;

  /**
   * Shape (σ of the underlying normal) shared by the bands. The lognormal's median sits at the
   * band's geometric midpoint; this controls how tightly draws concentrate around it before
   * truncation. Larger values flatten the band towards uniform.
   */
  public static double shape = 0.40;

  /**
   * Aligns every band with the global trip-distance range, so that a run which does not configure
   * bands behaves as one undivided band. Called from {@link Pars#setSimulationParameters()} after
   * the global range is resolved.
   */
  public static void setDefaults() {
    dayMin = RouteChoicePars.minTripDistance;
    dayMax = RouteChoicePars.maxTripDistance;
    eveningMin = RouteChoicePars.minTripDistance;
    eveningMax = RouteChoicePars.maxTripDistance;
    predawnMin = RouteChoicePars.minTripDistance;
    predawnMax = RouteChoicePars.maxTripDistance;
  }

  /** The band a 0–23 clock hour falls in. */
  public static Band bandFor(int clockHour) {
    if (clockHour >= TimePars.NIGHT_START_HOUR || clockHour < PREDAWN_START_HOUR) {
      return Band.EVENING;
    }
    if (clockHour < TimePars.DAY_START_HOUR) {
      return Band.PREDAWN;
    }
    return Band.DAY;
  }

  public static double min(Band band) {
    switch (band) {
      case EVENING:
        return eveningMin;
      case PREDAWN:
        return predawnMin;
      default:
        return dayMin;
    }
  }

  public static double max(Band band) {
    switch (band) {
      case EVENING:
        return eveningMax;
      case PREDAWN:
        return predawnMax;
      default:
        return dayMax;
    }
  }

  /**
   * Draws a trip distance from the band by inverse CDF, so a single uniform draw yields a distance
   * with no rejection loop — the caller already wraps this in the walk-share filter's bounded
   * resampling, and nesting a second rejection loop inside it would be costly.
   *
   * @param band the band to draw from.
   * @param uniform a uniform draw in {@code [0, 1)}.
   * @return the sampled distance in metres, within {@code [min, max]}.
   */
  public static double sample(Band band, double uniform) {
    double min = min(band);
    double max = max(band);
    if (max <= min) {
      return min;
    }

    // Median at the band's geometric midpoint: symmetric in log space, so the band's own width is
    // what sets the spread and no separate mean parameter is needed.
    double mu = 0.5 * (Math.log(min) + Math.log(max));
    double pLow = cdf(min, mu);
    double pHigh = cdf(max, mu);

    return invCdf(pLow + uniform * (pHigh - pLow), mu);
  }

  private static double cdf(double x, double mu) {
    return 0.5 * (1.0 + Erf.erf((Math.log(x) - mu) / (shape * Math.sqrt(2.0))));
  }

  private static double invCdf(double p, double mu) {
    return Math.exp(mu + shape * Math.sqrt(2.0) * Erf.erfInv(2.0 * p - 1.0));
  }
}
