package pedsim.activity.agents;

import java.time.LocalDateTime;
import pedsim.activity.parameters.ActivityPars;
import pedsim.core.parameters.TimePars;

/**
 * When people leave home, derived from what they are leaving home to do.
 *
 * <p>This replaces {@link TimePars#computeTimeStepShare}, which spreads the day's metres budget
 * over the day along a curve of tuned Gaussian peaks. That curve reproduces a plausible diurnal
 * profile, but it contains no behaviour: its peak times and widths were chosen because the shape
 * came out right, so a model carrying it cannot predict a diurnal profile — it is told one. The
 * comparison against observed presence then compares an input with an observation.
 *
 * <p>Here the same share is built from things that are set outside this model:
 *
 * <ul>
 *   <li><b>mandatory start windows</b> — when workers and students have to be somewhere
 *       ({@link Persona}), and which personas attend on this day of the week;
 *   <li><b>opening hours</b> — when each {@link ActivityPurpose} can be started at all, which come
 *       from OSM tagging rather than from this model;
 *   <li><b>persona preference weights</b> — what mix of purposes each persona pursues;
 *   <li><b>the realised persona mix</b> — census-conditioned per home zone, so a city of retirees
 *       gets a midday profile without anything being said about midday.
 * </ul>
 *
 * <p>Commutes are not here. They are generated per agent from whether that agent has a job and
 * walks to it (see {@code ActivityAgent.planMandatoryDeparture}); this curve times the
 * discretionary travel that happens around them.
 *
 * <p>None of those is fitted to a diurnal observation, so the profile that comes out is a
 * prediction. If it fits observed presence worse than the tuned curve did, that is a finding about
 * the behavioural model and not a reason to reach for the curve again.
 *
 * <p>The result is still a population-level share rather than a per-agent appointment: agents do
 * not hold departure times. What is agenda-derived is the aggregate timing, which is the part that
 * is testable.
 */
public final class DepartureProfile {

  /** Discretionary purposes, in the order of {@link Persona}'s preference-weight array. */
  private static final ActivityPurpose[] DISCRETIONARY = {
    ActivityPurpose.SHOPPING,
    ActivityPurpose.ERRANDS,
    ActivityPurpose.DINING,
    ActivityPurpose.NIGHTLIFE,
    ActivityPurpose.LEISURE,
    ActivityPurpose.STROLL
  };

  /**
   * Nobody sets out at 04:00 because a purpose is technically open. STROLL and NIGHTLIFE nominally
   * span the small hours; this is the window within which departures are placed at all, so the
   * overnight band stays thin without a peak being drawn there. A waking-hours bound, not a fitted
   * one.
   */
  private static final double WAKING_FROM = 6.0;

  private static final double WAKING_TO = 25.5; // 01:30 next day, for nightlife returns

  private final double[] density; // per bin, integrating to 1.0 across the day
  private final double binHours;

  private DepartureProfile(double[] density, double binHours) {
    this.density = density;
    this.binHours = binHours;
  }

  /**
   * Builds the discretionary departure profile.
   *
   * <p>No day-of-week parameter, and that is the point. The profile used to carry the commute as a
   * lump of mass spread over the mandatory start windows, so the weekend differed because a share
   * was computed to be zero. Commutes are now generated per agent from whether that agent has a job
   * and walks to it, which leaves this curve describing discretionary travel only - and a
   * discretionary day looks the same on a Tuesday as on a Saturday, as far as anything in this
   * model knows. The weekend difference falls out of workers not commuting, not out of a second
   * curve. If observed presence says Saturday afternoons differ in shape as well as in volume,
   * that is a finding this profile can now be wrong about.
   *
   * @param personaShares the realised mix of the sampled population, in WORKER, STUDENT, RETIREE,
   *     FLEX order - the census-conditioned mix the agents actually got, not the global constants
   * @return the profile, a density over the day integrating to 1.0
   */
  public static DepartureProfile discretionary(double[] personaShares) {
    int bins = (int) Math.round(24.0 * 60.0 / TimePars.releaseAgentsEveryMinutes);
    double binHours = 24.0 / bins;
    double[] density = new double[bins];

    Persona[] personas = {Persona.WORKER, Persona.STUDENT, Persona.RETIREE, Persona.FLEX};
    for (int p = 0; p < personas.length; p++) {
      double share = personaShares != null && p < personaShares.length ? personaShares[p] : 0.0;
      if (share > 0.0) {
        addDiscretionary(density, binHours, personas[p], share);
      }
    }

    normalise(density, binHours);
    return new DepartureProfile(density, binHours);
  }

  /** Spreads a persona's discretionary mass across its purposes' opening windows. */
  private static void addDiscretionary(
      double[] density, double binHours, Persona persona, double mass) {
    double[] weights = persona.getPurposeWeights();
    double total = 0.0;
    for (double w : weights) {
      total += w;
    }
    if (total <= 0.0) {
      spread(density, binHours, WAKING_FROM, WAKING_TO, mass);
      return;
    }
    for (int i = 0; i < DISCRETIONARY.length && i < weights.length; i++) {
      if (weights[i] <= 0.0) {
        continue;
      }
      ActivityPurpose purpose = DISCRETIONARY[i];
      double from = purpose.getOpenHour();
      double to = purpose.getCloseHour();
      if (to <= from) {
        to += 24.0; // wraps midnight (nightlife)
      }
      // Departures happen when the activity can be started, inside waking hours.
      from = Math.max(from, WAKING_FROM);
      to = Math.min(to, WAKING_TO);
      if (to <= from) {
        continue;
      }
      spread(density, binHours, from, to, mass * weights[i] / total);
    }
  }

  /** Adds {@code mass}, spread uniformly over {@code [from, to)} hours, into the bins. */
  private static void spread(
      double[] density, double binHours, double from, double to, double mass) {
    if (!(to > from) || mass <= 0.0) {
      return;
    }
    double perHour = mass / (to - from);
    for (int b = 0; b < density.length; b++) {
      double binFrom = b * binHours;
      double binTo = binFrom + binHours;
      // The window may run past midnight; the tail wraps into the early bins.
      double overlap = overlap(binFrom, binTo, from, to) + overlap(binFrom + 24.0, binTo + 24.0, from, to);
      if (overlap > 0.0) {
        density[b] += perHour * overlap;
      }
    }
  }

  private static double overlap(double aFrom, double aTo, double bFrom, double bTo) {
    return Math.max(0.0, Math.min(aTo, bTo) - Math.max(aFrom, bFrom));
  }

  /** Scales the density so its area over the day is exactly 1.0. */
  private static void normalise(double[] density, double binHours) {
    double area = 0.0;
    for (double d : density) {
      area += d;
    }
    if (area <= 0.0) {
      java.util.Arrays.fill(density, 1.0 / (density.length * binHours));
      return;
    }
    double scale = 1.0 / area;
    for (int b = 0; b < density.length; b++) {
      density[b] *= scale;
    }
  }

  /**
   * The share of the day's metres budget belonging to the release event at this time.
   *
   * <p>Same contract as {@link TimePars#computeTimeStepShare}: the values across a day's release
   * events sum to 1.0, so the whole budget is spent and no more.
   *
   * @param time the moment of the release event
   * @return the share, in {@code [0, 1]}
   */
  public double share(LocalDateTime time) {
    double hour =
        time.getHour() + time.getMinute() / 60.0 + time.getSecond() / 3600.0;
    int bin = (int) Math.floor(hour / binHours);
    if (bin < 0 || bin >= density.length) {
      return 0.0;
    }
    return density[bin];
  }

}
