package pedsim.activity.engine;

import java.time.LocalDate;
import pedsim.activity.parameters.ActivityPars;

/**
 * Per-day stochastic weather. Each simulated day is independently rainy with
 * {@link ActivityPars#rainyDayProbability}; the draw is a pure function of the run seed and the
 * date, so every query for the same day agrees (across threads and modules) and seeded runs are
 * reproducible without storing any state.
 *
 * <p>Rain acts in two places: the release budget is scaled by
 * {@link ActivityPars#rainReleaseMultiplier} (fewer people out overall) and the agenda's chained
 * discretionary probabilities by {@link ActivityPars#rainDiscretionaryMultiplier} (optional
 * outings are cut harder than commutes, which structurally still happen).
 */
public final class Weather {

  private Weather() {}

  /** Whether the given simulated date is rainy for the run with the given seed. */
  public static boolean isRainy(LocalDate date, long seed) {
    if (!ActivityPars.useWeather) {
      return false;
    }
    // One uniform draw per (seed, day). The key is finalised with SplitMix64 because raw
    // sequential keys (consecutive days) produce heavily correlated first draws in
    // java.util.Random — hashing first makes the per-day coin flips independent.
    double draw = uniform(seed * 31L + date.toEpochDay());
    return draw < ActivityPars.rainyDayProbability;
  }

  /** SplitMix64 finaliser mapping any long key to a uniform double in [0, 1). */
  private static double uniform(long key) {
    long z = key + 0x9E3779B97F4A7C15L;
    z = (z ^ (z >>> 30)) * 0xBF58476D1CE4E5B9L;
    z = (z ^ (z >>> 27)) * 0x94D049BB133111EBL;
    z = z ^ (z >>> 31);
    return (z >>> 11) / (double) (1L << 53);
  }
}
