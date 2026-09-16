package pedsim.night.parameters;

public class NightPars {
  public enum DirectionalLuxStatistic {
    MIN,
    MEAN
  }

  /**
   * Which directional statistic {@code NightImport} reads for the entrance view of an edge. MEAN:
   * the minimum over the visibility horizon is about one lamp spacing wide (Torino's mean
   * nearest-neighbour spacing is 11.7 m), so it measures how far the entry node falls from the
   * nearest lamp rather than how the street ahead is lit. Both columns are written by
   * {@code 04_directional_lighting.py}, so this is a column choice, not a pipeline re-run.
   */
  public static DirectionalLuxStatistic directionalLuxStatistic = DirectionalLuxStatistic.MEAN;

  // GUI Configurable parameters for light sensitivity (Lux)
  public static double minVulnerableLightSensitivity = 5.0;
  public static double maxVulnerableLightSensitivity = 15.0;
  // Non-vulnerable agents treat an edge as dark only below 5 lux (the pipeline's
  // UNLIT_LUX_THRESHOLD).
  public static double nonVulnerableLightSensitivity = 5.0;

  // Nominal illuminance (lux) credited to edges known lit only via the binary "lit" flag (no
  // continuous mean_lux), for the per-agent lux metric. Defaults to the lit/unlit threshold as a
  // conservative lower bound (a lit edge is at least this bright); raise for a more representative
  // lit-street value.
  public static double litEdgeNominalLux = 5.0;

  /**
   * A stretch of an edge below this illuminance is an unlit gap, and an edge carrying one fails
   * the lighting gate however bright its average is. 5.0 lux is the pipeline's service level
   * ({@code lighting.MIN_LUX}), which is what {@code min_lux} is measured against. Not the agent's
   * own sensitivity: {@code min_lux} is the darkest 2 m sample point on the whole edge, and asking
   * an extreme value to clear a 15-lux threshold fails nearly every street in the city.
   */
  public static double darkSpotLuxThreshold = 5.0;

  public static double crowdednessPercentile = 80.0;

  // A/B twin testing is an opt-in experimental mode: when true it spawns abTestPairs vulnerable/
  // non-vulnerable twins instead of the census-derived population. Off by default so a normal run
  // uses the full sampled population.
  public static boolean enableLightABTesting = false;

  // Number of vulnerable/non-vulnerable twin pairs spawned in A/B mode (2 agents per pair).
  // User-configurable; independent of the census-derived population size.
  public static int abTestPairs = 72;

  /**
   * Upper bound on P(reroute) in {@code NightBehaviour.rerouteOrIncreaseSpeed()} as the current
   * edge approaches full darkness. P(reroute) is 0.5 at or above the agent's own sensitivity
   * threshold and rises linearly toward this value as the edge darkens below it, so how dark the
   * street is decides whether the agent turns off it or merely walks it faster. A starting value,
   * not a calibrated one.
   */
  public static double maxRerouteProbabilityInDarkness = 0.9;

  /**
   * Ceiling on the route-planning cost multiplier applied to a <b>known</b> edge whose
   * {@code mean_lux} falls below the travelling agent's own sensitivity threshold; 1.0 disables the
   * penalty and restores planning that ignores light entirely. A starting value, not a calibrated
   * one.
   *
   * <p>This is the one parameter in the module that moves the <i>plan</i> rather than the
   * reaction, so it is also the one whose effect a lighting experiment most needs to state: with
   * it at 1.0 a night agent only ever reacts to a dark street it has already reached.
   */
  public static double maxKnownDarkEdgeCostMultiplier = 1.5;
}
