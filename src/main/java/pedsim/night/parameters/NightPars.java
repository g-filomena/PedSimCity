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

  /**
   * A street counts as busy, and so reassures rather than frightens after dark, when its agent count
   * is at or above this percentile of the non-empty streets at that moment. 20: the published
   * specification of this model (Filomena 2025, AGILE); the reassurance of others' presence is
   * Ferraro (1995), the number is the model's own.
   */
  public static double crowdednessPercentile = 20.0;

  // A/B twin testing is an opt-in experimental mode: when true it spawns abTestPairs vulnerable/
  // non-vulnerable twins instead of the census-derived population. Off by default so a normal run
  // uses the full sampled population.
  public static boolean enableLightABTesting = false;

  // Number of vulnerable/non-vulnerable twin pairs spawned in A/B mode (2 agents per pair).
  // User-configurable; independent of the census-derived population size.
  public static int abTestPairs = 72;

  /**
   * Illuminance at which darkness stops costing anything, in lux. Reassurance rises with
   * illuminance and plateaus: Fotios, Unwin and Farrall (2015) put the optimum near 10 lx, and
   * Portnov, Fotios et al. (2024) find the final breakpoint between 8.9 and 26 lx by location.
   */
  public static double reassuranceLux = 10.0;

  /**
   * How much a fully dark street costs a vulnerable walker, as a fraction of its length on top of
   * the length itself: at 1.0 an unlit street costs twice its length, a street at {@link
   * #reassuranceLux} or brighter costs its length. The direction is supported - women avoid
   * unlit routes at night (Basu, Sevtsuk et al. 2023) - the magnitude is not: no source gives the
   * metres of detour a given darkness is worth, so this is an assumption to state and sweep.
   */
  public static double darknessWeightVulnerable = 1.0;

  /**
   * The same weight for a non-vulnerable walker. Half the vulnerable one, from the ratio of
   * fear of walking alone after dark between women and men: 82% against 42% in parks and open
   * spaces (ONS 2022), over half against 26% near home (Gallup 2023). A ratio of reported fear
   * read as a ratio of costs is itself an assumption.
   */
  public static double darknessWeightNonVulnerable = 0.5;

  /**
   * Extra cost, as a fraction of length, of a street within a park or along water after dark, for
   * a vulnerable walker. Greenery and darkness together produce entrapment and avoidance (Malmö
   * focus groups, Urban Design International 2020). Magnitude unsourced; sweep it.
   */
  public static double parkWaterWeightVulnerable = 1.0;

  /** The same for a non-vulnerable walker, at the ratio used for darkness. */
  public static double parkWaterWeightNonVulnerable = 0.5;
}
