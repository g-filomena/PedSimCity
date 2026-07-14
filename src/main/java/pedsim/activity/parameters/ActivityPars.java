package pedsim.activity.parameters;

/**
 * Parameters for the activity-based behavioural layer: persona mix, habitual destination choice,
 * daily agenda size, the seasonal-daylight model, the walk-share filter, weather and the
 * persona-conditioned release.
 *
 * <p>All values have sensible defaults so cities without the corresponding data degrade
 * gracefully; each can be overridden per run before {@code runJobs()}.
 */
public class ActivityPars {

  // --- Persona mix (fractions of the sampled agent population; should sum to 1.0) ---
  public static double workerShare = 0.50;
  public static double studentShare = 0.15;
  public static double retireeShare = 0.20;
  public static double flexShare = 0.15;

  // --- Habitual destination choice ---
  /** Probability that a discretionary trip reuses one of the agent's favourite places. */
  public static double habitualDestinationProbability = 0.70;

  /** How many favourite places an agent keeps per activity purpose. */
  public static int maxFavouritesPerPurpose = 3;

  // --- Daily agenda ---
  /** Probability that a released non-working agent plans a second discretionary activity. */
  public static double secondActivityProbability = 0.40;

  /** Probability that a working agent chains one discretionary activity after work. */
  public static double postWorkActivityProbability = 0.45;

  /** Probability that a working agent chains a second discretionary activity after work. */
  public static double secondPostWorkActivityProbability = 0.15;

  // --- Seasonal daylight ---
  /**
   * Behavioural darkness follows the seasonal sunrise/sunset model in {@code Daylight} when true;
   * the fixed {@code TimePars.DAY_START_HOUR}/{@code NIGHT_START_HOUR} window otherwise. Exporter
   * day/night volume aggregation always uses the fixed window so outputs stay comparable.
   */
  public static boolean useSeasonalDaylight = true;

  /** Latitude used by the sunrise/sunset model (degrees; default Liverpool). */
  public static double latitudeDegrees = 53.4;

  /** Civil-twilight buffer: it is still light this many minutes before sunrise / after sunset. */
  public static double twilightBufferMinutes = 30;

  // --- Walk-share filter ---
  /**
   * Applies a logit-style walk probability to sampled trip distances at release time: most trips
   * under ~1 km are kept, few over ~3 km survive, reshaping the trip-length distribution toward
   * observed walking mode shares. The {@code metersPerDay} budget stays the anchor; the filter only
   * changes the mix of distances it is spent on.
   */
  public static boolean useWalkShareFilter = true;

  /** Distance (m) at which the walk probability is 50%. */
  public static double walkShareHalfDistance = 1800.0;

  /** Logit steepness (per metre); 0.0025 gives ~88% at 1 km and ~5% at 3 km. */
  public static double walkShareSteepness = 0.0025;

  // --- Weather ---
  /** Per-day stochastic weather: rainy days suppress walking, discretionary trips most. */
  public static boolean useWeather = true;

  /** Probability that any given simulated day is rainy. */
  public static double rainyDayProbability = 0.25;

  /** Release-budget multiplier on rainy days (suppresses overall walking volume). */
  public static double rainReleaseMultiplier = 0.75;

  /**
   * Multiplier on the agenda's chained-activity probabilities on rainy days: rain cuts optional
   * second stops and post-work outings harder than commutes (which still happen).
   */
  public static double rainDiscretionaryMultiplier = 0.6;

  // --- Census-conditioned personas ---
  /**
   * When the census zones carry age-structure shares ({@code retiree_pct}, {@code student_pct}),
   * personas are sampled per home zone instead of from the global shares above.
   */
  public static boolean useCensusPersonas = true;

  // --- Persona-conditioned release ---
  /** Weights who is released when by persona × hour (commuters at peaks, retirees midday). */
  public static boolean usePersonaReleaseWeights = true;

  // --- POI classification ---
  /** Maximum distance (m) between a tagged POI/building and the network node it attracts to. */
  public static double poiClaimRadius = 300.0;

  /**
   * Building footprint area (m²) worth one venue's attraction. POIs (venues) always weigh 1.0;
   * a classified building weighs {@code max(1, footprintArea / this)} so a 10,000 m² office block
   * attracts proportionally more WORK trips than a corner office. 200 m² is the preparation
   * pipeline's minimum building footprint, i.e. the smallest building ≈ one venue.
   */
  public static double buildingAreaPerAttractionUnit = 200.0;

  private ActivityPars() {}
}
