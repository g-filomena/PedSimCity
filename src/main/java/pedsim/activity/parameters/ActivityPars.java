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
  /**
   * Whether departures are timed by the agenda system rather than by the tuned diurnal curve in
   * {@code TimePars.computeTimeStepShare}.
   *
   * <p>On, the daily profile is a prediction built from mandatory start windows, opening hours and
   * persona preferences, things set outside this model, so it can be wrong, and being wrong is
   * informative. Off, the old curve is restored: useful for isolating the effect of this change,
   * not as a fallback when the prediction disappoints.
   */
  public static boolean useAgendaDepartureProfile = true;

  /**
   * Share of workers who walk to work, and of students who walk to their place of study.
   *
   * <p>Measured, not fitted: ISTAT, <i>Spostamenti quotidiani e nuove forme di mobilita</i>, anno
   * 2017, Figura 3: 12.0% of {@code occupati} and 27.9% of {@code scolari e studenti} reach work
   * or school on foot (17.4% of all commuters; 14.8% among employed women).
   *
   * <p>These replace the walk-share logit for the commute leg specifically. The logit's implied
   * figure was 59.4%, five times the observed one, which was enough on its own to make walking
   * commutes cost more than the entire daily metres budget.
   *
   * <p>National figures. Turin has a metro, four tram lines and above-average car ownership, so
   * the local value is plausibly lower still, but substituting a guess for a measurement would
   * give back exactly what these numbers were fetched to remove.
   */
  public static double walkShareCommuteWorker = 0.120;

  public static double walkShareCommuteStudent = 0.279;

  public static double workerShare = 0.50;
  public static double studentShare = 0.15;
  public static double retireeShare = 0.20;
  public static double flexShare = 0.15;

  // --- Habitual destination choice ---
  // Exploration vs preferential return, the two mechanisms Song, Koren, Wang & Barabasi (2010),
  // Nature Physics 6:818-823, measure on mobile-phone trajectories. The chance that the next trip
  // goes to a place never visited before decays with the number of places already known, as
  // P_new = rho * S^-gamma; the complement returns to a known place, chosen in proportion to how
  // often it has been visited. Both values are theirs: gamma = 0.21 +/- 0.02 fitted on the data,
  // rho normally distributed across users with mean 0.6. They replace a flat 0.70 reuse
  // probability that had no source, and they make the tendency to repeat a place something the
  // agent acquires rather than something it is issued with.
  /** Scale of the exploration probability. */
  public static double explorationRho = 0.60;

  /** Decay of the exploration probability in the number of places already known. */
  public static double explorationGamma = 0.21;

  /**
   * How many familiar places an agent holds at once, across all purposes.
   *
   * <p>Alessandretti, Sapiezynski, Sekara, Lehmann & Baronchelli (2018), Nature Human Behaviour
   * 2:485-491, follow about 40,000 individuals across four datasets and find that the number of
   * familiar locations a person visits at any point is a conserved quantity of roughly 25: the set
   * of places keeps turning over while its size does not grow. That is the capacity here, and it
   * replaces three-per-purpose, a figure with no source that also happened to freeze an agent's
   * geography permanently — once a purpose had three places, nothing new was ever recorded for it
   * again, so an agent could go on exploring and never learn. When the capacity is full the
   * least-visited place makes room, which is the turnover the paper measures.
   */
  public static int familiarLocationCapacity = 25;

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
