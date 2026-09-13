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
   * <p>Measured, not fitted, and <b>Turin-specific</b>: ISTAT, <i>Matrice del pendolarismo</i>,
   * 15th census (2011), restricted to residents of Torino (001/272) whose place of work or study is
   * in the same municipality - which is the only commute this model can represent. Of 233,399
   * intra-Turin work commutes, 16.3% are made on foot; of 121,512 study commutes, 38.0%. Derivation
   * and the full mode split in {@code COMMUTE_DISTANCE.md}.
   *
   * <p>These replace the national 12.0% / 27.9% (ISTAT 2017, <i>Spostamenti quotidiani</i>, Fig. 3),
   * which answered a broader question: every commute in Italy, including the half of Piedmont's that
   * leaves its own municipality and could never be walked.
   *
   * <p><b>These are now the check, not an input.</b> Nothing reads them to decide anything: the
   * per-agent decision is {@code ActivityAgent.decideCommuteMode}, a draw against the walk-share
   * curve at the agent's own home-work distance, and the population share that comes out is
   * logged against these two figures once per simulated day by
   * {@code ActivityTravelDemand.prepare}. They stopped being an input when the commute stopped
   * being a lottery: a share of the day's trips had to be told to the departure profile only
   * because the model could not generate the commutes it was certain to have.
   *
   * <p>Watch that log. The curve is English and pooled over all trip purposes, while these are
   * Italian and specific to commuting, and commutes are systematically less walked than
   * discretionary trips of the same length. A large gap is a statement about the curve, or about
   * where the model is putting workplaces, and not something to close by moving a number.

   *
   * <p>National figures. Turin has a metro, four tram lines and above-average car ownership, so
   * the local value is plausibly lower still, but substituting a guess for a measurement would
   * give back exactly what these numbers were fetched to remove.
   */
  public static double walkShareCommuteWorker = 0.163;

  public static double walkShareCommuteStudent = 0.380;

  /**
   * Distance decay on workplace choice: a WORK-tagged node's attraction is divided by
   * {@code max(10, d)^this} when {@code RouteChoicePars.useGravityModel} is set.
   *
   * <p><b>Uncalibrated, and currently wrong.</b> It was a bare local constant commented "a standard
   * gravity model decay parameter". With {@link #workplaceMinDistanceMetres} it concentrates
   * workplaces just past the floor, at a distance nearly everyone walks, and the model then walks
   * 47% of work commutes against the 16.3% ISTAT measures for Turin.
   *
   * <p>What it has to reproduce is in {@code COMMUTE_DISTANCE.md}: the walked share above, and the
   * four-band distribution of walked commute lengths. The share and the shape are one target -
   * matching 16.3% by pushing workplaces far away while the walked commutes come out too long is
   * worse than the present state, not better.
   */
  public static double workplaceDistanceDecay = 1.0;

  /**
   * Distance (m) at which a <b>commute</b> is walked with probability 0.5, and the logit steepness.
   *
   * <p>Separate from the pooled DfT National Travel Survey curve (NTS0308, England 2025: a
   * half-distance near 2,290 m at a steepness of 0.00084) that the model once used for every trip,
   * and that separation is the finding. The NTS curve pools every trip purpose; a commute is not a
   * discretionary trip and is walked far less at
   * the same distance. Holding one curve for both is what made the model walk 47% of work commutes
   * where Turin walks 16.3%, and no workplace distribution could repair it - the share and the
   * length distribution moved in opposite directions across the whole decay range.
   *
   * <p>Fitted, not invented: {@code CommuteCalibration} scores candidate curves against the ISTAT
   * commuting matrix for Torino (see {@code COMMUTE_DISTANCE.md}) - the 16.3% walked share and the
   * four-band length distribution, five targets for two parameters plus the decay. The optimum is
   * sharp: 800 m / 0.0015 at a decay of 1.0 reproduces 16.1% and 76.4 / 19.9 / 3.7 / 0.1 against
   * 16.3% and 76.2 / 19.0 / 3.4 / 1.3, and the misfit roughly quintuples one decay step either side.
   *
   * <p>Turin against England, at 800 m: 0.50 here, 0.78 on the NTS curve.
   */
  public static double walkShareCommuteHalfDistance = 800.0;

  public static double walkShareCommuteSteepness = 0.0015;

  /**
   * The same pair, for the journey to a place of study.
   *
   * <p>Separate from the work curve for the reason the work curve is separate from the general one:
   * purpose. A fourteen-year-old deciding how to get to school is not an adult deciding how to get
   * to work - no car of their own, a catchment rather than a labour market, sometimes a parent
   * walking them. ISTAT measures the two separately and they differ by more than a factor of two:
   * 38.0% of intra-Turin study commutes are walked against 16.3% of work commutes.
   *
   * <p>Fitted the same way and to the same five-target structure, on the study rows of the matrix:
   * with {@link #educationDistanceDecay} at 1.5, a curve of 1,200 m / 0.0019 gives 38.6% and
   * 87.7 / 11.3 / 1.0 / 0.0 against 38.0% and 87.5 / 11.1 / 0.9 / 0.5 - misfit 1.6.
   *
   * <p>The decay alone does not get there. At the work curve, the best education decay reproduces
   * the length distribution almost exactly and still walks only 29.8%, eight points short: students
   * walk further before giving up, which is a property of the curve and not of where schools are.
   */
  public static double walkShareStudentHalfDistance = 1200.0;

  public static double walkShareStudentSteepness = 0.0019;

  /**
   * Distance decay on the choice of a place of study, separate from
   * {@link #workplaceDistanceDecay}.
   *
   * <p>1.5 against work's 1.0: schools are more local than jobs, which is what a catchment means,
   * and ISTAT sees it directly - 87.5% of walked study trips are under fifteen minutes against
   * 76.2% of walked work trips. Sharing one decay between the two was never a decision, only the
   * same method called twice.
   */
  public static double educationDistanceDecay = 1.5;

  /**
   * Sweep the workplace decay against the ISTAT commuting matrix and exit, without simulating.
   * See {@code pedsim.activity.engine.CommuteCalibration}.
   */
  public static boolean calibrateCommute = false;

  /** Home locations drawn for that sweep. Cheap: no agents, no days, no routes. */
  public static int calibrationHomes = 20000;

  /**
   * Closest a workplace may be assigned, in metres.
   *
   * <p>No source. It exists because a {@code 1/d^2} decay with no floor gives the nearest tagged
   * node almost all the mass and everyone works next door. It was written as
   * {@code RouteChoicePars.minTripDistance * 0.6} - the discretionary walking range - which is the
   * conceptual error that produced the 2,700 m commute cap: a commute is not a discretionary trip
   * and has no business being sized by one. Stated in metres here so it is visibly a number
   * somebody chose, and calibrated together with {@link #workplaceDistanceDecay}.
   */
  public static double workplaceMinDistanceMetres = 0.0;

  public static double workerShare = 0.50;
  public static double studentShare = 0.15;
  public static double retireeShare = 0.20;
  public static double flexShare = 0.15;

  // --- Destination choice as a choice (experimental, behind the switch below) ---
  /**
   * Switches destination choice from "find a node at the sampled distance" to "choose among the
   * opportunities around here". See {@link pedsim.activity.agents.DestinationChoice}.
   *
   * <p>On by default since 11 September 2026. The switch stays so the two can still be compared on
   * the same seed: the old path is handed a trip-length distribution, this one produces one, and
   * whether the produced one matches what travel surveys observe is the question worth asking.
   */
  public static boolean useDestinationChoice = true;

  /**
   * Weight on the size term, {@code ln(1 + attraction)}. At 1.0 a node with ten times the
   * opportunities of another is about 2.4 utility points ahead of it, before distance.
   */
  public static double sizeWeight = 1.0;



  /**
   * Impedance per metre. **This is the coefficient that shapes the trip-length distribution**, and
   * the only one here that has to be calibrated: it is set so that the lengths the model produces
   * match an observed distribution, rather than being handed one. At 0.0012 the utility cost of a
   * kilometre is 1.2 points, which is roughly the pull of a node with three times the attraction.
   *
   * <p>Provisional. It has not been fitted to anything yet; the run that would fit it needs the
   * observed distribution to fit against, and for Italy that means the Audimob microdata.
   */
  public static double distanceWeight = 0.0012;

  /**
   * Bonus for a place the agent already knows. A term, not a branch: a familiar place competes with
   * a nearer or better one instead of overriding both, which is what the old reuse probability did.
   */
  public static double habitWeight = 1.5;

  /**
   * Walked trips one person makes on an average day.
   *
   * <p>ISFORT 22nd report: 2.53 trips a day for the mobile population, who are 80.8% of everyone,
   * so 2.04 trips per resident per day - across all modes. Walking is about 25% of trips in a large
   * north-western city, which gives **0.51 walked trips per resident per day**.
   *
   * <p>The mode share matters and is easy to drop: 2.04 counts every trip, most of them driven, and
   * using it whole would have the model walk four times what anyone walks.
   *
   * <p>This is a count of <i>legs</i>, which is what the survey counts, and it is the only figure
   * here the survey gives directly. What the model does with it is subtract the legs its structural
   * commutes will walk today and buy the remainder as discretionary chains, dividing by the chain
   * length {@code DailyAgenda.expectedLegs} computes rather than by a constant. It replaced
   * {@code tripChainsPerPersonPerDay = 0.21}, which was this figure with a chain length of 2.4
   * already divided into it - so changing an agenda probability silently changed how many trips the
   * population made, while the survey figure it came from stayed put.
   *
   * <p>Worth noting what it predicts. At about 1,734 m a leg, 0.51 legs a day comes to roughly
   * 885 m walked per resident per day - inside the 600-1,000 m that
   * {@code Pars.metersPerDayPerPerson} was derived from, by a route that shares only its first two
   * figures. Two derivations meeting is not proof, but it is the kind of check the metres anchor
   * could never offer, because it was the thing being hit rather than the thing being predicted.
   */
  public static double walkedTripsPerPersonPerDay = 0.51;

  /**
   * How many alternatives the choice is actually computed over.
   *
   * <p>Drawn uniformly from the opportunities within reach. Uniform sampling is what makes this
   * free: the correction it would need is identical for every alternative and cancels in the
   * softmax, so the sampled choice is the same choice. Enumerating them all cost roughly five times
   * as much per leg and changed nothing.
   */
  public static int choiceSetSize = 60;

  /**
   * How far out the choice set reaches, in metres.
   *
   * <p>It was described here as bounding the work rather than the behaviour, on the grounds that
   * anything beyond it has a utility far below the near candidates. **Measured, that is not true at
   * 3,000 m.** Full Torino, 1,693 agents, one day, the same fixed seed, varying only this:
   *
   * <pre>
   *   1,500 m   715 trips   mean leg 1,103 m
   *   3,000 m   644 trips   mean leg 1,384 m
   *   6,000 m   664 trips   mean leg 1,438 m
   *  12,000 m   639 trips   mean leg 1,371 m
   * </pre>
   *
   * <p>It converges by 6,000 m - the last step moves the mean by less than the run-to-run spread -
   * but 1,500 m truncates the trip-length distribution badly and 3,000 m still costs about 4% of
   * the mean leg against the converged value. Since the trip-length distribution is the headline
   * output of the destination-choice work, a 4% truncation is a result, not a rounding.
   *
   * <p>Either this should be 6,000 and the claim above becomes true again, or it stays at 3,000 and
   * the 4% is reported with every trip-length figure. That is a modelling call, not a tidy-up, so
   * the value is left where it was and the measurement written down.
   */
  public static double choiceSetRadiusMetres = 3000.0;

  // --- Habitual destination choice (the mechanism the switch above replaces) ---
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
