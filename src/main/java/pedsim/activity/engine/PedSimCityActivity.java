package pedsim.activity.engine;

import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import pedsim.activity.agents.ActivityAgent;
import pedsim.activity.agents.ActivityPurpose;
import pedsim.activity.agents.DailyAgenda;
import pedsim.activity.agents.DepartureProfile;
import pedsim.activity.agents.Persona;
import pedsim.activity.parameters.ActivityPars;
import pedsim.core.agents.Agent;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.parameters.Pars;
import pedsim.core.parameters.RouteChoicePars;
import pedsim.core.parameters.TimePars;
import pedsim.core.parameters.TripDistanceBands;
import pedsim.transit.TransitStop;
import pedsim.transit.TransitVehicle;
import sim.field.geo.VectorLayer;
import sim.graph.NodeGraph;

public class PedSimCityActivity extends PedSimCity {

  private static final java.util.logging.Logger logger = java.util.logging.Logger.getLogger(PedSimCityActivity.class.getName());

  // 24h activity clock: true between ~20:00 and ~06:00. Driven by ActivityEngine.onStepUpdate.
  public boolean isDark = false;

  // Raw census layer as loaded from <City>_censusData.gpkg: one polygon set carrying population
  // structure only (residence_pct, residents, plus module columns like vulnerability_pct).
  // Destination attraction comes from the OSM-tag purpose weights, not from the census.
  public static VectorLayer censusLayer = new VectorLayer();

  // Census zones built from the raw layer, each with its proximity-assigned network nodes and its
  // residence weight.
  public static List<CensusZone> censusZones = new ArrayList<>();

  // Optional dedicated POI layer (<City>_POIs.gpkg) carrying OSM-like use tags; buildings may
  // carry the same tags. Both feed the purpose classifier below.
  public static VectorLayer poisLayer = new VectorLayer();

  // Per-node attraction weights per activity purpose, derived from OSM-like use tags on the
  // buildings/POI layers by PoiClassifier. Empty when the city carries no tags.
  public static Map<ActivityPurpose, Map<NodeGraph, Double>> nodesPurposeWeight =
      new EnumMap<>(ActivityPurpose.class);

  // Multi-Modal Transit Static Data Structures
  public static List<TransitStop> allTransitStops = new ArrayList<>();
  public static Map<Integer, TransitStop> transitStopsByNodeId = new HashMap<>();
  public static List<TransitStop> metroStops = new ArrayList<>();
  public static List<TransitStop> tramStops = new ArrayList<>();
  public static List<TransitStop> busStops = new ArrayList<>();
  // Mutated from agent code, which runs in parallel when Pars.parallel is set, so it must be
  // concurrent like the other shared collections. Write through countTrip(), never put().
  public static Map<String, Integer> tripsByMode = new java.util.concurrent.ConcurrentHashMap<>();

  /** Records one completed trip against a mode ("WALK", "METRO", "TRAM", "BUS"). */
  public static void countTrip(String mode) {
    tripsByMode.merge(mode, 1, Integer::sum);
  }
  public static Map<Agent, TransitStop> agentTransitDestinations = new HashMap<>();

  public PedSimCityActivity(long seed, int job, ScenarioConfig scenarioConfig) {
    super(seed, job, scenarioConfig);
  }

  /**
   * Populates the environment with {@link pedsim.activity.agents.ActivityAgent}s using the
   * census-aware {@link ActivityPopulate} strategy.
   */
  @Override
  protected void populateEnvironment() {
    new ActivityPopulate().populate(this);
  }

  @Override
  protected void startMovingAgents() {
    super.startMovingAgents();

    // Spawn and schedule moving multi-modal transit vehicles if stations are present and transit is enabled
    if (pedsim.core.parameters.RouteChoicePars.usePublicTransport && !allTransitStops.isEmpty()) {
      if (!metroStops.isEmpty()) {
        for (int i = 0; i < 4; i++) {
          TransitVehicle metro = new TransitVehicle("METRO_M1_" + i, "METRO", "M1", 400);
          metro.stopSequence.addAll(metroStops);
          metro.currentStopIndex = (i * metroStops.size() / 4) % metroStops.size();
          schedule.scheduleRepeating(metro, 1, 1.0);
        }
      }
      if (!tramStops.isEmpty()) {
        for (int i = 0; i < 10; i++) {
          TransitVehicle tram = new TransitVehicle("TRAM_LINE_" + i, "TRAM", "4", 150);
          tram.stopSequence.addAll(tramStops);
          tram.currentStopIndex = (i * tramStops.size() / 10) % tramStops.size();
          schedule.scheduleRepeating(tram, 1, 1.0);
        }
      }
      if (!busStops.isEmpty()) {
        for (int i = 0; i < 20; i++) {
          TransitVehicle bus = new TransitVehicle("BUS_LINE_" + i, "BUS", "68", 80);
          bus.stopSequence.addAll(busStops);
          bus.currentStopIndex = (i * busStops.size() / 20) % busStops.size();
          schedule.scheduleRepeating(bus, 1, 1.0);
        }
      }
      logger.info(String.format("Multi-Modal Transit Vehicles scheduled: 4 Metro, 10 Tram, 20 Bus fleets active across %d stations.", allTransitStops.size()));
    }
  }

  @Override
  public void finish() {
    super.finish();
    printTransitSummary();
  }

  public static void printTransitSummary() {
    // Nothing to report when the city has no transit layer: every line would read zero, which
    // has previously been misread as "no trips were made".
    if (allTransitStops.isEmpty()) {
      return;
    }
    System.out.println("\n============================================================");
    System.out.println("            PEDSIMCITY MULTI-MODAL TRANSIT SUMMARY          ");
    System.out.println("============================================================");
    int metroTrips = tripsByMode.getOrDefault("METRO", 0);
    int tramTrips = tripsByMode.getOrDefault("TRAM", 0);
    int busTrips = tripsByMode.getOrDefault("BUS", 0);
    int walkTrips = tripsByMode.getOrDefault("WALK", 0);
    int totalTrips = metroTrips + tramTrips + busTrips + walkTrips;
    if (totalTrips == 0) totalTrips = 1;

    System.out.printf("  [MODE SPLIT ANALYSIS]\n");
    System.out.printf("  - METRO      : %6d trips (%.1f%%)\n", metroTrips, 100.0 * metroTrips / totalTrips);
    System.out.printf("  - TRAM       : %6d trips (%.1f%%)\n", tramTrips, 100.0 * tramTrips / totalTrips);
    System.out.printf("  - BUS        : %6d trips (%.1f%%)\n", busTrips, 100.0 * busTrips / totalTrips);
    System.out.printf("  - WALK ONLY  : %6d trips (%.1f%%)\n", walkTrips, 100.0 * walkTrips / totalTrips);
    System.out.println("------------------------------------------------------------");
    int totalWaiting = 0;
    for (TransitStop stop : allTransitStops) {
      totalWaiting += stop.waitingPassengers.size();
    }
    System.out.printf("  [STATION INFRASTRUCTURE]\n");
    System.out.printf("  - Active Transit Stops : %4d stops\n", allTransitStops.size());
    System.out.printf("  - Platform Queue Totals: %4d waiting agents\n", totalWaiting);
    System.out.println("============================================================\n");
  }

  /** Whether the current simulated day is rainy (see {@link Weather}). */
  public boolean isRainyNow() {
    return Weather.isRainy(
        TimePars.getTime(schedule.getSteps()).toLocalDate(), seed());
  }

  /** Rainy days suppress the walking volume: fewer releases per time step. */
  @Override
  public double releaseBudgetMultiplier(LocalDateTime time) {
    if (!ActivityPars.useWeather) {
      return 1.0;
    }
    return Weather.isRainy(time.toLocalDate(), seed()) ? ActivityPars.rainReleaseMultiplier : 1.0;
  }

  /**
   * Persona × hour release affinity: commuter personas are favoured at the morning/evening peaks,
   * retirees at midday (see {@link pedsim.activity.agents.Persona#releaseAffinity}).
   */
  @Override
  public double releaseCandidateWeight(Agent agent, int hour) {
    if (!ActivityPars.usePersonaReleaseWeights
        || !(agent instanceof ActivityAgent activityAgent)
        || activityAgent.getPersona() == null) {
      return 1.0;
    }
    return activityAgent.getPersona().releaseAffinity(hour);
  }

  /** Rebuilt when the simulated day changes; the profile depends on the day of week. */
  private DepartureProfile departureProfile;

  private java.time.LocalDate departureProfileDay;

  /**
   * Departures timed by the agenda system: mandatory start windows, opening hours and persona
   * preferences, instead of the tuned peaks of {@code TimePars.computeTimeStepShare}.
   *
   * <p>Rebuilt once per simulated day, because which personas attend a mandatory activity depends
   * on the day of week; the weekend profile then falls out of the personas not working, rather
   * than out of a second hand-shaped curve.
   */
  @Override
  public double departureShare(java.time.LocalDateTime time) {
    if (!ActivityPars.useAgendaDepartureProfile) {
      return super.departureShare(time);
    }
    java.time.LocalDate day = time.toLocalDate();
    if (departureProfile == null || !day.equals(departureProfileDay)) {
      double expectedTourMeters =
          DailyAgenda.expectedLegs(Persona.FLEX, false, isRainyNow())
              * expectedWalkedLegMeters(TripDistanceBands.bandFor(12));
      double commuteShare =
          DepartureProfile.commuteShareOfTours(
              day.getDayOfWeek(), Pars.metersPerDayPerPerson, expectedTourMeters);
      departureProfile = DepartureProfile.forDay(day.getDayOfWeek(), commuteShare);
      departureProfileDay = day;
    }
    return departureProfile.share(time);
  }

  /**
   * Tour length in legs: home, then optionally work, then the agenda's stops, then home. Delegates
   * to the agent so the expectation is computed from the same persona, commute test and weather
   * that {@link pedsim.activity.agents.DailyAgenda#build} will use when the agenda is actually
   * built.
   */
  @Override
  public double expectedTourLegs(Agent agent) {
    if (agent instanceof ActivityAgent activityAgent) {
      return activityAgent.expectedTourLegs();
    }
    // No agent in hand: the typical tour, used only to size the carried residual. Non-commuting
    // because most releases across the day are discretionary.
    return DailyAgenda.expectedLegs(null, false, isRainyNow())
        + ActivityPars.secondActivityProbability;
  }

  /**
   * Walk-share filter: logit acceptance of sampled trip distances, so most short trips are walked
   * and few long ones are — the released trip-length mix follows observed walking mode shares.
   */
  @Override
  public double tripAcceptanceProbability(double meters) {
    return walkShareProbability(meters);
  }

  /**
   * Probability that a trip of this length is walked rather than made some other way.
   *
   * <p>Separated from the draw so it can be integrated: the expected
   * length of a leg the model actually walks is not the mean of the distance band, because this
   * filter removes the long tail of it. Anything reasoning about how far a tour goes has to
   * integrate against this rather than use the band mean.
   *
   * @param meters the sampled trip distance
   * @return the acceptance probability, in {@code [0, 1]}
   */
  public static double walkShareProbability(double meters) {
    if (!ActivityPars.useWalkShareFilter) {
      return 1.0;
    }
    return 1.0
        / (1.0
            + Math.exp(
                ActivityPars.walkShareSteepness
                    * (meters - ActivityPars.walkShareHalfDistance)));
  }

  /**
   * Mean length of a leg the model actually walks, integrating the distance band against the
   * walk-share filter.
   *
   * <p>Quadrature over the band's inverse CDF: {@code TripDistanceBands.sample} maps a uniform
   * draw to a distance, so evaluating it on a regular grid of quantiles and weighting each by its
   * acceptance probability gives {@code E[d | walked]} without sampling.
   *
   * @param band the trip-distance band in force
   * @return the expected walked leg length in metres
   */
  public static double expectedWalkedLegMeters(TripDistanceBands.Band band) {
    int steps = 200;
    double weighted = 0.0;
    double weight = 0.0;
    for (int i = 0; i < steps; i++) {
      double u = (i + 0.5) / steps;
      double d = TripDistanceBands.sample(band, u);
      double accept = walkShareProbability(d);
      weighted += d * accept;
      weight += accept;
    }
    return weight > 0.0 ? weighted / weight : RouteChoicePars.avgTripDistance;
  }

  /** Clears all static data structures to allow for a clean simulation restart. */

  public static void clearStaticData() {
    // clear() the layers themselves: getGeometries() returns a defensive copy.
    censusLayer.clear();
    poisLayer.clear();
    censusZones.clear();
    nodesPurposeWeight.clear();
    allTransitStops.clear();
    transitStopsByNodeId.clear();
    metroStops.clear();
    tramStops.clear();
    busStops.clear();
    tripsByMode.clear();
    agentTransitDestinations.clear();
  }
}


