package pedsim.activity.engine;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import pedsim.activity.agents.ActivityPurpose;
import pedsim.core.engine.Exporter;
import pedsim.core.engine.FlowHandler;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.parameters.TimePars;
import sim.field.geo.VectorLayer;
import sim.graph.NodeGraph;

public class PedSimCityActivity extends PedSimCity {

  private static final java.util.logging.Logger logger =
      java.util.logging.Logger.getLogger(PedSimCityActivity.class.getName());

  // Seasonal darkness at the current simulation time, driven by ActivityEngine.onStepUpdate.
  public boolean isDark = false;

  /**
   * Legs that set off in seasonal darkness. Cleared at the end of each day.
   */
  public final java.util.concurrent.atomic.LongAdder legsInDarkness =
      new java.util.concurrent.atomic.LongAdder();

  /** Clears the day's darkness counters. */
  public void resetDarknessCounters() {
    legsInDarkness.reset();
  }

  /** Distance bands (m) the realised walking share is reported over. */
  public static final double[] MODE_DISTANCE_BANDS = {500.0, 1000.0, 2000.0, 5000.0};

  /** Rings (m from the city centre) the realised walking share is reported over. */
  public static final double[] MODE_RING_BANDS = {1500.0, 4000.0};

  private final java.util.concurrent.atomic.LongAdder[] legsOfferedByBand = adders(5);
  private final java.util.concurrent.atomic.LongAdder[] legsWalkedByBand = adders(5);
  private final java.util.concurrent.atomic.LongAdder[] legsOfferedByRing = adders(3);
  private final java.util.concurrent.atomic.LongAdder[] legsWalkedByRing = adders(3);
  private final java.util.concurrent.atomic.LongAdder[] metresOfferedByRing = adders(3);

  private static java.util.concurrent.atomic.LongAdder[] adders(int size) {
    java.util.concurrent.atomic.LongAdder[] adders =
        new java.util.concurrent.atomic.LongAdder[size];
    for (int i = 0; i < size; i++) {
      adders[i] = new java.util.concurrent.atomic.LongAdder();
    }
    return adders;
  }

  /**
   * Records one discretionary leg's mode decision, by how long the leg is and by how far from the
   * city centre the agent lives. The walking share these produce is the model's output, checked
   * against the survey rather than set from it.
   *
   * @param metres the leg's length as mode choice saw it
   * @param walked whether it is walked
   * @param home the agent's home node, for the ring; may be null
   */
  public void recordModeChoice(double metres, boolean walked, NodeGraph home) {
    int band = bandOf(metres, MODE_DISTANCE_BANDS);
    legsOfferedByBand[band].increment();
    if (walked) {
      legsWalkedByBand[band].increment();
    }
    double fromCentre = CityLocation.distanceFromCentre(home);
    if (Double.isNaN(fromCentre)) {
      return;
    }
    int ring = bandOf(fromCentre, MODE_RING_BANDS);
    legsOfferedByRing[ring].increment();
    metresOfferedByRing[ring].add(Math.round(metres));
    if (walked) {
      legsWalkedByRing[ring].increment();
    }
  }

  private static int bandOf(double value, double[] edges) {
    for (int i = 0; i < edges.length; i++) {
      if (value <= edges[i]) {
        return i;
      }
    }
    return edges.length;
  }

  /** Walked over offered, per distance band; NaN where the band saw no legs. */
  public double[] walkShareByBand() {
    return shares(legsWalkedByBand, legsOfferedByBand);
  }

  /** Walked over offered, per ring out from the city centre; NaN where the ring saw no legs. */
  public double[] walkShareByRing() {
    return shares(legsWalkedByRing, legsOfferedByRing);
  }

  /** Discretionary legs offered, per distance band. */
  public long[] legsOfferedByBand() {
    return sums(legsOfferedByBand);
  }

  /** Discretionary legs offered, per ring. */
  public long[] legsOfferedByRing() {
    return sums(legsOfferedByRing);
  }

  /**
   * Mean length of the legs a ring was offered, in metres - the quantity mode choice is deciding
   * on, and the one a periphery effect has to work through.
   */
  public double[] meanLegMetresByRing() {
    double[] means = new double[legsOfferedByRing.length];
    for (int i = 0; i < means.length; i++) {
      long legs = legsOfferedByRing[i].sum();
      means[i] = legs > 0 ? (double) metresOfferedByRing[i].sum() / legs : Double.NaN;
    }
    return means;
  }

  private static long[] sums(java.util.concurrent.atomic.LongAdder[] adders) {
    long[] totals = new long[adders.length];
    for (int i = 0; i < adders.length; i++) {
      totals[i] = adders[i].sum();
    }
    return totals;
  }

  /** Walked over offered across every discretionary leg the day offered; NaN when there were none. */
  public double realisedWalkShare() {
    long walked = 0;
    long offered = 0;
    for (int i = 0; i < legsOfferedByBand.length; i++) {
      walked += legsWalkedByBand[i].sum();
      offered += legsOfferedByBand[i].sum();
    }
    return offered > 0 ? (double) walked / offered : Double.NaN;
  }

  private static double[] shares(
      java.util.concurrent.atomic.LongAdder[] walked,
      java.util.concurrent.atomic.LongAdder[] offered) {
    double[] shares = new double[offered.length];
    for (int i = 0; i < offered.length; i++) {
      long total = offered[i].sum();
      shares[i] = total > 0 ? (double) walked[i].sum() / total : Double.NaN;
    }
    return shares;
  }

  /** Clears the day's mode-choice counters. */
  public void resetModeChoiceCounters() {
    for (int i = 0; i < legsOfferedByBand.length; i++) {
      legsOfferedByBand[i].reset();
      legsWalkedByBand[i].reset();
    }
    for (int i = 0; i < legsOfferedByRing.length; i++) {
      legsOfferedByRing[i].reset();
      legsWalkedByRing[i].reset();
      metresOfferedByRing[i].reset();
    }
  }

  @Override
  protected Exporter createExporter(FlowHandler flowHandler, String appName) {
    return new ActivityExporter(flowHandler, appName);
  }

  // Raw census layer as loaded from <City>_censusData.gpkg: one polygon set carrying population
  // structure only (residence_pct, residents, the persona shares and female_pct).
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

  /** Whether the current simulated day is rainy (see {@link Weather}). */
  public boolean isRainyNow() {
    return Weather.isRainy(TimePars.getTime(schedule.getSteps()).toLocalDate(), seed());
  }

  /** Travel demand for the activity tier; see {@link ActivityTravelDemand}. */
  @Override
  protected pedsim.core.engine.TravelDemand createTravelDemand() {
    return new ActivityTravelDemand(this);
  }

  /** Clears all static data structures to allow for a clean simulation restart. */
  public static void clearStaticData() {
    // clear() the layers themselves: getGeometries() returns a defensive copy.
    censusLayer.clear();
    poisLayer.clear();
    censusZones.clear();
    nodesPurposeWeight.clear();
  }
}
