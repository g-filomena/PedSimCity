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

  @Override
  protected Exporter createExporter(FlowHandler flowHandler, String appName) {
    return new ActivityExporter(flowHandler, appName);
  }

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
