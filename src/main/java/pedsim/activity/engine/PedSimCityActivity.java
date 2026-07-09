package pedsim.activity.engine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.locationtech.jts.index.strtree.STRtree;
import pedsim.core.agents.Agent;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.utilities.RobustVectorLayer;
import pedsim.transit.TransitStop;
import pedsim.transit.TransitVehicle;
import sim.graph.NodeGraph;

public class PedSimCityActivity extends PedSimCity {

  private static final java.util.logging.Logger logger = java.util.logging.Logger.getLogger(PedSimCityActivity.class.getName());

  // 24h activity clock: true between ~20:00 and ~06:00. Driven by ActivityEngine.onStepUpdate.
  public boolean isDark = false;


  // Raw census layer as loaded from <City>_censusData.gpkg: one polygon set carrying the
  // residence_pct, workplace_poi and night_poi columns (the full 24h activity pattern).
  public static RobustVectorLayer censusLayer = new RobustVectorLayer();

  // Census zones built from the raw layer, each with its proximity-assigned network nodes and the
  // per-zone residence / workplace / night weights.
  public static List<CensusZone> censusZones = new ArrayList<>();

  // Spatial index over the census zones, used to find candidate work zones near a home node.
  public static STRtree censusZonesIndex = new STRtree();

  // Per-node POI weights derived once from the zones (each zone's count split across its nodes).
  public static Map<NodeGraph, Double> nodesWorkplaceWeight = new HashMap<>();
  public static Map<NodeGraph, Double> nodesNightWeight = new HashMap<>();

  // Multi-Modal Transit Static Data Structures
  public static List<TransitStop> allTransitStops = new ArrayList<>();
  public static Map<Integer, TransitStop> transitStopsByNodeId = new HashMap<>();
  public static List<TransitStop> metroStops = new ArrayList<>();
  public static List<TransitStop> tramStops = new ArrayList<>();
  public static List<TransitStop> busStops = new ArrayList<>();
  public static Map<String, Integer> tripsByMode = new HashMap<>();
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


  /** Clears all static data structures to allow for a clean simulation restart. */

  public static void clearStaticData() {
    censusLayer.getGeometries().clear();
    censusZones.clear();
    censusZonesIndex = new STRtree();
    nodesWorkplaceWeight.clear();
    nodesNightWeight.clear();
    allTransitStops.clear();
    transitStopsByNodeId.clear();
    metroStops.clear();
    tramStops.clear();
    busStops.clear();
    tripsByMode.clear();
    agentTransitDestinations.clear();
  }
}


