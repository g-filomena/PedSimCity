package pedsim.activity.engine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.locationtech.jts.index.strtree.STRtree;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.utilities.RobustVectorLayer;
import sim.graph.NodeGraph;

public class PedSimCityActivity extends PedSimCity {

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

  /** Clears all static data structures to allow for a clean simulation restart. */
  public static void clearStaticData() {
    censusLayer.getGeometries().clear();
    censusZones.clear();
    censusZonesIndex = new STRtree();
    nodesWorkplaceWeight.clear();
    nodesNightWeight.clear();
  }
}
