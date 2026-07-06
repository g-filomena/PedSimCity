package pedsim.night.engine;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import org.javatuples.Pair;
import org.locationtech.jts.linearref.LengthIndexedLine;
import org.locationtech.jts.planargraph.DirectedEdge;
import pedsim.activity.engine.PedSimCityActivity;
import pedsim.core.engine.ScenarioConfig;
import sim.field.geo.VectorLayer;
import sim.graph.EdgeGraph;
import sim.graph.NodeGraph;

/**
 * Simulation state for the night module. Extends the activity-based
 * {@link PedSimCityActivity} (census + workplace + night POI) with the night perception/safety
 * layer: vulnerability, illuminated edges and directional lighting.
 *
 * <p>The 24h clock ({@code isDark}) and the activity data (census, workplace, night POI) live in
 * {@link PedSimCityActivity}. Vulnerability is carried as the {@code vulnerability_pct} column on the
 * unified census layer and resolved per node by {@code NightEnvironment}.
 */
public class PedSimCityNight extends PedSimCityActivity {

  private static final long serialVersionUID = 1L;

  // Illuminated edges dataset (contains mean_lux per edge for night simulation)
  public static VectorLayer illuminatedEdges = new VectorLayer();

  // --- Vulnerability dataset (node -> vulnerability %) ---
  public static Map<NodeGraph, Double> nodesVulnerabilityWeight = new HashMap<>();

  // Directional entrance light mapping (Node_A -> Node_B : min_lux), keyed by packed long.
  public static final Map<Long, Double> directionalLuxMap = new ConcurrentHashMap<>();

  /** Packs two node IDs into a single long to avoid String allocation per lookup. */
  public static long luxKey(int fromId, int toId) {
    return ((long) fromId << 32) | (toId & 0xFFFFFFFFL);
  }

  public static final Map<DirectedEdge, LengthIndexedLine> indexedEdgeCache =
      new ConcurrentHashMap<>();

  public static final Set<EdgeGraph> edges = ConcurrentHashMap.newKeySet();

  private static final int MAX_ROUTE_CACHE_SIZE = 5000;

  private static <K, V> Map<K, V> createBoundedCache() {
    return java.util.Collections.synchronizedMap(
        new java.util.LinkedHashMap<K, V>(MAX_ROUTE_CACHE_SIZE + 1, .75F, true) {
          @Override
          protected boolean removeEldestEntry(Map.Entry<K, V> eldest) {
            return size() > MAX_ROUTE_CACHE_SIZE;
          }
        });
  }

  // cached alternative routes for night movement
  public static Map<Pair<NodeGraph, NodeGraph>, List<DirectedEdge>> altRoutesVulnerable =
      createBoundedCache();
  public static Map<Pair<NodeGraph, NodeGraph>, List<DirectedEdge>> altRoutesNonVulnerable =
      createBoundedCache();

  /**
   * Constructs a new instance of the PedSimCity simulation environment.
   *
   * @param seed The random seed for the simulation.
   * @param job The current job number for multi-run simulations.
   */
  public PedSimCityNight(long seed, int job, ScenarioConfig scenarioConfig) {
    super(seed, job, scenarioConfig);
  }

  /**
   * Populates the simulation environment with agents and other entities based on the selected
   * simulation parameters. This method uses the Populate class to generate the agent population.
   */
  @Override
  protected void populateEnvironment() {
    NightPopulate populate = new NightPopulate();
    populate.populate(this);
  }

  // --- Getters & Setters for GUI ---

  public double getMinVulnerableLightSensitivity() {
    return pedsim.night.parameters.NightPars.minVulnerableLightSensitivity;
  }

  public void setMinVulnerableLightSensitivity(double minVulnerableLightSensitivity) {
    pedsim.night.parameters.NightPars.minVulnerableLightSensitivity = minVulnerableLightSensitivity;
  }

  public double getMaxVulnerableLightSensitivity() {
    return pedsim.night.parameters.NightPars.maxVulnerableLightSensitivity;
  }

  public void setMaxVulnerableLightSensitivity(double maxVulnerableLightSensitivity) {
    pedsim.night.parameters.NightPars.maxVulnerableLightSensitivity = maxVulnerableLightSensitivity;
  }

  public double getNonVulnerableLightSensitivity() {
    return pedsim.night.parameters.NightPars.nonVulnerableLightSensitivity;
  }

  public void setNonVulnerableLightSensitivity(double nonVulnerableLightSensitivity) {
    pedsim.night.parameters.NightPars.nonVulnerableLightSensitivity = nonVulnerableLightSensitivity;
  }

  public boolean getEnableLightABTesting() {
    return pedsim.night.parameters.NightPars.enableLightABTesting;
  }

  public void setEnableLightABTesting(boolean enableLightABTesting) {
    pedsim.night.parameters.NightPars.enableLightABTesting = enableLightABTesting;
  }

  public int getAbTestPairs() {
    return pedsim.night.parameters.NightPars.abTestPairs;
  }

  public void setAbTestPairs(int abTestPairs) {
    pedsim.night.parameters.NightPars.abTestPairs = abTestPairs;
  }

  public double getCrowdednessPercentile() {
    return pedsim.night.parameters.NightPars.crowdednessPercentile;
  }

  public void setCrowdednessPercentile(double crowdednessPercentile) {
    pedsim.night.parameters.NightPars.crowdednessPercentile = crowdednessPercentile;
  }

  // ---------------------------------

  public static void clearNightStaticData() {
    indexedEdgeCache.clear();
    edges.clear();
    altRoutesVulnerable.clear();
    altRoutesNonVulnerable.clear();
    directionalLuxMap.clear();
    illuminatedEdges.getGeometries().clear();
    nodesVulnerabilityWeight.clear();
  }
}
