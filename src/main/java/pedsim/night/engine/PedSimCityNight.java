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
 * {@link PedSimCityActivity} (census population + purpose weights) with the night
 * perception/safety layer: vulnerability, illuminated edges and directional lighting.
 *
 * <p>The 24h clock ({@code isDark}) and the activity data live in {@link PedSimCityActivity}.
 * Vulnerability is carried as the {@code vulnerability_pct} column on the census layer and
 * resolved per node by {@code NightEnvironment}.
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

  @Override
  public double getCrowdednessPercentile() {
    return pedsim.night.parameters.NightPars.crowdednessPercentile;
  }

  public void setCrowdednessPercentile(double crowdednessPercentile) {
    pedsim.night.parameters.NightPars.crowdednessPercentile = crowdednessPercentile;
  }

  /**
   * With light A/B testing enabled, day 1 releases one vulnerable/non-vulnerable twin pair per
   * release event (agents {@code 2i} and {@code 2i+1} for release event {@code i}, 72 pairs)
   * instead of the standard meters-based release.
   */
  @Override
  public int releaseAgentsOverride(double steps, int dayNumber) {
    if (!pedsim.night.parameters.NightPars.enableLightABTesting || dayNumber != 1) {
      return -1;
    }

    int pairIndex =
        (int) Math.round(steps / pedsim.core.parameters.TimePars.releaseAgentsEverySteps) - 1;
    if (pairIndex < 0 || pairIndex >= 72) {
      return 0;
    }

    pedsim.night.agents.NightAgent vulnAgent = null;
    pedsim.night.agents.NightAgent normalAgent = null;
    for (pedsim.core.agents.Agent agent : agentsList) {
      if (agent instanceof pedsim.night.agents.NightAgent nightAgent) {
        if (nightAgent.agentID == pairIndex * 2) {
          vulnAgent = nightAgent;
        } else if (nightAgent.agentID == pairIndex * 2 + 1) {
          normalAgent = nightAgent;
        }
      }
    }
    if (vulnAgent == null || normalAgent == null) {
      return 0;
    }

    double tripDistance = pedsim.core.parameters.RouteChoicePars.avgTripDistance;
    vulnAgent.setDistanceNextDestination(tripDistance);
    normalAgent.setDistanceNextDestination(tripDistance);
    vulnAgent.startWalkingAlone();
    normalAgent.startWalkingAlone();

    pedsim.core.utilities.LoggerUtil.getLogger()
        .fine(
            "A/B Testing: Released pair "
                + pairIndex
                + " (Agent "
                + vulnAgent.agentID
                + " & "
                + normalAgent.agentID
                + ") at step "
                + steps);
    return 2;
  }

  // ---------------------------------

  public static void clearNightStaticData() {
    indexedEdgeCache.clear();
    edges.clear();
    altRoutesVulnerable.clear();
    altRoutesNonVulnerable.clear();
    directionalLuxMap.clear();
    // clear() the layer itself: getGeometries() returns a defensive copy, so clearing that
    // copy would leave the layer (and its spatial index) populated across re-initialisations.
    illuminatedEdges.clear();
    nodesVulnerabilityWeight.clear();
  }
}
