package pedsim.night.engine;

import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import org.javatuples.Pair;
import org.locationtech.jts.linearref.LengthIndexedLine;
import org.locationtech.jts.planargraph.DirectedEdge;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.ScenarioConfig;
import sim.graph.EdgeGraph;
import sim.graph.NodeGraph;

/**
 * The PedSimCity class represents the main simulation environment.
 */
public class PedSimCityNight extends PedSimCity {

  private static final long serialVersionUID = 1L;
  public boolean isDark = false;

  // Parameters are now stored statically in NightPars

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
      return java.util.Collections.synchronizedMap(new java.util.LinkedHashMap<K, V>(MAX_ROUTE_CACHE_SIZE + 1, .75F, true) {
          @Override
          protected boolean removeEldestEntry(Map.Entry<K, V> eldest) {
              return size() > MAX_ROUTE_CACHE_SIZE;
          }
      });
  }

  // cached route
  public static Map<Pair<NodeGraph, NodeGraph>, List<DirectedEdge>> routesDay = createBoundedCache();
  public static Map<Pair<NodeGraph, NodeGraph>, List<DirectedEdge>> routesNonVulnerableNight = createBoundedCache();
  public static Map<Pair<NodeGraph, NodeGraph>, List<DirectedEdge>> routesVulnerableNight = createBoundedCache();

  // cached alternative routes for night movement
  public static Map<Pair<NodeGraph, NodeGraph>, List<DirectedEdge>> altRoutesVulnerable = createBoundedCache();
  public static Map<Pair<NodeGraph, NodeGraph>, List<DirectedEdge>> altRoutesNonVulnerable = createBoundedCache();

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

  @Override
  public void start() {
    loadDirectionalLighting();
    super.start();
  }

  private void loadDirectionalLighting() {
    directionalLuxMap.clear();
    try {
      String resourceName =
          pedsim.core.parameters.Pars.cityName
              + "/"
              + pedsim.core.parameters.Pars.cityName
              + "_directional_lighting_lookup.csv";
      java.net.URL fileUrl = getClass().getClassLoader().getResource(resourceName);
      if (fileUrl != null) {
        try (java.io.BufferedReader br =
            new java.io.BufferedReader(new java.io.InputStreamReader(fileUrl.openStream()))) {
          String line = br.readLine(); // skip header
          while ((line = br.readLine()) != null) {
            String[] parts = line.split(",");
            if (parts.length >= 5) {
              long key =
                  luxKey(Integer.parseInt(parts[0].trim()), Integer.parseInt(parts[1].trim()));
              double minLux = Double.parseDouble(parts[4]); // visibility_min_lux
              directionalLuxMap.put(key, minLux);
            }
          }
        }
        System.out.println("Loaded " + directionalLuxMap.size() + " directional lighting entries.");
      } else {
        System.out.println("Directional lighting lookup not found at: " + resourceName);
      }
    } catch (Exception e) {
      System.err.println("Failed to load directional lighting: " + e.getMessage());
    }
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
    routesDay.clear();
    routesNonVulnerableNight.clear();
    routesVulnerableNight.clear();
    altRoutesVulnerable.clear();
    altRoutesNonVulnerable.clear();
    directionalLuxMap.clear();
  }
}
