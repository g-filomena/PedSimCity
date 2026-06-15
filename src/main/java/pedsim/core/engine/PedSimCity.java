package pedsim.core.engine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import org.javatuples.Pair;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.Polygon;
import org.locationtech.jts.index.strtree.STRtree;
import org.locationtech.jts.linearref.LengthIndexedLine;
import org.locationtech.jts.planargraph.DirectedEdge;
import pedsim.core.agents.Agent;
import pedsim.core.cognition.cityimage.Barrier;
import pedsim.core.cognition.cityimage.Gateway;
import pedsim.core.cognition.cityimage.Region;
import pedsim.core.utilities.RobustVectorLayer;
import sim.engine.SimState;
import sim.engine.Stoppable;
import sim.field.geo.VectorLayer;
import sim.graph.Building;
import sim.graph.EdgeGraph;
import sim.graph.Graph;
import sim.graph.NodeGraph;
import sim.util.geo.MasonGeometry;

/**
 * The PedSimCity class represents the main simulation environment.
 */
public class PedSimCity extends SimState {
  private static final long serialVersionUID = 1L;
  protected String appName;

  // Urban elements: graphs, buildings, etc.
  public static VectorLayer roads = new VectorLayer();
  public static VectorLayer buildings = new VectorLayer();
  public static VectorLayer barriers = new VectorLayer();
  public static VectorLayer junctions = new VectorLayer();
  public static RobustVectorLayer sightLines = new RobustVectorLayer();
  public static RobustVectorLayer censusZones = new RobustVectorLayer();

  // Optional night/vulnerability data layers
  public static RobustVectorLayer censusZonesVulnerability = new RobustVectorLayer();
  public static RobustVectorLayer nightPoiDensities = new RobustVectorLayer();
  public static RobustVectorLayer workplacePoiDensities = new RobustVectorLayer();

  // Illuminated edges dataset (contains mean_lux per edge for night simulation)
  public static VectorLayer illuminatedEdges = new VectorLayer();

  // --- Census (residence) dataset ---
  public static Map<NodeGraph, MasonGeometry> nodesCensusZonesMap = new HashMap<>();
  public static STRtree censusZonesSpatialIndex = new STRtree();
  public static List<MasonGeometry> censusZonesList = new ArrayList<>();
  public static Map<MasonGeometry, List<NodeGraph>> censusZonesNodesMap = new HashMap<>();

  // --- Vulnerability dataset (node -> vulnerability %) ---
  public static Map<NodeGraph, Double> nodesVulnerabilityWeight = new HashMap<>();

  // --- Night POI density dataset (node -> night POI count) ---
  public static Map<NodeGraph, Double> nodesNightPoiWeight = new HashMap<>();

  // --- Workplace POI density dataset (node -> workplace POI count) ---
  public static Map<NodeGraph, Double> nodesWorkplacePoiWeight = new HashMap<>();

  public static Graph network = new Graph();
  public static Graph dualNetwork = new Graph();
  public static Envelope MBR = null;

  // dual graph
  public static VectorLayer intersectionsDual = new VectorLayer();
  public static VectorLayer centroids = new VectorLayer();

  // supporting HashMaps, bags and Lists
  public static Map<Integer, Building> buildingsMap = new HashMap<>();
  public static Map<Integer, Region> regionsMap = new HashMap<>();
  public static Map<Integer, Barrier> barriersMap = new HashMap<>();
  public static Map<Integer, Gateway> gatewaysMap = new HashMap<>();
  public static Map<Integer, NodeGraph> nodesMap = new HashMap<>();
  public static Map<Integer, EdgeGraph> edgesMap = new HashMap<>();
  public static Map<Integer, NodeGraph> centroidsMap = new HashMap<>();
  public static Set<EdgeGraph> edges = new HashSet<>();

  // OD related variables
  public static List<MasonGeometry> startingNodes = new ArrayList<>();

  public static Map<DirectedEdge, LengthIndexedLine> indexedEdgeCache = new ConcurrentHashMap<>();
  public static Map<Pair<Coordinate, Coordinate>, Polygon> censusZonesCache =
      new ConcurrentHashMap<>();
  // cached alternative routes for night movement
  public static Map<Pair<NodeGraph, NodeGraph>, List<DirectedEdge>> alternativeRoutes =
      new ConcurrentHashMap<>();

  // used only when loading OD sets
  public int currentJob;
  public FlowHandler flowHandler; // Using a wildcard since we don't know the exact type

  public VectorLayer agents;
  public Set<Agent> agentsAtHome = ConcurrentHashMap.newKeySet();
  public Set<Agent> agentsWalking = ConcurrentHashMap.newKeySet();
  public Set<Agent> agentsList = ConcurrentHashMap.newKeySet();

  public ScenarioConfig scenarioConfig;

  /**
   * Per-thread state reference, set in the constructor.
   * Simulation threads each get their own job's state via {@link #currentForThread()}.
   */
  private static final ThreadLocal<PedSimCity> THREAD_STATE = new ThreadLocal<>();

  /**
   * Last-constructed instance. Kept only for {@link pedsim.core.applet.SimulationViewer},
   * which runs on the Swing EDT and cannot use the ThreadLocal. Do not read this from
   * simulation threads — use {@link #currentForThread()} instead.
   */
  public static volatile PedSimCity currentInstance;

  /** Returns the {@link PedSimCity} instance belonging to the calling simulation thread. */
  public static PedSimCity currentForThread() {
    return THREAD_STATE.get();
  }

  /**
   * Constructs a new instance of the PedSimCity simulation environment.
   *
   * @param seed The random seed for the simulation.
   * @param job The current job number for multi-run simulations.
   * @param scenarioConfig The configuration for simulation scenarios.
   */
  public PedSimCity(long seed, int job, ScenarioConfig scenarioConfig) {
    super(seed);
    this.currentJob = job;
    this.scenarioConfig = scenarioConfig;
    this.agents = new VectorLayer();
    this.appName = this.getClass().getSimpleName();
    this.flowHandler = new FlowHandler(job, this, appName);
    THREAD_STATE.set(this);
    currentInstance = this; // for SimulationViewer (EDT only)
  }

  /**
   * Initialises the simulation by defining the simulation mode, initialising edge volumes, and
   * preparing the simulation environment. It then proceeds to populate the environment with agents
   * and starts the agent movement.
   */
  @Override
  public void start() {
    super.start();
    prepareEnvironment();
    populateEnvironment();
    startMovingAgents();
  }

  /**
   * Prepares the environment for the simulation. This method sets up the minimum bounding rectangle
   * (MBR) to encompass both the road and building layers and updates the MBR of the road layer
   * accordingly.
   */
  protected void prepareEnvironment() {
    MBR = roads.getMBR();
    if (!buildings.getGeometries().isEmpty()) {
      MBR.expandToInclude(buildings.getMBR());
    }
    if (!barriers.getGeometries().isEmpty()) {
      MBR.expandToInclude(barriers.getMBR());
    }
    roads.setMBR(MBR);
  }

  /**
   * Populates the simulation environment with agents and other entities based on the selected
   * simulation parameters. This method uses the Populate class to generate the agent population.
   */
  protected void populateEnvironment() {
    Populate populate = new Populate();
    populate.populate(this);
  }

  /**
   * Starts moving agents in the simulation. This method schedules agents for repeated movement
   * updates and sets up the spatial index for agents.
   */
  protected void startMovingAgents() {
    for (Agent agent : agentsList) {
      Stoppable stop = schedule.scheduleRepeating(agent);
      agent.setStoppable(stop);
      schedule.scheduleRepeating(agents.scheduleSpatialIndexUpdater(), Integer.MAX_VALUE, 1.0);
    }
    agents.setMBR(MBR);
  }

  public Set<Agent> getAgentsList() {
    return this.agentsList;
  }

  // ---------------------------------------------------
  // Shared simulation core (used by GUI + headless)
  // ---------------------------------------------------
  /**
   * Completes the simulation by saving results and performing cleanup operations.
   */
  @Override
  public void finish() {
    super.finish();
  }

  /**
   * Clears all static data structures to allow for a clean simulation restart.
   */
  public static void clearStaticData() {
    roads.getGeometries().clear();
    buildings.getGeometries().clear();
    barriers.getGeometries().clear();
    junctions.getGeometries().clear();
    sightLines.getGeometries().clear();
    censusZones.getGeometries().clear();
    censusZonesVulnerability.getGeometries().clear();
    nightPoiDensities.getGeometries().clear();
    workplacePoiDensities.getGeometries().clear();
    illuminatedEdges.getGeometries().clear();
    intersectionsDual.getGeometries().clear();
    centroids.getGeometries().clear();

    nodesCensusZonesMap.clear();
    censusZonesList.clear();
    censusZonesNodesMap.clear();
    nodesVulnerabilityWeight.clear();
    nodesNightPoiWeight.clear();
    nodesWorkplacePoiWeight.clear();

    network = new Graph();
    dualNetwork = new Graph();

    buildingsMap.clear();
    regionsMap.clear();
    barriersMap.clear();
    gatewaysMap.clear();
    nodesMap.clear();
    edgesMap.clear();
    centroidsMap.clear();
    edges.clear();
    startingNodes.clear();

    indexedEdgeCache.clear();
    censusZonesCache.clear();
    alternativeRoutes.clear();

    censusZonesSpatialIndex = new STRtree();
    MBR = null;
  }
}
