package pedsim.core.engine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.linearref.LengthIndexedLine;
import org.locationtech.jts.planargraph.DirectedEdge;
import pedsim.core.agents.Agent;
import pedsim.core.cognition.cityimage.Barrier;
import pedsim.core.cognition.cityimage.Gateway;
import pedsim.core.cognition.cityimage.Region;
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
  public static VectorLayer sightLines = new VectorLayer();

  public static Graph network = new Graph();
  public static Graph dualNetwork = new Graph();
  public static Envelope MBR = null;

  // Data-availability flags, set at import and read by route choice (Heuristics) to deactivate
  // mechanisms whose data a given city does not provide.
  /** A dual graph was loaded; angular-change (simplest-path) routing is available. */
  public static boolean dualGraphLoaded = false;

  /** Building landmark scores were loaded; landmark-based navigation is available. */
  public static boolean landmarksLoaded = false;

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
    if (!buildings.isEmpty()) {
      MBR.expandToInclude(buildings.getMBR());
    }
    if (!barriers.isEmpty()) {
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
   * Percentile of per-edge agent volumes above which an edge counts as crowded (see
   * {@link Crowdness}). Modules may override to expose their own parameter.
   */
  public double getCrowdednessPercentile() {
    return Crowdness.DEFAULT_CROWDEDNESS_PERCENTILE;
  }

  /**
   * Module hook for taking over a single agent-release event (see {@link AgentReleaseManager}).
   * Returns the number of agents released ({@code >= 0}) when the module handled the event — the
   * standard meters-based release is then skipped — or {@code -1} to let it run.
   */
  public int releaseAgentsOverride(double steps, int dayNumber) {
    return -1;
  }

  /**
   * Multiplier applied to the meters-to-allocate release budget at the given moment (see
   * {@link AgentReleaseManager}). Modules may override — e.g. to suppress walking on rainy days.
   */
  public double releaseBudgetMultiplier(java.time.LocalDateTime time) {
    return 1.0;
  }

  /**
   * Acceptance probability in {@code [0, 1]} that the given agent is released at the given hour of
   * day (see {@link AgentReleaseManager}). Modules may override — e.g. to favour commuter personas
   * in the morning and leisure personas at midday. Must be thread-safe.
   */
  public double releaseCandidateWeight(Agent agent, int hour) {
    return 1.0;
  }

  /**
   * Whether a sampled trip distance (meters) is acceptable for release (see
   * {@link AgentReleaseManager}); rejected draws are resampled. Modules may override — e.g. a
   * walk-share filter that keeps most short trips and few long ones. Must be thread-safe.
   */
  public boolean acceptTripDistance(double meters) {
    return true;
  }

  /**
   * Starts moving agents in the simulation. This method schedules agents for repeated movement
   * updates and sets up the spatial index for agents.
   */
  protected void startMovingAgents() {
    for (Agent agent : agentsList) {
      Stoppable stop = schedule.scheduleRepeating(agent);
      agent.setStoppable(stop);
    }
    // A single end-of-step index refresh covers the whole layer; scheduling it per agent would
    // rebuild the quadtree N times every step.
    schedule.scheduleRepeating(agents.scheduleSpatialIndexUpdater(), Integer.MAX_VALUE, 1.0);
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
    // Clear the layers themselves: getGeometries() returns a defensive copy, so clearing that
    // copy left the layers' contents (and spatial indexes) intact and every re-run re-imported
    // the same features on top of the old ones.
    roads.clear();
    buildings.clear();
    barriers.clear();
    junctions.clear();
    sightLines.clear();
    intersectionsDual.clear();
    centroids.clear();

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
    MBR = null;

    dualGraphLoaded = false;
    landmarksLoaded = false;
  }
}
