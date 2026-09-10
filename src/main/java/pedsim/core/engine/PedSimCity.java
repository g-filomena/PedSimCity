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
import pedsim.core.parameters.TimePars;
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
   * Metres of route planned today, summed over every leg of every agent.
   *
   * <p>The release budget is charged an estimate, a sampled trip distance times an expected leg
   * count, because at release time neither the destination nor the route exists yet. The route
   * appears one step later and its length is then known exactly: on Torino_simplified a leg walks
   * about 1.75x what it was charged, because destination selection lands near, not on, the sampled
   * radius and the network path between two points exceeds the distance between them.
   *
   * <p>This ledger measures that gap and nothing more. It is deliberately not fed back into the
   * allocation: the charge covers a whole tour at once while its routes are planned over the
   * following hours, so the measurement always lags the charge, and an allocation that grows when
   * it sees less walking than it charged for is a positive feedback loop. Fixing the gap belongs
   * at its source, in destination selection.
   *
   * <p>A {@link java.util.concurrent.atomic.DoubleAdder} because agents step concurrently.
   */
  private final java.util.concurrent.atomic.DoubleAdder plannedRouteMeters =
      new java.util.concurrent.atomic.DoubleAdder();

  /**
   * Records a leg whose route has just been planned.
   *
   * @param meters the routed length, which is what will actually be walked
   */
  public void recordPlannedRoute(double meters) {
    if (meters > 0.0 && Double.isFinite(meters)) {
      plannedRouteMeters.add(meters);
    }
  }

  /** Metres of route planned so far today. */
  public double plannedRouteMeters() {
    return plannedRouteMeters.sum();
  }

  /**
   * Metres actually walked on the legs that finished today.
   *
   * <p>The planned ledger above counts a leg the moment its route is laid out. A tour still under
   * way when the day ends has its last leg counted there in full and walked only in part, which
   * biases any comparison of planned against charged in the same direction as the charge itself.
   * Recording the walked length separately, at the point where the route is replaced by the edges
   * the agent really covered, leaves the difference visible instead of buried.
   */
  private final java.util.concurrent.atomic.DoubleAdder walkedRouteMeters =
      new java.util.concurrent.atomic.DoubleAdder();

  /** Records a leg that has just finished, with the length actually covered. */
  public void recordWalkedRoute(double meters) {
    if (meters > 0.0 && Double.isFinite(meters)) {
      walkedRouteMeters.add(meters);
    }
  }

  /** Metres walked on legs completed so far today. */
  public double walkedRouteMeters() {
    return walkedRouteMeters.sum();
  }

  /**
   * Times a destination search had to widen its distance band, and times it gave up and fell back
   * to any node in the city.
   *
   * <p>Both used to happen silently. The band widens inside the lookup call, so a run could not
   * say whether a leg came from the band it asked for or from one three times wider; and the
   * fallback replaces the band with the whole network. Since leg length is the open question,
   * these two counts are the difference between an answer and a guess.
   */
  private final java.util.concurrent.atomic.LongAdder destinationWidenings =
      new java.util.concurrent.atomic.LongAdder();

  private final java.util.concurrent.atomic.LongAdder destinationFallbacks =
      new java.util.concurrent.atomic.LongAdder();

  /** Records that a destination search widened its band the given number of times. */
  public void recordDestinationWidening(int widenings) {
    if (widenings > 0) {
      destinationWidenings.add(widenings);
    }
  }

  /** Records a destination search that exhausted its band and took any node instead. */
  public void recordDestinationFallback() {
    destinationFallbacks.increment();
  }

  /** Total band widenings so far today. */
  public long destinationWidenings() {
    return destinationWidenings.sum();
  }

  /** Total band fallbacks so far today. */
  public long destinationFallbacks() {
    return destinationFallbacks.sum();
  }

  /** Clears the day's ledgers and counters. */
  public void resetPlannedRouteMeters() {
    plannedRouteMeters.reset();
    walkedRouteMeters.reset();
    destinationWidenings.reset();
    destinationFallbacks.reset();
  }

  /**
   * Share of the day's metres budget belonging to the release event at this moment.
   *
   * <p>Core uses {@link TimePars#computeTimeStepShare}, a curve of tuned peaks. Modules that model
   * why people leave home may override with a profile derived from that instead; see
   * {@code DepartureProfile}. Whatever supplies it, the shares across a day's release events must
   * sum to 1.0, or the day's budget is over- or under-spent.
   *
   * @param time the moment of the release event
   * @return the share, in {@code [0, 1]}
   */
  public double departureShare(java.time.LocalDateTime time) {
    return TimePars.computeTimeStepShare(time);
  }

  /**
   * How many walked legs releasing this agent is expected to produce.
   *
   * <p>The release budget is a quantity of metres to be walked, so it has to be charged for
   * everything the release causes, not for the first leg of it. A core agent walks out and comes
   * back, which is two; activity-based modules chain a tour through several stops and override
   * this. Charging one leg, as the manager did until this seam existed, under-spends the budget by
   * whatever the tour multiplier is, and the model then walks that factor more than
   * {@code metersPerDayPerPerson} says it should.
   *
   * <p>Called with a null agent to get the population-typical value, which is used to size the
   * residual carried between release events. Must be thread-safe.
   *
   * @param agent the candidate being released, or null for the module-typical tour
   * @return the expected number of legs, at least one
   */
  public double expectedTourLegs(Agent agent) {
    return 2.0;
  }

  /**
   * Probability that a sampled trip distance (metres) is kept for release; rejected draws are
   * resampled (see {@link AgentReleaseManager}). Modules may override — e.g. a walk-share filter
   * that keeps most short trips and few long ones.
   *
   * <p>A probability rather than a verdict, so that the draw itself happens in the release manager
   * against its seeded generator. When the state made the draw it reached for a thread-local
   * generator, and this filter decides which trips exist at all: an unseeded draw there put the
   * whole run beyond reach of its own seed.
   */
  public double tripAcceptanceProbability(double meters) {
    return 1.0;
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
