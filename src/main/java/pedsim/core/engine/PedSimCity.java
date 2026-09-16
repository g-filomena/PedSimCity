package pedsim.core.engine;

import ec.util.MersenneTwisterFast;
import java.util.ArrayList;
import java.util.Comparator;
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
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.cognition.elements.Barrier;
import pedsim.core.cognition.elements.Gateway;
import pedsim.core.cognition.elements.Region;
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

  /** The module's name, which is also the folder its outputs are written under. */
  public String appName() {
    return appName;
  }

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

  /** Completed trips belonging exclusively to this job. */
  public final TripRouteRecorder tripRecorder = new TripRouteRecorder();

  /**
   * Per-thread state reference, set in the constructor.
   * Simulation threads each get their own job's state via {@link #currentForThread()}.
   */
  private static final ThreadLocal<PedSimCity> THREAD_STATE = new ThreadLocal<>();

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
   * Who goes out, when, and by what mode. Built lazily so that a module's override is in place
   * before the first release event, and cached because the departure profile it holds is rebuilt
   * per simulated day rather than per query.
   */
  private TravelDemand travelDemand;

  /**
   * This run's travel demand; see {@link TravelDemand}.
   *
   * <p>Synchronised rather than built in the constructor: {@link #createTravelDemand()} is
   * overridden by subclasses, and calling an overridable method from a base constructor reads the
   * subclass before its fields exist. The first call comes from agent construction, which is
   * single-threaded today and need not stay that way.
   */
  public synchronized TravelDemand travelDemand() {
    if (travelDemand == null) {
      travelDemand = createTravelDemand();
    }
    return travelDemand;
  }

  /**
   * The travel demand this simulation runs on. Modules override to supply their own - the activity
   * tier's {@code ActivityTravelDemand}, the night module's {@code NightTravelDemand}.
   */
  protected TravelDemand createTravelDemand() {
    return new BaselineTravelDemand(this);
  }

  /** What this run measured about itself; see {@link RouteTrace}. Measurement only, never fed back. */
  private final RouteTrace trace = new RouteTrace();

  /** What this run measured about itself, and the per-leg record behind it. */
  public RouteTrace trace() {
    return trace;
  }

  /**
   * Seeds for the agents of this simulation, handed out in construction order.
   *
   * <p>Per state, deliberately. A static counter would be shared by every job in the JVM: jobs run
   * sequentially or, where the module allows it, through a parallel stream, so a JVM-wide counter
   * makes an agent's seed depend on how many agents other jobs built first, and under a parallel
   * stream on thread interleaving. Both defeat the point of seeding at all.
   */
  private final java.util.concurrent.atomic.AtomicLong agentSeedSequence =
      new java.util.concurrent.atomic.AtomicLong();

  /** The next agent seed, derived from this simulation's seed and the agent's construction order. */
  public long nextAgentSeed() {
    return seed() * 1_000_003L + agentSeedSequence.getAndIncrement();
  }

  /**
   * Starts moving agents in the simulation. This method schedules agents for repeated movement
   * updates and sets up the spatial index for agents.
   */
  /**
   * Keeps the spatial-index updater stoppable, so a simulation whose agents have all finished can
   * actually empty its schedule.
   *
   * <p>It must be stoppable because a module's engine may end its job when the schedule empties —
   * the cityImage and empirical engines loop on {@code while (state.schedule.step(state))} — and a
   * steppable that never stops keeps that condition true after the last agent has gone.
   */
  private Stoppable spatialIndexUpdater;

  protected void startMovingAgents() {
    // A random order, drawn from the model seed: random, and the same random on every machine.
    //
    // MASON breaks ties within a tick by the order steppables were scheduled, so this decides the
    // order agents step - and therefore the order they plan legs, occupy edges and observe each
    // other's crowding. There is no correct order (nothing makes one pedestrian move before
    // another), so it should be drawn rather than fixed; what it must not be is drawn from
    // something outside the seed.
    //
    // It was the latter: agentsList is a ConcurrentHashMap key set and Agent overrides no hashCode,
    // so it iterated in identity-hash order, which HotSpot derives from a per-JVM generator. The
    // same seed therefore stepped agents in a different order on a different machine. It shows up
    // only once several agents are walking at once, which is why the opening legs of a run matched
    // across machines and the rest did not.
    //
    // Ordering by a per-agent key rather than shuffling the list, so that nothing here depends on
    // the order agentsList happens to enumerate. A shuffle would: Fisher-Yates permutes positions,
    // so shuffling an identity-hash-ordered list turns one unknown order into another, and it would
    // need a canonical sort first to be reproducible - an invariant that looks redundant and
    // reintroduces this whole defect the day somebody removes it. scheduleKey depends only on the
    // agent and the seed, so the result is a function of which agents exist, never of how they were
    // reached.
    List<Agent> inScheduleOrder = new ArrayList<>(agentsList);
    inScheduleOrder.sort(
        Comparator.comparingLong((Agent agent) -> scheduleKey(agent.agentID))
            .thenComparingInt(agent -> agent.agentID));
    for (Agent agent : inScheduleOrder) {
      Stoppable stop = schedule.scheduleRepeating(agent);
      agent.setStoppable(stop);
    }
    // A single end-of-step index refresh covers the whole layer; scheduling it per agent would
    // rebuild the quadtree N times every step.
    spatialIndexUpdater =
        schedule.scheduleRepeating(agents.scheduleSpatialIndexUpdater(), Integer.MAX_VALUE, 1.0);
    agents.setMBR(MBR);
  }

  /**
   * This agent's position in the step order: a value drawn from the run's seed and the agent's own
   * ID, and from nothing else.
   *
   * <p>Ordering by it gives a random permutation that is a pure function of the seed, arrived at
   * without enumerating anything - which is the property the step order needs and the one it did
   * not have while it came from a hash set's iteration order. Because the key depends only on the
   * agent, the order cannot inherit the order the agents were reached in.
   *
   * <p>Seeded per agent in the same way as everything else here - a distinct multiplier off the
   * model seed, as {@code Populate.seedFrom} and {@link #nextAgentSeed()} do - so that this draws
   * from MASON's generator like every other draw in the simulation rather than introducing a
   * second, separately-reasoned-about source of randomness. Built once per agent at startup, so
   * the cost of constructing a generator does not matter. Two agents drawing the same long is
   * vanishingly unlikely over 64 bits but not impossible, which is what the ID tiebreak in the
   * comparator is for.
   */
  private long scheduleKey(int agentID) {
    return new MersenneTwisterFast(seed() * 15_485_863L + agentID).nextLong();
  }

  /**
   * Stops the spatial-index refresh. Called when the last agent has gone: there is nothing left to
   * index, and leaving it scheduled is what stopped a schedule-driven job from ever ending.
   */
  public void stopSpatialIndexUpdater() {
    if (spatialIndexUpdater != null) {
      spatialIndexUpdater.stop();
      spatialIndexUpdater = null;
    }
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
    SharedCognitiveMap.clearStaticData();
    edgesMap.clear();
    centroidsMap.clear();
    edges.clear();
    startingNodes.clear();

    indexedEdgeCache.clear();
    MBR = null;

    dualGraphLoaded = false;
    landmarksLoaded = false;
  }

  /**
   * The exporter this model writes its result files with.
   *
   * @param flowHandler the flow handler holding the volumes
   * @param appName the output folder name
   * @return the exporter; a model returns its own to add columns of its own
   */
  protected Exporter createExporter(FlowHandler flowHandler, String appName) {
    return new Exporter(flowHandler, appName);
  }
}
