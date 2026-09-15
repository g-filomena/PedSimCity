package pedsim.core.engine;

import com.fasterxml.jackson.annotation.JsonProperty;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicReference;
import java.util.logging.Logger;
import pedsim.core.agents.Agent;
import pedsim.core.utilities.LoggerUtil;
import tools.jackson.databind.ObjectMapper;
import tools.jackson.databind.exc.JsonNodeException;

/**
 * Thread-safe singleton that holds the live simulation state for the browser dashboard. The
 * simulation engine writes to this store on every step; the REST endpoint reads from it to serve
 * agent positions and statistics to the map.
 */
public final class SimulationStateStore {

  private static final Logger logger = LoggerUtil.getLogger();

  // ----------------------------------------------------------------
  // Singleton
  // ----------------------------------------------------------------

  private static final SimulationStateStore INSTANCE = new SimulationStateStore();

  private SimulationStateStore() {}

  public static SimulationStateStore getInstance() {
    return INSTANCE;
  }

  private RunReservation activeRun;

  /** Atomically reserves the process-wide simulation before any parameters are changed. */
  public synchronized RunReservation tryReserveRun() {
    if (activeRun != null || running) return null;
    reset();
    running = true;
    activeRun = new RunReservation();
    return activeRun;
  }

  /** A reservation may be handed from the HTTP thread to its simulation thread. */
  public final class RunReservation implements AutoCloseable {
    private boolean closed;

    private RunReservation() {}

    public void requireActive() {
      synchronized (SimulationStateStore.this) {
        if (closed || activeRun != this) {
          throw new IllegalStateException("Simulation run reservation is no longer active");
        }
      }
    }

    @Override
    public void close() {
      synchronized (SimulationStateStore.this) {
        if (closed) return;
        closed = true;
        if (activeRun == this) {
          activeRun = null;
          running = false;
          finished = true;
        }
      }
    }
  }

  // ----------------------------------------------------------------
  // State fields
  // ----------------------------------------------------------------

  public volatile int currentStep = 0;
  public volatile String simulationTime = "—";
  public volatile int walkingCount = 0;
  public volatile int atHomeCount = 0;
  public volatile int atDestCount = 0;
  public volatile boolean running = false;
  public volatile boolean finished = false;
  public volatile boolean stopRequested = false;
  public volatile String roadsGeoJson = null;

  /**
   * Active module reference. {@link AtomicReference} ensures the reference swap is visible across
   * threads; individual module field reads (e.g. NightPars) are expected to be either volatile or
   * written only during parameter setup before {@code runJobs()} begins.
   */
  private final AtomicReference<SimulationModule> activeModule = new AtomicReference<>(null);

  private double vulnTripDistanceSum = 0;
  private int vulnTripCount = 0;
  private double normalTripDistanceSum = 0;
  private int normalTripCount = 0;

  /** Live snapshot of all agent positions, updated each simulation step. */
  private final ConcurrentHashMap<Integer, AgentSnapshot> agents = new ConcurrentHashMap<>();

  private static final ObjectMapper MAPPER = new ObjectMapper();

  // ----------------------------------------------------------------
  // Write methods (called from simulation threads)
  // ----------------------------------------------------------------

  public synchronized void addCompletedTrip(boolean vulnerable, double distance) {
    if (vulnerable) {
      vulnTripDistanceSum += distance;
      vulnTripCount++;
    } else {
      normalTripDistanceSum += distance;
      normalTripCount++;
    }
  }

  public synchronized double getAvgVulnTripM() {
    return vulnTripCount > 0 ? vulnTripDistanceSum / vulnTripCount : -1;
  }

  public synchronized double getAvgNormalTripM() {
    return normalTripCount > 0 ? normalTripDistanceSum / normalTripCount : -1;
  }

  public synchronized void resetTripStats() {
    vulnTripDistanceSum = 0;
    vulnTripCount = 0;
    normalTripDistanceSum = 0;
    normalTripCount = 0;
  }

  public void updateAgent(Agent agent) {
    if (agent.getLocation() == null) return;
    var coord = agent.getLocation().geometry.getCoordinate();
    agents.put(
        agent.agentID,
        new AgentSnapshot(
            agent.agentID,
            coord.x,
            coord.y,
            agent.getStatus().toString(),
            agent.isVulnerableBoolean()));
  }

  public void removeAgent(int agentId) {
    agents.remove(agentId);
  }

  public void updateStep(
      int step, String simTime, int walkingCount, int atHomeCount, int atDestCount) {
    this.currentStep = step;
    this.simulationTime = simTime;
    this.walkingCount = walkingCount;
    this.atHomeCount = atHomeCount;
    this.atDestCount = atDestCount;
  }

  public void setRoadsGeoJson(String geoJson) {
    this.roadsGeoJson = geoJson;
  }

  public void requestStop() {
    this.stopRequested = true;
  }

  /**
   * Records which module is currently active. Called by {@code SimulationLauncher.headlessRun}
   * before parameter application and before {@code runJobs()}.
   */
  public void setActiveModule(SimulationModule module) {
    activeModule.set(module);
  }

  /**
   * One boolean from the active module's {@link SimulationModule#extraState()}, or {@code false}
   * when there is no active module, it publishes no such key, or the value is not a boolean.
   *
   * <p>For core code that has to branch on something only a module knows. The alternative in use
   * before this was {@code Class.forName("pedsim.night.parameters.NightPars")} inside
   * {@code HtmlExporter} — core naming a module by string, so the compiler could not see the
   * coupling, and renaming the field would have made the dashboard quietly report no A/B test
   * forever. A module already declares what it wants core to see; core should ask.
   *
   * @param key the key the module publishes
   */
  public boolean moduleFlag(String key) {
    SimulationModule active = activeModule.get();
    if (active == null) {
      return false;
    }
    try {
      return active.extraState().get(key) instanceof Boolean flag && flag;
    } catch (Exception e) {
      logger.warning("extraState() threw while reading " + key + ": " + e.getMessage());
      return false;
    }
  }

  /** Resets all transient state for a new run. Does not clear the active module. */
  public void reset() {
    currentStep = 0;
    simulationTime = "—";
    walkingCount = 0;
    atHomeCount = 0;
    atDestCount = 0;
    running = false;
    finished = false;
    stopRequested = false;
    agents.clear();
    resetTripStats();
  }

  // ----------------------------------------------------------------
  // Read methods (called from the REST endpoint thread)
  // ----------------------------------------------------------------

  public List<AgentSnapshot> getAgents() {
    return new ArrayList<>(agents.values());
  }

  public String toJson() throws JsonNodeException {
    return MAPPER.writeValueAsString(new StateSnapshot(this));
  }

  // ----------------------------------------------------------------
  // Inner record: per-agent snapshot
  // ----------------------------------------------------------------

  public record AgentSnapshot(
      @JsonProperty("id") int id,
      @JsonProperty("lon") double lon,
      @JsonProperty("lat") double lat,
      @JsonProperty("status") String status,
      @JsonProperty("vulnerable") boolean vulnerable) {}

  // ----------------------------------------------------------------
  // Inner class: full-state DTO for JSON serialisation
  // ----------------------------------------------------------------

  public static class StateSnapshot {

    @JsonProperty("currentStep")
    public final int currentStep;

    @JsonProperty("simulationTime")
    public final String simulationTime;

    @JsonProperty("walkingCount")
    public final int walkingCount;

    @JsonProperty("atHomeCount")
    public final int atHomeCount;

    @JsonProperty("atDestCount")
    public final int atDestCount;

    @JsonProperty("running")
    public final boolean running;

    @JsonProperty("finished")
    public final boolean finished;

    @JsonProperty("avgVulnTripM")
    public final double avgVulnTripM;

    @JsonProperty("avgNormalTripM")
    public final double avgNormalTripM;

    @JsonProperty("agents")
    public final List<AgentSnapshot> agents;

    /** Active module identifier (e.g. {@code "core"}, {@code "night"}). */
    @JsonProperty("module")
    public final String module;

    /**
     * Module-specific live state from {@link SimulationModule#extraState()}. For night: includes
     * {@code enableAB} and {@code crowdednessPercentile}. Empty map for core.
     *
     * <p>If {@code extraState()} throws for any reason the field is an empty map so that {@code
     * /api/state} never fails due to a module implementation bug.
     */
    @JsonProperty("moduleState")
    public final Map<String, Object> moduleState;

    StateSnapshot(SimulationStateStore store) {
      this.currentStep = store.currentStep;
      this.simulationTime = store.simulationTime;
      this.walkingCount = store.walkingCount;
      this.atHomeCount = store.atHomeCount;
      this.atDestCount = store.atDestCount;
      this.running = store.running;
      this.finished = store.finished;
      this.avgVulnTripM = store.getAvgVulnTripM();
      this.avgNormalTripM = store.getAvgNormalTripM();
      this.agents = store.getAgents();

      SimulationModule active = store.activeModule.get();
      this.module = active != null ? active.moduleId() : "core";

      Map<String, Object> extra = Map.of();
      if (active != null) {
        try {
          extra = active.extraState();
        } catch (Exception e) {
          logger.warning(
              "extraState() threw for module " + active.moduleId() + ": " + e.getMessage());
        }
      }
      this.moduleState = extra;
    }
  }
}
