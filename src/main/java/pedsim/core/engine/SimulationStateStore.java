package pedsim.core.engine;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import pedsim.core.agents.Agent;

/**
 * Thread-safe singleton that holds the live simulation state for the browser
 * dashboard. The simulation engine writes to this store on every step;
 * the REST endpoint reads from it to serve agent positions and statistics
 * to the Leaflet map.
 */
public final class SimulationStateStore {

  // ----------------------------------------------------------------
  // Singleton
  // ----------------------------------------------------------------

  private static final SimulationStateStore INSTANCE = new SimulationStateStore();

  private SimulationStateStore() {}

  /**
   * Returns the single shared instance.
   *
   * @return the SimulationStateStore singleton.
   */
  public static SimulationStateStore getInstance() {
    return INSTANCE;
  }

  // ----------------------------------------------------------------
  // State fields
  // ----------------------------------------------------------------

  /** Current simulation step count. */
  public volatile int currentStep = 0;

  /** Human-readable simulation time string. */
  public volatile String simulationTime = "—";

  /** Number of agents currently walking. */
  public volatile int walkingCount = 0;

  /** Number of agents currently at home. */
  public volatile int atHomeCount = 0;

  /** Number of agents currently at their destination. */
  public volatile int atDestCount = 0;

  /** Whether the simulation is currently running. */
  public volatile boolean running = false;

  /** Whether the simulation has finished. */
  public volatile boolean finished = false;

  /**
   * Whether the Javelit dashboard has requested the simulation to stop.
   * The engine's run loop polls this flag.
   */
  public volatile boolean stopRequested = false;

  /**
   * GeoJSON string for the road network.
   */
  public volatile String roadsGeoJson = null;

  /** Live snapshot of all agent positions, updated each simulation step. */
  private final CopyOnWriteArrayList<AgentSnapshot> agents = new CopyOnWriteArrayList<>();

  private static final ObjectMapper MAPPER = new ObjectMapper();

  // ----------------------------------------------------------------
  // Write methods (called from simulation threads)
  // ----------------------------------------------------------------

  /**
   * Updates the position and status of a single agent in the live snapshot.
   */
  public void updateAgent(Agent agent) {
    if (agent.getLocation() == null)
      return;
    var coord = agent.getLocation().geometry.getCoordinate();
    var snapshot = new AgentSnapshot(
        agent.agentID,
        coord.x,
        coord.y,
        agent.getClass().getSimpleName(),
        agent.isVulnerableBoolean()
    );
    // replace existing entry or add new one
    agents.removeIf(s -> s.id == agent.agentID);
    agents.add(snapshot);
  }

  /**
   * Removes an agent from the live snapshot so the browser dashboard no longer
   * shows it (called when an agent finishes its journey and reaches home).
   *
   * @param agentId the ID of the agent to remove.
   */
  public void removeAgent(int agentId) {
    agents.removeIf(s -> s.id == agentId);
  }

  /**
   * Updates aggregate step-level statistics.
   */
  public void updateStep(int step, String simTime,
      int walkingCount, int atHomeCount, int atDestCount) {
    this.currentStep  = step;
    this.simulationTime = simTime;
    this.walkingCount = walkingCount;
    this.atHomeCount  = atHomeCount;
    this.atDestCount  = atDestCount;
  }

  /**
   * Stores the GeoJSON representation of the road network.
   */
  public void setRoadsGeoJson(String geoJson) {
    this.roadsGeoJson = geoJson;
  }

  /**
   * Signals the engine to stop at the end of the current step.
   */
  public void requestStop() {
    this.stopRequested = true;
  }

  /**
   * Resets all state ready for a new simulation run.
   */
  public void reset() {
    currentStep    = 0;
    simulationTime = "—";
    walkingCount   = 0;
    atHomeCount    = 0;
    atDestCount    = 0;
    running        = false;
    finished       = false;
    stopRequested  = false;
    agents.clear();
  }

  // ----------------------------------------------------------------
  // Read methods (called from the REST endpoint thread)
  // ----------------------------------------------------------------

  /**
   * Returns a copy of the current agent snapshot list safe for JSON serialisation.
   */
  public List<AgentSnapshot> getAgents() {
    return new ArrayList<>(agents);
  }

  /**
   * Serialises the current state to a JSON string for the /api/state endpoint.
   */
  public String toJson() throws JsonProcessingException {
    return MAPPER.writeValueAsString(new StateSnapshot(this));
  }

  // ----------------------------------------------------------------
  // Inner record: per-agent snapshot
  // ----------------------------------------------------------------

  /**
   * Immutable snapshot of a single agent's position and status.
   */
  public record AgentSnapshot(
      @JsonProperty("id")         int id,
      @JsonProperty("lon")        double lon,
      @JsonProperty("lat")        double lat,
      @JsonProperty("status")     String status,
      @JsonProperty("vulnerable") boolean vulnerable
  ) {}

  // ----------------------------------------------------------------
  // Inner class: full-state DTO for JSON serialisation
  // ----------------------------------------------------------------

  /**
   * Data-transfer object wrapping the full simulation state for one JSON snapshot.
   */
  public static class StateSnapshot {

    @JsonProperty("currentStep")    public final int currentStep;
    @JsonProperty("simulationTime") public final String simulationTime;
    @JsonProperty("walkingCount")   public final int walkingCount;
    @JsonProperty("atHomeCount")    public final int atHomeCount;
    @JsonProperty("atDestCount")    public final int atDestCount;
    @JsonProperty("running")        public final boolean running;
    @JsonProperty("finished")       public final boolean finished;
    @JsonProperty("agents")         public final List<AgentSnapshot> agents;

    StateSnapshot(SimulationStateStore store) {
      this.currentStep    = store.currentStep;
      this.simulationTime = store.simulationTime;
      this.walkingCount   = store.walkingCount;
      this.atHomeCount    = store.atHomeCount;
      this.atDestCount    = store.atDestCount;
      this.running        = store.running;
      this.finished       = store.finished;
      this.agents         = store.getAgents();
    }
  }
}
