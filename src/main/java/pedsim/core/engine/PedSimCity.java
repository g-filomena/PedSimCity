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
 * 
 * @param <E>
 */
public class PedSimCity extends SimState {
  private static final long serialVersionUID = 1L;

  // Urban elements: graphs, buildings, etc.
  public static VectorLayer roads = new VectorLayer();
  public static VectorLayer buildings = new VectorLayer();
  public static VectorLayer barriers = new VectorLayer();
  public static VectorLayer junctions = new VectorLayer();
  public static VectorLayer sightLines = new VectorLayer();

  final public static Graph network = new Graph();
  final public static Graph dualNetwork = new Graph();
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
  public static List<Float> distances = new ArrayList<>();
  public static List<MasonGeometry> startingNodes = new ArrayList<>();

  public static Map<DirectedEdge, LengthIndexedLine> indexedEdgeCache = new ConcurrentHashMap<>();
  public static Map<Pair<Coordinate, Coordinate>, Polygon> visibilityPolygonsCache =
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
   * Constructs a new instance of the PedSimCity simulation environment.
   *
   * @param seed The random seed for the simulation.
   * @param job The current job number for multi-run simulations.
   * @param <E>
   * @param <E>
   */
  public PedSimCity(long seed, int job, ScenarioConfig scenarioConfig) {
    super(seed);
    this.currentJob = job;
    this.scenarioConfig = scenarioConfig;
    this.agents = new VectorLayer();
    this.flowHandler = new FlowHandler(job, this);
    this.agents = new VectorLayer();
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
}
