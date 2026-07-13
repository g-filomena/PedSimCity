package pedsim.core.agents;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;
import org.javatuples.Pair;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.Point;
import pedsim.core.cognition.cognitivemap.CognitiveMap;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.engine.PedSimCity;
import pedsim.core.parameters.TimePars;
import pedsim.core.routing.RoutePlanner;
import pedsim.core.utilities.StringEnum.AgentStatus;
import sim.engine.SimState;
import sim.engine.Steppable;
import sim.engine.Stoppable;
import sim.graph.Graph;
import sim.graph.GraphUtils;
import sim.graph.NodeGraph;
import sim.graph.NodesLookup;
import sim.routing.Route;
import sim.util.geo.MasonGeometry;

/**
 * This class represents an agent in the pedestrian simulation. Agents move
 * along paths between origin and destination nodes.
 */
public class Agent implements Steppable {

  protected static final long serialVersionUID = 1L;

  // JTS geometry factories are thread-safe; one instance serves every position update.
  protected static final GeometryFactory GEOMETRY_FACTORY = new GeometryFactory();
  protected PedSimCity state;
  public Integer agentID;

  protected AgentStatus status;
  protected double timeAtDestination = Double.MAX_VALUE;

  public NodeGraph originNode = null;
  public NodeGraph destinationNode = null;
  public List<Pair<NodeGraph, NodeGraph>> OD = new LinkedList<>();

  // in the community network
  public NodeGraph homeNode;
  public NodeGraph workNode;

  protected AgentProperties agentProperties;
  protected CognitiveMap cognitiveMap;

  protected Stoppable killAgent;
  public MasonGeometry currentLocation;
  protected final AtomicBoolean reachedDestination = new AtomicBoolean(false);

  protected Route route;
  protected NodeGraph lastDestination;
  protected Random random = new Random();
  protected AgentMovement agentMovement;
  protected double distanceNextDestination = 0.0;

  private int tripsDone = 0;
  public double metersWalkedTot = 0.0;
  public double metersWalkedDay = 0.0;
  public double tripStartStep = 0.0;
  public List<Coordinate> spookLocations = new ArrayList<>();

  private Heuristics heuristics;
  protected boolean hasWorkedToday = false;

  /**
   * Constructor Function. Creates a new agent with the specified agent
   * properties.
   *
   * @param state the PedSimCity simulation state.
   */
  public Agent(PedSimCity state) {
    this(state, true);
  }

  public Agent() {}

  public Agent(PedSimCity state, boolean registerSpatial) {
    this.state = state;
    cognitiveMap = new CognitiveMap(this);
    initialiseAgentProperties();
    status = AgentStatus.WAITING;

    // Always initialize currentLocation to prevent NullPointerException
    currentLocation = new MasonGeometry(GEOMETRY_FACTORY.createPoint(new Coordinate(0, 0)));
    currentLocation.isMovable = true;

    if (registerSpatial) {
      placeAgent();
    }
  }

  protected void placeAgent() {
    currentLocation = new MasonGeometry(GEOMETRY_FACTORY.createPoint(new Coordinate(10, 10)));
    currentLocation.isMovable = true;
    if (homeNode != null) {
      updateAgentPosition(homeNode.getCoordinate());
    }
  }

  /**
   * Initialises the agent properties.
   */
  protected void initialiseAgentProperties() {
    agentProperties = new AgentProperties();
  }

  /**
   * This is called every tick by the scheduler. It moves the agent along the
   * path.
   *
   * @param state the simulation state.
   */
  @Override
  public void step(SimState state) {

    if (isWaiting()) {
      return;
    }
    if (isWalkingAlone() && destinationNode == null) {
      {
        if (!cognitiveMap.formed) getCognitiveMap().formCognitiveMap();
        planTrip();
      }
    } else if (reachedDestination.get()) {
      handleReachedDestination();
    } else if (isAtDestination() && timeAtDestination <= state.schedule.getSteps()) {
      goHome();
    } else if (isAtDestination()) {
      ;
    } else {
      agentMovement.keepWalking();
    }
  }

  protected synchronized void planTrip() {
    defineOrigin();
    if (isGoingHome()) {
      destinationNode = homeNode;
    } else {
      if (shouldGoToWork()) {
        destinationNode = workNode;
      } else {
        defineRandomDestination();
      }
    }
    // safety check
    if (destinationNode.getID() == (originNode.getID())) {
      reachedDestination.set(true);
      return;
    }
    reinitializeMovementPath();
  }

  public void reinitializeMovementPath() {
    planRoute();
    spookLocations.clear();
    tripStartStep = state.schedule.getSteps();
    agentMovement = createMovement();
    agentMovement.initialisePath(getRoute());
  }

  /**
   * Whether the next non-home trip should target the work node. Core: a work node exists, the
   * agent has not worked today and it is daytime. Activity-based modules refine this with persona
   * work-start windows and day-of-week.
   */
  protected boolean shouldGoToWork() {
    return workNode != null && !hasWorkedToday && !isDark();
  }

  /**
   * Creates the movement handler for a new trip. Modules with specialised movement (e.g. the night
   * module's lighting-aware movement) override this so every trip — including chained trips planned
   * outside {@link #planTrip()} — uses the right handler.
   */
  protected AgentMovement createMovement() {
    return new AgentMovement(this);
  }

  /**
   * Walking-speed multiplier applied to the base move rate each step. Core agents all walk at the
   * average pedestrian speed; activity personas override this (e.g. slower retirees).
   */
  public double getSpeedFactor() {
    return 1.0;
  }

  public void startWalkingAlone() {
    destinationNode = null;
    status = AgentStatus.WALKING_ALONE;
    updateAgentLists(true, false);
  }

  protected void defineOrigin() {

    if (isWalkingAlone()) {
      originNode = homeNode;
    } else if (isGoingHome()) {
      if (currentLocation.getGeometry().getCoordinate() != lastDestination.getCoordinate()) {
        currentLocation.geometry = lastDestination.getMasonGeometry().geometry;
      }
      originNode = lastDestination;
    }
  }

  protected void defineRandomDestination() {

    double lowerLimit = distanceNextDestination * 0.90;
    double upperLimit = distanceNextDestination;
    Graph network = SharedCognitiveMap.getCommunityPrimalNetwork();
    Set<NodeGraph> knownNodes =
        new HashSet<>(
            GraphUtils.getNodesFromNodeIDs(
                getCognitiveMap().getAgentKnownNodes(), PedSimCity.nodesMap));
    List<NodeGraph> candidates = new ArrayList<>();
    int maxIterations = 100;
    int iterations = 0;
    while (candidates.isEmpty() && iterations < maxIterations) {
      candidates =
          NodesLookup.getNodesBetweenDistanceInterval(network, originNode, lowerLimit, upperLimit);
      candidates.retainAll(knownNodes);
      lowerLimit = lowerLimit * 0.90;
      upperLimit = upperLimit * 1.10;
      iterations++;
    }
    if (candidates.isEmpty()) {
      List<NodeGraph> allNodes = network.getNodes();
      candidates = new ArrayList<>(allNodes);
    }

    destinationNode = selectWeightedDestination(candidates, isDark());
  }

  /**
   * Whether the simulation currently considers it "night" for this agent. Core agents have no
   * day/night cycle and always return {@code false}; modules with a 24h clock override this.
   */
  protected boolean isDark() {
    return false;
  }

  /**
   * Selects a destination from a list of candidates weighted by POI counts.
   *
   * @param candidates List of potential destination nodes.
   * @param isDark     Whether to use night weights (true) or day weights (false).
   * @return The selected destination NodeGraph.
   */
  /**
   * Returns the POI-based selection weight for a candidate destination node.
   *
   * <p>Core agents have no destination data, so every node weighs 0.0 (uniform selection).
   * Modules override this to weight candidate nodes by their own attraction data.
   *
   * @param node   The candidate destination node.
   * @param isDark Whether the simulation currently considers it "Night".
   * @return The weight for that node (0.0 when no activity data is loaded).
   */
  protected double getPOIWeight(NodeGraph node, boolean isDark) {
    return 0.0;
  }

  protected NodeGraph selectWeightedDestination(List<NodeGraph> candidates, boolean isDark) {
    if (candidates == null || candidates.isEmpty()) return null;

    double totalWeight = 0;
    double[] weights = new double[candidates.size()];

    for (int i = 0; i < candidates.size(); i++) {
      weights[i] = getPOIWeight(candidates.get(i), isDark);
      totalWeight += weights[i];
    }

    // No activity/POI data (every weight 0) → pick uniformly at random instead of always
    // returning the first candidate (r would be 0 and 0 <= currentSum hits index 0).
    if (totalWeight <= 0.0) {
      return candidates.get(random.nextInt(candidates.size()));
    }

    double r = random.nextDouble() * totalWeight;
    double currentSum = 0;
    for (int i = 0; i < candidates.size(); i++) {
      currentSum += weights[i];
      if (r <= currentSum) {
        return candidates.get(i);
      }
    }

    return candidates.get(random.nextInt(candidates.size())); // Fallback
  }

  protected void handleReachedDestination() {

    reachedDestination.set(false);
    updateAgentPosition(destinationNode.getCoordinate());

    updateAgentLists(false, destinationNode == homeNode);
    originNode = null;
    lastDestination = destinationNode;
    destinationNode = null;
    switch (status) {
      case WALKING_ALONE:
        handleReachedSoloDestination();
        pedsim.core.engine.SimulationStateStore.getInstance().removeAgent(this.agentID);
        break;
      case GOING_HOME:
        handleReachedHome();
        break;
      default:
        break;
    }
  }

  /**
   * Moves the agent to the given coordinates.
   *
   * <p>The geometry is assigned directly: {@code VectorLayer.setGeometryLocation} queries and then
   * rebuilds the layer's entire quadtree on every call, which is O(agents) work per agent per step.
   * The layer's spatial index is instead refreshed once per step by the updater scheduled in
   * {@link pedsim.core.engine.PedSimCity#startMovingAgents()}.
   *
   * @param coordinate the coordinates.
   */
  public void updateAgentPosition(Coordinate coordinate) {
    Point newLocation = GEOMETRY_FACTORY.createPoint(coordinate);
    currentLocation.geometry = newLocation;
    pedsim.core.engine.SimulationStateStore.getInstance().updateAgent(this);
  }

  /**
   * Handles the agent's status when it reaches its solo destination.
   */
  private void handleReachedSoloDestination() {
    status = AgentStatus.AT_DESTINATION;
    if (lastDestination != null && lastDestination.equals(workNode)) {
      hasWorkedToday = true;
    }
    calculateTimeAtDestination(state.schedule.getSteps());
  }

  /**
   * Handles the agent's status when it reaches home.
   */
  protected void handleReachedHome() {
    status = AgentStatus.WAITING;
    hasWorkedToday = false; // Reset for the next day
    pedsim.core.engine.SimulationStateStore.getInstance().removeAgent(this.agentID);
  }

  /**
   * Calculates the time the agent will stay at its destination.
   *
   * @param steps the current simulation step.
   */
  protected void calculateTimeAtDestination(long steps) {
    int randomMinutes;
    if (lastDestination != null && lastDestination.equals(workNode)) {
      // Work stay: 6 to 9 hours (360 to 540 minutes)
      randomMinutes = 360 + random.nextInt(181);
    } else {
      // POI/Social stay: 15 to 120 minutes (original logic)
      randomMinutes = 15 + random.nextInt(106);
    }

    timeAtDestination = (randomMinutes * TimePars.MINUTE_TO_STEPS) + steps;
  }

  /**
   * Sets the ordered OD list for agents that run a fixed sequence of trips.
   * Places the agent at the first origin without triggering a layer update.
   */
  public void setOD(List<Pair<NodeGraph, NodeGraph>> odPairs) {
    this.OD = new LinkedList<>(odPairs);
    if (!this.OD.isEmpty()) {
      this.originNode = this.OD.get(0).getValue0();
      placeAtOriginWithoutLayerUpdate();
    }
  }

  // Avoids a layer update before the first step (agent placed directly at origin
  // geometry).
  protected void placeAtOriginWithoutLayerUpdate() {
    if (originNode == null) {
      return;
    }
    this.currentLocation.geometry = GEOMETRY_FACTORY.createPoint(originNode.getCoordinate());
  }

  protected void selectNodesFromOD() {
    Pair<NodeGraph, NodeGraph> pair = OD.get(getTripsDone());
    originNode = pair.getValue0();
    destinationNode = pair.getValue1();
  }

  /**
   * The agent goes home after reaching its destination.
   */
  protected void goHome() {

    state.agentsWalking.add(this);
    status = AgentStatus.GOING_HOME;
    planTrip();
  }

  /**
   * Updates the agent's status in the agent lists.
   *
   * @param isWalking   indicates whether the agent is walking or not.
   * @param reachedHome indicates whether the agent has reached home.
   */
  public void updateAgentLists(boolean isWalking, boolean reachedHome) {

    state.agentsList.add(this);
    if (isWalking) {
      state.agentsWalking.add(this);
      state.agentsAtHome.remove(this);
    } else {
      if (reachedHome) {
        state.agentsAtHome.add(this);
      }
      state.agentsWalking.remove(this);
    }
  }

  /**
   * Plans the route for the agent.
   */
  protected void planRoute() {
    // Initialise and store the agent's heuristics so that other components
    // (e.g. landmark-based navigation) can safely access them via getHeuristics().
    heuristics = new Heuristics(this);
    heuristics.defineHeuristic(false);
    RoutePlanner planner = new RoutePlanner(originNode, destinationNode, this);
    setRoute(planner.definePath());
  }

  /**
   * Sets the stoppable reference for the agent.
   *
   * @param a The stoppable reference.
   */
  public void setStoppable(Stoppable a) {
    this.killAgent = a;
  }

  /**
   * Removes the agent from the simulation.
   *
   */
  protected void removeAgent() {
    state.agentsList.remove(this);
    killAgent.stop();
    if (state.agentsList.isEmpty()) {
      state.finish();
    }
  }

  /**
   * Gets the geometry representing the agent's location.
   *
   * @return The geometry representing the agent's location.
   */
  public MasonGeometry getLocation() {
    return currentLocation;
  }

  /**
   * Gets the agent's properties.
   *
   * @return The agent's properties.
   */
  public AgentProperties getProperties() {
    return agentProperties;
  }

  /**
   * Gets the agent's cognitive map.
   *
   * @return The cognitive map.
   */
  public CognitiveMap getCognitiveMap() {
    return cognitiveMap;
  }

  public AgentStatus getStatus() {
    return status;
  }

  public void setStatus(AgentStatus status) {
    this.status = status;
  }


  /**
   * Checks if the agent is waiting.
   *
   * @return true if the agent is waiting, false otherwise.
   */
  protected boolean isWaiting() {
    return status.equals(AgentStatus.WAITING);
  }

  /**
   * Checks if the agent is walking alone.
   *
   * @return true if the agent is walking alone, false otherwise.
   */
  protected boolean isWalkingAlone() {
    return status.equals(AgentStatus.WALKING_ALONE);
  }

  /**
   * Checks if the agent is going home.
   *
   * @return true if the agent is going home, false otherwise.
   */
  protected boolean isGoingHome() {
    return status.equals(AgentStatus.GOING_HOME);
  }

  /**
   * Checks if the agent is at its destination.
   *
   * @return true if the agent is at its destination, false otherwise.
   */
  protected boolean isAtDestination() {
    return status.equals(AgentStatus.AT_DESTINATION);
  }

  /**
   * Gets the total distance the agent has walked.
   *
   * @return The total distance the agent has walked in kilometers.
   */
  public double getTotalMetersWalked() {
    return metersWalkedTot;
  }

  /**
   * Gets the distance the agent has walked in the current day.
   *
   * @return The distance walked by the agent today in kilometers.
   */
  public double getMetersWalkedDay() {
    return metersWalkedDay;
  }

  /**
   * Sets the distance to the next destination for the agent.
   *
   * @param distanceNextDestination The distance to the next destination.
   */
  public void setDistanceNextDestination(double distanceNextDestination) {
    this.distanceNextDestination = distanceNextDestination;
  }

  /**
   * Gets the simulation state of the agent.
   *
   * @return The PedSimCity simulation state.
   */
  public PedSimCity getState() {
    return state;
  }

  /**
   * The agent-type category this agent's volumes are tallied under. Plain agents have no sub-type,
   * so they fall under a single {@code DEFAULT} category (never null). Modules that group agents
   * (night = vulnerability, learning = learner, …) override this.
   */
  public Enum<?> getAgentScenario() {
    return pedsim.core.utilities.StringEnum.Default.DEFAULT;
  }

  /**
   * The simulation (time-context) scenario this agent's volumes are tallied under, or {@code null}
   * if the module has none. Activity overrides this to the hour of day; night to day/night.
   */
  public Enum<?> getSimulationScenario() {
    return null;
  }

  /**
   * Mean illuminance (lux) experienced on the just-completed trip, or {@code NaN} when the module
   * tracks no lighting. Modules that track lighting override this; recorded per trip by {@link
   * pedsim.core.engine.TripRouteRecorder}.
   */
  public double getTripMeanLux() {
    return Double.NaN;
  }

  public Heuristics getHeuristics() {
    return heuristics;
  }

  /**
   * @return the route
   */
  public Route getRoute() {
    return route;
  }

  /**
   * @param route the route to set
   */
  public void setRoute(Route route) {
    this.route = route;
  }

  public void setHomeWorkLoctations(NodeGraph homeNode, NodeGraph workNode) {
    this.homeNode = homeNode;
    this.workNode = workNode;
  }

  /**
   * Gets the home node for the agent in the cognitive map.
   *
   * @return The home node for the agent.
   */
  public NodeGraph getHome() {
    return homeNode;
  }

  public NodeGraph getWork() {
    return workNode;
  }

  protected boolean vulnerable = false;

  public boolean isVulnerableBoolean() {
    return vulnerable;
  }

  public void setVulnerable(boolean vulnerable) {
    this.vulnerable = vulnerable;
  }

  public void setReachedDestination(boolean reached) {
    this.reachedDestination.set(reached);
  }

  /**
   * @param tripsDone the tripsDone to set
   */
  public void setTripsDone(int tripsDone) {
    this.tripsDone = tripsDone;
  }

  /**
   * @return the tripsDone
   */
  public int getTripsDone() {
    return tripsDone;
  }
}
