package pedsim.core.agents;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;
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
 * This class represents an agent in the pedestrian simulation. Agents move along paths between
 * origin and destination nodes.
 */
public class Agent implements Steppable {

  protected static final long serialVersionUID = 1L;
  protected PedSimCity state;
  public Integer agentID;

  protected AgentStatus status;
  protected double timeAtDestination = Double.MAX_VALUE;

  public NodeGraph originNode = null;
  public NodeGraph destinationNode = null;
  public List<Pair<NodeGraph, NodeGraph>> OD = new LinkedList<>();

  // in the community network
  protected NodeGraph homeNode;
  protected NodeGraph workNode;

  protected AgentProperties agentProperties;
  protected CognitiveMap cognitiveMap;

  protected Stoppable killAgent;
  protected MasonGeometry currentLocation;
  protected final AtomicBoolean reachedDestination = new AtomicBoolean(false);

  protected Route route;
  protected NodeGraph lastDestination;
  protected Random random = new Random();
  protected AgentMovement agentMovement;
  protected double distanceNextDestination = 0.0;

  public double metersWalkedTot = 0.0;
  public double metersWalkedDay = 0.0;

  private Heuristics heuristics;
  Enum<?> agentScenario;

  /**
   * Constructor Function. Creates a new agent with the specified agent properties.
   *
   * @param state the PedSimCity simulation state.
   */
  public Agent(PedSimCity state) {

    this.state = state;
    cognitiveMap = new CognitiveMap(this);
    initialiseAgentProperties();
    status = AgentStatus.WAITING;
    placeAgent();
  }

  protected void placeAgent() {
    final GeometryFactory fact = new GeometryFactory();
    currentLocation = new MasonGeometry(fact.createPoint(new Coordinate(10, 10)));
    currentLocation.isMovable = true;
    if (homeNode != null) {
      updateAgentPosition(homeNode.getCoordinate());
    }
  }

  public Agent() {}

  /**
   * Initialises the agent properties.
   */
  protected void initialiseAgentProperties() {
    agentProperties = new AgentProperties(this);
  }

  /**
   * This is called every tick by the scheduler. It moves the agent along the path.
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
        if (!cognitiveMap.formed)
          getCognitiveMap().formCognitiveMap();
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
      defineRandomDestination();
    }
    // safety check
    if (destinationNode.getID() == originNode.getID()) {
      reachedDestination.set(true);
      return;
    }
    planRoute();
    agentMovement = new AgentMovement(this);
    agentMovement.initialisePath(getRoute());
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

  private void defineRandomDestination() {

    double lowerLimit = distanceNextDestination * 0.90;
    double upperLimit = distanceNextDestination;
    Graph network = SharedCognitiveMap.getCommunityPrimalNetwork();
    List<NodeGraph> candidates = new ArrayList<>();
    while (candidates.isEmpty()) {
      candidates =
          NodesLookup.getNodesBetweenDistanceInterval(network, originNode, lowerLimit, upperLimit);
      candidates.retainAll(GraphUtils.getNodesFromNodeIDs(getCognitiveMap().getAgentKnownNodes(),
          PedSimCity.nodesMap));
      lowerLimit = lowerLimit * 0.90;
      upperLimit = upperLimit * 1.10;
    }
    destinationNode = NodesLookup.randomNodeFromList(candidates);
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
   * @param c the coordinates.
   */
  public void updateAgentPosition(Coordinate coordinate) {
    GeometryFactory geometryFactory = new GeometryFactory();
    Point newLocation = geometryFactory.createPoint(coordinate);
    state.agents.setGeometryLocation(currentLocation, newLocation);
    currentLocation.geometry = newLocation;
  }

  /**
   * Handles the agent's status when it reaches its solo destination.
   */
  private void handleReachedSoloDestination() {
    status = AgentStatus.AT_DESTINATION;
    calculateTimeAtDestination(state.schedule.getSteps());
  }

  /**
   * Handles the agent's status when it reaches home.
   */
  protected void handleReachedHome() {
    status = AgentStatus.WAITING;
  }

  /**
   * Calculates the time the agent will stay at its destination.
   *
   * @param steps the current simulation step.
   */
  protected void calculateTimeAtDestination(long steps) {
    // Generate a random number between 15 (inclusive) and 120 (inclusive)
    int randomMinutes = 15 + random.nextInt(106);
    // Multiply with MINUTES_IN_STEPS
    timeAtDestination = (randomMinutes * TimePars.MINUTE_TO_STEPS) + steps;
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
   * @param isWalking indicates whether the agent is walking or not.
   * @param reachedHome indicates whether the agent has reached home.
   */
  public void updateAgentLists(boolean isWalking, boolean reachedHome) {

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
   *
   * @throws Exception
   */
  protected void planRoute() {
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

  public Enum<?> getAgentScenario() {
    return agentScenario;
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

}
