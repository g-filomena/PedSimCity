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
 * This class represents an agent in the pedestrian simulation. Agents move
 * along paths between origin and destination nodes.
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
	Enum<?> agentScenario;
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

	public Agent() {
	}

	public Agent(PedSimCity state, boolean registerSpatial) {
		this.state = state;
		cognitiveMap = new CognitiveMap(this);
		initialiseAgentProperties();
		status = AgentStatus.WAITING;

		// Always initialize currentLocation to prevent NullPointerException
		final GeometryFactory fact = new GeometryFactory();
		currentLocation = new MasonGeometry(fact.createPoint(new Coordinate(0, 0)));
		currentLocation.isMovable = true;

		if (registerSpatial) {
			placeAgent();
		}
	}

	protected void placeAgent() {
		final GeometryFactory fact = new GeometryFactory();
		currentLocation = new MasonGeometry(fact.createPoint(new Coordinate(10, 10)));
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
			// If it's day (not dark) and they haven't worked today, go to work!
			if (workNode != null && !hasWorkedToday && !state.getClass().getSimpleName().contains("Night")) {
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
		planRoute();
		spookLocations.clear();
		tripStartStep = state.schedule.getSteps();
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

	private static final java.util.concurrent.ConcurrentHashMap<String, List<NodeGraph>> candidateCache = new java.util.concurrent.ConcurrentHashMap<>();

	public static List<NodeGraph> getNodesBetweenDistanceIntervalOptimized(Graph network, NodeGraph originNode,
			double lowerLimit, double upperLimit) {
		if (candidateCache.size() > 50_000) {
			candidateCache.clear();
		}
		String key = originNode.getID() + "_" + lowerLimit + "_" + upperLimit;
		return candidateCache.computeIfAbsent(key, k -> {
			double minEuc = lowerLimit / 2.5;
			double maxEuc = upperLimit;
			Coordinate originCoord = originNode.getCoordinate();
			boolean hasEucCandidates = false;
			for (Object obj : network.getNodes()) {
				NodeGraph node = (NodeGraph) obj;
				Coordinate c = node.getCoordinate();
				double dx = c.x - originCoord.x;
				double dy = c.y - originCoord.y;
				double euc = Math.sqrt(dx * dx + dy * dy);
				if (euc >= minEuc && euc <= maxEuc) {
					hasEucCandidates = true;
					break;
				}
			}
			if (!hasEucCandidates) {
				return new ArrayList<>();
			}
			return NodesLookup.getNodesBetweenDistanceInterval(network, originNode, lowerLimit, upperLimit);
		});
	}

	private void defineRandomDestination() {

		double lowerLimit = distanceNextDestination * 0.90;
		double upperLimit = distanceNextDestination;
		Graph network = SharedCognitiveMap.getCommunityPrimalNetwork();
		List<NodeGraph> candidates = new ArrayList<>();
		int maxIterations = 100;
		int iterations = 0;
		while (candidates.isEmpty() && iterations < maxIterations) {
			candidates = getNodesBetweenDistanceIntervalOptimized(network, originNode, lowerLimit, upperLimit);
			candidates.retainAll(
					GraphUtils.getNodesFromNodeIDs(getCognitiveMap().getAgentKnownNodes(), PedSimCity.nodesMap));
			lowerLimit = lowerLimit * 0.90;
			upperLimit = upperLimit * 1.10;
			iterations++;
		}
		if (candidates.isEmpty()) {
			List<NodeGraph> allNodes = network.getNodes();
			candidates = new ArrayList<>(allNodes);
		}

		destinationNode = selectWeightedDestination(candidates, false);
	}

	/**
	 * Selects a destination from a list of candidates weighted by POI counts.
	 * 
	 * @param candidates List of potential destination nodes.
	 * @param isDark     Whether to use night weights (true) or day weights (false).
	 * @return The selected destination NodeGraph.
	 */
	protected NodeGraph selectWeightedDestination(List<NodeGraph> candidates, boolean isDark) {
		if (candidates == null || candidates.isEmpty())
			return null;

		double totalWeight = 0;
		double[] weights = new double[candidates.size()];

		for (int i = 0; i < candidates.size(); i++) {
			weights[i] = pedsim.core.engine.Populate.getPOIWeight(candidates.get(i), isDark);
			totalWeight += weights[i];
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
	 * @param coordinate the coordinates.
	 */
	public void updateAgentPosition(Coordinate coordinate) {
		GeometryFactory geometryFactory = new GeometryFactory();
		Point newLocation = geometryFactory.createPoint(coordinate);
		state.agents.setGeometryLocation(currentLocation, newLocation);
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
		GeometryFactory geometryFactory = new GeometryFactory();
		this.currentLocation.geometry = geometryFactory.createPoint(originNode.getCoordinate());
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
		heuristics.defineHeuristic(originNode, destinationNode, false);
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
