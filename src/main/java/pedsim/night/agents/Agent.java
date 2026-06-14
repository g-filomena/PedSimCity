package pedsim.night.agents;

import java.util.List;

import pedsim.core.agents.Heuristics;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.utilities.StringEnum;
import pedsim.core.utilities.StringEnum.Vulnerable;
import pedsim.night.engine.PedSimCityNight;
import pedsim.night.routing.RoutePlanner;
import sim.engine.SimState;
import sim.engine.Steppable;
import sim.graph.Graph;
import sim.graph.NodeGraph;
import sim.graph.NodesLookup;

/**
 * This class represents an agent in the pedestrian simulation. Agents move
 * along paths between origin and destination nodes.
 */
public class Agent extends pedsim.core.agents.Agent implements Steppable {

	private static final long serialVersionUID = 1L;
	public Agent abTestTwin = null;
	public StringEnum.Vulnerable vulnerable;
	private Graph agentNetwork;
	protected pedsim.night.agents.AgentMovement nightMovement;
	protected PedSimCityNight state;
	private boolean nightRoute = false;

	public double lightSensitivityThreshold;
	public double accumulatedLux = 0.0;
	public int edgesWalked = 0;

	/**
	 * Constructor Function. Creates a new agent with the specified agent
	 * properties.
	 *
	 * @param state the PedSimCity simulation state.
	 */
	public Agent(PedSimCityNight state) {
		this(state, true);
	}

	public Agent(PedSimCityNight state, boolean registerSpatial) {
		super(state, registerSpatial);
		this.state = state;
		this.agentNetwork = SharedCognitiveMap.getCommunityPrimalNetwork();
	}

	public void initSensitivity() {
		// Initialize light sensitivity threshold based on vulnerability
		if (isVulnerable()) {
			double min = state.getMinVulnerableLightSensitivity();
			double max = state.getMaxVulnerableLightSensitivity();
			this.lightSensitivityThreshold = min + (state.random.nextDouble() * (max - min));
		} else {
			this.lightSensitivityThreshold = state.getNonVulnerableLightSensitivity();
		}
	}

	/**
	 * This is called every tick by the scheduler. It moves the agent along the
	 * path.
	 *
	 * @param state the simulation state.
	 */
	@Override
	public void step(SimState state) {

		if (isWaiting())
			return;

		if (isWalkingAlone() && destinationNode == null) {
			// simple cognitive map in this sim
			if (!cognitiveMap.formed) {
				getCognitiveMap().buildSimpleActivityBone();
				cognitiveMap.formed = true;
			}
			if (this.state.isDark) {
				planNightTrip();
				nightRoute = true;
			} else
				planTrip();
		} else if (reachedDestination.get()) {
			nightRoute = false;
			handleReachedDestination();
		} else if (isAtDestination() && timeAtDestination <= state.schedule.getSteps())
			goHome();
		else if (isAtDestination())
			;
		else if (nightRoute)
			nightMovement.keepWalking();
		else
			agentMovement.keepWalking();
	}

	private synchronized void planNightTrip() {
		defineOrigin();
		if (isGoingHome()) {
			destinationNode = homeNode;
		} else {
			defineRandomDestination();
		}
		// safety check
		if (destinationNode.getID().equals(originNode.getID())) {
			reachedDestination.set(true);
			return;
		}
		planNightRoute();
		tripStartStep = state.schedule.getSteps();
		nightMovement = new AgentMovement(this);
		nightMovement.initialisePath(getRoute());
	}

	/**
	 * Plans the route for the agent.
	 */
	protected void planNightRoute() {

		Heuristics heuristics = new Heuristics(this);
		heuristics.defineHeuristic(originNode, destinationNode, true);
		RoutePlanner planner = new RoutePlanner(originNode, destinationNode, this);
		setRoute(planner.definePath());
	}

	/**
	 * Plans the route for the agent.
	 */
	@Override
	protected void planTrip() {
		defineOrigin();
		if (isGoingHome()) {
			destinationNode = homeNode;
		} else {
			// If it's day (not dark) and they haven't worked today, go to work!
			if (workNode != null && !hasWorkedToday && !state.isDark) {
				destinationNode = workNode;
			} else {
				defineRandomDestination();
			}
		}
		// safety check
		if (destinationNode.getID().equals(originNode.getID())) {
			reachedDestination.set(true);
			return;
		}
		planRoute();
		tripStartStep = state.schedule.getSteps();
		agentMovement = new AgentMovement(this);
		agentMovement.initialisePath(getRoute());
	}

	/**
	 * Plans the route for the agent.
	 */
	@Override
	protected void planRoute() {
		Heuristics heuristics = new Heuristics(this);
		heuristics.defineHeuristic(originNode, destinationNode, true);
		pedsim.night.routing.RoutePlanner planner = new RoutePlanner(originNode, destinationNode, this);
		setRoute(planner.definePath());
	}

	/**
	 * Randomly selects a destination node within a specified distance range.
	 */
	private void defineRandomDestination() {
		if (abTestTwin != null && abTestTwin.destinationNode != null && abTestTwin.destinationNode != abTestTwin.homeNode) {
			this.destinationNode = abTestTwin.destinationNode;
			return;
		}

		// Initialise limits for distance calculation
		double lowerLimit = distanceNextDestination * 0.90;
		double upperLimit = distanceNextDestination * 1.10;

		// Get the network graph from the cognitive map

		// Loop until a valid destination is found
		while (destinationNode == null) {

			// Get candidate nodes between the current distance range
			List<NodeGraph> destinationCandidates = pedsim.core.agents.Agent.getNodesBetweenDistanceIntervalOptimized(agentNetwork,
					originNode, lowerLimit, upperLimit);

			if (destinationCandidates.isEmpty()) {
				// Skip this iteration and adjust the limits if no candidates found
				lowerLimit *= 0.90;
				upperLimit *= 1.10;
				continue; // Continue with the next loop iteration
			}

			// Select a weighted destination node from the list of candidates
			destinationNode = selectWeightedDestination(destinationCandidates, state.isDark);

			// If it's dark, filter out destination nodes that lie in parks or along rivers
			if (state.isDark && destinationNode.getEdges().stream()
					.anyMatch(SharedCognitiveMap.getEdgesWithinParksOrAlongWater()::contains)) {
				destinationNode = null; // Set destination to null and try again

				// Adjust the distance range for the next iteration
				lowerLimit *= 0.90;
				upperLimit *= 1.10;
			}
		}
	}

	/**
	 * Checks if the agent is vulnerable.
	 *
	 * @return true if the agent is vulnerable, false otherwise.
	 */
	public boolean isVulnerable() {
		return vulnerable == Vulnerable.VULNERABLE;
	}

	/**
	 * Gets the simulation state of the agent.
	 *
	 * @return The PedSimCity simulation state.
	 */
	@Override
	public PedSimCityNight getState() {
		return state;
	}
}
