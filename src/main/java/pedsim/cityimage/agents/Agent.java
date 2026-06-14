package pedsim.cityimage.agents;

import java.util.List;
import java.util.logging.Logger;

import org.javatuples.Pair;

import pedsim.cityimage.engine.PedSimCityImage;
import pedsim.cityimage.parameters.TestPars;
import pedsim.cityimage.utilities.StringEnum.RouteChoice;
import pedsim.core.agents.AgentMovement;
import pedsim.core.routing.RoutePlanner;
import pedsim.core.utilities.LoggerUtil;
import sim.engine.SimState;
import sim.graph.NodeGraph;

public final class Agent extends pedsim.core.agents.Agent {

	private static final long serialVersionUID = 1L;

	private static final Logger LOGGER = LoggerUtil.getLogger();

	private final RouteChoice routeChoice;

	public Agent(PedSimCityImage state, RouteChoice routeChoice, List<Pair<NodeGraph, NodeGraph>> odPairs) {

		super(state, false);

		this.routeChoice = routeChoice;
		this.agentProperties = new AgentProperties();
		((AgentProperties) this.agentProperties).setRouteChoice(routeChoice);

		this.agentMovement = new AgentMovement(this);
		setOD(odPairs);
	}

	@Override
	public void step(SimState state) {
		if (reachedDestination.get() || destinationNode == null) {
			handleReachedDestination();
		} else {
			agentMovement.keepWalking();
		}
	}

	@Override
	protected void handleReachedDestination() {
		boolean completedTrip = reachedDestination.get();
		reachedDestination.set(false);

		if (completedTrip) {
			setTripsDone(getTripsDone() + 1);
		}

		if (getTripsDone() >= OD.size()) {
			removeAgent();
			return;
		}

		selectNodesFromOD();
		updateAgentPosition(originNode.getCoordinate());
		planRoute();
		agentMovement.initialisePath(route);
	}

	@Override
	protected void planRoute() {
		if (TestPars.verboseMode) {
			LOGGER.info(String.format("CityImage agent %d | model=%s | trip=%d | origin=%s | destination=%s", agentID,
					routeChoice, getTripsDone(), originNode, destinationNode));
		}

		RoutePlanner planner = new RoutePlanner(originNode, destinationNode, this);
		setRoute(planner.definePath());
	}

	@Override
	public Enum<?> getAgentScenario() {
		return routeChoice;
	}

	public RouteChoice getRouteChoice() {
		return routeChoice;
	}
}