package pedsim.cityimage.agents;

import java.util.LinkedList;
import java.util.List;
import java.util.logging.Logger;

import org.javatuples.Pair;
import org.locationtech.jts.geom.GeometryFactory;

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

	public void setOD(List<Pair<NodeGraph, NodeGraph>> odPairs) {
		this.OD = new LinkedList<>(odPairs);

		if (!this.OD.isEmpty()) {
			this.originNode = this.OD.get(0).getValue0();
			placeAtOriginWithoutLayerUpdate();
		}
	}

	private void placeAtOriginWithoutLayerUpdate() {
		if (originNode == null) {
			return;
		}

		GeometryFactory geometryFactory = new GeometryFactory();
		this.currentLocation.geometry = geometryFactory.createPoint(originNode.getCoordinate());
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

	private void selectNodesFromOD() {
		Pair<NodeGraph, NodeGraph> pair = OD.get(getTripsDone());

		originNode = pair.getValue0();
		destinationNode = pair.getValue1();
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