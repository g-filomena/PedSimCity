package pedsim.cityimage.agents;

import java.util.List;
import java.util.logging.Logger;

import org.javatuples.Pair;

import pedsim.cityimage.engine.PedSimCityImage;
import pedsim.cityimage.parameters.TestPars;
import pedsim.cityimage.utilities.StringEnum.RouteChoice;
import pedsim.core.agents.AgentMovement;
import pedsim.core.agents.OdAgent;
import pedsim.core.routing.RoutePlanner;
import pedsim.core.utilities.LoggerUtil;
import sim.graph.NodeGraph;

public final class CityImageAgent extends OdAgent {

	private static final long serialVersionUID = 1L;

	private static final Logger LOGGER = LoggerUtil.getLogger();

	private final RouteChoice routeChoice;

	public CityImageAgent(PedSimCityImage state, RouteChoice routeChoice, List<Pair<NodeGraph, NodeGraph>> odPairs) {
		super(state);
		this.routeChoice = routeChoice;
		this.agentProperties = new AgentProperties();
		((AgentProperties) this.agentProperties).setRouteChoice(routeChoice);
		this.agentMovement = new AgentMovement(this);
		setOD(odPairs);
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
