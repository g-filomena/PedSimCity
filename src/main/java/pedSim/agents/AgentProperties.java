package pedSim.agents;

import pedSim.utilities.StringEnum;
import pedSim.utilities.StringEnum.BarrierType;
import pedSim.utilities.StringEnum.LandmarkType;
import pedSim.utilities.StringEnum.RouteChoice;

/**
 * The `AgentProperties` class represents the properties and preferences of an
 * agent in the simulation. These properties influence the agent's navigation
 * behaviour and route choices.
 */
public class AgentProperties {

	public int agentID;

	// for general routing
	public RouteChoice routeChoice;
	public boolean onlyMinimising = false;
	public boolean minimisingDistance = false;
	public boolean minimisingAngular = false;
	public boolean minimisingTurns = false;

	public boolean localHeuristicDistance = false;
	public boolean localHeuristicAngular = false;

	// landmarkNavigation related parameters
	public boolean usingLocalLandmarks = false;
	public boolean usingDistantLandmarks = false;

	// region- and barrier-based parameters
	public boolean regionBasedNavigation = false;
	public boolean barrierBasedNavigation = false;
	public boolean preferenceNaturalBarriers = false;
	public boolean aversionSeveringBarriers = false;

	public double naturalBarriers = 0.0;
	public double naturalBarriersSD = 0.10;
	public double severingBarriers = 0.0;
	public double severingBarriersSD = 0.10;

	// the ones possibly used as sub-goals []
	public BarrierType barrierType;
	public LandmarkType landmarkType;

	public Agent agent;

	/**
	 * Constructs an AgentProperties instance associated with the given agent.
	 *
	 * @param agent The agent for which this AgentProperties instance is created.
	 */
	public AgentProperties(Agent agent) {
		this.agent = agent;
	}

	/**
	 * Sets the route choice for the agent and updates related properties
	 * accordingly.
	 * 
	 * @param routeChoice The selected route choice for the agent.
	 */
	public void setRouteChoice(RouteChoice routeChoice) {
		this.routeChoice = routeChoice;
		onlyMinimising = false;
		minimisingDistance = routeChoice == RouteChoice.ROAD_DISTANCE;
		minimisingAngular = routeChoice == RouteChoice.ANGULAR_CHANGE;

		onlyMinimising = minimisingDistance || minimisingAngular;
		if (onlyMinimising)
			return;

		// localHeuristics
		localHeuristicDistance = containsDistance(routeChoice);
		localHeuristicAngular = containsAngular(routeChoice);

		// landmarks
		regionBasedNavigation = containsRegion(routeChoice);
		barrierBasedNavigation = containsBarrier(routeChoice);
		usingLocalLandmarks = containsLocal(routeChoice);
		usingDistantLandmarks = containsGlobal(routeChoice);

		if (routeChoice.toString().contains("LANDMARKS")) {
			usingDistantLandmarks = true;
			if (!routeChoice.toString().contains("DISTANT"))
				usingLocalLandmarks = true;
		}

		if (usingLocalLandmarks)
			landmarkType = StringEnum.LandmarkType.LOCAL;

		if (barrierBasedNavigation) {
			preferenceNaturalBarriers = true;
			aversionSeveringBarriers = true;
			naturalBarriers = 0.70;
			severingBarriers = 1.30;
			barrierType = BarrierType.ALL;
		}
	}

	/**
	 * Checks if the given route choice contains a "DISTANCE" component.
	 *
	 * @param routeChoice The route choice to check.
	 * @return True if "DISTANCE" is found in the route choice, false otherwise.
	 */
	public static boolean containsDistance(RouteChoice routeChoice) {
		return routeChoice.toString().contains("DISTANCE");
	}

	/**
	 * Checks if the given route choice contains an "ANGLE" component.
	 *
	 * @param routeChoice The route choice to check.
	 * @return True if "ANGLE" is found in the route choice, false otherwise.
	 */
	public boolean containsAngular(RouteChoice routeChoice) {
		return routeChoice.toString().contains("ANGLE");
	}

	/**
	 * Checks if the given route choice contains a "REGION" component.
	 *
	 * @param routeChoice The route choice to check.
	 * @return True if "REGION" is found in the route choice, false otherwise.
	 */
	public boolean containsRegion(RouteChoice routeChoice) {
		return routeChoice.toString().contains("REGION");
	}

	/**
	 * Checks if the given route choice contains a "BARRIER" component.
	 *
	 * @param routeChoice The route choice to check.
	 * @return True if "BARRIER" is found in the route choice, false otherwise.
	 */
	public boolean containsBarrier(RouteChoice routeChoice) {
		return routeChoice.toString().contains("BARRIER");
	}

	/**
	 * Checks if the given route choice contains a "LOCAL" component.
	 *
	 * @param routeChoice The route choice to check.
	 * @return True if "LOCAL" is found in the route choice, false otherwise.
	 */
	public boolean containsLocal(RouteChoice routeChoice) {
		return routeChoice.toString().contains("LOCAL");
	}

	/**
	 * Checks if the given route choice contains a "DISTANT" component.
	 *
	 * @param routeChoice The route choice to check.
	 * @return True if "DISTANT" is found in the route choice, false otherwise.
	 */
	public boolean containsGlobal(RouteChoice routeChoice) {
		return routeChoice.toString().contains("DISTANT");
	}
}
