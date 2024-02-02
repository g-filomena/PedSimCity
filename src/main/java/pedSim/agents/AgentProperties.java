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

	// for general routing
	public RouteChoice routeChoice;
	public boolean onlyMinimising = false;
	public boolean minimisingDistance = false;
	public boolean minimisingAngular = false;

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

	// the ones possibly used as sub-goals
	public BarrierType barrierType;
	public LandmarkType landmarkType;

	/**
	 * Constructs an AgentProperties instance associated with the given agent.
	 *
	 */
	public AgentProperties() {

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
		localHeuristicDistance = containsDistance();
		localHeuristicAngular = containsAngular();

		// landmarks
		activateLandmarks();
		regionBasedNavigation = containsRegion();
		barrierBasedNavigation = containsBarrier();

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
	 * @return True if "DISTANCE" is found in the route choice descriptor, false
	 *         otherwise.
	 */
	private boolean containsDistance() {
		return routeChoice.toString().contains("DISTANCE");
	}

	/**
	 * Checks if the given route choice contains an "ANGLE" component.
	 *
	 * @return True if "ANGLE" is found in the route choice descriptor, false
	 *         otherwise.
	 */
	private boolean containsAngular() {
		return routeChoice.toString().contains("ANGLE");
	}

	/**
	 * Checks if the given route choice contains a "REGION" component.
	 *
	 * @return True if "REGION" is found in the route choice descriptor, false
	 *         otherwise.
	 */
	private boolean containsRegion() {
		return routeChoice.toString().contains("REGION");
	}

	/**
	 * Checks if the given route choice contains a "BARRIER" component.
	 *
	 * @return True if "BARRIER" is found in the route choice descriptor, false
	 *         otherwise.
	 */
	public boolean containsBarrier() {
		return routeChoice.toString().contains("BARRIER");
	}

	/**
	 * Activates the use of local and/or global landmarks based on the specified
	 * conditions. Sets the values for usingLocalLandmarks and usingDistantLandmarks
	 * based on the presence of local and global landmarks.
	 */
	private void activateLandmarks() {

		usingLocalLandmarks = containsLocal();
		usingDistantLandmarks = containsGlobal();

		if (containsAllLandmarks()) {
			usingDistantLandmarks = true;
			if (!routeChoice.toString().contains("DISTANT"))
				usingLocalLandmarks = true;
		}

		if (usingLocalLandmarks)
			landmarkType = StringEnum.LandmarkType.LOCAL;
	}

	/**
	 * Checks if the given route choice contains a "LOCAL" component.
	 *
	 * @return True if "LOCAL" is found in the route choice descriptor, false
	 *         otherwise.
	 */
	private boolean containsLocal() {
		return routeChoice.toString().contains("LOCAL");
	}

	/**
	 * Checks if the given route choice contains a "DISTANT" component.
	 *
	 * @return True if "DISTANT" is found in the route choice descriptor, false
	 *         otherwise.
	 */
	private boolean containsGlobal() {
		return routeChoice.toString().contains("DISTANT");
	}

	/**
	 * Checks if the given route choice contains a "DISTANT" component.
	 *
	 * @return True if "DISTANT" is found in the route choice descriptor, false
	 *         otherwise.
	 */
	private boolean containsAllLandmarks() {
		return routeChoice.toString().contains("LANDMARKS");
	}
}
