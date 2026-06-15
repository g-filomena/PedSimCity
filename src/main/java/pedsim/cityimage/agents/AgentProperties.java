package pedsim.cityimage.agents;

import pedsim.cityimage.utilities.StringEnum.RouteChoice;
import pedsim.core.utilities.StringEnum.AgentBarrierType;
import pedsim.core.utilities.StringEnum.LandmarkType;
import pedsim.core.utilities.StringEnum.LocalHeuristicMode;
import pedsim.core.utilities.StringEnum.MinimisationMode;

/**
 * The `AgentProperties` class represents the properties and preferences of an agent in the
 * simulation. These properties influence the agent's navigation behaviour and route choices.
 */
public class AgentProperties extends pedsim.core.agents.AgentProperties {

  // for general routing
  public RouteChoice routeChoice;

  public AgentProperties() {
    super();
  }

  /**
   * Sets the route choice for the agent and updates related properties accordingly.
   *
   * @param routeChoice The selected route choice for the agent.
   */
  public void setRouteChoice(RouteChoice routeChoice) {
    reset();
    this.routeChoice = routeChoice;

    if (routeChoice == null) {
      return;
    }

    // Pure minimisation modes
    if (routeChoice == RouteChoice.ROAD_DISTANCE) {
      setMinimisationMode(MinimisationMode.DISTANCE);
      return;
    }

    if (routeChoice == RouteChoice.ANGULAR_CHANGE) {
      setMinimisationMode(MinimisationMode.ANGULAR);
      return;
    }

    // Local heuristic modes
    if (containsDistance()) {
      setLocalHeuristicMode(LocalHeuristicMode.DISTANCE);
    } else if (containsAngular()) {
      setLocalHeuristicMode(LocalHeuristicMode.ANGULAR);
    }

    // Landmark-based navigation
    activateLandmarks();

    // Region / barrier navigation
    setRegionBasedNavigation(containsRegion());
    setBarrierBasedNavigation(containsBarrier());

    if (isBarrierBasedNavigation()) {
      setPreferenceNaturalBarriers(true);
      setAversionSeveringBarriers(true);
      setNaturalBarriersMean(0.70);
      setSeveringBarriersMean(1.30);
      setBarrierType(AgentBarrierType.ALL);
    }
  }

  private void activateLandmarks() {
    boolean usingLocal = containsLocal();
    boolean usingDistant = containsGlobal();

    if (containsAllLandmarks()) {
      usingDistant = true;

      // "LANDMARKS" without explicit "DISTANT" means both local and distant
      if (!routeChoice.toString().contains("DISTANT")) {
        usingLocal = true;
      }
    }

    setUsingLocalLandmarks(usingLocal);
    setUsingDistantLandmarks(usingDistant);
    if (usingLocal) setLandmarkType(LandmarkType.LOCAL);
  }

  /**
   * Checks if the given route choice contains a "DISTANCE" component.
   *
   * @return True if "DISTANCE" is found in the route choice descriptor, false otherwise.
   */
  private boolean containsDistance() {
    return routeChoice.toString().contains("DISTANCE");
  }

  /**
   * Checks if the given route choice contains an "ANGULAR" component.
   *
   * @return True if "ANGULAR" is found in the route choice descriptor, false otherwise.
   */
  private boolean containsAngular() {
    return routeChoice.toString().contains("ANGULAR");
  }

  /**
   * Checks if the given route choice contains a "REGION" component.
   *
   * @return True if "REGION" is found in the route choice descriptor, false otherwise.
   */
  private boolean containsRegion() {
    return routeChoice.toString().contains("REGION");
  }

  /**
   * Checks if the given route choice contains a "BARRIER" component.
   *
   * @return True if "BARRIER" is found in the route choice descriptor, false otherwise.
   */
  public boolean containsBarrier() {
    return routeChoice.toString().contains("BARRIER");
  }

  /**
   * Checks if the given route choice contains a "LOCAL" component.
   *
   * @return True if "LOCAL" is found in the route choice descriptor, false otherwise.
   */
  private boolean containsLocal() {
    return routeChoice.toString().contains("LOCAL");
  }

  /**
   * Checks if the given route choice contains a "DISTANT" component.
   *
   * @return True if "DISTANT" is found in the route choice descriptor, false otherwise.
   */
  private boolean containsGlobal() {
    return routeChoice.toString().contains("DISTANT");
  }

  private boolean containsAllLandmarks() {
    return routeChoice.toString().contains("LANDMARKS");
  }
}
