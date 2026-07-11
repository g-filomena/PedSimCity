package pedsim.cityimage.agents;

import pedsim.cityimage.utilities.StringEnum.RouteChoice;
import pedsim.core.utilities.StringEnum.AgentBarrierType;
import pedsim.core.utilities.StringEnum.LandmarkType;
import pedsim.core.utilities.StringEnum.LocalHeuristicMode;
import pedsim.core.utilities.StringEnum.MinimisationMode;

/**
 * Route-choice properties for cityImage test agents: maps a {@link RouteChoice} scenario onto the
 * core {@link pedsim.core.agents.AgentProperties} configuration (minimisation mode, local
 * heuristic, landmark/region/barrier elements).
 */
public class CityImageAgentProperties extends pedsim.core.agents.AgentProperties {

  // for general routing
  public RouteChoice routeChoice;

  public CityImageAgentProperties() {
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

  // ----------------------------------------------------------------
  // RouteChoice token checks: the RouteChoice enum names are composed of tokens
  // (DISTANCE, ANGULAR, REGION, BARRIER, LOCAL, DISTANT, LANDMARKS), so each
  // component is detected by its token in the enum name.
  // ----------------------------------------------------------------

  private boolean containsDistance() {
    return routeChoice.toString().contains("DISTANCE");
  }

  private boolean containsAngular() {
    return routeChoice.toString().contains("ANGULAR");
  }

  private boolean containsRegion() {
    return routeChoice.toString().contains("REGION");
  }

  public boolean containsBarrier() {
    return routeChoice.toString().contains("BARRIER");
  }

  private boolean containsLocal() {
    return routeChoice.toString().contains("LOCAL");
  }

  private boolean containsGlobal() {
    return routeChoice.toString().contains("DISTANT");
  }

  private boolean containsAllLandmarks() {
    return routeChoice.toString().contains("LANDMARKS");
  }
}
