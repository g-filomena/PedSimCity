package pedsim.core.routing;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import pedsim.core.agents.Agent;
import pedsim.core.agents.AgentProperties;
import pedsim.core.parameters.RouteChoicePars;
import pedsim.core.routing.elements.BarrierBasedNavigation;
import pedsim.core.routing.elements.GlobalLandmarkNavigation;
import pedsim.core.routing.elements.LandmarkNavigation;
import pedsim.core.routing.elements.RegionBasedNavigation;
import pedsim.core.routing.elements.RegionLandmarkNavigation;
import pedsim.core.routing.pathfinder.AngularChangePathFinder;
import pedsim.core.routing.pathfinder.GlobalLandmarksPathFinder;
import pedsim.core.routing.pathfinder.RoadDistancePathFinder;
import pedsim.core.utilities.LoggerUtil;
import sim.graph.GraphUtils;
import sim.graph.NodeGraph;
import sim.routing.Route;

/**
 * The `RoutePlanner` class is responsible for calculating a route for an agent
 * within a pedestrian simulation. It considers the agent's route choice
 * properties and strategies to determine the optimal path from an origin node
 * to a destination node.
 */
public class RoutePlanner {

  protected NodeGraph originNode;
  protected NodeGraph destinationNode;
  protected AgentProperties properties;
  protected List<NodeGraph> nodesSequence;
  protected Agent agent;
  protected Route route = new Route();

  public RoutePlanner() {}

  /**
   * Constructs a `RoutePlanner` instance for calculating a route.
   *
   * @param originNode      The starting node of the route.
   * @param destinationNode The destination node of the route.
   * @param agent           The agent for which the route is being planned.
   */
  public RoutePlanner(NodeGraph originNode, NodeGraph destinationNode, Agent agent) {
    this.originNode = originNode;
    this.destinationNode = destinationNode;
    this.agent = agent;
    this.properties = agent.getProperties();
    this.nodesSequence = new ArrayList<>();
    warnIfUnconfigured();
  }

  /**
   * Warns once when a planner is built on properties nothing has configured: the route then falls
   * back to road distance, which is not the agent's route choice. {@code Agent.planRoute()} calls
   * {@code initialiseHeuristics()} first and the module property classes set a mode themselves.
   */
  private void warnIfUnconfigured() {
    if (properties.isConfigured() || unconfiguredWarningIssued) {
      return;
    }
    unconfiguredWarningIssued = true;
    LoggerUtil.getLogger()
        .warning(
            "RoutePlanner built on unconfigured AgentProperties (agent "
                + agent.agentID
                + "): no minimisation mode, local heuristic or route-choice element is set, so the"
                + " route falls back to road distance. Plan through Agent.planRoute(), which calls"
                + " initialiseHeuristics() first. Reported once per run.");
  }

  /** One warning is the point; a per-trip one would bury the run's own output. */
  private static volatile boolean unconfiguredWarningIssued = false;

  /**
   * Defines the path for the agent based on route choice properties and
   * strategies.
   *
   * @return A `Route` object representing the calculated route.
   */
  public Route definePath() {

    // === Use only minimisation-based navigation
    if (properties.shouldOnlyUseMinimization()) {
      if (properties.isMinimisingDistance() || !angularAvailable()) {
        return new RoadDistancePathFinder().roadDistance(originNode, destinationNode, agent);
      }
      return new AngularChangePathFinder().angularChangeBased(originNode, destinationNode, agent);
    }

    // === Region-based navigation
    boolean regionBased = isRegionBasedNavigation();
    if (regionBased) {
      RegionBasedNavigation regionsPath =
          new RegionBasedNavigation(originNode, destinationNode, agent);
      nodesSequence = regionsPath.computeSequence();
    } else {
      agent.getProperties().setRegionBasedNavigation(false);
    }

    // Barrier-based navigation (only barriers, no regions)
    if (properties.isBarrierBasedNavigation() && !regionBased) {
      BarrierBasedNavigation barriersPath =
          new BarrierBasedNavigation(originNode, destinationNode, agent, false);
      nodesSequence = barriersPath.computeSequence();
    }

    // Local landmarks navigation
    else if (properties.isUsingLocalLandmarks()) {
      LandmarkNavigation landmarkNav =
          regionBased && !nodesSequence.isEmpty()
              ? new RegionLandmarkNavigation(originNode, destinationNode, agent, nodesSequence)
              : new GlobalLandmarkNavigation(originNode, destinationNode, agent);
      nodesSequence = landmarkNav.computeSequence();
      route.setVisitedLocations(new HashSet<>(landmarkNav.getOnRouteMarks()));
    }

    // Only Distant landmarks navigation (not active in empirical-based simulation)
    else if (properties.isUsingDistantLandmarks() && !properties.shouldUseLocalHeuristic()) {
      GlobalLandmarksPathFinder finder = new GlobalLandmarksPathFinder();
      route =
          !nodesSequence.isEmpty()
              ? finder.globalLandmarksPathSequence(nodesSequence, agent)
              : finder.globalLandmarksPath(originNode, destinationNode, agent);
      return route;
    }

    // The local heuristic routes each leg between the sub-goals chosen above. The test is
    // positive - is it angular - so LocalHeuristicMode.NONE, which means no heuristic was chosen,
    // routes by distance: angular is a stated preference, shortest path is what is left without
    // one.
    boolean angular = properties.isLocalHeuristicAngular() && angularAvailable();
    route =
        nodesSequence.isEmpty()
            ? (angular
                ? new AngularChangePathFinder()
                    .angularChangeBased(originNode, destinationNode, agent)
                : new RoadDistancePathFinder().roadDistance(originNode, destinationNode, agent))
            : (angular
                ? new AngularChangePathFinder().angularChangeBasedSequence(nodesSequence, agent)
                : new RoadDistancePathFinder().roadDistanceSequence(nodesSequence, agent));
    return route;
  }

  /**
   * Whether angular-change routing can run at all: it searches the dual graph, so a city that
   * shipped no dual layers cannot serve it.
   *
   * <p>{@link pedsim.core.agents.Heuristics} constrains both angular modes to distance when the
   * dual graph is absent, which covers every agent routing through {@code Agent.planRoute()}. The
   * check is repeated here because {@code RoutePlanner} is also constructed directly, and without it
   * the failure is a {@code NullPointerException} on a null dual node inside
   * {@code NodeGraph.getDualNodes}, which names nothing about the missing layer.
   */
  private boolean angularAvailable() {
    return pedsim.core.engine.PedSimCity.dualGraphLoaded;
  }

  /**
   * Verifies if region-based navigation should be enabled for route planning
   * based on distance thresholds. If not, it disables region-based navigation in
   * agent properties.
   */
  private boolean isRegionBasedNavigation() {
    return properties.isRegionBasedNavigation()
        && GraphUtils.nodesDistance(originNode, destinationNode)
            >= RouteChoicePars.regionNavActivationThreshold
        && originNode.getRegionID() != destinationNode.getRegionID();
  }
}
