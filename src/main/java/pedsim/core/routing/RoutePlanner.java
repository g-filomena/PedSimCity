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
import pedsim.core.routing.routers.AngularChangePathFinder;
import pedsim.core.routing.routers.GlobalLandmarksPathFinder;
import pedsim.core.routing.routers.RoadDistancePathFinder;
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
  }

  /**
   * Defines the path for the agent based on route choice properties and
   * strategies.
   *
   * @return A `Route` object representing the calculated route.
   */
  public Route definePath() {

    // === Use only minimisation-based navigation
    if (properties.shouldOnlyUseMinimization()) {
      if (properties.isMinimisingDistance()) {
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
    boolean angular = properties.isLocalHeuristicAngular();
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
