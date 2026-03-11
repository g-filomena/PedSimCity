package pedsim.core.routing;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import pedsim.core.agents.Agent;
import pedsim.core.agents.AgentProperties;
import pedsim.core.parameters.RouteChoicePars;
import pedsim.core.routing.elements.BarrierBasedNavigation;
import pedsim.core.routing.elements.GlobalLandmarkNavigation;
import pedsim.core.routing.elements.RegionBasedNavigation;
import pedsim.core.routing.elements.RegionLandmarkNavigation;
import pedsim.core.routing.pathfinder.AngularChangePathFinder;
import pedsim.core.routing.pathfinder.GlobalLandmarksPathFinder;
import pedsim.core.routing.pathfinder.RoadDistancePathFinder;
import sim.graph.GraphUtils;
import sim.graph.NodeGraph;
import sim.routing.Route;

/**
 * The `RoutePlanner` class is responsible for calculating a route for an agent within a pedestrian
 * simulation. It considers the agent's route choice properties and strategies to determine the
 * optimal path from an origin node to a destination node.
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
   * @param originNode The starting node of the route.
   * @param destinationNode The destination node of the route.
   * @param agent The agent for which the route is being planned.
   */
  public RoutePlanner(NodeGraph originNode, NodeGraph destinationNode, Agent agent) {
    this.originNode = originNode;
    this.destinationNode = destinationNode;
    this.agent = agent;
    this.properties = agent.getProperties();
    this.nodesSequence = new ArrayList<>();
    agent.getHeuristics().defineHeuristic(originNode, destinationNode);
  }

  /**
   * Defines the path for the agent based on route choice properties and strategies.
   *
   * @return A `Route` object representing the calculated route.
   */
  public Route definePath() {

    // === Use only minimization-based navigation
    if (properties.shouldOnlyUseMinimization()) {
      if (properties.minimisingDistance) {
        return new RoadDistancePathFinder().roadDistance(originNode, destinationNode, agent);
      }
      return new AngularChangePathFinder().angularChangeBased(originNode, destinationNode, agent);
    }

    // === Region-based navigation
    if (isRegionBasedNavigation()) {
      RegionBasedNavigation regionsPath =
          new RegionBasedNavigation(originNode, destinationNode, agent);
      nodesSequence = regionsPath.sequenceRegions();
    } else {
      agent.getProperties().regionBasedNavigation = false;
    }

    // Barrier-based navigation (only barriers, no regions)
    if (properties.barrierBasedNavigation && !isRegionBasedNavigation()) {
      BarrierBasedNavigation barriersPath =
          new BarrierBasedNavigation(originNode, destinationNode, agent, false);
      nodesSequence = barriersPath.sequenceBarriers();
    }

    // Local landmarks navigation
    else if (properties.usingLocalLandmarks) {
      GlobalLandmarkNavigation globalNav =
          new GlobalLandmarkNavigation(originNode, destinationNode, agent);
      RegionLandmarkNavigation regionNav =
          new RegionLandmarkNavigation(originNode, destinationNode, agent);

      nodesSequence = isRegionBasedNavigation() && !nodesSequence.isEmpty()
          ? regionNav.regionOnRouteMarks(nodesSequence)
          : globalNav.onRouteMarks();

      // depending on which was used, pull visited locations from the right one
      route.setVisitedLocations(new HashSet<>(
          isRegionBasedNavigation() && !nodesSequence.isEmpty() ? regionNav.getOnRouteMarks()
              : globalNav.getOnRouteMarks()));
    }

    // Only Distant landmarks navigation (not active in empirical-based simulation)
    else if (properties.usingDistantLandmarks && !properties.shouldUseLocalHeuristic()) {
      GlobalLandmarksPathFinder finder = new GlobalLandmarksPathFinder();
      route = !nodesSequence.isEmpty() ? finder.globalLandmarksPathSequence(nodesSequence, agent)
          : finder.globalLandmarksPath(originNode, destinationNode, agent);
      return route;
    }

    // Fallback or finalize the route based on the local heuristic
    route =
        nodesSequence.isEmpty()
            ? (properties.localHeuristicDistance
                ? new RoadDistancePathFinder().roadDistance(originNode, destinationNode, agent)
                : new AngularChangePathFinder().angularChangeBased(originNode, destinationNode,
                    agent))
            : (properties.localHeuristicDistance
                ? new RoadDistancePathFinder().roadDistanceSequence(nodesSequence, agent)
                : new AngularChangePathFinder().angularChangeBasedSequence(nodesSequence, agent));
    return route;
  }

  /**
   * Verifies if region-based navigation should be enabled for route planning based on distance
   * thresholds. If not, it disables region-based navigation in agent properties.
   */
  private boolean isRegionBasedNavigation() {
    return properties.regionBasedNavigation
        && GraphUtils.nodesDistance(originNode,
            destinationNode) >= RouteChoicePars.regionNavActivationThreshold
        && originNode.getRegionID() != destinationNode.getRegionID();
  }
}
