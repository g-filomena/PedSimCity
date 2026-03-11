package pedsim.core.routing.elements;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;
import org.javatuples.Pair;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import pedsim.core.agents.Agent;
import pedsim.core.cognition.cityimage.Barrier;
import pedsim.core.cognition.cityimage.Region;
import pedsim.core.cognition.metrics.BarrierIntegration;
import pedsim.core.engine.PedSimCity;
import pedsim.core.utilities.StringEnum.BarrierType;
import sim.graph.EdgeGraph;
import sim.graph.GraphUtils;
import sim.graph.NodeGraph;
import sim.util.geo.GeometryUtilities;
import sim.util.geo.MasonGeometry;
import sim.util.geo.Utilities;

/**
 * Series of functions for computing a sequence of barrier sub-goals in the space between an origin
 * and a destination.
 */
public class BarrierBasedNavigation {

  List<NodeGraph> sequence = new ArrayList<>();
  private NodeGraph originNode;
  private NodeGraph currentLocation;
  private NodeGraph destinationNode;
  private Agent agent;
  boolean regionBasedNavigation = false;
  // it stores the barriers that the agent has already been exposed to
  Set<Integer> visitedBarriers;
  Map<Integer, Double> validBarriers;

  /**
   * Initialises a new instance of the BarrierBasedNavigation class.
   *
   * @param originNode the origin node;
   * @param destinationNode the destination node;
   * @param agent the agent for navigation.
   * @param regionBasedNavigation
   */
  public BarrierBasedNavigation(NodeGraph originNode, NodeGraph destinationNode, Agent agent,
      boolean regionBasedNavigation) {
    this.originNode = originNode;
    this.destinationNode = destinationNode;
    this.agent = agent;
    this.regionBasedNavigation = regionBasedNavigation;
    visitedBarriers = new HashSet<>();
    validBarriers = new HashMap<>();
  }

  /**
   * Computes a sequence of nodes, including origin, destination, and barrier sub-goals, and
   * identifies traversed regions.
   *
   * @return an ArrayList of NodeGraph representing the sequence of sub-goals.
   * @throws Exception
   */
  public List<NodeGraph> sequenceBarriers() {

    currentLocation = originNode;
    // sub-goals
    sequence.add(originNode);

    while (true) {
      // in the agent cognitive Map
      validBarriers.clear();
      validBarriers = findValidBarriers(currentLocation, null);
      if (validBarriers.isEmpty()) {
        break;
      }

      Pair<EdgeGraph, Integer> barrierGoal = identifyBarrierSubGoal(validBarriers, null);
      if (barrierGoal == null) {
        break;
      }

      EdgeGraph edgeGoal = barrierGoal.getValue0();
      int barrier = barrierGoal.getValue1();
      NodeGraph subGoal =
          GraphUtils.nodesDistance(currentLocation, edgeGoal.getFromNode()) < GraphUtils
              .nodesDistance(currentLocation, edgeGoal.getToNode()) ? edgeGoal.getFromNode()
                  : edgeGoal.getToNode();

      sequence.add(subGoal);
      currentLocation = subGoal;
      visitedBarriers.add(barrier); // to avoid visiting them again
      if (subGoal.equals(destinationNode)) {
        break;
      }
    }

    if (!sequence.contains(destinationNode)) {
      sequence.add(destinationNode);
    }

    return sequence;
  }

  /**
   * Finds the valid barriers along the line of movement towards the destination within the given
   * region.
   *
   * @param currentLocation The current node location (gateway when region-based navigation).
   * @param region the region of the currentLocation, only for region-based navigation.
   * @return
   */
  protected Map<Integer, Double> findValidBarriers(NodeGraph currentLocation, Region region) {

    this.currentLocation = currentLocation;
    BarrierIntegration barrierIntegration = new BarrierIntegration();

    // check if there are good barriers in line of movement towards the destination,
    // known by the agent
    Map<Geometry, Set<Integer>> viewFieldIntersectingBarriers =
        barrierIntegration.intersectingBarriers(currentLocation, destinationNode, agent);

    // no barriers
    if (viewFieldIntersectingBarriers.isEmpty()) {
      return validBarriers;
    }

    // length is always max 1
    Geometry viewField = new ArrayList<>(viewFieldIntersectingBarriers.keySet()).get(0);
    Set<Integer> intersectingBarriers = viewFieldIntersectingBarriers.get(viewField);

    // When region-based, only keep barriers within the region boundaries known to
    // the agent
    filterBarriersByRegion(region, intersectingBarriers);

    // assuming that these have been "visited" already
    identifyAdjacentBarriers();
    if (intersectingBarriers.isEmpty()) {
      return validBarriers;
    }

    Coordinate currentCoordinate = currentLocation.getCoordinate();
    Coordinate destinationCoordinate = destinationNode.getCoordinate();

    // Check each candidate barrier and add it to validBarriers if it meets criteria
    assessCandidateBarriers(intersectingBarriers, viewField, currentCoordinate,
        destinationCoordinate);
    return validBarriers;
  }

  /**
   * Filter barriers by region if region-based navigation is active and the region is known.
   *
   * @param region The region object for filtering barriers.
   * @param intersectingBarriers The set of intersecting barriers to filter.
   */
  private void filterBarriersByRegion(Region region, Set<Integer> intersectingBarriers) {
    if (regionBasedNavigation && agent.getCognitiveMap().isRegionKnown(region.regionID)) {
      intersectingBarriers.retainAll(region.barriers);
    }
  }

  /**
   * Identifies the barriers surrounding the current location by examining the incoming edges of the
   * node. Adds the identified barriers to the list of adjacent barriers.
   */
  private void identifyAdjacentBarriers() {
    // identify barriers around this currentLocation
    List<EdgeGraph> incomingEdges = currentLocation.getEdges();
    for (EdgeGraph edge : incomingEdges) {
      ArrayList<Integer> edgeBarriers = edge.attributes.get("barriers").getArray();
      if (!edgeBarriers.isEmpty()) {
        visitedBarriers.addAll(edgeBarriers);
      }
    }
  }

  /**
   * Check each candidate barrier and add it to validBarriers if it meets the distance criteria.
   *
   * @param intersectingBarriers The set of intersecting barriers to check.
   * @param validBarriers The map to store valid barriers and their distances.
   * @param currentCoordinate The current location coordinate.
   * @param destinationCoordinate The destination node coordinate.
   */
  private void assessCandidateBarriers(Set<Integer> intersectingBarriers, Geometry viewField,
      Coordinate currentCoordinate, Coordinate destinationCoordinate) {

    //// System.out.println("currentCorrdinate " + currentCoordinate + "viewField -- " + viewField);
    // intersectingBarriers.parallelStream().filter(barrierID ->
    //// !visitedBarriers.contains(barrierID))
    // .forEach(barrierID -> {
    // MasonGeometry barrierGeometry = PedSimCity.barriersMap.get(barrierID).masonGeometry;
    //
    //// System.out.println("barrierID " + barrierID + " barrierGeo " + barrierGeometry);
    // Coordinate[] intersections =
    //// viewField.intersection(barrierGeometry.geometry).getCoordinates();
    //
    // double minDistance = Arrays.stream(intersections).parallel()
    // .mapToDouble(intersection -> GraphUtils.euclideanDistance(currentCoordinate, intersection))
    // .min().orElse(Double.MAX_VALUE);
    //
    //// System.out.println("barrierID " + barrierID + " minDistance min " + minDistance);
    //
    // if (minDistance <= GraphUtils.euclideanDistance(currentCoordinate, destinationCoordinate)) {
    // validBarriers.put(barrierID, minDistance);
    // }
    // });

    // for each candidate barrier, check whether it complies with the criteria
    for (int barrierID : intersectingBarriers) {
      if (visitedBarriers.contains(barrierID)) {
        continue;
      }
      MasonGeometry barrierGeometry = PedSimCity.barriersMap.get(barrierID).masonGeometry;
      Coordinate[] intersections =
          viewField.intersection(barrierGeometry.geometry).getCoordinates();

      double minDistance = Arrays.stream(intersections).parallel()
          .mapToDouble(
              intersection -> GeometryUtilities.euclideanDistance(currentCoordinate, intersection))
          .min().orElse(Double.MAX_VALUE);

      // barriers that are more distant than the destinationNode are disregarded
      if (minDistance > GeometryUtilities.euclideanDistance(currentCoordinate,
          destinationCoordinate)) {
        continue;
      }

      validBarriers.put(barrierID, minDistance);
    }

  }

  /**
   * Identifies an edge-subGoals associated with the most attractive barrier, complying with certain
   * criteria, and finds the closest edge to them.
   *
   * @param validBarriers a set of valid, intersecting barriers towards the destination.
   * @param region the region of the currentLocation, only for region-based navigation.
   * @return a Pair of EdgeGraph and Integer representing the closest edge to the barrier and the
   *         barrierID.
   */
  protected Pair<EdgeGraph, Integer> identifyBarrierSubGoal(Map<Integer, Double> validBarriers,
      Region region) {

    List<EdgeGraph> regionEdges = new ArrayList<>();
    if (regionBasedNavigation && agent.getCognitiveMap().isRegionKnown(region.regionID)) {
      regionEdges = region.edges;
    }

    // When more than one barrier is identified, the farthest water body barrier is
    // chosen; if no water bodies are identified, the agent picks the farthest park
    // barrier, if any, or, otherwise, the farthest viable severing barrier.

    // sorted by distance (further away first, as it leads your further away)
    // System.out.println("-- valid " + validBarriers);
    Map<Integer, Double> validSorted = Utilities.sortByValue(validBarriers, true);

    List<Integer> barrierIDs = new ArrayList<>();
    List<EdgeGraph> possibleEdgeGoals = new ArrayList<>();

    int waterCounter = 0;
    int parkCounter = 0;

    for (int barrierID : validSorted.keySet()) {
      Barrier barrier = PedSimCity.barriersMap.get(barrierID);
      BarrierType type = barrier.type;
      List<EdgeGraph> edgesAlong = new ArrayList<>(barrier.edgesAlong);

      // for region-based, only consider edges in the region
      if (regionBasedNavigation && agent.getCognitiveMap().isRegionKnown(region.regionID)) {
        edgesAlong.retainAll(regionEdges);
        if (edgesAlong.isEmpty()) {
          continue;
        }
      }

      // only known edges
      edgesAlong = edgesAlong.stream()
          .filter(edge -> agent.getCognitiveMap().getEdgesInKnownNetwork().contains(edge))
          .collect(Collectors.toList());

      // only edges with certain requirements
      Map<EdgeGraph, Double> thisBarrierEdgeGoals = keepValidSubGoals(edgesAlong);
      if (thisBarrierEdgeGoals.isEmpty()) {
        continue;
      }

      // this is considered a good Edge, sort by distance and takes the closest to the
      // current location.
      Map<EdgeGraph, Double> thisBarrierSubGoalSorted =
          Utilities.sortByValue(thisBarrierEdgeGoals, false);
      EdgeGraph possibleEdgeGoal = thisBarrierSubGoalSorted.keySet().iterator().next();

      switch (type) {
        case WATER -> {
          barrierIDs.add(waterCounter, barrierID);
          possibleEdgeGoals.add(waterCounter, possibleEdgeGoal);
          waterCounter++;
          parkCounter++;
        }
        case PARK -> {
          barrierIDs.add(parkCounter);
          possibleEdgeGoals.add(parkCounter, possibleEdgeGoal);
          parkCounter++;
        }
        default -> {
          barrierIDs.add(barrierID);
          possibleEdgeGoals.add(possibleEdgeGoal);
        }
      }
    }

    if (possibleEdgeGoals.isEmpty() || possibleEdgeGoals.get(0) == null) {
      return null;
    }

    return new Pair<>(possibleEdgeGoals.get(0), barrierIDs.get(0));
  }

  /**
   * Checks requirements for selecting barrier sub-goals from a list of edges along a barrier.
   *
   * @param edgesAlong a list of edges along a barrier;
   * @return a HashMap of EdgeGraph and Double representing eligible barrier sub-goals and their
   *         distances.
   */
  private Map<EdgeGraph, Double> keepValidSubGoals(List<EdgeGraph> edgesAlong) {
    return edgesAlong.stream().filter(edge -> {
      double distanceToEdge = GeometryUtilities.euclideanDistance(currentLocation.getCoordinate(),
          edge.getCoordsCentroid());
      double distanceToDestination = GraphUtils.nodesDistance(currentLocation, destinationNode);
      return distanceToEdge <= distanceToDestination;
    }).filter(edge -> sequence.stream().noneMatch(node -> node.getEdges().contains(edge)))
        .filter(edge -> !currentLocation.getEdges().contains(edge))
        .collect(Collectors.toMap(edge -> edge, edge -> GeometryUtilities
            .euclideanDistance(currentLocation.getCoordinate(), edge.getCoordsCentroid())));
  }
}
