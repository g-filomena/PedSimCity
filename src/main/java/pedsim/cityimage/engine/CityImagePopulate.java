package pedsim.cityimage.engine;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.TreeMap;
import org.javatuples.Pair;
import pedsim.cityimage.agents.CityImageAgent;
import pedsim.cityimage.parameters.TestPars;
import pedsim.cityimage.utilities.StringEnum.Scenario;
import pedsim.core.engine.NetworkCircuity;
import pedsim.core.engine.PedSimCity;
import pedsim.core.parameters.Pars;
import sim.graph.Building;
import sim.graph.EdgeGraph;
import sim.graph.Graph;
import sim.graph.NodeGraph;
import sim.graph.NodesLookup;

/**
 * Populates the city-image testing module.
 *
 * The conceptual model is:
 *
 * - generate one shared OD matrix; - instantiate one agent per route-choice
 * model; - each agent runs the same OD matrix using its assigned route-choice
 * strategy.
 *
 * This reproduces the historical testing logic while using the current core
 * engine and core agent lifecycle.
 */
public class CityImagePopulate extends pedsim.core.engine.Populate {

  private static final int LANDMARK_TEST_DESTINATIONS = 255;

  /**
   * Route length range for the urban-subdivision test, in <b>walked</b> metres like
   * {@link Pars#minRouteLength}. Converted to a straight-line interval at the point of search, since
   * that is what {@code NodesLookup} takes; passing them raw asked for routes a city's circuity
   * longer than intended.
   */
  private static final double SUBDIVISION_MIN_ROUTE_LENGTH = 1000.0;

  private static final double SUBDIVISION_MAX_ROUTE_LENGTH = 3000.0;

  /** Height of one storey, turning a building's height into a floor count for floor area. */
  private static final double STOREY_HEIGHT_METRES = 3.0;

  /**
   * Nodes the destination at an observed length is drawn from: enough for the floor-area weighting
   * to choose between, few enough that the search interval stays a few metres wide.
   */
  private static final int LENGTH_CANDIDATES = 30;

  private final ArrayList<Pair<NodeGraph, NodeGraph>> odMatrix = new ArrayList<>();

  private PedSimCityImage state;
  private Graph network;

  public void populateTests(PedSimCityImage state) {
    this.state = state;
    seedFrom(state);
    this.network = PedSimCity.network;
    this.odMatrix.clear();

    if (TestPars.testingSpecificOD) {
      prepareManualODmatrix();
    } else {
      generateTestODmatrix();
    }

    generateTestAgents();
  }

  private void prepareManualODmatrix() {
    if (TestPars.originsTmp == null || TestPars.destinationsTmp == null) {
      throw new IllegalStateException("Specific OD mode requires originsTmp and destinationsTmp.");
    }

    if (TestPars.originsTmp.length != TestPars.destinationsTmp.length) {
      throw new IllegalStateException("originsTmp and destinationsTmp must have the same length.");
    }

    for (int i = 0; i < TestPars.originsTmp.length; i++) {
      NodeGraph originNode = PedSimCity.nodesMap.get(TestPars.originsTmp[i]);
      NodeGraph destinationNode = PedSimCity.nodesMap.get(TestPars.destinationsTmp[i]);

      if (originNode == null || destinationNode == null) {
        throw new IllegalStateException(
            "Invalid specific OD pair: "
                + TestPars.originsTmp[i]
                + " -> "
                + TestPars.destinationsTmp[i]);
      }

      odMatrix.add(new Pair<>(originNode, destinationNode));
    }
  }

  private void generateTestODmatrix() {
    int numberTrips = Math.max(1, TestPars.numberTripsPerAgent);

    if (TestPars.testingLandmarks) {
      numberTrips = LANDMARK_TEST_DESTINATIONS;
      generateLandmarkODmatrix(numberTrips);
      return;
    }

    if (TestPars.testingSubdivisions) {
      generateSubdivisionODmatrix(numberTrips);
      return;
    }

    generateGenericODmatrix(numberTrips);
  }

  private void generateLandmarkODmatrix(int numberTrips) {
    NodeGraph originNode = getLandmarkTestingOrigin();

    for (int i = 0; i < numberTrips; i++) {
      NodeGraph destinationNode =
          NodesLookup.randomNodeFromDistancesSet(
              network, PedSimCity.junctions, originNode, TestPars.distances, random);

      if (destinationNode == null || destinationNode.gateway) {
        destinationNode = randomDestination(originNode);
      }

      odMatrix.add(new Pair<>(originNode, destinationNode));
    }
  }

  private NodeGraph getLandmarkTestingOrigin() {
    if (TestPars.originsTmp != null && TestPars.originsTmp.length > 0) {
      NodeGraph originNode = PedSimCity.nodesMap.get(TestPars.originsTmp[0]);

      if (originNode != null) {
        return originNode;
      }
    }

    if (!PedSimCity.startingNodes.isEmpty()) {
      return NodesLookup.randomNodeFromGeometries(network, PedSimCity.startingNodes, random);
    }

    return NodesLookup.randomNode(network, random);
  }

  private void generateSubdivisionODmatrix(int numberTrips) {
    for (int i = 0; i < numberTrips; i++) {
      NodeGraph originNode = randomSubdivisionOrigin();
      NodeGraph destinationNode =
          randomDestination(
              originNode,
              NetworkCircuity.straightLineFor(SUBDIVISION_MIN_ROUTE_LENGTH),
              NetworkCircuity.straightLineFor(SUBDIVISION_MAX_ROUTE_LENGTH));

      odMatrix.add(new Pair<>(originNode, destinationNode));
    }
  }

  private NodeGraph randomSubdivisionOrigin() {
    if (!PedSimCity.startingNodes.isEmpty()) {
      return NodesLookup.randomNodeFromGeometries(network, PedSimCity.startingNodes, random);
    }

    return NodesLookup.randomNode(network, random);
  }

  private void generateGenericODmatrix(int numberTrips) {
    Map<NodeGraph, Double> floorArea =
        TestPars.weightODByFloorArea ? floorAreaByNode() : Collections.emptyMap();

    boolean observedLengths = !TestPars.routeLengthMetres.isEmpty();
    if (observedLengths) {
      logger.info(
          String.format(
              "OD route lengths drawn from %s_%s.csv: %d observed lengths",
              Pars.cityName, TestPars.routeLengths, TestPars.routeLengthMetres.size()));
    }

    for (int i = 0; i < numberTrips; i++) {
      NodeGraph originNode =
          floorArea.isEmpty() ? randomGenericOrigin() : weightedDraw(floorArea.keySet(), floorArea);
      if (observedLengths) {
        odMatrix.add(new Pair<>(originNode, destinationAtObservedLength(originNode, floorArea)));
        continue;
      }
      double minimumDistance = NetworkCircuity.straightLineFor(Pars.minRouteLength);
      double maximumDistance = NetworkCircuity.straightLineFor(Pars.maxRouteLength);
      NodeGraph destinationNode =
          floorArea.isEmpty()
              ? randomDestination(originNode, minimumDistance, maximumDistance)
              : weightedDestination(originNode, minimumDistance, maximumDistance, floorArea);

      odMatrix.add(new Pair<>(originNode, destinationNode));
    }
  }

  /**
   * A destination at a walked length drawn from the observed distribution, measured along the
   * network: the destination is drawn from the nodes whose shortest-path distance from the origin
   * is nearest that length - uniformly, or in proportion to floor area when the matrix is weighted.
   * The interval starts at a metre either side and doubles until it holds {@link
   * #LENGTH_CANDIDATES} nodes, so it stays tight where the network is dense. Measuring along the
   * network rather than through a circuity factor matters for short trips, which are more crooked
   * than the city's average.
   */
  private NodeGraph destinationAtObservedLength(
      NodeGraph originNode, Map<NodeGraph, Double> floorArea) {
    double length = drawObservedLength();
    double reach = 2.0 * length + 200.0;
    List<Map.Entry<NodeGraph, Double>> reachable =
        networkDistancesWithin(originNode, reach).entrySet().stream()
            .filter(entry -> entry.getKey() != originNode && !entry.getKey().gateway)
            .filter(entry -> floorArea.isEmpty() || floorArea.containsKey(entry.getKey()))
            .sorted(Comparator.comparingInt(entry -> entry.getKey().getID()))
            .toList();
    List<NodeGraph> candidates = List.of();
    for (double tolerance = 1.0;
        candidates.size() < LENGTH_CANDIDATES && length - tolerance > -reach;
        tolerance *= 2.0) {
      double low = length - tolerance;
      double high = length + tolerance;
      candidates =
          reachable.stream()
              .filter(entry -> entry.getValue() >= low && entry.getValue() <= high)
              .map(Map.Entry::getKey)
              .toList();
    }
    if (candidates.isEmpty()) {
      return randomDestination(originNode);
    }
    return floorArea.isEmpty()
        ? candidates.get(random.nextInt(candidates.size()))
        : weightedDraw(candidates, floorArea);
  }

  /** Shortest-path distance from {@code origin} to every node within {@code limit} metres. */
  private static Map<NodeGraph, Double> networkDistancesWithin(NodeGraph origin, double limit) {
    Map<NodeGraph, Double> settled = new HashMap<>();
    Map<NodeGraph, Double> best = new HashMap<>();
    PriorityQueue<Map.Entry<NodeGraph, Double>> queue =
        new PriorityQueue<>(
            Comparator.<Map.Entry<NodeGraph, Double>>comparingDouble(Map.Entry::getValue)
                .thenComparingInt(entry -> entry.getKey().getID()));
    queue.add(Map.entry(origin, 0.0));
    best.put(origin, 0.0);
    while (!queue.isEmpty()) {
      Map.Entry<NodeGraph, Double> next = queue.poll();
      NodeGraph node = next.getKey();
      if (settled.containsKey(node)) {
        continue;
      }
      settled.put(node, next.getValue());
      for (EdgeGraph edge : node.getEdges()) {
        NodeGraph other = edge.getOtherNode(node);
        double distance = next.getValue() + edge.getLength();
        if (distance <= limit && distance < best.getOrDefault(other, Double.MAX_VALUE)) {
          best.put(other, distance);
          queue.add(Map.entry(other, distance));
        }
      }
    }
    return settled;
  }

  /** One walked length from {@link TestPars#routeLengthMetres}, in proportion to its weight. */
  private double drawObservedLength() {
    double total = 0.0;
    for (double weight : TestPars.routeLengthWeights) {
      total += weight;
    }
    double target = random.nextDouble() * total;
    for (int i = 0; i < TestPars.routeLengthMetres.size(); i++) {
      target -= TestPars.routeLengthWeights.get(i);
      if (target < 0.0) {
        return TestPars.routeLengthMetres.get(i);
      }
    }
    return TestPars.routeLengthMetres.get(TestPars.routeLengthMetres.size() - 1);
  }

  /**
   * Floor area attached to each node: footprint area times storeys ({@code height} over {@link
   * #STOREY_HEIGHT_METRES}, at least one), summed over the buildings whose nearest junction it is.
   * Nodes without buildings are absent. Ordered by nodeID, so draws do not depend on hash order.
   */
  private Map<NodeGraph, Double> floorAreaByNode() {
    if (PedSimCity.buildingsMap.isEmpty()) {
      throw new IllegalStateException(
          "weightODByFloorArea needs a buildings layer, and none was loaded for " + Pars.cityName);
    }
    Map<NodeGraph, Double> byNode = new TreeMap<>(Comparator.comparingInt(NodeGraph::getID));
    for (Building building : PedSimCity.buildingsMap.values()) {
      if (building.node == null || building.node.gateway) {
        continue;
      }
      Double height = building.geometry.getDoubleAttribute("height");
      double storeys =
          height != null && Double.isFinite(height)
              ? Math.max(1.0, height / STOREY_HEIGHT_METRES)
              : 1.0;
      byNode.merge(building.node, building.geometry.getGeometry().getArea() * storeys, Double::sum);
    }
    logger.info(
        String.format(
            "OD matrix weighted by floor area: %d of %d nodes carry buildings, %.1f km2 in total",
            byNode.size(),
            network.getNodes().size(),
            byNode.values().stream().mapToDouble(Double::doubleValue).sum() / 1e6));
    return byNode;
  }

  /**
   * A destination between the two straight-line distances, drawn in proportion to floor area; the
   * uniform draw when no candidate in the interval carries buildings.
   */
  private NodeGraph weightedDestination(
      NodeGraph originNode,
      double minimumDistance,
      double maximumDistance,
      Map<NodeGraph, Double> floorArea) {
    List<NodeGraph> candidates =
        NodesLookup.getNodesBetweenDistanceInterval(
                network, originNode, minimumDistance, maximumDistance)
            .stream()
            .filter(floorArea::containsKey)
            .sorted(Comparator.comparingInt(NodeGraph::getID))
            .toList();
    if (candidates.isEmpty()) {
      return randomDestination(originNode, minimumDistance, maximumDistance);
    }
    return weightedDraw(candidates, floorArea);
  }

  private NodeGraph weightedDraw(Collection<NodeGraph> nodes, Map<NodeGraph, Double> weights) {
    double total = 0.0;
    for (NodeGraph node : nodes) {
      total += weights.get(node);
    }
    double target = random.nextDouble() * total;
    NodeGraph last = null;
    for (NodeGraph node : nodes) {
      target -= weights.get(node);
      last = node;
      if (target < 0.0) {
        return node;
      }
    }
    return last;
  }

  private NodeGraph randomGenericOrigin() {
    if (!PedSimCity.startingNodes.isEmpty()) {
      return NodesLookup.randomNodeFromGeometries(network, PedSimCity.startingNodes, random);
    }

    return NodesLookup.randomNode(network, random);
  }

  private NodeGraph randomDestination(NodeGraph originNode) {
    return randomDestination(
        originNode,
        NetworkCircuity.straightLineFor(Pars.minRouteLength),
        NetworkCircuity.straightLineFor(Pars.maxRouteLength));
  }

  private NodeGraph randomDestination(
      NodeGraph originNode, double minimumDistance, double maximumDistance) {

    NodeGraph destinationNode =
        NodesLookup.randomNodeBetweenDistanceInterval(
            network, originNode, minimumDistance, maximumDistance, random);

    int attempts = 0;

    while ((destinationNode == null || destinationNode.gateway) && attempts < 100) {
      destinationNode =
          NodesLookup.randomNodeBetweenDistanceInterval(
              network, originNode, minimumDistance, maximumDistance, random);
      attempts++;
    }

    if (destinationNode == null) {
      destinationNode = NodesLookup.randomNode(network, random);
    }

    return destinationNode;
  }

  private void generateTestAgents() {
    Scenario[] scenarios = scenarios();

    for (int agentID = 0; agentID < scenarios.length; agentID++) {
      CityImageAgent agent = new CityImageAgent(state, scenarios[agentID], odMatrix);
      addAgent(agent, agentID);
    }
  }

  private Scenario[] scenarios() {
    if (TestPars.scenarios != null && TestPars.scenarios.length > 0) {
      return TestPars.scenarios;
    }

    return new Scenario[] {Scenario.ROAD_DISTANCE, Scenario.ANGULAR_CHANGE};
  }

  private void addAgent(CityImageAgent agent, int agentID) {
    agent.agentID = agentID;

    state.agents.addGeometry(agent.getLocation());
    state.agentsList.add(agent);
  }

  public List<Pair<NodeGraph, NodeGraph>> getOdMatrix() {
    return odMatrix;
  }
}
