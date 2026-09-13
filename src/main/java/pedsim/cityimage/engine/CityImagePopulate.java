package pedsim.cityimage.engine;

import java.util.ArrayList;
import java.util.List;
import org.javatuples.Pair;
import pedsim.core.parameters.Pars;
import pedsim.core.engine.NetworkCircuity;
import pedsim.cityimage.agents.CityImageAgent;
import pedsim.cityimage.parameters.TestPars;
import pedsim.cityimage.utilities.StringEnum.RouteChoice;
import pedsim.core.engine.PedSimCity;
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
    for (int i = 0; i < numberTrips; i++) {
      NodeGraph originNode = randomGenericOrigin();
      NodeGraph destinationNode =
          randomDestination(
              originNode, NetworkCircuity.straightLineFor(Pars.minRouteLength), NetworkCircuity.straightLineFor(Pars.maxRouteLength));

      odMatrix.add(new Pair<>(originNode, destinationNode));
    }
  }

  private NodeGraph randomGenericOrigin() {
    if (!PedSimCity.startingNodes.isEmpty()) {
      return NodesLookup.randomNodeFromGeometries(network, PedSimCity.startingNodes, random);
    }

    return NodesLookup.randomNode(network, random);
  }

  private NodeGraph randomDestination(NodeGraph originNode) {
    return randomDestination(
        originNode, NetworkCircuity.straightLineFor(Pars.minRouteLength), NetworkCircuity.straightLineFor(Pars.maxRouteLength));
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
    RouteChoice[] routeChoiceModels = routeChoiceModels();

    for (int agentID = 0; agentID < routeChoiceModels.length; agentID++) {
      CityImageAgent agent = new CityImageAgent(state, routeChoiceModels[agentID], odMatrix);
      addAgent(agent, agentID);
    }
  }

  private RouteChoice[] routeChoiceModels() {
    if (TestPars.routeChoiceModels != null && TestPars.routeChoiceModels.length > 0) {
      return TestPars.routeChoiceModels;
    }

    return new RouteChoice[] {RouteChoice.ROAD_DISTANCE, RouteChoice.ANGULAR_CHANGE};
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
