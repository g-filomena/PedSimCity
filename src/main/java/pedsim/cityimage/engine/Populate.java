package pedsim.cityimage.engine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import org.javatuples.Pair;
import pedsim.cityimage.agents.Agent;
import pedsim.cityimage.parameters.TestPars;
import pedsim.cityimage.utilities.StringEnum.RouteChoice;
import pedsim.core.engine.PedSimCity;
import pedsim.core.parameters.RouteChoicePars;
import sim.graph.Graph;
import sim.graph.NodeGraph;
import sim.graph.NodesLookup;

/**
 * The Populate class is responsible for generating test agents, building the OD matrix, and
 * populating empirical groups for pedestrian simulation.
 */
public class Populate extends pedsim.core.engine.Populate {

  private Graph network = new Graph();
  private final ArrayList<Pair<NodeGraph, NodeGraph>> OD = new ArrayList<>();
  private PedSimCityImage state;

  public static HashMap<String, Double> destinationsDMA = new HashMap<>();
  List<Integer> testOrigins = new ArrayList<>();
  List<Integer> testDestinations = new ArrayList<>();

  /**
   * Populates test agents, OD matrix, and empirical groups for pedestrian simulation.
   *
   * @param state The PedSimCity simulation state.
   */
  public void populateTests(PedSimCityImage state) {

    this.state = state;
    this.network = PedSimCity.network;

    if (TestPars.testingSpecificOD)
      prepareManualODmatrix();
    generateTestODmatrix();
    generateTestAgents();
  }

  /**
   * Prepares a manual OD matrix based on specified test origins and destinations.
   */
  public void prepareManualODmatrix() {

    testOrigins.clear();
    testDestinations.clear();
    for (Integer nodeID : TestPars.originsTmp)
      testOrigins.add(nodeID);
    for (Integer nodeID : TestPars.destinationsTmp)
      testDestinations.add(nodeID);
  }

  /**
   * Generates the OD matrix for the test-based simulations.
   */
  private void generateTestODmatrix() {

    // a trip per agent
    for (int i = 0; i < TestPars.numberTripsPerAgent; i++) {
      NodeGraph originNode = null;
      NodeGraph destinationNode = null;

      if (TestPars.testingSpecificOD) {
        originNode = PedSimCity.nodesMap.get(testOrigins.get(i));
        destinationNode = PedSimCity.nodesMap.get(testDestinations.get(i));
      } else if (TestPars.testingLandmarks) {
        originNode = NodesLookup.randomNode(network);
        destinationNode = NodesLookup.randomNodeFromDistancesSet(network, PedSimCity.junctions,
            originNode, PedSimCityImage.distances);
      } else if (TestPars.testingSubdivisions) {
        originNode = NodesLookup.randomNodeFromGeometriest(network, PedSimCity.startingNodes);
        destinationNode =
            NodesLookup.randomNodeBetweenDistanceInterval(network, originNode, 1000, 3000);
      } else if (TestPars.testingModels) {
        originNode = NodesLookup.randomNodeFromGeometriest(network, PedSimCity.startingNodes);
        destinationNode = NodesLookup.randomNodeBetweenDistanceInterval(network, originNode,
            RouteChoicePars.minTripDistance, RouteChoicePars.maxTripDistance);
      }

      Pair<NodeGraph, NodeGraph> pair = new Pair<>(originNode, destinationNode);
      this.OD.add(pair);
    }
  }

  /**
   * Generates test agents for the simulation.
   */
  private void generateTestAgents() {

    // One Model, One Agent
    TestPars.numAgents = TestPars.routeChoiceModels.length;
    final RouteChoice[] routeChoiceModels = TestPars.routeChoiceModels;
    for (int agentID = 0; agentID < TestPars.numAgents; agentID++) {
      Agent agent = new Agent(this.state);
      agent.initialiseAgentProperties();
      agent.getProperties().setRouteChoice(routeChoiceModels[agentID]);
      addAgent(agent, agentID, OD);
    }
  }

  /**
   * Adds an agent to the simulation.
   *
   * @param agent The agent to be added.
   * @param agentID The identifier of the agent.
   * @param thisAgentODs The OD matrix for this agent.
   */
  private void addAgent(Agent agent, int agentID,
      ArrayList<Pair<NodeGraph, NodeGraph>> thisAgentODs) {

    agent.OD = new LinkedList<>(thisAgentODs);
    agent.agentID = agentID;
    state.agents.addGeometry(agent.getGeometry());
    state.agentsList.add(agent);
  }
}
