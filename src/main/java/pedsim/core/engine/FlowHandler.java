package pedsim.core.engine;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;
import pedsim.core.agents.Agent;
import pedsim.core.cognition.cognitivemap.CognitiveMap;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.utilities.RouteData;
import sim.graph.EdgeGraph;
import sim.graph.GraphUtils;
import sim.routing.Route;
import sim.util.geo.MasonGeometry;

/**
 * The Flow class provides methods for updating various data related to agent movement and route
 * storing in the simulation.
 *
 * @param <E>
 */
public class FlowHandler {

  public Map<Integer, Map<String, Integer>> volumesMap = new HashMap<>();
  public List<RouteData> routesData = new ArrayList<>();
  public int job;

  Map<Integer, Map<String, Integer>> knownEdgesMap = new HashMap<>();
  Map<Integer, Map<String, Integer>> knownLandmarksMap = new HashMap<>();
  private final PedSimCity state;
  private final Enum<?>[] enumAgentScenarios;
  private final Enum<?>[] enumScenarios;
  private String[] scenarios;
  Exporter exporter;

  public FlowHandler(int job, PedSimCity state) {
    this.job = job;
    this.state = state;
    this.enumAgentScenarios = getAgentScenarioValues();
    this.enumScenarios = getSimulationScenarioValues();

    if (enumAgentScenarios != null && enumScenarios != null) {
      scenarios =
          Arrays.stream(enumAgentScenarios).flatMap(agentScenario -> Arrays.stream(enumScenarios)
              .map(simScenario -> agentScenario + "_" + simScenario)).toArray(String[]::new);
    } else if (enumAgentScenarios != null) {
      scenarios = Arrays.stream(enumAgentScenarios).map(Enum::toString).toArray(String[]::new);
    } else if (enumScenarios != null) {
      scenarios = Arrays.stream(enumScenarios).map(Enum::toString).toArray(String[]::new);
    } else {
      scenarios = new String[0];
    }

    initializeEdgeVolumes();
    initializeCognitiveMapCollector();
    exporter = new Exporter(this);
  }

  public Enum<?>[] getAgentScenarioValues() {
    return state.scenarioConfig.getAgentScenarioValues();
  }

  public Enum<?>[] getSimulationScenarioValues() {
    return state.scenarioConfig.getSimulationScenarioValues();
  }

  /**
   * Initialises the edge volumes for the simulation. This method assigns initial volume values to
   * edges based on the selected route choice models or empirical agent groups. If the simulation is
   * not empirical, it initialises volumes based on the route choice models. If the simulation is
   * empirical-based, it initialises volumes based on empirical agent groups.
   */
  private void initializeEdgeVolumes() {
    for (int edgeID : PedSimCity.edgesMap.keySet()) {
      Map<String, Integer> edgeVolumes =
          Arrays.stream(scenarios).collect(Collectors.toMap(s -> s, s -> 0));
      volumesMap.put(edgeID, edgeVolumes);
    }
  }

  private void initializeCognitiveMapCollector() {

    for (Integer edgeID : PedSimCity.edgesMap.keySet()) {
      Map<String, Integer> edgeMap =
          Arrays.stream(scenarios).collect(Collectors.toMap(s -> s, s -> 0));
      knownEdgesMap.put(edgeID, edgeMap);
    }

    for (Integer buildingID : PedSimCity.buildingsMap.keySet()) {
      Map<String, Integer> buildingMap =
          Arrays.stream(scenarios).collect(Collectors.toMap(s -> s, s -> 0));
      knownLandmarksMap.put(buildingID, buildingMap);
    }
  }

  /**
   * Updates the edge data on the basis of the passed agent's route and its edges sequence.
   *
   * @param agent The agent for which edge data is updated.
   * @param route The route taken by the agent.
   * @param night Boolean flag indicating whether it is night or day.
   */
  public synchronized void updateFlowsData(Agent agent, Route route, Enum<?> agentScenarioValue,
      Enum<?> scenarioValue) {

    String attribute = (agentScenarioValue != null && scenarioValue != null)
        ? agentScenarioValue.toString() + "_" + scenarioValue.toString()
        : (agentScenarioValue != null ? agentScenarioValue.toString() : "DEFAULT");

    RouteData routeData = createRouteData(agent, route, attribute);
    for (EdgeGraph edgeGraph : route.edgesSequence) {
      Map<String, Integer> edgeVolume = volumesMap.get(edgeGraph.getID());
      edgeVolume.replace(attribute, edgeVolume.get(attribute) + 1);
      volumesMap.replace(edgeGraph.getID(), edgeVolume);
    }
    routeData.edgeIDsSequence = GraphUtils.getEdgeIDs(route.edgesSequence);
    routesData.add(routeData);
  }

  /**
   * Creates and initialises a new RouteData object for the given agent.
   *
   * @param agent The agent for which route data is created.
   * @return A RouteData object containing route information.
   */
  private RouteData createRouteData(Agent agent, Route route, String attribute) {
    RouteData routeData = new RouteData();
    routeData.origin = agent.originNode.getID();
    routeData.destination = agent.destinationNode.getID();
    routeData.lineGeometry = route.getLineString();
    routeData.scenario = attribute;
    return routeData;
  }

  /**
   * Updates the edge data on the basis of the passed agent's route and its edges sequence.
   *
   * @param agent The agent for which edge data is updated.
   * @param directedEdgesSequence The sequence of directed edges travelled by the agent.
   * @throws Exception
   */
  public synchronized void updateCognitiveMapsData(Enum<?> scenarioValue) throws Exception {

    for (Agent agent : state.agentsList) {
      String agentScenarioValueStr =
          (agent.getAgentScenario() != null) ? agent.getAgentScenario().toString() : null;
      String attribute = (scenarioValue != null && agentScenarioValueStr != null)
          ? agentScenarioValueStr + "_" + scenarioValue.toString()
          : (agentScenarioValueStr != null ? agentScenarioValueStr : "DEFAULT");

      CognitiveMap cognitiveMap = agent.getCognitiveMap();
      if (!cognitiveMap.formed) {
        continue;
      }

      for (EdgeGraph edgeGraph : cognitiveMap.getEdgesInKnownNetwork()) {
        Map<String, Integer> edgeMap = knownEdgesMap.get(edgeGraph.getID());
        edgeMap.replace(attribute, edgeMap.get(attribute) + 1);
        knownEdgesMap.replace(edgeGraph.getID(), edgeMap);
      }

      Set<Integer> buildings = cognitiveMap.getLocalLandmarksIDs();
      List<MasonGeometry> buildingGeoemetries =
          SharedCognitiveMap.getBuildings().getGeometriesFromIDs(buildings);

      for (MasonGeometry buildingGeometry : buildingGeoemetries) {
        int buildingID = buildingGeometry.getIntegerAttribute("buildingID");
        Map<String, Integer> buildingMap = knownLandmarksMap.get(buildingID);
        buildingMap.replace(attribute, buildingMap.get(attribute) + 1);
        knownLandmarksMap.replace(buildingID, buildingMap);
      }
    }
  }

  /**
   * Exports the flows data for the specified day.
   *
   * @param day The day for which the flow data should be exported.
   * @throws Exception if there is an error during the export process.
   */
  public void exportFlowsData(int day) throws Exception {
    exporter.savePedestrianVolumes(day, scenarios);
    exporter.saveRoutes(day);
    initializeEdgeVolumes();
  }

  public void exportCognitiveMapsData(int day) throws Exception {

    exporter.saveCognitiveMapsData(day, scenarios);
    exporter.saveKnownLandmarksData(day, scenarios);
    initializeCognitiveMapCollector();
  }
}
