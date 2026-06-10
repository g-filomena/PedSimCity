package pedsim.cityimage.engine;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import pedsim.cityimage.agents.Agent;
import pedsim.cityimage.agents.AgentProperties;
import pedsim.empirical.agent.EmpiricalAgentProperties;
import pedsim.cityimage.parameters.TestPars;
import pedsim.cityimage.utilities.RouteData;
import pedsim.core.engine.PedSimCity;
import sim.graph.EdgeGraph;
import sim.graph.GraphUtils;
import sim.routing.Route;

/**
 * The Flow class provides methods for updating various data related to agent movement and route
 * storing in the simulation.
 */
public class FlowHandler extends pedsim.core.engine.FlowHandler {

  public List<RouteData> routesData = new ArrayList<>();

  /**
   * Initialises the edge volumes for the simulation. This method assigns initial volume values to
   * edges based on the selected route choice models or empirical agent groups. If the simulation is
   * not empirical, it initialises volumes based on the route choice models. If the simulation is
   * empirical-based, it initialises volumes based on empirical agent groups.
   */

  public FlowHandler(int job, PedSimCity state) {
    super(job, state);
  }

  @Override
  protected void initializeEdgeVolumes() {

    for (int edgeID : PedSimCity.edgesMap.keySet()) {
      Map<String, Integer> edgeVolumes = new HashMap<String, Integer>();
      if (!TestPars.empirical)
        edgeVolumes = Arrays.stream(scenarios).collect(Collectors.toMap(s -> s, s -> 0));
      else
        edgeVolumes = PedSimCityImage.empiricalGroups.stream().collect(Collectors
            .toMap(empiricalGroup -> empiricalGroup.groupName.toString(), empiricalGroup -> 0));

      volumesMap.put(edgeID, edgeVolumes);
    }
  }

  /**
   * Updates the edge data on the basis of the passed agent's route and its edges sequence.
   *
   * @param agent The agent for which edge data is updated.
   * @param route The sequence of directed edges travelled by the agent.
   */
  public synchronized void updateFlowsData(Agent agent, Route route) {

    AgentProperties agentProperties = agent.getProperties();
    String attribute =
        TestPars.empirical ? ((EmpiricalAgentProperties) agentProperties).groupName.toString()
            : agentProperties.routeChoice.toString();

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
    if (TestPars.empirical)
      routeData = getDataFromEmpiricalAgent(agent, routeData);
    return routeData;
  }

  /**
   * Extracts data from an empirical agent's properties and populates the route data.
   *
   * @param agent The empirical agent for which route data is extracted.
   * @param route The RouteData object to be populated with data.
   * @return A RouteData object containing route information.
   */
  private static RouteData getDataFromEmpiricalAgent(Agent agent, RouteData routeData) {
    EmpiricalAgentProperties agentProperties = (EmpiricalAgentProperties) agent.getProperties();
    routeData.group = agentProperties.groupName.toString();
    routeData.routeID = agent.originNode.getID() + "-" + agent.destinationNode.getID();
    routeData.minimisingDistance = agentProperties.minimisingDistance ? true : false;
    routeData.minimisingAngular = agentProperties.minimisingAngular ? true : false;
    routeData.localHeuristicDistance = agentProperties.localHeuristicDistance ? true : false;
    routeData.localHeuristicAngular = agentProperties.localHeuristicAngular ? true : false;
    routeData.barrierSubGoals = agentProperties.barrierBasedNavigation ? true : false;
    routeData.distantLandmarks = agentProperties.usingDistantLandmarks ? true : false;
    routeData.regionBased = agentProperties.regionBasedNavigation ? true : false;
    routeData.onRouteMarks = agentProperties.usingLocalLandmarks ? true : false;
    routeData.naturalBarriers = agentProperties.naturalBarriers;
    routeData.severingBarriers = agentProperties.severingBarriers;
    return routeData;
  }
}
