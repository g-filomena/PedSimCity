package pedsim.night.engine;

import java.util.Map;
import pedsim.core.agents.Agent;
import pedsim.core.engine.PedSimCity;
import pedsim.core.utilities.StringEnum.Vulnerable;
import sim.graph.NodeGraph;
import sim.util.geo.MasonGeometry;

/**
 * The Populate class is responsible for generating agents, building the OD matrix, and populating
 * empirical groups for pedestrian the simulation.
 */
public class Populate extends pedsim.core.engine.Populate {

  private PedSimCityNight state;

  @Override
  public void populate(PedSimCity state) {
    this.state = (PedSimCityNight) state;
    if (this.state.getEnableLightABTesting()) {
      populateABTest();
    } else {
      super.populate(state);
    }
  }

  private void populateABTest() {
    int pairs = 50;
    int currentAgentID = 0;
    for (int i = 0; i < pairs; i++) {
      // Create a dummy agent just to get valid home/work locations from the core logic
      pedsim.night.agents.Agent dummy = new pedsim.night.agents.Agent(this.state, false);
      super.defineHomeWorkLocations(dummy);
      
      // Vulnerable twin
      pedsim.night.agents.Agent vulnerableTwin = new pedsim.night.agents.Agent(this.state, false);
      vulnerableTwin.agentID = currentAgentID++;
      vulnerableTwin.setHomeWorkLoctations(dummy.homeNode, dummy.workNode);
      vulnerableTwin.setVulnerable(true);
      vulnerableTwin.vulnerable = Vulnerable.VULNERABLE;
      
      // Normal twin
      pedsim.night.agents.Agent normalTwin = new pedsim.night.agents.Agent(this.state, false);
      normalTwin.agentID = currentAgentID++;
      normalTwin.setHomeWorkLoctations(dummy.homeNode, dummy.workNode);
      normalTwin.setVulnerable(false);
      normalTwin.vulnerable = Vulnerable.NON_VULNERABLE;
      
      registerAgent(vulnerableTwin);
      registerAgent(normalTwin);
    }
    System.out.println("Spawned " + pairs + " A/B testing identical twin pairs (Vulnerable vs Normal).");
  }

  private void registerAgent(pedsim.night.agents.Agent agent) {
    agent.updateAgentLists(false, true);
    this.state.agentsList.add(agent);
    if (agent.homeNode != null) {
      agent.currentLocation.geometry =
          new org.locationtech.jts.geom.GeometryFactory().createPoint(agent.homeNode.getCoordinate());
    }
    this.state.agents.addGeometry(agent.getLocation());
  }

  /**
   * Creates a new agent with an assigned vulnerability status based on the vulnerability dataset.
   * If the vulnerability dataset was not loaded, all agents default to NON_VULNERABLE.
   *
   * @param agentID The identifier of the agent.
   * @return The created agent.
   */
  @Override
  protected pedsim.core.agents.Agent createAgent(int agentID) {

    pedsim.night.agents.Agent agent = new pedsim.night.agents.Agent(this.state, false);
    agent.agentID = agentID;

    defineHomeWorkLocations(agent);
    assignVulnerabilityStatus(agent);
    agent.vulnerable =
        agent.isVulnerableBoolean() ? Vulnerable.VULNERABLE : Vulnerable.NON_VULNERABLE;

    agent.updateAgentLists(false, true);
    return agent;
  }

  /**
   * Assigns vulnerability to the agent based on its home node and the vulnerability dataset.
   * If the vulnerability dataset was not loaded (empty map) or the node has no zone,
   * the agent defaults to NOT vulnerable.
   *
   * @param agent The agent to assign vulnerability to.
   */
  private void assignVulnerabilityStatus(pedsim.night.agents.Agent agent) {

    Map<NodeGraph, Double> vulnMap = PedSimCity.nodesVulnerabilityWeight;

    if (vulnMap.isEmpty() || agent.homeNode == null) {
      agent.setVulnerable(false);
      return;
    }

    double vulnProb = vulnMap.getOrDefault(agent.homeNode, 0.0);

    // Normalise: values > 1.0 are assumed to be percentages (e.g. 45.2 -> 0.452)
    if (vulnProb > 1.0) {
      vulnProb /= 100.0;
    }

    agent.setVulnerable(random.nextDouble() < vulnProb);
  }
}
