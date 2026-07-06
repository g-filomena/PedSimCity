package pedsim.night.engine;

import java.util.Map;
import pedsim.activity.engine.ActivityPopulate;
import pedsim.core.agents.Agent;
import pedsim.core.engine.PedSimCity;
import pedsim.night.agents.NightAgent;
import pedsim.night.parameters.NightPars;
import sim.graph.NodeGraph;

/**
 * Populate strategy for the night module. Extends the activity-based {@link ActivityPopulate}
 * (census residence + workplace home/work selection) and adds per-agent vulnerability assignment and
 * A/B-test twin generation.
 */
public class NightPopulate extends ActivityPopulate {

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

  /**
   * Spawns identical vulnerable/non-vulnerable twin pairs sharing the same home/work locations, for a
   * controlled A/B comparison. The number of pairs is the user-set {@link NightPars#abTestPairs}
   * (2 agents per pair), independent of the census-derived population. Vulnerability here is assigned
   * by construction, not sampled from the census {@code vulnerability_pct}.
   */
  private void populateABTest() {
    int pairs = Math.max(1, NightPars.abTestPairs);
    int currentAgentID = 0;
    for (int i = 0; i < pairs; i++) {
      NightAgent vulnerableTwin = new NightAgent(this.state, false);
      vulnerableTwin.agentID = currentAgentID++;
      defineHomeWorkLocations(vulnerableTwin);
      vulnerableTwin.setVulnerable(true);
      vulnerableTwin.initSensitivity();

      NightAgent normalTwin = new NightAgent(this.state, false);
      normalTwin.agentID = currentAgentID++;
      normalTwin.setHomeWorkLoctations(vulnerableTwin.homeNode, vulnerableTwin.workNode);
      normalTwin.setVulnerable(false);
      normalTwin.initSensitivity();

      vulnerableTwin.abTestTwin = normalTwin;
      normalTwin.abTestTwin = vulnerableTwin;

      registerAgent(vulnerableTwin);
      registerAgent(normalTwin);
    }
    System.out.println("Spawned " + pairs + " A/B twin pairs (vulnerable vs non-vulnerable).");
  }

  private void registerAgent(NightAgent agent) {
    if (agent.homeNode != null) {
      agent.currentLocation.geometry =
          new org.locationtech.jts.geom.GeometryFactory()
              .createPoint(agent.homeNode.getCoordinate());
    }
    this.state.agents.addGeometry(agent.getLocation());
    agent.updateAgentLists(false, true); // adds to agentsList + agentsAtHome
  }

  @Override
  protected Agent createAgent(int agentID) {
    NightAgent agent = new NightAgent(this.state, false);
    agent.agentID = agentID;
    defineHomeWorkLocations(agent);
    assignVulnerabilityStatus(agent);
    agent.initSensitivity();
    return agent;
  }

  /**
   * Assigns vulnerability from the agent's home-zone {@code vulnerability_pct}. Defaults to not
   * vulnerable when the vulnerability dataset was not loaded or the home node has no zone value.
   * ({@code vulnerability_pct} is already a [0,1] rate from the census pipeline.)
   */
  private void assignVulnerabilityStatus(NightAgent agent) {
    Map<NodeGraph, Double> vulnMap = PedSimCityNight.nodesVulnerabilityWeight;
    if (vulnMap.isEmpty() || agent.homeNode == null) {
      agent.setVulnerable(false);
      return;
    }
    agent.setVulnerable(random.nextDouble() < vulnMap.getOrDefault(agent.homeNode, 0.0));
  }
}
