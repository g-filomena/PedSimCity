package pedsim.night.engine;

import pedsim.activity.engine.ActivityPopulate;
import pedsim.core.agents.Agent;
import pedsim.core.engine.PedSimCity;
import pedsim.core.utilities.LoggerUtil;
import pedsim.night.agents.NightAgent;
import pedsim.night.parameters.NightPars;

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
    seedFrom(state);
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
   * by construction and does not read the agent's sex.
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
      normalTwin.setHomeAndWorkplace(vulnerableTwin.homeNode, vulnerableTwin.workNode);
      normalTwin.setVulnerable(false);
      normalTwin.initSensitivity();
      // These two are built here rather than through defineHomeWorkLocations, so the commute mode
      // has to be settled explicitly; without it both twins defaulted to walking their commute.
      vulnerableTwin.decideCommuteMode();
      normalTwin.decideCommuteMode();

      vulnerableTwin.abTestTwin = normalTwin;
      normalTwin.abTestTwin = vulnerableTwin;

      registerAgent(vulnerableTwin);
      registerAgent(normalTwin);
    }
    LoggerUtil.getLogger()
        .info("Spawned " + pairs + " A/B twin pairs (vulnerable vs non-vulnerable).");
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
   * <b>This module's vulnerable group is women.</b> The agent's sex is a census fact settled in
   * {@link ActivityPopulate}; which groups walk differently after dark is the night model's
   * judgement, and it is made here and nowhere else. Age does not enter it: every agent the model
   * builds is an adult.
   */
  private void assignVulnerabilityStatus(NightAgent agent) {
    agent.setVulnerable(agent.isFemale());
  }
}
