package pedsim.night.engine;

import pedsim.core.engine.PedSimCity;
import pedsim.core.utilities.StringEnum.Vulnerable;

/**
 * The Populate class is responsible for generating agents, building the OD matrix, and populating
 * empirical groups for pedestrian the simulation.
 */
public class Populate extends pedsim.core.engine.Populate {

  private PedSimCityNight state;

  @Override
  public void populate(PedSimCity state) {
    this.state = (PedSimCityNight) state;
    super.populate(state);
  }

  /**
   * Creates a new agent with a assigned vulnerability status based on census data.
   * Does NOT register it with simulation fields (VectorLayer, etc).
   * Adds a new agent to the simulation with a assigned vulnerability status based on census data.
   *
   * @param agentID The identifier of the agent.
   * @return The created agent.
   */
  @Override
  protected pedsim.core.agents.Agent createAgent(int agentID) {

    pedsim.night.agents.Agent agent = new pedsim.night.agents.Agent(this.state, false);
    pedsim.night.agents.Agent agent = new pedsim.night.agents.Agent(this.state);
    agent.agentID = agentID;
    defineHomeWorkLocations(agent);
    
    // Synchronize the night-specific Vulnerable enum with the core boolean set by defineHomeWorkLocations
    agent.vulnerable = agent.isVulnerableBoolean() ? 
        Vulnerable.VULNERABLE : Vulnerable.NON_VULNERABLE;

    return agent;
    this.state.agentsList.add(agent);
    agent.updateAgentLists(false, true);
  }
}
