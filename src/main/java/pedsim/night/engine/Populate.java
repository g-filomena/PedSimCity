package pedsim.night.engine;

import java.util.Random;
import pedsim.core.utilities.StringEnum.Vulnerable;

/**
 * The Populate class is responsible for generating agents, building the OD matrix, and populating
 * empirical groups for pedestrian the simulation.
 */
public class Populate extends pedsim.core.engine.Populate {

  private PedSimCityNight state;

  /**
   * Adds a new agent to the simulation with a randomly assigned vulnerability status. The agent is
   * added to the list of agents and its cognitive map is initialised.
   *
   * @param agentID The identifier of the agent to be added.
   */
  @Override
  protected void addAgent(int agentID) {

    pedsim.night.agents.Agent agent = new pedsim.night.agents.Agent(this.state);
    defineHomeWorkLocations(agent);
    agent.agentID = agentID;
    agent.vulnerable = assignRandomVulernability();
    this.state.agentsList.add(agent);
    agent.updateAgentLists(false, true);
  }

  /**
   * Assigns a random vulnerability status (either vulnerable or non-vulnerable) to an agent with a
   * 55% chance of being vulnerable.
   *
   * @return A randomly assigned vulnerability status.
   */
  public static Vulnerable assignRandomVulernability() {
    double p = new Random().nextDouble();
    return p < 0.55 ? Vulnerable.VULNERABLE : Vulnerable.NON_VULNERABLE;
  }
}
