package pedsim.core.engine;

import java.util.logging.Logger;
import java.util.stream.IntStream;
import pedsim.core.agents.Agent;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.parameters.Pars;
import pedsim.core.parameters.RouteChoicePars;
import pedsim.core.utilities.LoggerUtil;
import sim.graph.NodeGraph;
import sim.graph.NodesLookup;

/**
 * The Populate class is responsible for generating test agents, building the OD matrix, and
 * populating empirical groups for pedestrian simulation.
 */
public class Populate {

  protected PedSimCity state;
  protected static final Logger logger = LoggerUtil.getLogger();

  /**
   * Populates agents, OD matrix, for the simulation. It creates a set of agents with the learner
   * status and updates their cognitive maps. The agents are then added to the simulation state.
   *
   * @param state The PedSimCity simulation state.
   */
  public void populate(PedSimCity state) {

    this.state = state;

    // Create agents with parameter true
    int totalAgents = Pars.numAgents;
    logger.info("Creating " + totalAgents + " Agents. Building Their Cognitive Maps");
    IntStream.range(0, totalAgents).parallel().forEach(agentID -> {
      addAgent(agentID); // Must be thread-safe!
    });

    for (Agent agent : state.agentsList) {
      state.agents.addGeometry(agent.getLocation());
    }
    logger.info(state.agentsList.size() + " agents created");
  }

  /**
   * Adds a new agent to the simulation with a randomly assigned vulnerability status. The agent is
   * added to the list of agents and its cognitive map is Initialised.
   *
   * @param agentID The identifier of the agent to be added.
   */
  protected void addAgent(int agentID) {

    Agent agent = new Agent(this.state);
    defineHomeWorkLocations(agent);
    agent.agentID = agentID;
    state.agentsList.add(agent);
    agent.updateAgentLists(false, true);
  }

  // TODO to improve with census data
  protected void defineHomeWorkLocations(Agent agent) {

    // in the community network
    NodeGraph homeNode = null;
    NodeGraph workNode = null;
    while (homeNode == null) {
      homeNode = NodesLookup.randomNodeDMA(SharedCognitiveMap.getCommunityPrimalNetwork(), "live");

      workNode = NodesLookup.randomNodeBetweenDistanceIntervalDMA(
          SharedCognitiveMap.getCommunityPrimalNetwork(), homeNode, RouteChoicePars.minTripDistance,
          RouteChoicePars.maxTripDistance, "work");
    }
    agent.setHomeWorkLoctations(homeNode, workNode);
  }
}
