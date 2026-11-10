package pedsim.learning.engine;

import pedsim.activity.agents.Persona;
import pedsim.activity.engine.ActivityPopulate;
import pedsim.core.agents.Agent;
import pedsim.core.utilities.StringEnum.Learner;
import pedsim.learning.agents.LearningAgent;
import pedsim.learning.parameters.LearningPars;

/**
 * Generates the learning-module agent population.
 *
 * <p>Reuses {@link ActivityPopulate} for everything: agent creation streaming, registration, and
 * crucially {@code defineHomeWorkLocations(Agent)} for home/work assignment (census-zone →
 * DMA → uniform-random fallback). Home/work belongs to the agent, exactly as in core, and the
 * cognitive map reads them from the agent.
 *
 * <p>Learner status alternates per agent ID (even = {@link Learner#LEARNER}, odd =
 * {@link Learner#NOT_LEARNER}) so each run contains a balanced split for the learning-vs-static
 * comparison declared by {@code ScenarioConfig(Learner.values(), null)}.
 *
 * <p>With {@link LearningPars#shadowPairs} every agent is a learner, and gets a shadow: an agent
 * with the same home, workplace and person that is not placed in the simulation and only plans.
 * The learner's share of the community's network follows {@link LearningPars#knowledgeLevels}.
 */
public class LearningPopulate extends ActivityPopulate {

  @Override
  protected Agent createAgent(int agentID) {
    LearningAgent agent = new LearningAgent(this.state, false);
    agent.agentID = agentID;
    agent.setLearner(
        LearningPars.shadowPairs || agentID % 2 == 0 ? Learner.LEARNER : Learner.NOT_LEARNER);
    defineHomeWorkLocations(agent); // core: census-zone → DMA → uniform random
    agent.getLearningCognitiveMap().setCommunityShare(LearningPars.knowledgeShare(agentID));
    if (LearningPars.shadowPairs) {
      LearningAgent shadow = new LearningAgent(this.state, false);
      shadow.agentID = SHADOW_ID_OFFSET + agentID;
      shadow.setLearner(Learner.NOT_LEARNER);
      shadow.setHomeAndWorkplace(agent.homeNode, agent.workNode);
      shadow.copyPersonFrom(agent);
      agent.setShadow(shadow);
    }
    return agent;
  }

  /** Added to a learner's ID to give its shadow an ID of its own, outside the population's. */
  private static final int SHADOW_ID_OFFSET = 1_000_000;

  /** The persona every learner has when {@link LearningPars#fixedPersona} names one. */
  @Override
  protected Persona samplePersona() {
    if (!LearningPars.fixedPersona.isBlank()) {
      return Persona.valueOf(LearningPars.fixedPersona.trim().toUpperCase());
    }
    return super.samplePersona();
  }
}
