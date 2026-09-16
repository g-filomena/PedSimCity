package pedsim.empirical.agents;

import pedsim.core.agents.AgentMovement;
import pedsim.core.agents.OdAgent;
import pedsim.core.agents.RouteChoiceModel;
import pedsim.core.engine.PedSimCity;
import pedsim.core.routing.RoutePlanner;

/**
 * OD-based empirical agent.
 *
 * This mirrors the archive/cityimage OD-trip behaviour but depends on
 * core.Agent, not on cityimage.Agent.
 */
public final class EmpiricalAgent extends OdAgent {

  private static final long serialVersionUID = 1L;

  private final EmpiricalGroup groupName;

  private final EmpiricalAgentProperties properties;

  public EmpiricalAgent(PedSimCity state, EmpiricalAgentsGroup group) {
    super(state);
    this.groupName = group.groupName;
    this.agentProperties = new EmpiricalAgentProperties(this, group);
    this.properties = (EmpiricalAgentProperties) this.agentProperties;
    this.agentMovement = new AgentMovement(this);
  }

  @Override
  protected void planRoute() {
    initialiseHeuristics(false);
    RoutePlanner planner = new RoutePlanner(originNode, destinationNode, this);
    initialiseRoute(planner.definePath());
  }

  /**
   * Drawn afresh for every trip, from this agent's survey cluster.
   *
   * <p>Per trip rather than per agent: a cluster is a distribution over ways of getting somewhere,
   * not a label fixed to a person, so the agent samples it each time it plans. v1.11 did the same,
   * from {@code findNewAStarPath}. Drawing once per agent instead makes each agent a single point
   * sample of its cluster, so the group's realised mix is N draws rather than N x trips and the
   * contrast between groups is noisier for no modelling reason.
   */
  @Override
  protected RouteChoiceModel assignedRouteChoice() {
    properties.randomizeRouteChoiceParameters();
    return properties.toModel();
  }

  @Override
  public Enum<?> getAgentScenario() {
    return groupName;
  }
}
