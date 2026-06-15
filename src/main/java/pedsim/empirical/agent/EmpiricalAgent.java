package pedsim.empirical.agent;

import pedsim.core.agents.AgentMovement;
import pedsim.core.agents.OdAgent;
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

  public EmpiricalAgent(PedSimCity state, EmpiricalAgentsGroup group) {
    super(state);
    this.groupName = group.groupName;
    this.agentProperties = new EmpiricalAgentProperties(this, group);
    ((EmpiricalAgentProperties) this.agentProperties).randomizeRouteChoiceParameters();
    this.agentMovement = new AgentMovement(this);
  }

  @Override
  protected void planRoute() {
    RoutePlanner planner = new RoutePlanner(originNode, destinationNode, this);
    route = planner.definePath();
  }

  @Override
  public Enum<?> getAgentScenario() {
    return groupName;
  }
}
