package pedsim.empirical.agent;

import java.util.LinkedList;
import java.util.List;
import org.javatuples.Pair;
import pedsim.core.agents.AgentMovement;
import pedsim.core.engine.PedSimCity;
import pedsim.core.routing.RoutePlanner;
import sim.engine.SimState;
import sim.graph.NodeGraph;

/**
 * OD-based empirical agent.
 *
 * This mirrors the archive/cityimage OD-trip behaviour but depends on core.Agent, not on
 * cityimage.Agent.
 */
public final class EmpiricalAgent extends pedsim.core.agents.Agent {

  private static final long serialVersionUID = 1L;

  private final EmpiricalGroup groupName;

  public EmpiricalAgent(PedSimCity state, EmpiricalAgentsGroup group) {
    super(state, false);
    this.groupName = group.groupName;
    this.agentProperties = new EmpiricalAgentProperties(this, group);
    ((EmpiricalAgentProperties) this.agentProperties).randomizeRouteChoiceParameters();
    this.agentMovement = new AgentMovement(this);
  }

  public void setOD(List<Pair<NodeGraph, NodeGraph>> odPairs) {
    this.OD = new LinkedList<>(odPairs);

    if (!this.OD.isEmpty()) {
      this.originNode = this.OD.get(0).getValue0();
      updateAgentPosition(this.originNode.getCoordinate());
    }
  }

  @Override
  public void step(SimState state) {
    if (reachedDestination.get() || destinationNode == null) {
      handleReachedDestination();
    } else {
      agentMovement.keepWalking();
    }
  }

  @Override
  protected void handleReachedDestination() {
    boolean completedTrip = reachedDestination.get();
    reachedDestination.set(false);

    if (completedTrip) {
      setTripsDone(getTripsDone() + 1);
    }

    if (getTripsDone() >= OD.size()) {
      removeAgent();
      return;
    }

    selectNodesFromOD();
    updateAgentPosition(originNode.getCoordinate());
    planRoute();
    agentMovement.initialisePath(route);
  }

  private void selectNodesFromOD() {
    Pair<NodeGraph, NodeGraph> pair = OD.get(getTripsDone());
    originNode = pair.getValue0();
    destinationNode = pair.getValue1();
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