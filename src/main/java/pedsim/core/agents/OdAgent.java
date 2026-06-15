package pedsim.core.agents;

import pedsim.core.engine.PedSimCity;
import sim.engine.SimState;

/**
 * Base class for agents that walk a fixed sequence of origin-destination pairs.
 * Subclasses only need to implement {@link #planRoute()}.
 */
public abstract class OdAgent extends Agent {

  protected OdAgent(PedSimCity state) {
    super(state, false);
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
}
