package pedsim.night.agents;

import java.util.List;

import pedsim.core.agents.Heuristics;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.utilities.StringEnum;
import pedsim.core.utilities.StringEnum.Vulnerable;
import pedsim.night.engine.PedSimCityNight;
import pedsim.night.routing.RoutePlanner;
import sim.engine.SimState;
import sim.engine.Steppable;
import sim.graph.Graph;
import sim.graph.NodeGraph;
import sim.graph.NodesLookup;

/** Agent implementation for night simulations. */
public class Agent extends pedsim.core.agents.Agent implements Steppable {

  private static final long serialVersionUID = 1L;

  public StringEnum.Vulnerable vulnerable;

  private final Graph agentNetwork;
  protected pedsim.night.agents.AgentMovement nightMovement;
  protected PedSimCityNight state;
  private boolean nightRoute = false;

  public Agent(PedSimCityNight state) {
    this(state, true);
  }

  public Agent(PedSimCityNight state, boolean registerSpatial) {
    super(state, registerSpatial);
    this.state = state;
    this.agentNetwork = SharedCognitiveMap.getCommunityPrimalNetwork();
  }

  @Override
  public void step(SimState state) {
    if (isWaiting()) {
      return;
    }

    if (isWalkingAlone() && destinationNode == null) {
      if (!cognitiveMap.formed) {
        getCognitiveMap().buildSimpleActivityBone();
        cognitiveMap.formed = true;
      }

      if (this.state.isDark) {
        planNightTrip();
        nightRoute = true;
      } else {
        planTrip();
        nightRoute = false;
      }
      return;
    }

    if (reachedDestination.get()) {
      nightRoute = false;
      handleReachedDestination();
    } else if (isAtDestination() && timeAtDestination <= state.schedule.getSteps()) {
      goHome();
    } else if (isAtDestination()) {
      return;
    } else if (nightRoute) {
      nightMovement.keepWalking();
    } else {
      agentMovement.keepWalking();
    }
  }

  private synchronized void planNightTrip() {
    defineOrigin();
    if (isGoingHome()) {
      destinationNode = homeNode;
    } else {
      defineRandomDestination();
    }

    if (destinationNode.getID() == originNode.getID()) {
      reachedDestination.set(true);
      return;
    }

    planNightRoute();
    nightMovement = new AgentMovement(this);
    nightMovement.initialisePath(getRoute());
  }

  protected void planNightRoute() {
    Heuristics heuristics = new Heuristics(this);
    heuristics.defineHeuristic(originNode, destinationNode, true);
    RoutePlanner planner = new RoutePlanner(originNode, destinationNode, this);
    setRoute(planner.definePath());
  }

  @Override
  protected synchronized void planTrip() {
    defineOrigin();
    if (isGoingHome()) {
      destinationNode = homeNode;
    } else if (workNode != null && !hasWorkedToday && !state.isDark) {
      destinationNode = workNode;
    } else {
      defineRandomDestination();
    }

    if (destinationNode.getID() == originNode.getID()) {
      reachedDestination.set(true);
      return;
    }

    planRoute();
    agentMovement = new pedsim.core.agents.AgentMovement(this);
    agentMovement.initialisePath(getRoute());
  }

  @Override
  protected void planRoute() {
    Heuristics heuristics = new Heuristics(this);
    heuristics.defineHeuristic(originNode, destinationNode, true);

    // Daytime behaviour inside a night simulation should use the core route planner.
    pedsim.core.routing.RoutePlanner planner =
        new pedsim.core.routing.RoutePlanner(originNode, destinationNode, this);
    setRoute(planner.definePath());
  }

  private void defineRandomDestination() {
    double lowerLimit = distanceNextDestination * 0.90;
    double upperLimit = distanceNextDestination * 1.10;

    while (destinationNode == null) {
      List<NodeGraph> destinationCandidates =
          NodesLookup.getNodesBetweenDistanceInterval(agentNetwork, originNode, lowerLimit, upperLimit);

      if (destinationCandidates.isEmpty()) {
        lowerLimit *= 0.90;
        upperLimit *= 1.10;
        continue;
      }

      destinationNode = selectWeightedDestination(destinationCandidates, state.isDark);

      if (state.isDark
          && destinationNode.getEdges().stream()
              .anyMatch(SharedCognitiveMap.getEdgesWithinParksOrAlongWater()::contains)) {
        destinationNode = null;
        lowerLimit *= 0.90;
        upperLimit *= 1.10;
      }
    }
  }

  public boolean isVulnerable() {
    return isVulnerableBoolean() || vulnerable == Vulnerable.VULNERABLE;
  }

  @Override
  public PedSimCityNight getState() {
    return state;
  }
}
