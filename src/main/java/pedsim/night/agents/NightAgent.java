package pedsim.night.agents;

import java.util.List;
import pedsim.activity.agents.ActivityAgent;
import pedsim.core.agents.Heuristics;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.utilities.StringEnum.Vulnerable;
import pedsim.night.engine.PedSimCityNight;
import pedsim.night.routing.pathfinder.RoadDistancePathFinder;
import sim.engine.SimState;
import sim.graph.Graph;
import sim.graph.NodeGraph;

/**
 * Pedestrian agent for the night module. Inherits the 24h activity pattern (time-of-day destination
 * selection) from {@link ActivityAgent} and adds the night perception/safety layer:
 * vulnerability-aware, lighting-aware routing and avoidance of parks/water after dark.
 */
public class NightAgent extends ActivityAgent {

  private static final long serialVersionUID = 1L;

  public NightAgent abTestTwin = null;
  protected PedSimCityNight state;
  private final Graph agentNetwork;

  public double lightSensitivityThreshold;
  // Per-trip lighting metric: mean over edges that actually had a measured mean_lux.
  public double accumulatedMeasuredLux = 0.0;
  public int edgesWithMeasuredLux = 0;
  public int edgesWalked = 0;

  public NightAgent(PedSimCityNight state) {
    this(state, true);
  }

  public NightAgent(PedSimCityNight state, boolean registerSpatial) {
    super(state, registerSpatial);
    this.state = state;
    this.agentNetwork = SharedCognitiveMap.getCommunityPrimalNetwork();
  }

  /**
   * Sets the light-sensitivity threshold: a random draw in [min, max] for vulnerable agents, the
   * fixed non-vulnerable value otherwise.
   */
  public void initSensitivity() {
    if (isVulnerable()) {
      double min = state.getMinVulnerableLightSensitivity();
      double max = state.getMaxVulnerableLightSensitivity();
      this.lightSensitivityThreshold = min + state.random.nextDouble() * (max - min);
    } else {
      this.lightSensitivityThreshold = state.getNonVulnerableLightSensitivity();
    }
  }

  /** Called every tick: plans a trip when idle, otherwise advances along the night-aware path. */
  @Override
  public void step(SimState simState) {
    if (isWaiting()) {
      return;
    }
    if (isWalkingAlone() && destinationNode == null) {
      if (!cognitiveMap.formed) {
        getCognitiveMap().buildSimpleActivityBone();
        cognitiveMap.formed = true;
      }
      planTrip();
    } else if (reachedDestination.get()) {
      handleReachedDestination();
    } else if (isAtDestination()) {
      if (timeAtDestination <= state.schedule.getSteps()) {
        goHome();
      }
      // else: still resting at the destination.
    } else {
      agentMovement.keepWalking();
    }
  }

  /**
   * Plans one trip. Destination is home when heading home, the workplace during the day (before work
   * is done), otherwise a random reachable node — avoiding parks/water when dark. Routing is
   * night-aware after dark and plain shortest path during the day (see {@link #planRoute()}).
   */
  @Override
  protected void planTrip() {
    defineOrigin();
    if (isGoingHome()) {
      destinationNode = homeNode;
    } else if (workNode != null && !hasWorkedToday && !state.isDark) {
      destinationNode = workNode;
    } else {
      defineRandomDestination();
    }
    if (sameOriginDestination()) {
      return;
    }
    planRoute();
    tripStartStep = state.schedule.getSteps();
    agentMovement = new NightAgentMovement(this);
    agentMovement.initialisePath(getRoute());
  }

  /**
   * Night-aware road-distance routing after dark (vulnerable agents avoid parks/water and unknown
   * regions); plain shortest path during the day, so the defensive routing is a night-only response.
   */
  @Override
  protected void planRoute() {
    new Heuristics(this).defineHeuristic(originNode, destinationNode, true);
    RoadDistancePathFinder pathFinder = new RoadDistancePathFinder();
    setRoute(
        state.isDark
            ? pathFinder.roadDistanceNight(originNode, destinationNode, this)
            : pathFinder.roadDistance(originNode, destinationNode, this));
  }

  private boolean sameOriginDestination() {
    if (destinationNode.getID() == originNode.getID()) {
      reachedDestination.set(true);
      return true;
    }
    return false;
  }

  /**
   * Selects a random destination within a distance band around the origin. When dark, nodes on
   * park/water edges are avoided. Bounded to a fixed number of attempts; if none is found the agent
   * falls back to any reachable node so it can never stall.
   */
  private void defineRandomDestination() {
    if (abTestTwin != null
        && abTestTwin.destinationNode != null
        && abTestTwin.destinationNode != abTestTwin.homeNode) {
      this.destinationNode = abTestTwin.destinationNode;
      return;
    }

    double lowerLimit = distanceNextDestination * 0.90;
    double upperLimit = distanceNextDestination * 1.10;

    for (int attempt = 0; destinationNode == null && attempt < 100; attempt++) {
      List<NodeGraph> candidates =
          getNodesBetweenDistanceIntervalOptimized(agentNetwork, originNode, lowerLimit, upperLimit);
      if (candidates.isEmpty()) {
        lowerLimit *= 0.90;
        upperLimit *= 1.10;
        continue;
      }

      destinationNode = selectWeightedDestination(candidates, state.isDark);

      if (state.isDark
          && destinationNode != null
          && destinationNode.getEdges().stream()
              .anyMatch(SharedCognitiveMap.getEdgesWithinParksOrAlongWater()::contains)) {
        destinationNode = null;
        lowerLimit *= 0.90;
        upperLimit *= 1.10;
      }
    }

    if (destinationNode == null) {
      // No park/water-free destination found; accept any reachable node so the agent proceeds.
      List<NodeGraph> nodes = agentNetwork.getNodes();
      destinationNode = nodes.get(random.nextInt(nodes.size()));
    }
  }

  /** True when this agent is vulnerable. Single source of truth: the base {@code vulnerable} flag. */
  public boolean isVulnerable() {
    return isVulnerableBoolean();
  }

  /**
   * Night agents are grouped by vulnerability for volume tallying. The time dimension (hour, and the
   * day/night aggregation derived from it) is inherited from {@link ActivityAgent}.
   */
  @Override
  public Enum<?> getAgentScenario() {
    return isVulnerable() ? Vulnerable.VULNERABLE : Vulnerable.NON_VULNERABLE;
  }

  /**
   * Mean illuminance experienced on the just-completed trip, over lit edges only: measured
   * {@code mean_lux} where available, else the nominal {@code litEdgeNominalLux} for edges known lit
   * only via the binary flag, divided by the count of such edges. {@code NaN} until the trip contains
   * at least one lit edge (fully-dark edges are excluded).
   */
  @Override
  public double getTripMeanLux() {
    return edgesWithMeasuredLux > 0 ? accumulatedMeasuredLux / edgesWithMeasuredLux : Double.NaN;
  }

  @Override
  public PedSimCityNight getState() {
    return state;
  }
}
