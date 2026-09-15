package pedsim.night.agents;

import java.util.ArrayList;
import java.util.List;
import pedsim.activity.agents.ActivityAgent;
import pedsim.activity.parameters.ActivityPars;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.engine.PedSimCity;
import pedsim.core.utilities.StringEnum.Vulnerable;
import pedsim.night.engine.PedSimCityNight;
import pedsim.night.routing.pathfinder.RoadDistancePathFinder;
import sim.engine.SimState;
import sim.graph.Graph;
import sim.graph.GraphUtils;
import sim.graph.NodeGraph;
import sim.graph.NodesLookup;

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
  // Per-trip lighting metric: illuminance integrated over the metres walked, dark metres included.
  public double accumulatedLuxMetres = 0.0;
  public double metresWalkedForLux = 0.0;
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
      this.lightSensitivityThreshold = min + random.nextDouble() * (max - min);
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
   * Plans one trip. Destination is home when heading home, the workplace when the persona's work
   * rule fires (weekday, inside the start window, daylight), otherwise a random reachable node —
   * avoiding parks/water when dark. Routing is night-aware after dark and plain shortest path
   * during the day (see {@link #planRoute()}).
   */
  @Override
  protected void planTrip() {
    defineOrigin();
    if (isGoingHome()) {
      destinationNode = homeNode;
    } else if (shouldGoToWork()) {
      destinationNode = workNode;
    } else {
      defineRandomDestination();
    }
    if (sameOriginDestination()) {
      return;
    }
    // This override bypasses ActivityAgent.planTrip(), where the mode counter lives; without
    // this the night module reported a mode split of all zeros against a full trip count.
    pedsim.activity.engine.PedSimCityActivity.countTrip("WALK");
    planRoute();
    tripStartStep = state.schedule.getSteps();
    agentMovement = createMovement();
    agentMovement.initialisePath(getRoute());
  }

  /** Every trip — including chained agenda trips — uses the lighting-aware movement handler. */
  @Override
  protected pedsim.core.agents.AgentMovement createMovement() {
    return new NightAgentMovement(this);
  }

  /**
   * Night-aware road-distance routing after dark (vulnerable agents avoid parks/water and unknown
   * regions); plain shortest path during the day, so the defensive routing is a night-only response.
   */
  @Override
  protected void planRoute() {
    initialiseHeuristics(true);
    RoadDistancePathFinder pathFinder = new RoadDistancePathFinder();
    initialiseRoute(
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
   * Selects a destination from the places the agent knows. When dark, nodes on park/water edges
   * are avoided. Bounded to a fixed number of attempts; if none is found the agent
   * falls back to any reachable node so it can never stall. The trip purpose is resolved first, so
   * the candidate weighting (via {@code getPOIWeight}) is purpose-aware; habitual-place reuse is
   * intentionally skipped — the park/water avoidance must stay free to reject any candidate.
   */
  @Override
  protected void defineRandomDestination() {
    ensureCurrentPurpose();
    if (abTestTwin != null
        && abTestTwin.destinationNode != null
        && abTestTwin.destinationNode != abTestTwin.homeNode) {
      this.destinationNode = abTestTwin.destinationNode;
      return;
    }

    // An A/B twin chooses by utility whichever way useDestinationChoice is set. The experiment's
    // manipulated variable is vulnerability, so the destination mechanism has to be held fixed for
    // the pair to be controlled - and pinning it to the model's real one is what let the twins'
    // shared trip *length* go. That length was drawn from a band of metres and was the last such
    // draw anywhere in the activity tier, which is the tier that is not meant to have one.
    if (abTestTwin != null || ActivityPars.useDestinationChoice) {
      chooseDestinationAvoidingParksAfterDark();
      return;
    }

    // Everywhere this agent knows. Night agents are not individualised - their known edges are a
    // preference signal rather than a statement about what is reachable - so this narrows what
    // they would choose, not what they could reach.
    List<NodeGraph> candidates =
        new ArrayList<>(
            GraphUtils.getNodesFromNodeIDs(
                getCognitiveMap().getAgentKnownNodes(), PedSimCity.nodesMap));
    if (candidates.isEmpty()) {
      candidates = new ArrayList<>(agentNetwork.getNodes());
    }

    // A park or waterside candidate is dropped from the choice set and another drawn from the same
    // set. The previous version answered a rejection by widening the distance interval, which let a
    // constraint that has nothing to do with distance push the destination further out.
    while (destinationNode == null && !candidates.isEmpty()) {
      destinationNode = selectWeightedDestination(candidates);
      if (destinationNode == null) {
        break;
      }
      if (state.isDark
          && destinationNode.getEdges().stream()
              .anyMatch(SharedCognitiveMap.getEdgesWithinParksOrAlongWater()::contains)) {
        candidates.remove(destinationNode);
        destinationNode = null;
      }
    }

    if (destinationNode == null) {
      // Every known candidate is on a park or waterside edge; accept any reachable node
      // so the agent proceeds.
      state.trace().recordDestinationFallback();
      destinationNode = NodesLookup.randomNode(agentNetwork, random);
    }
  }

  /**
   * Destination choice as a choice, with the night module's one extra condition kept.
   *
   * <p>After dark a candidate on a park or waterside edge is refused, and another is drawn. The
   * refusal is applied after the choice rather than folded into the utility on purpose: avoiding
   * those edges is not a preference being traded off against attraction and distance, it is a
   * constraint, and writing it as a large negative weight would let a bright enough destination buy
   * its way past it.
   *
   * <p><b>This applies to every night agent, not only vulnerable ones</b>, and so does not match
   * {@code DijkstraRoadDistanceNight}, where the same park/water avoidance is applied to vulnerable
   * agents only. The two are deliberately different decisions: declining to spend an evening in an
   * unlit park is a general one, while declining to walk past one on the way somewhere else is a
   * vulnerability response. Neither site is an oversight in the other; they share a rule and not a
   * gate. Changing either changes what the A/B experiment's one manipulated variable means.
   */
  private void chooseDestinationAvoidingParksAfterDark() {
    for (int attempt = 0; attempt < 20; attempt++) {
      chooseDestination();
      if (destinationNode == null) {
        break;
      }
      if (!state.isDark
          || destinationNode.getEdges().stream()
              .noneMatch(SharedCognitiveMap.getEdgesWithinParksOrAlongWater()::contains)) {
        return;
      }
      destinationNode = null;
    }
    if (destinationNode == null) {
      state.trace().recordDestinationFallback();
      destinationNode = NodesLookup.randomNode(agentNetwork, random);
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
   * Mean illuminance experienced on the just-completed trip, over every metre walked.
   *
   * <p>Two things were wrong with the figure this replaces. It averaged over lit edges only, so it
   * could not fall: a darker route did not lower the number, it removed edges from the denominator,
   * and a trip through unlit streets came out as bright as one along a boulevard. And it weighted
   * every edge equally, so a 20 m alley counted as much as a 200 m avenue.
   *
   * <p>Now dark metres count, at the illuminance they actually have, and each edge counts for its
   * length. {@code NaN} only when nothing was walked.
   */
  @Override
  public double getTripMeanLux() {
    return metresWalkedForLux > 0.0 ? accumulatedLuxMetres / metresWalkedForLux : Double.NaN;
  }

  @Override
  public PedSimCityNight getState() {
    return state;
  }
}
