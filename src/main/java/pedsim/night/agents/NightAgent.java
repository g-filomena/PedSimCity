package pedsim.night.agents;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import pedsim.activity.agents.ActivityAgent;
import pedsim.activity.parameters.ActivityPars;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.engine.PedSimCity;
import pedsim.core.utilities.StringEnum.Vulnerable;
import pedsim.night.engine.PedSimCityNight;
import pedsim.night.parameters.NightPars;
import pedsim.night.routing.NightRouteCost;
import pedsim.night.routing.routers.RoadDistancePathFinder;
import sim.engine.SimState;
import sim.graph.EdgeGraph;
import sim.graph.Graph;
import sim.graph.GraphUtils;
import sim.graph.NodeGraph;
import sim.graph.NodesLookup;
import sim.routing.Route;

/**
 * Pedestrian agent for the night module. Inherits the 24h activity pattern (time-of-day destination
 * selection) from {@link ActivityAgent} and adds the night layer: routes priced by darkness and by
 * parks and water after dark, re-planned when a street turns out darker than assumed.
 *
 * <p><b>Darkness weighs on whether to walk, not only on where.</b> After dark the decision to walk
 * a discretionary leg reads the walk-share curve at the leg's length times what its night route
 * costs per metre ({@link NightRouteCost#perceivedLengthRatio}), so a leg through dark, empty or
 * enclosed streets is walked as rarely as a longer one would be. The cost is the route planner's,
 * weights and all, so a vulnerable agent declines more dark legs than a non-vulnerable one for the
 * same reasons it detours more. Fewer people walk after dark than in daylight at the same clock
 * time (Uttley and Fotios 2017; Fotios, Uttley and Fox 2019).
 *
 * <p><b>A night agent walks its whole trip.</b> The model has no transit, so an evening journey
 * that a real person would make by bus or metro is either walked in full or, where the commute
 * mode choice refuses it, not made at all. The bias
 * runs both ways and does not cancel: long journeys are walked that would not be, and the access
 * and egress walks around stops - which concentrate night-time volume at stations, and around
 * lighting that differs from the streets either side - are missing entirely.
 */
public class NightAgent extends ActivityAgent {

  private static final long serialVersionUID = 1L;

  public NightAgent abTestTwin = null;
  protected PedSimCityNight state;
  private final Graph agentNetwork;

  /** Whether this agent is vulnerable after dark; drawn per agent by {@code NightPopulate}. */
  private boolean vulnerable;

  /**
   * The illuminance, in lux, at and above which this agent is not troubled by the dark; see {@link
   * #drawReassuranceLux()}.
   */
  public double reassuranceLux;

  /** The night route a dark leg's mode decision was priced on, walked if the agent walks the leg. */
  private Route pricedRoute;

  private NodeGraph pricedOrigin;
  private NodeGraph pricedDestination;

  /** Edges whose lighting this agent has seen on the current trip; cleared as each trip starts. */
  final Set<EdgeGraph> lightingSeenThisTrip = new HashSet<>();

  /**
   * Whether the agent knows how an edge is lit: it is a street the agent knows, or one it has
   * already seen on this trip.
   */
  public boolean knowsLightingOf(EdgeGraph edge) {
    return lightingSeenThisTrip.contains(edge) || getCognitiveMap().isEdgeKnown(edge);
  }

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
   * Draws the agent's reassurance level uniformly in [{@link NightPars#minReassuranceLux}, {@link
   * NightPars#maxReassuranceLux}], whatever its vulnerability: people differ in how much light they
   * need, and vulnerability acts through the weights. It sets how dark every street reads to the
   * agent, in the route cost and in the decision to walk faster.
   */
  public void drawReassuranceLux() {
    double min = NightPars.minReassuranceLux;
    double max = NightPars.maxReassuranceLux;
    this.reassuranceLux = min + random.nextDouble() * (max - min);
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

  /** Every trip — including chained agenda trips — uses the lighting-aware movement handler. */
  @Override
  protected pedsim.core.agents.AgentMovement createMovement() {
    return new NightAgentMovement(this);
  }

  /**
   * Night-cost routing after dark ({@code DijkstraRoadDistanceNight}: darkness and parks or water
   * priced by vulnerability); plain shortest path during the day. A leg whose mode decision was
   * priced on a night route walks that route.
   */
  @Override
  protected void planRoute() {
    Route priced = pricedRoute;
    pricedRoute = null;
    if (priced != null
        && state.isDark
        && originNode == pricedOrigin
        && destinationNode == pricedDestination) {
      initialiseRoute(priced);
      return;
    }
    initialiseHeuristics(true);
    RoadDistancePathFinder pathFinder = new RoadDistancePathFinder();
    initialiseRoute(
        state.isDark
            ? pathFinder.roadDistanceNight(originNode, destinationNode, this)
            : pathFinder.roadDistance(originNode, destinationNode, this));
  }

  /**
   * After dark, the leg's metres times what its night route costs per metre; by day, the metres.
   * The route is planned here and kept, so a leg the agent decides to walk is walked on the route it
   * was judged by.
   */
  @Override
  protected double perceivedLegMetres(NodeGraph origin, NodeGraph destination, double metres) {
    pricedRoute = null;
    if (!state.isDark || origin.equals(destination)) {
      return metres;
    }
    initialiseHeuristics(true);
    Route route = new RoadDistancePathFinder().roadDistanceNight(origin, destination, this);
    pricedRoute = route;
    pricedOrigin = origin;
    pricedDestination = destination;
    if (route == null || route.directedEdgesSequence == null) {
      return metres;
    }
    return metres * NightRouteCost.perceivedLengthRatio(this, route.directedEdgesSequence);
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

  @Override
  public boolean isVulnerable() {
    return vulnerable;
  }

  /**
   * Sets the agent's vulnerability.
   *
   * @param vulnerable whether the agent is vulnerable
   */
  public void setVulnerable(boolean vulnerable) {
    this.vulnerable = vulnerable;
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
