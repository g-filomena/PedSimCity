package pedsim.night.agents;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Predicate;
import java.util.stream.Collectors;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.planargraph.DirectedEdge;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.engine.PedSimCity;
import pedsim.night.engine.NightLighting;
import pedsim.night.engine.PedSimCityNight;
import pedsim.night.parameters.NightPars;
import sim.graph.EdgeGraph;
import sim.graph.Graph;
import sim.graph.GraphUtils;
import sim.graph.NodeGraph;
import sim.routing.Astar;
import sim.routing.Route;

/**
 * Handles movement along paths, including night-time rerouting and speed
 * adjustments.
 *
 * <h2>Routing model</h2>
 *
 * <ol>
 * <li><b>Prospective planning</b>: a night-aware Dijkstra computes the global
 * route before the agent starts moving. This route is the anchor for the whole
 * journey and is kept intact in {@link #originalEdgesSequence}.
 * <li><b>Situated planning</b>: at every edge the agent re-evaluates local
 * safety (lighting / social conditions) with <em>full</em> demands — there is
 * no carry-over of relaxed standards from one edge to the next. If the upcoming
 * edge is judged unsafe and a reroute is allowed, the agent looks for a
 * <em>local bypass</em>: it scans the remaining original route in order and,
 * for the earliest node it can reach <em>safely</em> via A*, detours to that
 * node and then rejoins the original plan from there.
 * <li><b>Fallback</b>: if no safe re-entry point exists anywhere on the
 * remaining original route, the agent accepts the problematic edge and
 * increases its walking speed rather than stalling.
 * </ol>
 *
 * <p>
 * The original route is never abandoned: the agent only ever bypasses a
 * problematic stretch and rejoins the pre-processed plan, so the safety
 * guarantees established at planning time are preserved for the un-bypassed
 * remainder. Because each edge is re-evaluated with full demands, an agent that
 * was forced onto one unsafe segment becomes "demanding" again immediately
 * afterwards.
 */
public class NightAgentMovement extends pedsim.core.agents.AgentMovement {

  /**
   * The IDs of {@link #edgesToAvoid}, kept in step with it by {@code defineEdgesToAvoid}.
   *
   * <p>A* takes IDs rather than edges, and the translation is over the whole avoid-set, so it is
   * done once where the set is built rather than at each call.
   */
  private final Set<Integer> edgeIDsToAvoid = new HashSet<>();

  private final NightBehaviour nightBehaviour;
  private final PedSimCityNight state;
  private final Graph network;

  /**
   * Immutable copy of the pre-processed (prospective) route. The anchor for all
   * rerouting.
   */
  private List<DirectedEdge> originalEdgesSequence = new ArrayList<>();

  /**
   * Index, into {@link #originalEdgesSequence}, of the edge the agent is
   * currently traversing if it is on the original route; while on a bypass it
   * points to the last original edge actually walked (i.e. the bypass's eventual
   * re-entry edge always lies strictly ahead of it).
   *
   * <p>
   * Kept in sync by {@link #syncOriginalRouteIndex()} on every
   * {@link #setupEdge}.
   */
  private int originalRouteIndex = 0;

  /** Bypasses taken on the current leg, against {@link NightPars#maxReroutesPerLeg}. */
  private int reroutesThisLeg = 0;

  public NightAgentMovement(NightAgent agent) {
    super(agent);
    this.network = SharedCognitiveMap.getCommunityPrimalNetwork();
    this.nightBehaviour = new NightBehaviour(agent, this);
    this.state = (PedSimCityNight) agent.getState();
  }

  /** Initialises the directed edge sequence for the agent. */
  @Override
  public void initialisePath(Route route) {
    if (route == null
        || route.directedEdgesSequence == null
        || route.directedEdgesSequence.isEmpty()) {
      agent.setReachedDestination(true);
      return;
    }

    // Reset the per-trip lux accumulators (they live on the agent and persist across trips).
    NightAgent nightAgent = (NightAgent) agent;
    nightAgent.accumulatedLuxMetres = 0.0;
    nightAgent.metresWalkedForLux = 0.0;
    nightAgent.edgesWalked = 0;

    indexOnSequence = 0;
    originalRouteIndex = 0;
    reroutesThisLeg = 0;
    this.directedEdgesSequence = route.directedEdgesSequence;
    // Keep an independent copy of the original planned route so bypasses cannot
    // mutate it.
    this.originalEdgesSequence = new ArrayList<>(route.directedEdgesSequence);

    firstDirectedEdge = directedEdgesSequence.get(indexOnSequence);
    currentNode = (NodeGraph) firstDirectedEdge.getFromNode();
    agent.updateAgentPosition(currentNode.getCoordinate());
    setupEdge(firstDirectedEdge);
  }

  /**
   * Sets the agent up to proceed along a specified edge.
   *
   * <p>
   * Night-behaviour flags are reset to their defaults here so that each edge is
   * evaluated independently, with full safety demands and no carry-over from a
   * previous problematic edge.
   *
   * @param directedEdge the edge to traverse next
   */
  @Override
  protected void setupEdge(DirectedEdge directedEdge) {
    // Reset per-edge night flags: every edge starts from a clean, fully-demanding
    // evaluation.
    nightBehaviour.avoidParksWater = false;
    nightBehaviour.increaseSpeedAtNight = false;

    currentDirectedEdge = directedEdge;
    currentEdge = (EdgeGraph) currentDirectedEdge.getEdge();

    // Keep our position on the original route in sync before any situated decision
    // is made.
    syncOriginalRouteIndex();

    // checkLightLevel() may trigger computeAlternativeRoute(), which can replace
    // currentDirectedEdge
    // and currentEdge with the first edge of a bypass. Everything below therefore
    // operates on the
    // edge the agent will actually walk, not on the (skipped) problematic edge.
    // The first edge is checked too: rerouting is still impossible at the origin
    // (indexOnSequence == 0), but the agent can react by speeding up on a dark first edge.
    if (state.isDark) {
      nightBehaviour.checkLightLevel();
    }

    recordLightingMetric(currentEdge);

    updateCounts();

    if (PedSimCity.indexedEdgeCache.containsKey(currentDirectedEdge)) {
      indexedSegment = PedSimCity.indexedEdgeCache.get(currentDirectedEdge);
    } else {
      addIndexedSegment(currentEdge);
      indexedSegment = PedSimCity.indexedEdgeCache.get(currentDirectedEdge);
    }

    currentIndex = indexedSegment.getStartIndex();
    endIndex = indexedSegment.getEndIndex();
  }

  /**
   * Records lighting exposure for the edge actually walked, integrated over its length: the
   * measured {@code mean_lux} where it exists, the nominal {@link NightPars#litEdgeNominalLux} for
   * edges known lit only via the binary flag, and zero for edges that are neither. Dark metres
   * count, which is what lets the per-trip figure fall on a dark route.
   */
  private void recordLightingMetric(EdgeGraph edge) {
    NightAgent nightAgent = (NightAgent) agent;
    nightAgent.edgesWalked++;

    var meanLuxAttr = edge.attributes.get("mean_lux");
    double lux;
    if (meanLuxAttr != null) {
      lux = meanLuxAttr.getDouble();
    } else if (NightLighting.isTaggedLit(edge)) {
      lux = NightPars.litEdgeNominalLux;
    } else {
      lux = 0.0;
    }
    double metres = edge.getLength();
    nightAgent.accumulatedLuxMetres += lux * metres;
    nightAgent.metresWalkedForLux += metres;
  }

  /** Moves the agent along the current path. */
  @Override
  public void keepWalking() {
    resetReach();

    if (nightBehaviour.increaseSpeedAtNight) {
      increaseReach();
    }

    currentIndex += reach;

    if (currentIndex > endIndex) {
      final Coordinate currentPos = indexedSegment.extractPoint(endIndex);
      agent.updateAgentPosition(currentPos);
      double residualMove = currentIndex - endIndex;
      transitionToNextEdge(residualMove);
    } else {
      final Coordinate currentPos = indexedSegment.extractPoint(currentIndex);
      agent.updateAgentPosition(currentPos);
    }
  }

  /**
   * Realigns {@link #originalRouteIndex} with the edge currently being set up.
   *
   * <p>
   * The pointer only ever moves forward. If the current edge is the one already
   * pointed to, nothing changes. If it is an original edge further ahead (normal
   * progression by one, or a jump to a re-entry edge after a bypass), the pointer
   * advances to it. If the current edge is a bypass edge (not found ahead in the
   * original sequence) the pointer is left untouched, so it keeps marking the
   * agent's last known position on the original plan.
   */
  private void syncOriginalRouteIndex() {
    if (originalRouteIndex < originalEdgesSequence.size()
        && currentDirectedEdge.equals(originalEdgesSequence.get(originalRouteIndex))) {
      return;
    }
    for (int i = originalRouteIndex + 1; i < originalEdgesSequence.size(); i++) {
      if (currentDirectedEdge.equals(originalEdgesSequence.get(i))) {
        originalRouteIndex = i;
        return;
      }
    }
    // Current edge is part of a bypass: leave the pointer marking the agent's committed progress
    // along the original route, which is the re-entry position the bypass is heading for.
  }

  /**
   * Attempts a safe local bypass of the current (problematic) edge.
   *
   * <p>
   * Scans the remaining original route in order and, for the earliest node
   * reachable from the current origin via A* under full night-time avoidance
   * rules, builds a new path consisting of the bypass to that re-entry node
   * followed by the untouched remainder of the original route. The earliest
   * reachable re-entry point is preferred because it minimises the deviation from
   * the original plan. The destination is the last candidate, so reaching it
   * leaves no remainder to splice and the agent arrives by the bypass itself.
   *
   * <p>
   * If no node on the remaining original route can be reached safely, the agent
   * keeps to the problematic edge and speeds up instead of stalling.
   */
  void computeAlternativeRoute() {
    final NodeGraph routeOrigin = (NodeGraph) currentDirectedEdge.getFromNode();
    agent.spookLocations.add(routeOrigin.getCoordinate());

    final Predicate<EdgeGraph> edgeAllowed = edgeAllowedForBypass();

    // Candidate re-entry points, in route order, each mapped to the earliest position it occupies:
    // a node the route visits twice is rejoined at the first opportunity.
    final Map<NodeGraph, Integer> positionOf = new LinkedHashMap<>();
    for (int idx = originalRouteIndex + 1; idx < originalEdgesSequence.size(); idx++) {
      final NodeGraph reentryNode = (NodeGraph) originalEdgesSequence.get(idx).getFromNode();
      // A bypass to where we already are makes no progress.
      if (!reentryNode.equals(routeOrigin)) {
        positionOf.putIfAbsent(reentryNode, idx);
      }
    }

    // The loop above takes each remaining edge's from-node, which leaves out the route's final
    // to-node. The destination is a re-entry point like any other and is added here, at position
    // size() - past every other candidate, so it is reached for only when nothing earlier can be,
    // and the preference for rejoining the plan is unchanged. A bypass that ends there leaves an
    // empty remainder to splice: the agent arrives by its own way rather than rejoining, which is
    // the only way the last edge of a route can be avoided. putIfAbsent keeps the earlier position
    // for a route that already passes through its destination.
    if (!originalEdgesSequence.isEmpty()) {
      final NodeGraph finalNode =
          (NodeGraph) originalEdgesSequence.get(originalEdgesSequence.size() - 1).getToNode();
      if (!finalNode.equals(routeOrigin)) {
        positionOf.putIfAbsent(finalNode, originalEdgesSequence.size());
      }
    }

    final Astar aStar = new Astar();
    Route bestBypass = null;
    int bestPosition = Integer.MAX_VALUE;
    Collection<NodeGraph> candidates = positionOf.keySet();

    // A* settles the CHEAPEST target first, and the rule here is the EARLIEST one on the remaining
    // route, so one multi-target search is not enough on its own. Each search does, however, rule
    // out everything at or beyond the position it reaches, so re-searching over just the earlier
    // candidates converges on the earliest reachable one - in a handful of searches rather than one
    // per candidate, and, when none is reachable, in a single search rather than one exhaustive
    // failure per candidate.
    while (!candidates.isEmpty()) {
      final Route bypass = aStar.astarRouteAllowing(routeOrigin, candidates, network, edgeAllowed);
      if (!isValidRoute(bypass)) {
        break;
      }
      final Integer position = positionOf.get(aStar.reachedTarget());
      if (position == null || position >= bestPosition) {
        break;
      }
      bestBypass = bypass;
      bestPosition = position;

      final int earlierThan = bestPosition;
      candidates =
          positionOf.entrySet().stream()
              .filter(entry -> entry.getValue() < earlierThan)
              .map(Map.Entry::getKey)
              .collect(Collectors.toCollection(LinkedHashSet::new));
    }

    if (bestBypass != null) {
      // New path: safe bypass to the re-entry node + untouched remainder of the original route.
      final List<DirectedEdge> newPath = new ArrayList<>(bestBypass.directedEdgesSequence);
      newPath.addAll(originalEdgesSequence.subList(bestPosition, originalEdgesSequence.size()));

      edgesToAvoid.clear();
      nightBehaviour.avoidParksWater = false;
      // The agent has committed to rejoining the original route at bestPosition, so that is its
      // progress along it: everything before it lies behind. Advancing the pointer here is what
      // confines a further reroute - taken while still on this bypass, from a node off the original
      // route - to re-entry points beyond the one already chosen. Left where the agent departed the
      // route, the pointer offers those earlier positions back, and the earliest-re-entry rule
      // prefers them precisely because they are earliest, turning the agent around.
      originalRouteIndex = bestPosition;
      reroutesThisLeg++;
      resetPath(newPath);
      return;
    }

    // No safe re-entry point anywhere ahead: keep to the current edge, just walk faster.
    edgesToAvoid.clear();
    nightBehaviour.avoidParksWater = false;
    nightBehaviour.increaseSpeedAtNight = true;
  }

  /**
   * Which edges a bypass may use, as a test applied to the edges the search actually reaches.
   *
   * <p>The same rule {@code defineEdgesToAvoid} states as a set, asked one edge at a time. A* reaches
   * a few hundred edges; enumerating the rule over the whole network first is tens of thousands of
   * set operations per reroute, and the sets it draws on are already held elsewhere.
   */
  private Predicate<EdgeGraph> edgeAllowedForBypass() {

    final Set<EdgeGraph> unlit =
        agent.isVulnerable()
            ? NightLighting.unlitEdgesOutsideCommunityKnown(
                edgesOutsideCommunityKnown(), ((NightAgent) agent).lightSensitivityThreshold)
            : NightLighting.unlitEdgesOutsideCommunityKnown(edgesOutsideCommunityKnown());

    // A set, not the list the lookup returns: this is a membership test per expanded edge.
    final Set<EdgeGraph> agentKnown =
        agent.isVulnerable()
            ? new HashSet<>(
                GraphUtils.getEdgesFromEdgeIDs(
                    agent.getCognitiveMap().getAgentKnownEdges(), PedSimCity.edgesMap))
            : Set.of();

    final Set<EdgeGraph> parksAndWater =
        (agent.isVulnerable() || nightBehaviour.avoidParksWater)
            ? SharedCognitiveMap.getEdgesWithinParksOrAlongWater()
            : Set.of();

    final EdgeGraph fledEdge = currentEdge;
    final Set<EdgeGraph> destinationEdges = new HashSet<>(agent.destinationNode.getEdges());

    return edge -> {
      // Edges incident to the destination are never avoided, so the tail of the route stays
      // reachable. This is tested first because it overrides everything below it.
      if (destinationEdges.contains(edge)) {
        return true;
      }
      if (edge.equals(fledEdge) || parksAndWater.contains(edge)) {
        return false;
      }
      return !unlit.contains(edge) || agentKnown.contains(edge);
    };
  }

  /**
   * Resets the agent onto a new directed-edge sequence.
   *
   * <p>
   * Overrides the base implementation, which anchors {@code currentNode} to the
   * journey's very first edge ({@code firstDirectedEdge}); for a mid-journey
   * reroute that would momentarily place the agent at the trip origin. Here
   * {@code currentNode} is taken from the first edge of the new sequence, which
   * equals the agent's actual position at reroute time.
   *
   * @param newSequence the new sequence of directed edges to follow
   */
  @Override
  protected void resetPath(List<DirectedEdge> newSequence) {
    indexOnSequence = 0;
    this.directedEdgesSequence = newSequence;
    currentDirectedEdge = newSequence.get(indexOnSequence);
    currentEdge = (EdgeGraph) currentDirectedEdge.getEdge();
    currentNode = (NodeGraph) currentDirectedEdge.getFromNode();
    edgesToAvoid.clear();
    agent.updateAgentPosition(currentNode.getCoordinate());
  }

  /**
   * Every city edge outside the community-known network: the fixed half of a vulnerable agent's
   * avoid-set. Built once per network rather than per reroute, where it meant copying all 44,278
   * Torino edges into a fresh HashSet each time an agent was spooked. Invalidated by
   * {@link PedSimCityNight#clearNightStaticData()}.
   */
  private static volatile Set<EdgeGraph> edgesOutsideCommunityKnown;

  private static Set<EdgeGraph> edgesOutsideCommunityKnown() {
    Set<EdgeGraph> cached = edgesOutsideCommunityKnown;
    if (cached == null) {
      cached = new HashSet<>(SharedCognitiveMap.getCommunityPrimalNetwork().getEdges());
      cached.removeAll(SharedCognitiveMap.getCommunityKnownEdges());
      edgesOutsideCommunityKnown = cached;
    }
    return cached;
  }

  /** Drops the cached network set, so a re-imported network is not answered from the old one. */
  public static void clearCachedNetworkSets() {
    edgesOutsideCommunityKnown = null;
  }

  /**
   * The edges this agent will not route through when bypassing the one that frightened it.
   *
   * <p><b>One rule, at two thresholds: avoid what is neither lit nor familiar.</b> A vulnerable
   * agent retreats toward light or toward ground it personally knows - of the edges outside the
   * community-known network it avoids those that also read as unlit at its <i>own</i> drawn
   * sensitivity, less the streets it knows itself. A non-vulnerable one asks the same question with
   * the community as its familiar set and {@link NightPars#nonVulnerableLightSensitivity} as its
   * threshold. Parks and water are added for the vulnerable, and for anyone currently avoiding
   * them.
   *
   * <p><b>The lighting term on the vulnerable branch is load-bearing.</b> Without it the avoid-set
   * is knowledge-only, so an agent frightened by darkness detours toward <i>familiarity</i> and any
   * improvement in the light it ends up under is a coincidence. What frightens an agent and what it
   * detours around are the same measurement here - {@link NightLighting#isLit}, the rule its own
   * gate uses - which is what makes the A/B's manipulated variable mean what it says.
   *
   * <p>The edge being fled is avoided by both, which is why it is added after the branch and after
   * the vulnerable branch's {@code removeAll} - inside the branch it would be subtracted straight
   * back out, since the problematic edge is normally a street the agent or the community knows, and
   * A* could then return a "bypass" that ran down it. Edges incident to the destination are never
   * avoided, so the tail of the route stays reachable; {@code canReroute()} guarantees the fled edge
   * is not one of them.
   */
  private void defineEdgesToAvoid() {
    edgesToAvoid.clear();

    if (agent.isVulnerable()) {
      edgesToAvoid.addAll(
          NightLighting.unlitEdgesOutsideCommunityKnown(
              edgesOutsideCommunityKnown(), ((NightAgent) agent).lightSensitivityThreshold));
      edgesToAvoid.removeAll(
          GraphUtils.getEdgesFromEdgeIDs(
              agent.getCognitiveMap().getAgentKnownEdges(), PedSimCity.edgesMap));
    } else {
      edgesToAvoid.addAll(
          NightLighting.unlitEdgesOutsideCommunityKnown(edgesOutsideCommunityKnown()));
    }

    edgesToAvoid.add(currentEdge);

    if (agent.isVulnerable() || nightBehaviour.avoidParksWater) {
      edgesToAvoid.addAll(SharedCognitiveMap.getEdgesWithinParksOrAlongWater());
    }

    // Never avoid edges incident to the destination, so the tail of the route stays
    // reachable.
    edgesToAvoid.removeAll(agent.destinationNode.getEdges());

    // A* takes IDs, so fill them here in one pass over the set just built. Deriving them at the
    // call site cost a List and a Set on top of this one, per reroute.
    edgeIDsToAvoid.clear();
    for (EdgeGraph edge : edgesToAvoid) {
      edgeIDsToAvoid.add(edge.getID());
    }
  }

  /**
   * Checks whether the agent may attempt a situated reroute.
   *
   * <p>
   * Three of the four constraints are structural — there is nothing to bypass into if the current
   * edge leads straight to the destination, on the very first edge, or when no original route
   * remains ahead to rejoin. The fourth is {@link NightPars#maxReroutesPerLeg}, a bound on how many
   * bypasses one leg may take; past it the agent keeps to its route and walks faster. Being on a
   * bypass rather than on the original route is not itself a constraint.
   *
   * @return true if the agent can reroute; false otherwise
   */
  protected boolean canReroute() {
    return !currentEdge.getNodes().contains(agent.destinationNode)
        && indexOnSequence != 0
        && hasRemainingOriginalRoute()
        && reroutesThisLeg < NightPars.maxReroutesPerLeg;
  }

  /**
   * @return true if at least one edge of the original route still lies ahead of
   *         the agent
   */
  private boolean hasRemainingOriginalRoute() {
    return originalRouteIndex + 1 < originalEdgesSequence.size();
  }

  /**
   * @param route the route to validate
   * @return true if {@code route} is non-null and contains at least one directed
   *         edge
   */
  private static boolean isValidRoute(Route route) {
    return route != null
        && route.directedEdgesSequence != null
        && !route.directedEdgesSequence.isEmpty();
  }
}
