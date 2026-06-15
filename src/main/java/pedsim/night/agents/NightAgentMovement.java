package pedsim.night.agents;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.javatuples.Pair;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.planargraph.DirectedEdge;

import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.engine.PedSimCity;
import pedsim.night.engine.PedSimCityNight;
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

	public NightAgentMovement(NightAgent agent) {
		super(agent);
		this.network = SharedCognitiveMap.getCommunityPrimalNetwork();
		this.nightBehaviour = new NightBehaviour(agent, this);
		this.state = (PedSimCityNight) agent.getState();
	}

	/** Initialises the directed edge sequence for the agent. */
	@Override
	public void initialisePath(Route route) {
		if (route == null || route.directedEdgesSequence == null || route.directedEdgesSequence.isEmpty()) {
			agent.setReachedDestination(true);
			return;
		}

		indexOnSequence = 0;
		originalRouteIndex = 0;
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
		if (state.isDark && currentDirectedEdge != firstDirectedEdge) {
			nightBehaviour.checkLightLevel();
		}

		if (currentEdge.attributes.get("mean_lux") != null) {
			((pedsim.night.agents.NightAgent) agent).accumulatedLux += currentEdge.attributes.get("mean_lux").getDouble();
		}
		((pedsim.night.agents.NightAgent) agent).edgesWalked++;

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
		// Current edge is part of a bypass: leave the pointer marking the last original
		// edge walked.
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
	 * the original plan.
	 *
	 * <p>
	 * If no node on the remaining original route can be reached safely, the agent
	 * keeps to the problematic edge and speeds up instead of stalling.
	 */
	void computeAlternativeRoute() {
		final NodeGraph routeOrigin = (NodeGraph) currentDirectedEdge.getFromNode();
		agent.spookLocations.add(routeOrigin.getCoordinate());

		final Map<Pair<NodeGraph, NodeGraph>, List<DirectedEdge>> cache = (agent.isVulnerableBoolean()
				|| nightBehaviour.avoidParksWater) ? PedSimCityNight.altRoutesVulnerable
						: PedSimCityNight.altRoutesNonVulnerable;

		defineEdgesToAvoid();
		final Set<Integer> edgeIDsToAvoid = new HashSet<>(GraphUtils.getEdgeIDs(edgesToAvoid));
		final Astar aStar = new Astar();

		// Scan the remaining original route for the earliest safely-reachable re-entry
		// point.
		for (int idx = originalRouteIndex + 1; idx < originalEdgesSequence.size(); idx++) {
			final DirectedEdge reentryEdge = originalEdgesSequence.get(idx);
			final NodeGraph reentryNode = (NodeGraph) reentryEdge.getFromNode();

			// A bypass to where we already are makes no progress.
			if (reentryNode.equals(routeOrigin)) {
				continue;
			}

			final Pair<NodeGraph, NodeGraph> routeKey = Pair.with(routeOrigin, reentryNode);
			List<DirectedEdge> bypassEdges = null;

			if (cache.containsKey(routeKey)) {
				bypassEdges = new ArrayList<>(cache.get(routeKey));
			} else {
				final Route bypass = aStar.astarRoute(routeOrigin, reentryNode, network, edgeIDsToAvoid);
				if (isValidRoute(bypass)) {
					bypassEdges = new ArrayList<>(bypass.directedEdgesSequence);
					cache.put(routeKey, new ArrayList<>(bypassEdges));
				}
			}

			if (bypassEdges != null) {
				// New path: safe bypass to the re-entry node + untouched remainder of the
				// original route.
				final List<DirectedEdge> newPath = new ArrayList<>(bypassEdges);
				newPath.addAll(originalEdgesSequence.subList(idx, originalEdgesSequence.size()));

				edgesToAvoid.clear();
				nightBehaviour.avoidParksWater = false;
				// originalRouteIndex is intentionally NOT advanced here: it is updated
				// organically by
				// syncOriginalRouteIndex() once the agent actually reaches the re-entry edge,
				// so that a
				// further reroute while still on this bypass still sees the full remaining
				// original route.
				resetPath(newPath);
				return;
			}
		}

		// No safe re-entry point anywhere ahead: keep to the current edge, just walk
		// faster.
		edgesToAvoid.clear();
		nightBehaviour.avoidParksWater = false;
		nightBehaviour.increaseSpeedAtNight = true;
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

	/** Defines the set of edges the agent should avoid during rerouting. */
	private void defineEdgesToAvoid() {
		edgesToAvoid.clear();
		edgesToAvoid.add(currentEdge);
		edgesToAvoid.addAll(SharedCognitiveMap.getEdgesNonLitNonCommunityKnown());

		if (agent.isVulnerableBoolean()) {
			edgesToAvoid.addAll(SharedCognitiveMap.getCommunityPrimalNetwork().getEdges());
			edgesToAvoid.removeAll(SharedCognitiveMap.getCommunityKnownEdges());

			Set<EdgeGraph> knownEdges = new HashSet<>(
					GraphUtils.getEdgesFromEdgeIDs(agent.getCognitiveMap().getAgentKnownEdges(), PedSimCity.edgesMap));
			edgesToAvoid.removeAll(knownEdges);
		}

		if (agent.isVulnerableBoolean() || nightBehaviour.avoidParksWater) {
			edgesToAvoid.addAll(SharedCognitiveMap.getEdgesWithinParksOrAlongWater());
		}

		// Never avoid edges incident to the destination, so the tail of the route stays
		// reachable.
		edgesToAvoid.removeAll(agent.destinationNode.getEdges());
	}

	/**
	 * Checks whether the agent may attempt a situated reroute.
	 *
	 * <p>
	 * This no longer gates on whether the agent is still on its original route: a
	 * local bypass may be attempted any number of times. The remaining constraints
	 * are purely structural — there is nothing to bypass into if the current edge
	 * leads straight to the destination, on the very first edge, or when no
	 * original route remains ahead to rejoin.
	 *
	 * @return true if the agent can reroute; false otherwise
	 */
	protected boolean canReroute() {
		return !currentEdge.getNodes().contains(agent.destinationNode) && indexOnSequence != 0
				&& hasRemainingOriginalRoute();
	}

	/**
	 * @return true if at least one edge of the original route still lies ahead of
	 *         the agent
	 */
	private boolean hasRemainingOriginalRoute() {
		return originalRouteIndex + 1 < originalEdgesSequence.size();
	}

	/**
	 * Checks if the edge is known by the agent.
	 *
	 * @param edge the edge to check
	 * @return true if the edge is known by the agent
	 */
	protected boolean isEdgeKnown(EdgeGraph edge) {
		return agent.getCognitiveMap().isEdgeKnown(edge);
	}

	/**
	 * @param route the route to validate
	 * @return true if {@code route} is non-null and contains at least one directed
	 *         edge
	 */
	private static boolean isValidRoute(Route route) {
		return route != null && route.directedEdgesSequence != null && !route.directedEdgesSequence.isEmpty();
	}
}