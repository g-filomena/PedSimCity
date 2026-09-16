package pedsim.core.routing.routers;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.locationtech.jts.planargraph.DirectedEdge;
import pedsim.core.agents.Agent;
import pedsim.core.engine.PedSimCity;
import pedsim.core.routing.search.DijkstraAngularChange;
import sim.graph.NodeGraph;
import sim.routing.Route;
import sim.routing.RoutingUtils;

/**
 * Router for least-cumulative-angular-change route calculations.
 * This class extends the functionality of the base class PathFinder.
 */
public class AngularChangePathFinder extends PathFinder {

  /**
   * Formulates the least cumulative angular change shortest path between an
   * origin and a destination node.
   *
   * @param originNode      The origin node for the route.
   * @param destinationNode The destination node for the route.
   * @param agent           The agent for which the route is completed.
   * @return A Route object representing the calculated route based on angular
   *         change.
   */
  public Route angularChangeBased(NodeGraph originNode, NodeGraph destinationNode, Agent agent) {

    if (agent != null && agent.getState() != null) {
      agent.getState().trace().recordAngularAttempt();
    }
    this.agent = agent;
    previousJunction = null;

    // Origin and destination joined by a single street: that is the route, and it has no turn in it
    // for the angular model to weigh. Handled here rather than falling out of the candidate loops,
    // because the pair of centroids that would express it is the same centroid on both sides - it
    // is incident to both endpoints - and the loops skip a pair of identical centroids as
    // degenerate. A destination with only this edge incident, a dead end, therefore produced no
    // candidate pair at all and was served by the distance fallback and counted as one.
    DirectedEdge directEdge = network.getDirectedEdgeBetween(originNode, destinationNode);
    if (directEdge != null) {
      List<DirectedEdge> single = new ArrayList<>();
      single.add(directEdge);
      route.directedEdgesSequence = single;
      route.computeRouteSequences();
      return route;
    }

    // Every centroid incident on each endpoint, best-aligned first, rather than only the single
    // best-aligned one. The dual search is confined to the agent's known dual subgraph while the
    // centroid was chosen from the geometry of the street, so the one centroid geometry prefers is
    // regularly not in that subgraph and no path can exist - 92 of 420 angular routes a day on
    // full Torino. getDualNodes says as much itself: "when computing paths within subgraphs, some
    // specific segments may be indeed unreachable". The retry loop this replaces called the
    // singular getDualNode up to a hundred times with identical arguments; it is a minimum over
    // incident edges with no randomness in it, so all hundred calls returned the same centroid.
    List<NodeGraph> dualOrigins =
        new ArrayList<>(
            originNode.getDualNodes(originNode, destinationNode, false, previousJunction).keySet());
    List<NodeGraph> dualDestinations =
        new ArrayList<>(
            destinationNode
                .getDualNodes(originNode, destinationNode, false, previousJunction)
                .keySet());
    if (dualOrigins.isEmpty() || dualDestinations.isEmpty()) {
      return route; // no dual representation at an endpoint; return empty route
    }

    hadDualPath = false;
    anyPairKnown = false;

    Route found = searchCandidates(originNode, destinationNode, dualOrigins, dualDestinations);
    if (found != null) {
      return found;
    }

    // Nothing the agent knows connects these centroids. Widen the search to the whole dual network
    // rather than swap the route-choice model: an agent who cannot get there through the turns it
    // knows still walks, and it still walks minimising angular change. Shortest path stays the last
    // resort, for when the city's own dual graph has no path either.
    if (agent != null
        && agent.getCognitiveMap() != null
        && agent.getCognitiveMap().individualised) {
      searchFullNetwork = true;
      found = searchCandidates(originNode, destinationNode, dualOrigins, dualDestinations);
      searchFullNetwork = false;
      if (found != null) {
        if (agent.getState() != null) {
          agent.getState().trace().recordFullNetworkEscalation(true);
        }
        return found;
      }
    }

    if (!anyPairKnown) {
      recordEndpointsUnknown(agent);
    }
    return distanceFallback(originNode, destinationNode, agent, hadDualPath);
  }

  /** Set while the candidate search is allowed to leave the agent's known dual network. */
  private boolean searchFullNetwork = false;

  /** Whether any candidate pair produced a dual path, before cleaning. */
  private boolean hadDualPath = false;

  /** Whether the agent knew both ends of at least one candidate pair. */
  private boolean anyPairKnown = false;

  /**
   * Tries each pair of candidate centroids, best-aligned first, and returns the first route that
   * survives cleaning.
   *
   * @return the route, or {@code null} if no pair yielded one.
   */
  private Route searchCandidates(
      NodeGraph originNode,
      NodeGraph destinationNode,
      List<NodeGraph> dualOrigins,
      List<NodeGraph> dualDestinations) {

    for (NodeGraph dualOrigin : dualOrigins) {
      for (NodeGraph dualDestination : dualDestinations) {
        if (dualDestination.equals(dualOrigin)) {
          continue;
        }
        anyPairKnown |= endpointsKnown(agent, dualOrigin, dualDestination);

        NodeGraph commonJunction = RoutingUtils.getPrimalJunction(dualOrigin, dualDestination);
        if (commonJunction != null) {
          List<DirectedEdge> shortcut =
              edgesViaCommonJunction(originNode, commonJunction, destinationNode);
          if (shortcut != null) {
            route.directedEdgesSequence = shortcut;
            // Without this the shortcut returns a route whose node and edge sequences were never
            // built - no origin, no destination, no line geometry, and a length of zero, which the
            // day trace then rejects. Every other exit from this class computes them.
            route.computeRouteSequences();
            return route;
          }
          // This network cannot represent the shortcut; fall through to the dual search, which may
          // still connect this pair of centroids the long way round.
        }

        DijkstraAngularChange dijkstra = new DijkstraAngularChange();
        if (searchFullNetwork) {
          dijkstra.ignoreKnownNetwork();
        }
        partialSequence =
            dijkstra.dijkstraAlgorithm(
                dualOrigin,
                dualDestination,
                destinationNode,
                new HashSet<>(centroidsToAvoid),
                previousJunction,
                agent);
        if (partialSequence.isEmpty()) {
          continue;
        }
        hadDualPath = true;
        cleanDualPath(originNode, destinationNode);
        partialSequence = sequenceOnCommunityNetwork(partialSequence);
        if (partialSequence.isEmpty()) {
          continue; // cleaning trimmed it away; try the next pair of centroids
        }
        route.directedEdgesSequence = partialSequence;
        route.computeRouteSequences();
        return route;
      }
    }
    return null;
  }

  /**
   * Whether the agent knows both ends of this candidate pair.
   *
   * <p>The search is confined to the agent's known dual subgraph while the candidates are chosen
   * from the geometry of the street, so a pair outside what the agent knows has no path by
   * construction however well connected the city's dual graph is. Reported when no candidate pair
   * was fully known, which is the case this diagnoses; an agent with no individualised dual map is
   * not restricted and so counts as knowing them.
   */
  private boolean endpointsKnown(Agent agent, NodeGraph dualOrigin, NodeGraph dualDest) {
    if (agent == null || agent.getState() == null || agent.getCognitiveMap() == null) {
      return true;
    }
    Set<NodeGraph> known = agent.getCognitiveMap().getNodesInKnownDualNetwork();
    if (known == null || known.isEmpty()) {
      return true;
    }
    return known.contains(dualOrigin) && known.contains(dualDest);
  }

  private void recordEndpointsUnknown(Agent agent) {
    if (agent != null && agent.getState() != null) {
      agent.getState().trace().recordAngularEndpointUnknown();
    }
  }

  /**
   * Shortest path instead, when no angular-change path survives.
   *
   * <p>Two ways to end up with nothing. The dual Dijkstra can find no path at all for any pair of
   * candidate centroids - the dual graph has components the primal graph does not, so a pair
   * connected on the street can be unreachable through turns. And {@code cleanDualPath} guards
   * {@code size < 2} but not {@code size == 2}: a two-edge path that is both "one edge ahead" and
   * carrying an unnecessary first edge loses both, which is why short trips were the ones that fell
   * over.
   *
   * <p>Handing the empty sequence on was a crash, not a degradation:
   * {@code Route.computeRouteSequences} reads {@code nodesSequence.get(0)} unguarded, so every run
   * on a city with a dual graph died the first time a pedestrian met this - which is every run on
   * the full Torino network. Returning an empty route instead would only move the failure into the
   * movement code, and the agent does still have somewhere to be, so it walks the shortest path and
   * the substitution is counted rather than hidden.
   */
  private Route distanceFallback(
      NodeGraph originNode, NodeGraph destinationNode, Agent agent, boolean trimmed) {
    PedSimCity state = agent.getState();
    if (state != null) {
      state.trace().recordAngularFallback(trimmed);
    }
    return new RoadDistancePathFinder().roadDistance(originNode, destinationNode, agent);
  }

  /**
   * Formulates the least cumulative angular change path through a sequence of
   * intermediate nodes [originNode, ..., destinationNode] using the provided
   * agent properties. It allows combining the angular-change local minimisation
   * heuristic with navigational strategies based on the usage of urban elements.
   *
   * @param sequenceNodes A list of nodes representing intermediate nodes.
   * @param agent         The agent for which the route is completed.
   * @return A Route object representing the calculated sequence of routes based
   *         on angular change.
   */
  public Route angularChangeBasedSequence(List<NodeGraph> sequenceNodes, Agent agent) {

    if (agent != null && agent.getState() != null) {
      agent.getState().trace().recordAngularAttempt();
    }
    this.agent = agent;
    this.regionBased = agent.getProperties().isRegionBasedNavigation();
    this.sequenceNodes = new ArrayList<>(sequenceNodes);

    originNode = sequenceNodes.get(0);
    tmpOrigin = originNode;
    destinationNode = sequenceNodes.get(this.sequenceNodes.size() - 1);
    this.sequenceNodes.remove(0);

    for (final NodeGraph currentNode : this.sequenceNodes) {

      moveOn = false; // for path cleaning and already traversed edges
      tmpDestination = currentNode;
      partialSequence = new ArrayList<>();

      if (tmpOrigin != originNode) {
        centroidsToAvoid = RoutingUtils.getCentroidsFromEdgesSequence(completeSequence);
        previousJunction = RoutingUtils.getPreviousJunction(completeSequence);

        // check if tmpDestination traversed already
        if (nodesFromEdgesSequence(completeSequence).contains(tmpDestination)) {
          controlPath(tmpDestination);
          tmpOrigin = tmpDestination;
          continue;
        }
      }
      // check if edge in between
      if (haveEdgesBetween()) {
        continue;
      }

      List<NodeGraph> dualNodesOrigin = getDualNodes(tmpOrigin, previousJunction);
      List<NodeGraph> dualNodesDestination = getDualNodes(tmpDestination, null);

      for (NodeGraph tmpDualOrigin : dualNodesOrigin) {
        for (NodeGraph tmpDualDestination : dualNodesDestination) {
          // check if just one node separates them
          NodeGraph commonJunction =
              RoutingUtils.getPrimalJunction(tmpDualOrigin, tmpDualDestination);

          if (commonJunction != null) {
            addEdgesCommonJunction(commonJunction);
          } else {
            final DijkstraAngularChange search = new DijkstraAngularChange();
            Set<NodeGraph> centroidsToAvoidSet = new HashSet<>(centroidsToAvoid);
            partialSequence =
                search.dijkstraAlgorithm(
                    tmpDualOrigin,
                    tmpDualDestination,
                    destinationNode,
                    centroidsToAvoidSet,
                    tmpOrigin,
                    agent);
          }
          if (!partialSequence.isEmpty()) {
            break;
          }
        }
        if (!partialSequence.isEmpty()) {
          break;
        }
      }

      while (partialSequence.isEmpty() && !moveOn) {
        dualBacktracking();
      }
      if (moveOn) {
        tmpOrigin = tmpDestination;
        continue;
      }
      cleanDualPath(tmpOrigin, tmpDestination);
      completeSequence.addAll(partialSequence);
      tmpOrigin = tmpDestination;
    }
    completeSequence = sequenceOnCommunityNetwork(completeSequence);
    if (completeSequence.isEmpty()) {
      return distanceFallback(originNode, destinationNode, agent, false);
    }
    route.directedEdgesSequence = completeSequence;
    route.computeRouteSequences();
    return route;
  }

  /**
   * Adds directed primal edges between the current origin and the common junction
   * node, and between the common junction and the current destination to the
   * partial sequence.
   *
   * @param commonJunction The common junction node between the origin and
   *                       destination.
   */
  private void addEdgesCommonJunction(NodeGraph commonJunction) {
    List<DirectedEdge> edges = edgesViaCommonJunction(tmpOrigin, commonJunction, tmpDestination);
    if (edges == null) {
      return; // leaves partialSequence empty, so the caller tries the next pair of centroids
    }
    partialSequence.addAll(edges);
  }

  /**
   * The one or two primal edges joining origin to destination through the junction their two
   * centroids share, or {@code null} when this network cannot represent that walk.
   *
   * <p>{@code getPrimalJunction} returns the endpoint the two primal edges have <i>in common</i>,
   * and that endpoint is regularly the origin or the destination itself - when one centroid already
   * spans the whole trip, the "junction" is simply the far end of it. {@code getDirectedEdgeBetween}
   * is then asked for an edge from a node to itself and answers {@code null}, which used to go
   * straight into the route and surface much later as a {@code NullPointerException} inside
   * {@code Route.nodesSequence}. The walk in that case is a <i>single</i> edge, not two.
   *
   * <p>That is the only way a {@code null} arises on today's call paths, and the deduction is worth
   * keeping: {@code network} is {@code SharedCognitiveMap.getCommunityPrimalNetwork()}, which is
   * assigned {@code PedSimCity.network} - the whole primal graph, not a subset - and
   * {@code getDualNodes} iterates the node's own incident edges, so any junction other than the
   * endpoint itself is the far end of an edge that exists. The null return below is therefore a
   * guard, not a case anything currently reaches; {@code network} is a seam, and pointing it at a
   * real subgraph would make it live. Do not cite it as an explanation for a failure.
   */
  private List<DirectedEdge> edgesViaCommonJunction(
      NodeGraph originNode, NodeGraph commonJunction, NodeGraph destinationNode) {

    List<DirectedEdge> sequence = new ArrayList<>();
    if (!commonJunction.equals(originNode)) {
      DirectedEdge first = network.getDirectedEdgeBetween(originNode, commonJunction);
      if (first == null) {
        return null;
      }
      sequence.add(first);
    }
    if (!commonJunction.equals(destinationNode)) {
      DirectedEdge second = network.getDirectedEdgeBetween(commonJunction, destinationNode);
      if (second == null) {
        return null;
      }
      sequence.add(second);
    }
    // Both equal means origin and destination are the same node: no walk at all.
    return sequence.isEmpty() ? null : sequence;
  }
}
