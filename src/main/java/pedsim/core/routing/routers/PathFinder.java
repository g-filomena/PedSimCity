package pedsim.core.routing.routers;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import org.locationtech.jts.planargraph.DirectedEdge;
import pedsim.core.agents.Agent;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.routing.search.DijkstraRoadDistance;
import sim.graph.EdgeGraph;
import sim.graph.Graph;
import sim.graph.NodeGraph;
import sim.routing.Route;

/**
 * The `PathFinder` class provides common functionality for computing navigation paths using various
 * algorithms and graph representations.
 */
public class PathFinder {

  protected Agent agent;
  protected Route route = new Route();

  protected Graph network = SharedCognitiveMap.getCommunityPrimalNetwork();
  protected NodeGraph originNode, destinationNode;
  protected NodeGraph tmpOrigin, tmpDestination;

  List<NodeGraph> sequenceNodes = new ArrayList<>();
  protected Set<DirectedEdge> directedEdgesToAvoid = new HashSet<>();

  protected List<DirectedEdge> completeSequence = new ArrayList<>();
  protected List<DirectedEdge> partialSequence = new ArrayList<>();

  protected boolean regionBased = false;
  boolean moveOn = false;

  /** Routes one leg of a sub-goal sequence, from {@code tmpOrigin} to {@code tmpDestination}. */
  @FunctionalInterface
  protected interface LegRouter {
    List<DirectedEdge> route();
  }

  /**
   * Walks a sequence of sub-goals, routing each leg with {@code legRouter} and accumulating the
   * result into {@code completeSequence}.
   *
   * <p>The loop is shared because the sub-goal sequence is the same idea whatever routes its legs:
   * take the waypoints in order, skip one already traversed, take a direct edge where there is one,
   * otherwise route to it and backtrack if that fails. Only the leg routing differs.
   *
   * @param sequence the sub-goals, origin first and destination last
   * @param agent the agent being routed
   * @param legRouter routes one leg between the current pair
   */
  protected void routeSequence(List<NodeGraph> sequence, Agent agent, LegRouter legRouter) {
    routeSequence(
        sequence,
        agent,
        legRouter,
        () -> {
          directedEdgesToAvoid = new HashSet<>(completeSequence);
          return new DijkstraRoadDistance()
              .dijkstraAlgorithm(
                  tmpOrigin, tmpDestination, destinationNode, directedEdgesToAvoid, agent);
        });
  }

  /**
   * As {@link #routeSequence(List, Agent, LegRouter)}, with {@code backtrackRouter} routing the leg
   * again from each node backtracking retreats to.
   */
  protected void routeSequence(
      List<NodeGraph> sequence, Agent agent, LegRouter legRouter, LegRouter backtrackRouter) {

    this.agent = agent;
    this.sequenceNodes = new ArrayList<>(sequence);

    originNode = this.sequenceNodes.get(0);
    tmpOrigin = originNode;
    destinationNode = sequence.get(sequence.size() - 1);
    this.sequenceNodes.remove(0);

    for (NodeGraph currentNode : this.sequenceNodes) {
      moveOn = false;
      tmpDestination = currentNode;

      if (nodesFromEdgesSequence(completeSequence).contains(tmpDestination)) {
        controlPath(tmpDestination);
        tmpOrigin = tmpDestination;
        continue;
      }

      if (haveEdgesBetween()) {
        continue;
      }

      partialSequence = legRouter.route();

      while (partialSequence.isEmpty() && !moveOn) {
        backtracking(tmpDestination, backtrackRouter);
      }

      if (moveOn) {
        // backtracking sets moveOn two ways. Reaching the origin means this sub-goal was skipped
        // and the agent has not moved, so tmpOrigin stays; finding a direct edge on the way back
        // means it has, and updateTmpOrigin has already moved it.
        if (tmpOrigin != originNode) {
          tmpOrigin = tmpDestination;
        }
        continue;
      }

      // The node the leg starts from: checkEdgesSequence walks forward from it and flips any edge
      // the search returned reversed, so handing it the leg's far end corrects nothing and
      // corrupts the rest.
      checkEdgesSequence(tmpOrigin);
      completeSequence.addAll(partialSequence);
      tmpOrigin = tmpDestination;
    }

    completeSequence = sequenceOnCommunityNetwork(completeSequence);
  }

  /**
   * Performs backtracking to compute a path in a primal graph from the current temporary origin
   * node to the given temporary destination node. If the temporary origin node is the same as the
   * original origin node, it attempts to skip the temporary destination. Otherwise, it updates the
   * temporary origin node, checks for the existence of a direct segment between the new origin and
   * the destination, and if not found, computes a path from the new origin to the destination while
   * avoiding specified segments.
   *
   * @param tmpDestination The temporary destination node.
   * @param backtrackRouter Routes the leg from the new temporary origin.
   */
  protected void backtracking(NodeGraph tmpDestination, LegRouter backtrackRouter) {

    if (tmpOrigin.equals(originNode)) {
      // try skipping this tmpDestination
      moveOn = true;
      return;
    }
    // determine new tmpOrigin
    updateTmpOrigin();

    // check if there's a segment between the new tmpOrigin and the destination
    final DirectedEdge edge = network.getDirectedEdgeBetween(tmpOrigin, tmpDestination);
    if (edge != null) {
      if (!completeSequence.contains(edge)) {
        completeSequence.add(edge);
      }
      moveOn = true; // No need to backtrack anymore
      return;
    }

    // If not, try to compute the path from the new tmpOrigin
    partialSequence = backtrackRouter.route();
  }

  /**
   * Updates the temporary origin node based on the current state of the path. If there are fewer
   * than two segments in the complete sequence, it clears the sequence and sets the temporary
   * origin to the original origin node. Otherwise, it removes the last problematic segment from the
   * complete sequence and updates the temporary origin accordingly.
   */
  private void updateTmpOrigin() {
    if (completeSequence.size() < 2) {
      completeSequence.clear();
      tmpOrigin = originNode;
    } else {
      // remove the last problematic segment
      completeSequence.remove(completeSequence.size() - 1);
      tmpOrigin = (NodeGraph) completeSequence.get(completeSequence.size() - 1).getToNode();
    }
  }

  /**
   * Cuts the walk so far at the first point it reaches {@code destinationNode}, a sub-goal it has
   * already passed.
   *
   * @param destinationNode The sub-goal already traversed.
   */
  protected void controlPath(NodeGraph destinationNode) {
    for (final DirectedEdge directedEdge : completeSequence) {
      if (directedEdge.getToNode().equals(destinationNode)) {
        int lastIndex = completeSequence.indexOf(directedEdge);
        completeSequence = new ArrayList<>(completeSequence.subList(0, lastIndex + 1));
        return;
      }
    }
  }

  /**
   * Reorders the sequence of DirectedEdges based on their fromNode and toNode, ensuring that they
   * follow a consistent order. This method is used to correct the sequence of DirectedEdges when
   * performing region-based navigation to ensure a smooth path.
   *
   * @param tmpOrigin The examined primal origin node.
   */
  protected void checkEdgesSequence(NodeGraph tmpOrigin) {
    NodeGraph previousNode = tmpOrigin;

    final List<DirectedEdge> copyPartial = new ArrayList<>(partialSequence);
    for (final DirectedEdge edge : copyPartial) {
      NodeGraph nextNode = (NodeGraph) edge.getToNode();
      // need to swap
      if (nextNode.equals(previousNode)) {
        nextNode = (NodeGraph) edge.getFromNode();
        // the same street the other way round; a lookup by the node pair could answer a parallel
        // street instead
        DirectedEdge correctEdge = edge.getSym();
        partialSequence.set(partialSequence.indexOf(edge), correctEdge);
      }
      previousNode = nextNode;
    }
  }

  /**
   * Checks if there are directed edges between two nodes.
   *
   * @return true if edges exist, false otherwise.
   */
  protected boolean haveEdgesBetween() {
    // check if edge in between
    DirectedEdge edge = network.getDirectedEdgeBetween(tmpOrigin, tmpDestination);

    if (edge == null) {
      return false;
    }
    if (!completeSequence.contains(edge)) {
      completeSequence.add(edge);
    }
    tmpOrigin = tmpDestination;
    return true;
  }

  protected List<DirectedEdge> sequenceOnCommunityNetwork(List<DirectedEdge> partialSequence) {

    List<DirectedEdge> newSequence = new ArrayList<>();
    for (DirectedEdge directedEdge : partialSequence) {
      EdgeGraph edge = (EdgeGraph) directedEdge.getEdge();
      // EdgeGraph parentEdge = agentNetwork.getParentEdge(edge);
      if (edge != null) {
        if (edge.getDirEdge(0).getCoordinate().equals(directedEdge.getCoordinate())) {
          newSequence.add(edge.getDirEdge(0));
        } else {
          newSequence.add(edge.getDirEdge(1));
        }
      }
    }
    return newSequence;
  }

  public List<NodeGraph> nodesFromEdgesSequence(List<DirectedEdge> directedEdgesSequence) {

    List<NodeGraph> nodes = new ArrayList<>();
    if (directedEdgesSequence.isEmpty()) {
      return nodes;
    }

    nodes =
        directedEdgesSequence.stream()
            .map(directedEdge -> ((EdgeGraph) directedEdge.getEdge()).getFromNode())
            .collect(Collectors.toList());

    EdgeGraph lastEdge =
        (EdgeGraph) directedEdgesSequence.get(directedEdgesSequence.size() - 1).getEdge();
    nodes.add(lastEdge.getToNode());

    return nodes;
  }
}
