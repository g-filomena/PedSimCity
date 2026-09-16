package pedsim.night.routing.search;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.locationtech.jts.planargraph.DirectedEdge;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.routing.search.DijkstraRoadDistance;
import pedsim.night.agents.NightAgent;
import pedsim.night.engine.NightLighting;
import pedsim.night.engine.PedSimCityNight;
import pedsim.night.parameters.NightPars;
import sim.graph.EdgeGraph;
import sim.graph.NodeGraph;

/**
 * Road-distance shortest path for night-time routing.
 *
 * <p>Resolution proceeds as a graded fallback so that constraints are relaxed only as far as
 * necessary:
 * <ol>
 *   <li><b>First attempt</b>: avoid parks/water <em>and</em> edges in unknown regions.
 *   <li><b>Second attempt</b>: relax the unknown-region constraint but keep avoiding parks/water.
 *   <li><b>Final fallback</b>: an unconstrained {@link DijkstraRoadDistance} shortest path.
 * </ol>
 *
 * <p>Both constrained attempts apply only to vulnerable agents; for non-vulnerable agents the
 * filter is a no-op and night avoidance is left to situated navigation.
 *
 * <p><b>Darkness reaches the plan here, and only here.</b> Every other lighting rule in this module
 * fires once the agent is standing on the edge; {@link #lightingCostMultiplier} is what lets it
 * prefer a lit way round before setting off. It applies to <b>known</b> edges only, so planning and
 * situated reaction cannot charge for the same darkness twice.
 *
 * <p><b>Deliberately narrower than the destination rule.</b>
 * {@code NightAgent.chooseDestinationAvoidingParksAfterDark} refuses a park or waterside
 * destination for every night agent after dark, while the avoidance here gates on vulnerability.
 * Walking past a dark park and choosing to spend the evening in one are different decisions, so
 * they share a rule and not a gate. See that method for the other half of the reasoning.
 *
 * ## Possible to do - add agents filter out zero light edges but add look up table so when they
 * arrive at a node and the start of an edge is dark they reroute
 */
public class DijkstraRoadDistanceNight extends DijkstraRoadDistance {

  public Set<NodeGraph> disregardedNodes = new HashSet<>();
  protected boolean secondAttempt;

  /**
   * Finds the minimum distances for adjacent nodes in the primal graph.
   *
   * <p>The neighbour filter is applied on every attempt; the degree of relaxation is governed by
   * {@link #secondAttempt} via {@link #shouldAvoidEdgeAtNight(EdgeGraph, boolean)}.
   *
   * <p>Performance: neighbours are reached by iterating the node's own outgoing directed edges,
   * which yields the target node, the undirected edge and the directed edge in one object — no
   * {@code getEdgeBetween}/{@code getDirectedEdgeBetween} map lookups are needed. The cost noise
   * comes from the job's own seeded RNG (see {@code Dijkstra.drawFromDistribution}) instead of the
   * shared static generator in GeoMason, and at {@code RouteChoicePars.perceptionErrorSD}, which is
   * the sigma a pinned comparison sets to zero.
   *
   * <p><b>Barrier preferences are not part of route choice in this module, and the decision is
   * made upstream.</b> Core, activity, night and learning agents take their model from {@link
   * pedsim.core.agents.Heuristics}, which builds every one with {@code BarrierPreferences.NONE} -
   * so for a night agent {@code costPerceptionError} has no barrier branch to take and already
   * reduces to the plain perception error. Barrier perception is an OD-module mechanism: cityImage
   * configures it per scenario and empirical draws it from a survey cluster, and neither kind of
   * agent walks after dark.
   *
   * <p>It asks {@code costPerceptionError} rather than drawing the error itself so that the sigma
   * is {@code RouteChoicePars.perceptionErrorSD} and nothing else. That is what
   * {@code --perceptionErrorSD=0} pins, and a paired A/B whose one manipulated variable is lighting
   * is exactly where an unpinned draw does the most damage.
   *
   * @param currentNode the current node in the primal graph
   */
  @Override
  protected void findMinDistances(NodeGraph currentNode) {
    boolean anyValidNeighbour = false;

    for (DirectedEdge outEdge : currentNode.getOutDirectedEdges()) {
      NodeGraph targetNode = (NodeGraph) outEdge.getToNode();
      EdgeGraph commonEdge = (EdgeGraph) outEdge.getEdge();
      if (!canMoveToNodeAtNight(targetNode, commonEdge)) {
        continue;
      }
      anyValidNeighbour = true;

      // The parent's three skips, and they come *after* anyValidNeighbour on purpose: a node is
      // disregarded when night's own constraints leave it no way out, not when its neighbours
      // happen to be settled already. Each is a no-op on today's call path - the search settles
      // every node once, night agents are not individualised, and the three-argument entry builds
      // no avoid-set - and each stops this override from meaning something different from its
      // parent the day one of those stops being true.
      if (visitedNodes.contains(targetNode)
          || (restrictToKnownNetwork() && !isEdgeKnown(commonEdge))
          || edgesToAvoid.contains(commonEdge)) {
        continue;
      }

      double error = costPerceptionError(targetNode, commonEdge, false);
      double edgeCost = commonEdge.getLength() * error * lightingCostMultiplier(commonEdge);
      computeTentativeCost(currentNode, targetNode, edgeCost);
      isBest(currentNode, targetNode, outEdge);
    }

    if (!anyValidNeighbour) {
      disregardedNodes.add(currentNode);
    }
  }

  /**
   * Planning cost multiplier for a <b>known</b> edge, by how far its measured {@code mean_lux}
   * falls below the travelling agent's own sensitivity threshold: 1.0 at or above it, rising
   * linearly toward {@link NightPars#maxKnownDarkEdgeCostMultiplier} at total darkness.
   *
   * <p>1.0 - no penalty - in daylight, for a caller that is not a {@link NightAgent}, for an edge
   * the agent does not know, and for an edge with no continuous lux reading. The agent's own
   * threshold does the vulnerable/non-vulnerable split, since a vulnerable agent already draws a
   * higher one.
   *
   * <p><b>Only while it is dark</b>, by the same {@code isDark} the situated gate in
   * {@code NightAgentMovement.setupEdge} consults - a night agent's day is a whole 24 hours and it
   * plans plenty of trips in daylight, where how brightly a street is lit decides nothing. Without
   * this the module's one planning rule would be the only lighting rule in it that fired at noon.
   *
   * <p><b>Known edges only, deliberately.</b> An agent reacts to an unknown dark street when it
   * reaches one ({@code NightBehaviour}); charging for it here as well would price the same
   * darkness twice, once in the plan and once in the reaction. A night agent's known edges are its
   * simple activity bone - the home and work regions - so this bites there and nowhere else.
   */
  private double lightingCostMultiplier(EdgeGraph edge) {
    if (!(agent instanceof NightAgent nightAgent)
        || !(nightAgent.getState() instanceof PedSimCityNight night)
        || !night.isDark
        || !nightAgent.getCognitiveMap().isEdgeKnown(edge)) {
      return 1.0;
    }
    double depth = NightLighting.darknessDepth(edge, nightAgent.lightSensitivityThreshold);
    return 1.0 + (NightPars.maxKnownDarkEdgeCostMultiplier - 1.0) * depth;
  }

  /**
   * Determines whether an edge should be avoided at night.
   *
   * <p>The {@code secondAttempt} term is tested first so that, on the relaxed attempt, the
   * region-knowledge lookup is short-circuited away entirely.
   *
   * @param edge the edge to evaluate
   * @param secondAttempt if true, relaxes the unknown-region avoidance criterion (parks/water are
   *     still avoided)
   * @return true if the edge should be avoided at night
   */
  protected boolean shouldAvoidEdgeAtNight(EdgeGraph edge, boolean secondAttempt) {
    if (edge.getNodes().contains(destinationNode)) {
      return false;
    }
    return SharedCognitiveMap.getEdgesWithinParksOrAlongWater().contains(edge)
        || (!secondAttempt && !isRegionKnown(edge.getRegionID()));
  }

  /**
   * @param targetNode the candidate neighbour
   * @param edge the already-resolved edge between the current node and {@code targetNode}
   * @return true if the agent may move onto {@code targetNode} at night
   */
  private boolean canMoveToNodeAtNight(NodeGraph targetNode, EdgeGraph edge) {
    return (!agent.isVulnerable() || !shouldAvoidEdgeAtNight(edge, secondAttempt))
        && !disregardedNodes.contains(targetNode);
  }

  /**
   * Reconstructs the sequence of directed edges composing the path.
   *
   * <p>Performance: each predecessor wrapper is fetched once per step (instead of three map
   * lookups), and edges are appended then reversed once, avoiding the O(n^2) cost of repeated
   * head insertions on an {@link ArrayList}.
   *
   * @return the reconstructed directed-edge sequence
   */
  @Override
  protected List<DirectedEdge> reconstructSequence() {
    List<DirectedEdge> directedEdgesSequence = new ArrayList<>();
    NodeGraph step = destinationNode;

    if (nodeWrappersMap.get(destinationNode) != null && nodeWrappersMap.size() > 1) {
      while (true) {
        var wrapper = nodeWrappersMap.get(step);
        if (wrapper.nodeFrom == null) {
          break;
        }
        directedEdgesSequence.add(wrapper.directedEdgeFrom);
        step = wrapper.nodeFrom;
      }
      Collections.reverse(directedEdgesSequence);
    }

    if (directedEdgesSequence.isEmpty()) {
      if (!secondAttempt) {
        secondAttempt = true;
        // The relaxed attempt must not inherit dead-ends pruned under the stricter constraints.
        disregardedNodes.clear();
        directedEdgesSequence = dijkstraAlgorithm(originNode, destinationNode, agent);
      }
      if (directedEdgesSequence.isEmpty()) {
        directedEdgesSequence =
            new DijkstraRoadDistance().dijkstraAlgorithm(originNode, destinationNode, agent);
      }
    }
    return directedEdgesSequence;
  }
}
