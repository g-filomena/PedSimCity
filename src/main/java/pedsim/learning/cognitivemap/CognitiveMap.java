package pedsim.learning.cognitivemap;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.locationtech.jts.geom.Coordinate;
import pedsim.core.agents.Agent;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.learning.agents.LearningAgent;
import pedsim.learning.parameters.LearningPars;
import sim.graph.EdgeGraph;
import sim.graph.NodeGraph;

/**
 * Incremental, individual cognitive map for the learning model.
 *
 * <p>Extends the core {@link pedsim.core.cognition.cognitivemap.CognitiveMap} - which already
 * provides the activity bone, the known-network construction, and {@code
 * readjustCognitiveMap(List&lt;Polygon&gt;)} (reading home/work from the agent) - and adds the
 * learning-specific layer that core leaves out because core agents do not learn:
 *
 * <ul>
 *   <li>after the base map is formed, the agent's route memory is seeded from the routes between
 *       its cognitive anchors via {@code IncrementalLearning.buildBasicMemory()};
 *   <li><b>adaptive recognition</b>: the local-landmark threshold and the wayfinding-easiness
 *       threshold, which core fixes at the community's value and at 0, are driven by the agent's
 *       <em>vividness</em> via {@link ElementsSensitivity} - novices need salient landmarks and easy
 *       routes, experts tolerate subtler landmarks and harder routes.
 * </ul>
 *
 * <p>Vividness is grid-based ({@link VividnessGrid#vividnessBetween}) once the agent has
 * accumulated route memory, blended with the stable per-agent {@code spatialAbility}; before any
 * memory exists it falls back to {@code spatialAbility} alone.
 */
public class CognitiveMap extends pedsim.core.cognition.cognitivemap.CognitiveMap {

  /** Maps vividness to landmark/easiness thresholds (novice → expert shaping). */
  private final ElementsSensitivity sensitivity = new ElementsSensitivity();

  private final RecognitionStats localLandmarkStats = new RecognitionStats();
  private final RecognitionStats easinessStats = new RecognitionStats();

  /**
   * The thresholds this map has handed out since the last call, with the vividness they followed;
   * the counts start again from zero.
   */
  public RecognitionStats drainLocalLandmarkStats() {
    return localLandmarkStats.drain();
  }

  /** As {@link #drainLocalLandmarkStats()}, for the wayfinding-easiness threshold. */
  public RecognitionStats drainEasinessStats() {
    return easinessStats.drain();
  }

  public CognitiveMap(Agent agent) {
    super(agent);
  }

  /** The share of the community's known network this agent starts with; 1 is all of it. */
  private double communityShare = 1.0;

  /**
   * With a radius above 0, the agent starts knowing the community streets within it of an anchor
   * instead of a share of them.
   */
  private double communityRadius = 0.0;

  private Set<Integer> keptCommunityNodes;
  private Set<Integer> keptCommunityEdges;

  /** The streets the agent knew from the start and never forgets: its bone and its community share. */
  private Set<Integer> baseKnownEdges = new HashSet<>();

  /**
   * Sets the share of the community's known network this agent knows, the streets nearest its
   * anchors first. Below 1 the agent does not know the city-centre regions either.
   */
  public void setCommunityShare(double share) {
    this.communityShare = Math.max(0.0, Math.min(1.0, share));
    keptCommunityNodes = null;
    keptCommunityEdges = null;
  }

  /**
   * Makes the agent start knowing the community's streets within {@code metres} of an anchor, and
   * nothing else of the community's network, the city centre included.
   */
  public void setCommunityRadius(double metres) {
    this.communityShare = 0.0;
    this.communityRadius = Math.max(0.0, metres);
    keptCommunityNodes = null;
    keptCommunityEdges = null;
  }

  @Override
  protected boolean knowsCityCentre() {
    return communityShare >= 1.0;
  }

  @Override
  protected void fuseBoneWithCommunityNetwork() {
    baseKnownEdges = new HashSet<>(activityBoneEdges);
    if (communityShare >= 1.0) {
      super.fuseBoneWithCommunityNetwork();
      baseKnownEdges.addAll(
          sim.graph.GraphUtils.getEdgeIDs(SharedCognitiveMap.getCommunityKnownEdges()));
      return;
    }
    if (keptCommunityEdges == null) {
      keepNearestCommunity();
    }
    baseKnownEdges.addAll(keptCommunityEdges);
    agentKnownNodes.addAll(activityBoneNodes);
    agentKnownEdges.addAll(activityBoneEdges);
    agentKnownNodes.addAll(keptCommunityNodes);
    agentKnownEdges.addAll(keptCommunityEdges);
  }

  /**
   * Picks the community edges this agent knows: the {@code communityShare} of them nearest to its
   * anchors, by the straight line from an edge's centre to the closest anchor.
   */
  private void keepNearestCommunity() {
    List<EdgeGraph> edges = new ArrayList<>(SharedCognitiveMap.getCommunityKnownEdges());
    edges.sort(Comparator.comparingInt(EdgeGraph::getID));
    double[] distance = new double[edges.size()];
    for (int i = 0; i < distance.length; i++) {
      Coordinate centre = edges.get(i).getLine().getEnvelopeInternal().centre();
      double nearest = Double.POSITIVE_INFINITY;
      for (NodeGraph anchor : anchors) {
        nearest = Math.min(nearest, anchor.getCoordinate().distance(centre));
      }
      distance[i] = nearest;
    }
    Integer[] order = new Integer[distance.length];
    for (int i = 0; i < order.length; i++) {
      order[i] = i;
    }
    java.util.Arrays.sort(order, Comparator.comparingDouble(i -> distance[i]));
    int keep = (int) Math.ceil(communityShare * order.length);
    if (communityRadius > 0.0) {
      keep = 0;
      while (keep < order.length && distance[order[keep]] <= communityRadius) {
        keep++;
      }
    }
    keptCommunityEdges = new HashSet<>();
    keptCommunityNodes = new HashSet<>();
    for (int k = 0; k < keep; k++) {
      EdgeGraph edge = edges.get(order[k]);
      keptCommunityEdges.add(edge.getID());
      keptCommunityNodes.add(edge.getFromNode().getID());
      keptCommunityNodes.add(edge.getToNode().getID());
    }
  }

  @Override
  public void copyKnowledgeFrom(pedsim.core.cognition.cognitivemap.CognitiveMap other) {
    super.copyKnowledgeFrom(other);
    if (other instanceof CognitiveMap learning) {
      baseKnownEdges = new HashSet<>(learning.baseKnownEdges);
    }
  }

  /**
   * How familiar a street is to the agent, from 0 to 1. 1 for a street it knew from the start;
   * otherwise the mean over points along it of {@code w / (w + memoryThreshold)}, {@code w} being
   * the strength in walks of the vividness-grid cell the point is in.
   */
  public double familiarity(EdgeGraph edge) {
    if (baseKnownEdges.contains(edge.getID())) {
      return 1.0;
    }
    VividnessGrid grid = grid();
    if (grid == null) {
      return 0.0;
    }
    var line = edge.getLine();
    double length = line.getLength();
    int steps = (int) Math.max(1, Math.min(6, Math.ceil(length / 25.0)));
    var indexed = new org.locationtech.jts.linearref.LengthIndexedLine(line);
    double sum = 0.0;
    for (int i = 0; i <= steps; i++) {
      Coordinate point = indexed.extractPoint(length * i / steps);
      double w = grid.getValueAt(point.x, point.y);
      sum += w / (w + LearningPars.memoryThreshold);
    }
    return sum / (steps + 1);
  }

  /**
   * Whether the agent knows of the place at a node: it is in the space the agent knows, or a cell
   * of the vividness grid around it holds at least {@code LearningPars.awarenessThreshold} walks.
   */
  public boolean isAwareOf(NodeGraph node) {
    if (getAgentKnownNodes().contains(node.getID())) {
      return true;
    }
    VividnessGrid grid = grid();
    return grid != null
        && grid.getValueAt(node.getCoordinate().x, node.getCoordinate().y)
            >= LearningPars.awarenessThreshold;
  }

  private VividnessGrid grid() {
    return agent instanceof LearningAgent learning && learning.learning != null
        ? learning.learning.getVividnessGrid()
        : null;
  }

  /**
   * Forms the base cognitive map (core), seeds the agent's initial route memory, and hands what it
   * now knows to its shadow, if it has one.
   */
  @Override
  public void formCognitiveMap() {
    super.formCognitiveMap();
    if (agent instanceof LearningAgent learningAgent) {
      learningAgent.learning.buildBasicMemory();
      learningAgent.startShadow();
    }
  }

  /**
   * Effective vividness in [0,1] for a specific origin–destination, blending the agent's accumulated
   * route-memory grid with its stable spatial ability. Falls back to {@code spatialAbility} when no
   * route memory has been built yet.
   */
  public double effectiveVividness(NodeGraph originNode, NodeGraph destinationNode) {
    double ability = clamp01(spatialAbility);
    if (agent instanceof LearningAgent learningAgent && learningAgent.learning != null) {
      VividnessGrid grid = learningAgent.learning.getVividnessGrid();
      if (grid != null) {
        double space =
            grid.vividnessBetween(originNode.getCoordinate(), destinationNode.getCoordinate());
        return VividnessGrid.effectiveVividness(space, ability);
      }
    }
    return ability;
  }

  /**
   * Local-landmark recognition threshold for a route between two places, driven by the agent's
   * vividness between them.
   */
  @Override
  public double getLocalLandmarkThreshold(NodeGraph originNode, NodeGraph destinationNode) {
    double vividness = effectiveVividness(originNode, destinationNode);
    double threshold = sensitivity.calculateLocalLandmarksThreshold(vividness);
    localLandmarkStats.record(vividness, threshold);
    return threshold;
  }

  /**
   * Wayfinding-easiness threshold for a space: novices of it require easy routes (high threshold),
   * those who know it well tolerate harder ones (low threshold).
   */
  @Override
  public double getWayfindingEasinessThreshold(NodeGraph from, NodeGraph to, boolean regionBased) {
    double vividness = effectiveVividness(from, to);
    double threshold = ElementsSensitivity.deriveWayfindingEasinessThreshold(vividness);
    easinessStats.record(vividness, threshold);
    return threshold;
  }

  private static double clamp01(double v) {
    return Math.max(0.0, Math.min(1.0, v));
  }
}
