package pedsim.core.agents;

import java.util.Random;
import pedsim.core.parameters.RouteChoicePars;
import sim.graph.NodeGraph;

/**
 * HeuristicMixer: compute route-choice weights based on vividness and memory ability.
 */
public final class Heuristics {

  // Probabilities of activation minimisation
  private double probabilityDistanceMinimisation;
  private double probabilityAngularMinimisation;

  // Probabilities of activation elements
  private double probabilityDistantLandmarks;
  private double probabilityUsingRegions;
  private double probabilityBarrierSubGoals;

  private AgentProperties ap;
  private Agent agent;
  final Random random = new Random();

  public Heuristics(Agent agent) {
    this.agent = agent;
    this.ap = agent.getProperties();
  }

  public void defineHeuristic(NodeGraph originNode, NodeGraph destinationNode) {

    defineRouteChoiceMechanisms();
  }

  public double getLocallLandmarksThreshold() {
    return RouteChoicePars.localLandmarkThresholdCommunity;
  }

  // Public outputs you already had:
  private double globalLandmarknessWeightDistance =
      RouteChoicePars.globalLandmarknessWeightDistanceCommunity;
  private double globalLandmarknessWeightAngular =
      RouteChoicePars.globalLandmarknessWeightAngularCommunity;


  public double getGlobalLandmarkWeight(boolean angular) {
    if (angular)
      return globalLandmarknessWeightAngular;
    return globalLandmarknessWeightDistance;
  }

  /**
   * Randomly assigns route choice parameters to the agent based on its derived vividness profile
   * (probabilities).
   * 
   * Uses the probabilities for heuristics/sub-goals/landmarks/regions that were pre-computed by
   * HeuristicMixer. Ensures consistency with novice ↔ expert gradient.
   */
  public void defineRouteChoiceMechanisms() {

    ap.reset(); // clear flags
    double r = random.nextDouble();

    // --- Global Minimisation heuristics (rare, no usage of Urban Elements)
    if (probabilityDistanceMinimisation > 0.90 || probabilityAngularMinimisation > 0.90) {
      // One heuristic dominates strongly → force assignment
      ap.minimisingDistance = probabilityDistanceMinimisation > probabilityAngularMinimisation;
      ap.minimisingAngular = !ap.minimisingDistance;
      return;
    }

    // Otherwise set them as local Minimisation heuristics
    ap.localHeuristicDistance = r < probabilityDistanceMinimisation;
    ap.localHeuristicAngular = !ap.localHeuristicDistance;

    // --- Barrier sub-goals vs Local landmarks (mutually exclusive) ---
    double rBL = random.nextDouble();
    if (rBL < probabilityBarrierSubGoals) {
      ap.barrierBasedNavigation = true;
      ap.usingLocalLandmarks = false;
    } else {
      ap.usingLocalLandmarks = true;
      ap.barrierBasedNavigation = false;
    }

    // --- Distant/global landmarks (orientation anchors) ---
    ap.usingDistantLandmarks = random.nextDouble() < probabilityDistantLandmarks;

    // --- Region-based segmentation (independent switch) ---
    ap.regionBasedNavigation = random.nextDouble() < probabilityUsingRegions;
  }
}
