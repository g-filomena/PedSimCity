package pedsim.core.agents;

import java.util.Objects;
import java.util.Random;
import pedsim.core.engine.PedSimCity;
import pedsim.core.parameters.RouteChoicePars;
import pedsim.core.utilities.StringEnum.LocalHeuristicMode;
import pedsim.core.utilities.StringEnum.MinimisationMode;
import pedsim.core.utilities.StringEnum.RouteChoiceElement;
import sim.graph.NodeGraph;

public final class Heuristics {

  private double probabilityDistanceMinimisation;
  private double probabilityAngularMinimisation;
  private double probabilityDistantLandmarks;
  private double probabilityUsingRegions;
  private double probabilityBarrierSubGoals;

  private final Agent agent;
  private final AgentProperties ap;
  private final Random random;

  private final double globalLandmarknessWeightDistance =
      RouteChoicePars.globalLandmarknessWeightDistanceCommunity;
  private final double globalLandmarknessWeightAngular =
      RouteChoicePars.globalLandmarknessWeightAngularCommunity;

  public Heuristics(Agent agent) {
    this.agent = Objects.requireNonNull(agent);
    this.ap = Objects.requireNonNull(agent.getProperties());
    this.random = new Random();
  }

  public void defineHeuristic(
      NodeGraph originNode, NodeGraph destinationNode, boolean onlyDistanceMinimsation) {
    if (onlyDistanceMinimsation) {
      ap.reset();
      ap.setMinimisationMode(MinimisationMode.DISTANCE);
      return;
    }
    defineRouteChoiceMechanisms();
  }

  public void defineRouteChoiceMechanisms() {

    // No empirical (cluster) data drives this agent: use pure minimisation, alternating shortest
    // path (distance) and least-turn (angular / simplest path) by the default distribution.
    // Angular needs a dual graph, so primal-only cities always minimise distance.
    if (!hasEmpiricalActivation()) {
      ap.setMinimisationMode(defaultMinimisationMode());
      return;
    }

    // Empirical-driven route choice, with each mechanism gated by the data the city actually loaded.
    if (isGlobalMinimisationDominant()) {
      ap.setMinimisationMode(constrainMinimisation(sampleMinimisationMode()));
      return;
    }

    ap.setLocalHeuristicMode(constrainLocalHeuristic(sampleLocalHeuristicMode()));

    if (barriersAvailable() && random.nextDouble() < probabilityBarrierSubGoals) {
      ap.addElement(RouteChoiceElement.BARRIER_BASED_NAVIGATION);
    } else if (landmarksAvailable()) {
      ap.addElement(RouteChoiceElement.LOCAL_LANDMARKS);
    }

    if (landmarksAvailable() && random.nextDouble() < probabilityDistantLandmarks) {
      ap.addElement(RouteChoiceElement.DISTANT_LANDMARKS);
    }

    if (regionsAvailable() && random.nextDouble() < probabilityUsingRegions) {
      ap.addElement(RouteChoiceElement.REGION_BASED_NAVIGATION);
    }
  }

  /** Whether any empirical (cluster-derived) activation probability has been set for this agent. */
  private boolean hasEmpiricalActivation() {
    return probabilityDistanceMinimisation > 0.0
        || probabilityAngularMinimisation > 0.0
        || probabilityDistantLandmarks > 0.0
        || probabilityUsingRegions > 0.0
        || probabilityBarrierSubGoals > 0.0;
  }

  /**
   * Minimisation mode when no empirical data drives the agent: samples distance vs angular by the
   * {@link RouteChoicePars} default split, but only offers angular when a dual graph is loaded.
   */
  private MinimisationMode defaultMinimisationMode() {
    if (!dualAvailable()) {
      return MinimisationMode.DISTANCE;
    }
    double d = Math.max(0.0, RouteChoicePars.defaultProbabilityDistanceMinimisation);
    double a = Math.max(0.0, RouteChoicePars.defaultProbabilityAngularMinimisation);
    double total = d + a;
    if (total == 0.0) {
      return MinimisationMode.DISTANCE;
    }
    return random.nextDouble() < d / total ? MinimisationMode.DISTANCE : MinimisationMode.ANGULAR;
  }

  /** Angular minimisation needs the dual graph; fall back to distance when it is absent. */
  private MinimisationMode constrainMinimisation(MinimisationMode mode) {
    return (mode == MinimisationMode.ANGULAR && !dualAvailable()) ? MinimisationMode.DISTANCE : mode;
  }

  /** Angular local heuristic needs the dual graph; fall back to distance when it is absent. */
  private LocalHeuristicMode constrainLocalHeuristic(LocalHeuristicMode mode) {
    return (mode == LocalHeuristicMode.ANGULAR && !dualAvailable())
        ? LocalHeuristicMode.DISTANCE
        : mode;
  }

  private boolean dualAvailable() {
    return PedSimCity.dualGraphLoaded;
  }

  private boolean landmarksAvailable() {
    return PedSimCity.landmarksLoaded;
  }

  private boolean regionsAvailable() {
    return !PedSimCity.regionsMap.isEmpty();
  }

  private boolean barriersAvailable() {
    return !PedSimCity.barriersMap.isEmpty();
  }

  private boolean isGlobalMinimisationDominant() {
    return probabilityDistanceMinimisation > 0.90 || probabilityAngularMinimisation > 0.90;
  }

  private MinimisationMode sampleMinimisationMode() {
    return sampleWeightedMode(probabilityDistanceMinimisation, probabilityAngularMinimisation)
            == BinaryMode.DISTANCE
        ? MinimisationMode.DISTANCE
        : MinimisationMode.ANGULAR;
  }

  private LocalHeuristicMode sampleLocalHeuristicMode() {
    return sampleWeightedMode(probabilityDistanceMinimisation, probabilityAngularMinimisation)
            == BinaryMode.DISTANCE
        ? LocalHeuristicMode.DISTANCE
        : LocalHeuristicMode.ANGULAR;
  }

  private BinaryMode sampleWeightedMode(double distanceWeight, double angularWeight) {
    double d = Math.max(0.0, distanceWeight);
    double a = Math.max(0.0, angularWeight);
    double total = d + a;

    if (total == 0.0) {
      return BinaryMode.DISTANCE;
    }

    return random.nextDouble() < (d / total) ? BinaryMode.DISTANCE : BinaryMode.ANGULAR;
  }

  private enum BinaryMode {
    DISTANCE,
    ANGULAR
  }

  public void setActivationProbabilities(
      double probabilityDistanceMinimisation,
      double probabilityAngularMinimisation,
      double probabilityDistantLandmarks,
      double probabilityUsingRegions,
      double probabilityBarrierSubGoals) {

    this.probabilityDistanceMinimisation = probabilityDistanceMinimisation;
    this.probabilityAngularMinimisation = probabilityAngularMinimisation;
    this.probabilityDistantLandmarks = probabilityDistantLandmarks;
    this.probabilityUsingRegions = probabilityUsingRegions;
    this.probabilityBarrierSubGoals = probabilityBarrierSubGoals;
  }

  public double getLocalLandmarksThreshold() {
    return RouteChoicePars.localLandmarkThresholdCommunity;
  }

  public double getGlobalLandmarkWeight(boolean angular) {
    return angular ? globalLandmarknessWeightAngular : globalLandmarknessWeightDistance;
  }

  public Agent getAgent() {
    return agent;
  }

  public AgentProperties getAgentProperties() {
    return ap;
  }
}
