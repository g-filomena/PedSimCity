package pedsim.core.agents;

import java.util.Objects;
import java.util.Random;
import pedsim.core.engine.PedSimCity;
import pedsim.core.parameters.RouteChoicePars;
import pedsim.core.utilities.StringEnum.LocalHeuristicMode;
import pedsim.core.utilities.StringEnum.MinimisationMode;
import pedsim.core.utilities.StringEnum.RouteChoiceElement;

/**
 * Samples an agent's route-choice configuration into its {@link AgentProperties}.
 *
 * <p>Two paths:
 *
 * <ul>
 *   <li><b>Default</b> (no activation probabilities set): pure minimisation, sampling shortest
 *       (distance) vs least-turn (angular) by the {@link RouteChoicePars} default split.
 *   <li><b>Activated</b> (after {@link #setActivationProbabilities}): probability-driven choice of
 *       minimisation, local heuristic and route-choice elements (landmarks, regions, barriers),
 *       with each mechanism gated by the data the city actually loaded — angular modes need the
 *       dual graph, landmark elements need landmark scores, and so on.
 * </ul>
 */
public final class Heuristics {

  private double probabilityDistanceMinimisation;
  private double probabilityAngularMinimisation;
  private double probabilityDistantLandmarks;
  private double probabilityUsingRegions;
  private double probabilityBarrierSubGoals;

  private final AgentProperties ap;
  private final Random random;

  private final double globalLandmarknessWeightDistance =
      RouteChoicePars.globalLandmarknessWeightDistanceCommunity;
  private final double globalLandmarknessWeightAngular =
      RouteChoicePars.globalLandmarknessWeightAngularCommunity;

  public Heuristics(Agent agent) {
    this.ap = Objects.requireNonNull(Objects.requireNonNull(agent).getProperties());
    this.random = new Random();
  }

  /**
   * Configures the agent's route choice for the next trip. With {@code onlyDistanceMinimisation}
   * the properties are reset to plain shortest-path routing; otherwise the route-choice mechanisms
   * are sampled (see class doc).
   */
  public void defineHeuristic(boolean onlyDistanceMinimisation) {
    if (onlyDistanceMinimisation) {
      ap.reset();
      ap.setMinimisationMode(MinimisationMode.DISTANCE);
      return;
    }
    defineRouteChoiceMechanisms();
  }

  public void defineRouteChoiceMechanisms() {

    // No activation probabilities set for this agent: use pure minimisation, alternating shortest
    // path (distance) and least-turn (angular / simplest path) by the default distribution.
    // Angular needs a dual graph, so primal-only cities always minimise distance.
    if (!hasActivationProbabilities()) {
      ap.setMinimisationMode(defaultMinimisationMode());
      return;
    }

    // Probability-driven route choice, with each mechanism gated by the data the city loaded.
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

  /** Whether any activation probability has been set for this agent. */
  private boolean hasActivationProbabilities() {
    return probabilityDistanceMinimisation > 0.0
        || probabilityAngularMinimisation > 0.0
        || probabilityDistantLandmarks > 0.0
        || probabilityUsingRegions > 0.0
        || probabilityBarrierSubGoals > 0.0;
  }

  /**
   * Minimisation mode when no activation probabilities drive the agent: samples distance vs
   * angular by the {@link RouteChoicePars} default split, but only offers angular when a dual
   * graph is loaded.
   */
  private MinimisationMode defaultMinimisationMode() {
    if (!dualAvailable()) {
      return MinimisationMode.DISTANCE;
    }
    return sampleDistanceOverAngular(
            RouteChoicePars.defaultProbabilityDistanceMinimisation,
            RouteChoicePars.defaultProbabilityAngularMinimisation)
        ? MinimisationMode.DISTANCE
        : MinimisationMode.ANGULAR;
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
    return sampleDistanceOverAngular(
            probabilityDistanceMinimisation, probabilityAngularMinimisation)
        ? MinimisationMode.DISTANCE
        : MinimisationMode.ANGULAR;
  }

  private LocalHeuristicMode sampleLocalHeuristicMode() {
    return sampleDistanceOverAngular(
            probabilityDistanceMinimisation, probabilityAngularMinimisation)
        ? LocalHeuristicMode.DISTANCE
        : LocalHeuristicMode.ANGULAR;
  }

  /** Weighted coin flip between the two minimisation flavours; distance wins ties and zeros. */
  private boolean sampleDistanceOverAngular(double distanceWeight, double angularWeight) {
    double d = Math.max(0.0, distanceWeight);
    double a = Math.max(0.0, angularWeight);
    double total = d + a;

    if (total == 0.0) {
      return true;
    }

    return random.nextDouble() < (d / total);
  }

  /**
   * Sets the per-mechanism activation probabilities that switch this agent from the default pure
   * minimisation to probability-driven route choice (e.g. sampled from group-level parameters).
   */
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
}
