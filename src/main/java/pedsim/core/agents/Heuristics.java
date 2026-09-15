package pedsim.core.agents;

import ec.util.MersenneTwisterFast;
import java.util.EnumSet;
import java.util.Objects;
import pedsim.core.engine.PedSimCity;
import pedsim.core.parameters.RouteChoicePars;
import pedsim.core.utilities.StringEnum.LocalHeuristicMode;
import pedsim.core.utilities.StringEnum.MinimisationMode;
import pedsim.core.utilities.StringEnum.RouteChoiceElement;

/**
 * Samples a {@link RouteChoiceModel} for an agent that does not bring one of its own.
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
 *
 * <p>It returns a model rather than writing into the agent's properties. An agent that was built
 * with a model of its own - cityImage's one-per-scenario, empirical's cluster draw - simply never
 * asks, so there is no way for a sampled model to overwrite an assigned one.
 */
public final class Heuristics {

  private double probabilityDistanceMinimisation;
  private double probabilityAngularMinimisation;
  private double probabilityDistantLandmarks;
  private double probabilityUsingRegions;
  private double probabilityBarrierSubGoals;

  private final MersenneTwisterFast random;

  private final double globalLandmarknessWeightDistance =
      RouteChoicePars.globalLandmarknessWeightDistanceCommunity;
  private final double globalLandmarknessWeightAngular =
      RouteChoicePars.globalLandmarknessWeightAngularCommunity;

  public Heuristics(Agent agent) {
    Objects.requireNonNull(agent);
    // The agent's own seeded generator: route choice is sampled per trip, so an unseeded one here
    // put every routing decision of the run beyond replay.
    this.random = agent.getRandom();
  }

  /**
   * Samples the route choice for the next trip.
   *
   * @param onlyDistanceMinimisation route by plain shortest path, sampling nothing
   * @return the model for this trip
   */
  public RouteChoiceModel defineHeuristic(boolean onlyDistanceMinimisation) {
    if (onlyDistanceMinimisation) {
      return RouteChoiceModel.minimising(MinimisationMode.DISTANCE);
    }
    return defineRouteChoiceMechanisms();
  }

  /**
   * Samples a model from the activation probabilities, or falls back to pure minimisation when none
   * are set.
   *
   * @return the sampled model
   */
  public RouteChoiceModel defineRouteChoiceMechanisms() {

    // No activation probabilities set for this agent: use pure minimisation, alternating shortest
    // path (distance) and least-turn (angular / simplest path) by the default distribution.
    // Angular needs a dual graph, so primal-only cities always minimise distance.
    if (!hasActivationProbabilities()) {
      return RouteChoiceModel.minimising(defaultMinimisationMode());
    }

    // Probability-driven route choice, with each mechanism gated by the data the city loaded.
    if (isGlobalMinimisationDominant()) {
      return RouteChoiceModel.minimising(constrainMinimisation(sampleMinimisationMode()));
    }

    LocalHeuristicMode localHeuristic = constrainLocalHeuristic(sampleLocalHeuristicMode());
    EnumSet<RouteChoiceElement> elements = EnumSet.noneOf(RouteChoiceElement.class);

    if (barriersAvailable() && random.nextDouble() < probabilityBarrierSubGoals) {
      elements.add(RouteChoiceElement.BARRIER_BASED_NAVIGATION);
    } else if (landmarksAvailable()) {
      elements.add(RouteChoiceElement.LOCAL_LANDMARKS);
    }

    if (landmarksAvailable() && random.nextDouble() < probabilityDistantLandmarks) {
      elements.add(RouteChoiceElement.DISTANT_LANDMARKS);
    }

    if (regionsAvailable() && random.nextDouble() < probabilityUsingRegions) {
      elements.add(RouteChoiceElement.REGION_BASED_NAVIGATION);
    }

    return RouteChoiceModel.usingElements(
        localHeuristic, elements, null, RouteChoiceModel.BarrierPreferences.NONE);
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
    return (mode == MinimisationMode.ANGULAR && !dualAvailable())
        ? MinimisationMode.DISTANCE
        : mode;
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
