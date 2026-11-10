package pedsim.core.agents;

import ec.util.MersenneTwisterFast;
import java.util.EnumSet;
import java.util.Objects;
import pedsim.core.engine.PedSimCity;
import pedsim.core.parameters.RouteChoicePars;
import pedsim.core.utilities.StringEnum.LandmarkType;
import pedsim.core.utilities.StringEnum.LocalHeuristicMode;
import pedsim.core.utilities.StringEnum.MinimisationMode;
import pedsim.core.utilities.StringEnum.RouteChoiceElement;

/**
 * Samples a {@link RouteChoiceModel} for an agent that does not bring one of its own.
 *
 * <p>Two paths:
 *
 * <ul>
 *   <li><b>Default</b> (no activation set): pure minimisation, sampling shortest (distance) vs
 *       least-turn (angular) by the {@link RouteChoicePars} default split.
 *   <li><b>Activated</b> (after {@link #setActivation}): the survey-style draw the empirical
 *       module makes per respondent - elements or a pure minimisation; if elements, a local
 *       heuristic, region gateways, local-landmark or barrier sub-goals and distant landmarks - with
 *       each mechanism gated by the data the city actually loaded: landmark elements need landmark
 *       scores, barrier sub-goals need barriers, regions need regions. A trip for which nothing is
 *       available is a pure minimisation.
 * </ul>
 *
 * <p>It returns a model rather than writing into the agent's properties. An agent that was built
 * with a model of its own - cityImage's one-per-scenario, empirical's cluster draw - simply never
 * asks, so there is no way for a sampled model to overwrite an assigned one.
 */
public final class Heuristics {

  private RouteChoiceActivation activation;

  private final MersenneTwisterFast random;

  private final double globalLandmarknessWeightDistance =
      RouteChoicePars.globalLandmarknessWeightDistanceCommunity;
  private final double globalLandmarknessWeightAngular =
      RouteChoicePars.globalLandmarknessWeightAngularCommunity;

  public Heuristics(Agent agent) {
    this(Objects.requireNonNull(agent).getRandom());
  }

  /**
   * @param random the agent's own seeded generator: route choice is sampled per trip, so an
   *     unseeded one would put every routing decision of the run beyond replay
   */
  Heuristics(MersenneTwisterFast random) {
    this.random = random;
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
   * Samples a model from the activation, or falls back to pure minimisation when none is set.
   *
   * @return the sampled model
   */
  public RouteChoiceModel defineRouteChoiceMechanisms() {

    // No activation set for this agent: use pure minimisation, alternating shortest path
    // (distance) and least-turn (angular / simplest path) by the default distribution.
    if (activation == null) {
      return RouteChoiceModel.minimising(defaultMinimisationMode());
    }

    boolean landmarks = landmarksAvailable();
    boolean regions = regionsAvailable();
    boolean barriers = barriersAvailable();

    if (!(landmarks || regions || barriers) || random.nextDouble() >= activation.usingElements()) {
      return RouteChoiceModel.minimising(
          sampleDistanceOverAngular(activation.onlyDistance(), activation.onlyAngular())
              ? MinimisationMode.DISTANCE
              : MinimisationMode.ANGULAR);
    }

    LocalHeuristicMode localHeuristic =
        sampleDistanceOverAngular(activation.localDistance(), activation.localAngular())
            ? LocalHeuristicMode.DISTANCE
            : LocalHeuristicMode.ANGULAR;
    EnumSet<RouteChoiceElement> elements = EnumSet.noneOf(RouteChoiceElement.class);

    if (regions && random.nextDouble() < activation.regions()) {
      elements.add(RouteChoiceElement.REGION_BASED_NAVIGATION);
    }

    // One sub-goal mechanism at most: local landmarks, barriers, or neither, in proportion to
    // their weights and to what is left over.
    double landmarkWeight = landmarks ? activation.localLandmarks() : 0.0;
    double barrierWeight = barriers ? activation.barrierSubGoals() : 0.0;
    double neither = Math.max(0.0, 1.0 - landmarkWeight - barrierWeight);
    double draw = random.nextDouble() * (landmarkWeight + barrierWeight + neither);
    boolean localLandmarks = draw < landmarkWeight;
    boolean barrierSubGoals = !localLandmarks && draw < landmarkWeight + barrierWeight;
    if (localLandmarks) {
      elements.add(RouteChoiceElement.LOCAL_LANDMARKS);
    } else if (barrierSubGoals) {
      elements.add(RouteChoiceElement.BARRIER_BASED_NAVIGATION);
    }

    if (landmarks && random.nextDouble() < activation.distantLandmarks()) {
      elements.add(RouteChoiceElement.DISTANT_LANDMARKS);
    }

    // The survey's own fallback: a respondent who uses elements and activated none uses local
    // landmarks - or, with no landmark scores in the city, a barrier sub-goal.
    if (elements.isEmpty()) {
      if (landmarks) {
        elements.add(RouteChoiceElement.LOCAL_LANDMARKS);
        localLandmarks = true;
      } else if (barriers) {
        elements.add(RouteChoiceElement.BARRIER_BASED_NAVIGATION);
        barrierSubGoals = true;
      } else {
        return RouteChoiceModel.minimising(defaultMinimisationMode());
      }
    }

    return RouteChoiceModel.usingElements(
        localHeuristic,
        elements,
        localLandmarks || barrierSubGoals ? LandmarkType.LOCAL : null,
        activation.barriers());
  }

  /**
   * Minimisation mode when no activation drives the agent: samples distance vs angular by the
   * {@link RouteChoicePars} default split.
   */
  private MinimisationMode defaultMinimisationMode() {
    return sampleDistanceOverAngular(
            RouteChoicePars.defaultProbabilityDistanceMinimisation,
            RouteChoicePars.defaultProbabilityAngularMinimisation)
        ? MinimisationMode.DISTANCE
        : MinimisationMode.ANGULAR;
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
   * Switches this agent from the default pure minimisation to the survey-style draw of {@link
   * #defineRouteChoiceMechanisms()}.
   *
   * @param activation the agent's own probabilities, or null for the default
   */
  public void setActivation(RouteChoiceActivation activation) {
    this.activation = activation;
  }

  public double getGlobalLandmarkWeight(boolean angular) {
    return angular ? globalLandmarknessWeightAngular : globalLandmarknessWeightDistance;
  }
}
