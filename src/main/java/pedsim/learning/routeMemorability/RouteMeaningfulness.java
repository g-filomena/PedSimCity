package pedsim.learning.routeMemorability;

import pedsim.learning.parameters.LearningPars;

/**
 * How much a walked route counts in the agent's memory beyond being walked.
 *
 * <p>A route is meaningful to the extent that it is new to the agent and that it shows landmarks
 * along the way; both are shares in [0, 1] and the meaningfulness is their mean. The route then
 * counts as {@code 1 + meaningfulnessGain * meaningfulness} walks, so it never counts for less
 * than one. Neither the choice of the two factors nor the gain has a source: they are an
 * assumption of the model, with the gain in {@link LearningPars#meaningfulnessGain}.
 *
 * <p>Novelty is measured against the agent's vividness grid, so nothing is kept per route.
 */
public final class RouteMeaningfulness {

  private RouteMeaningfulness() {}

  /**
   * The meaningfulness of a route.
   *
   * @param novelty share of the route's space that is new to the agent, in [0, 1]
   * @param landmarkExposure share of the route's nodes with a landmark in view, in [0, 1]
   * @return their mean, in [0, 1]
   */
  public static double of(double novelty, double landmarkExposure) {
    return (clamp01(novelty) + clamp01(landmarkExposure)) / 2.0;
  }

  /**
   * The number of walks a route counts as.
   *
   * @param novelty share of the route's space that is new to the agent, in [0, 1]
   * @param landmarkExposure share of the route's nodes with a landmark in view, in [0, 1]
   * @return at least 1
   */
  public static double walks(double novelty, double landmarkExposure) {
    return 1.0 + Math.max(0.0, LearningPars.meaningfulnessGain) * of(novelty, landmarkExposure);
  }

  private static double clamp01(double v) {
    return Double.isNaN(v) ? 0.0 : Math.max(0.0, Math.min(1.0, v));
  }
}
