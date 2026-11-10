package pedsim.learning.cognitivemap;

/**
 * Maps an agent's vividness of a space, in [0, 1], to how readily it recognises local landmarks and
 * how hard a route it will take on: a novice needs salient landmarks and easy routes, an expert
 * tolerates subtler landmarks and harder routes.
 */
public final class ElementsSensitivity {

  /** Lowest local-landmark score anyone recognises. */
  private static final double MIN_LANDMARK_SCORE = 0.25;

  /** Local-landmark recognition threshold at zero vividness (novice) and at full (expert). */
  private static final double LOCAL_LM_THRESHOLD_NOVICE = 0.85;

  private static final double LOCAL_LM_THRESHOLD_EXPERT = 0.25;

  /**
   * Curve exponent of the local-landmark threshold. Below 1 the threshold falls slowly at first and
   * sharply near expertise: rarely adopted by novices, steadily by experts.
   */
  private static final double EXP_THRESHOLD_LOCAL = 0.6;

  /** Wayfinding-easiness threshold at full vividness (expert) and at zero (novice). */
  private static final double EASINESS_THRESHOLD_EXPERT = 0.3;

  private static final double EASINESS_THRESHOLD_NOVICE = 0.95;

  /**
   * The local-landmark recognition threshold for a vividness in [0, 1]: a landmark is recognised
   * when its local-landmarkness is above it.
   */
  public double calculateLocalLandmarksThreshold(double vividness) {
    double threshold =
        LOCAL_LM_THRESHOLD_NOVICE
            + (LOCAL_LM_THRESHOLD_EXPERT - LOCAL_LM_THRESHOLD_NOVICE)
                * Math.pow(vividness, EXP_THRESHOLD_LOCAL);
    return Math.max(MIN_LANDMARK_SCORE, threshold);
  }

  /**
   * The wayfinding-easiness threshold for a vividness in [0, 1]: a space is easy enough to be
   * navigated by landmarks when its easiness is above it. Novices need easy routes (high
   * threshold), experts tolerate harder ones.
   */
  public static double deriveWayfindingEasinessThreshold(double vividness) {
    double v = Math.max(0.0, Math.min(1.0, vividness));
    return EASINESS_THRESHOLD_NOVICE - (EASINESS_THRESHOLD_NOVICE - EASINESS_THRESHOLD_EXPERT) * v;
  }
}
