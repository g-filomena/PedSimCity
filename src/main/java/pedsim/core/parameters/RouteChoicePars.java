package pedsim.core.parameters;

/**
 * The Parameters class contains global parameters and settings for the PedSimCity simulation. These
 * parameters are used to configure various aspects of the simulation, including simulation mode,
 * agent behavior, and data import options.
 */
public class RouteChoicePars {

  public static boolean useGravityModel = true;
  public static boolean usePublicTransport = true;
  public static double thresholdTurn = 45;

  // Default route-choice split used when NO empirical (cluster) data drives the agent: how often
  // agents minimise road distance (shortest path) vs angular change (least-turn / simplest path).
  // Angular is only ever used when a dual graph is loaded; otherwise agents fall back to distance.
  public static double defaultProbabilityDistanceMinimisation = 0.5;
  public static double defaultProbabilityAngularMinimisation = 0.5;

  public static Integer[] cityCentreRegionsID = {};
  public static boolean includeTertiary = true;

  // Landmark Integration
  public static double distanceNodeLandmark = 50.0;
  public static double distanceAnchors = 2000;
  public static double threshold3dVisibility = 300;
  // Threshold Percentile to identify salient nodes
  public static double salientNodesPercentile = 0.90;

  // to speed-up, it can be higher; it can be lower for more prototypical
  public static int nrAnchors = 25;

  public static double globalLandmarkThresholdCommunity = 0.30;
  public static double localLandmarkThresholdCommunity = 0.35;
  // weight Global Landmarkness in combination with edge costs (road distance)
  public static double globalLandmarknessWeightDistanceCommunity = 0.85;
  // weight Global Landmarkness in combination with edge costs (angular change)
  public static double globalLandmarknessWeightAngularCommunity = 0.95;

  // Wayfinding Easiness threshold
  public static double wayfindingEasinessThresholdCommunity = 0.95; // global navigation for local
  // landmark identification
  // within regions for local landmark identification
  public static double wayfindingEasinessThresholdRegionsCommunity = 0.85;

  // Region-based navigation Threshold - meters
  public static double regionNavActivationThreshold = 500;

  /**
   * Standard deviation of the multiplier {@code Dijkstra.costPerceptionError} applies to every edge
   * cost, on every relaxation, in every model: an agent does not perceive a street's cost exactly.
   *
   * <p>It is behaviour, not noise to be switched off casually. But over a route it does not average
   * out enough to ignore - at 0.10 it produces roughly 0.20 volume divergence and 0.43 edge overlap
   * between two models whether or not those models differ - so <b>any comparison between
   * route-choice models must set it to 0 first</b>, or it reads the dice as the model.
   *
   * <p>At 0 the draw is degenerate and the multiplier is exactly 1.0, so {@code ROAD_DISTANCE} is
   * the true shortest path and is minimal on every OD pair by construction. That is the check that
   * makes a silent substitution impossible to miss: if another model beats it, the distance
   * baseline is not minimising distance and nothing else in the comparison can be trusted.
   *
   * <p>The draw is made either way, so the random stream is the same at any sigma and a run pinned
   * here stays comparable with one that is not in everything except the perception error.
   */
  public static double perceptionErrorSD = 0.10;
}
