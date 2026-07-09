package pedsim.core.parameters;

/**
 * The Parameters class contains global parameters and settings for the PedSimCity simulation. These
 * parameters are used to configure various aspects of the simulation, including simulation mode,
 * agent behavior, and data import options.
 */
public class RouteChoicePars {

  public static boolean usingDMA = false;
  public static boolean useGravityModel = true;
  public static boolean usePublicTransport = true;
  public static double thresholdTurn = 45;

  // Default route-choice split used when NO empirical (cluster) data drives the agent: how often
  // agents minimise road distance (shortest path) vs angular change (least-turn / simplest path).
  // Angular is only ever used when a dual graph is loaded; otherwise agents fall back to distance.
  public static double defaultProbabilityDistanceMinimisation = 0.5;
  public static double defaultProbabilityAngularMinimisation = 0.5;

  // Distance between Origin and Destination
  public static double minTripDistance = 700;
  public static double avgTripDistance = 1800;
  public static double maxTripDistance = 2500;
  public static double maxTripsPerDay = 6;

  public static Integer[] originsTmp = {};
  public static Integer[] destinationsTmp = {};
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

  public static void setMinMaxTripDistance() {
    minTripDistance = avgTripDistance * 0.50;
    maxTripDistance = avgTripDistance * 1.50;
  }
}
