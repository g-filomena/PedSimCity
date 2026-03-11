package pedsim.night.parameters;

/**
 * The Parameters class contains global parameters and settings for the PedSimCity simulation. These
 * parameters are used to configure various aspects of the simulation, including simulation mode,
 * agent behavior, and data import options.
 */
public class RouteChoicePars {

  public static boolean usingDMA = false;
  public static double thresholdTurn = 45;
  public static int MIN_WALKED_ROUTES_SIZE = 5;

  // Euclidean Distance between Origin and Destination -> Walking distance will be
  // probably EuclideanDistance*1.25
  public static double avgTripDistance = 1300;
  public static double minTripDistance;
  public static double maxTripDistance;

  public Integer[] originsTmp = {};
  public static Integer[] destinationsTmp = {};
  public static Integer[] cityCentreRegionsID = {0, 2, 13, 33};

  public static double naturalBarriers = 0.15;
  public static double naturalBarriersSD = 0.10;

  // Landmark Integration
  public static double distanceNodeBuilding = 40.0;
  public static double salientNodesPercentile = 0.90; // Threshold Percentile to identify salient
                                                      // nodes

  public static void setMinMaxTripDistance() {
    minTripDistance = avgTripDistance * 0.50;
    maxTripDistance = avgTripDistance * 1.50;
  }
}
