package pedSim.engine;

import pedSim.utilities.StringEnum.RouteChoice;

/**
 * The Parameters class contains global parameters and settings for the
 * PedSimCity simulation. These parameters are used to configure various aspects
 * of the simulation, including simulation mode, agent behavior, and data import
 * options.
 */
public class Parameters {

	// General parameters
	public static String cityName = "Muenster";
	public static String stringMode = "";
	public static int jobs = 1;
	public static int numAgents = 1;
	public static int numberTripsPerAgent = 2;

	// in seconds. One step == 10 minutes,
	public static double stepTimeUnit = 600;
	// One step == 10 minutes, average speed 1 meter/sec., --> moveRate = 60*10
	// meters per second
	private static double pedestrianSpeed = 1.42;
	// meters per step;
	public static double moveRate;

	public static boolean testingLandmarks = false;
	public static boolean testingSubdivisions = false;
	public static boolean testingModels = false;
	public static boolean testingSpecificOD = false;

	public static boolean testing = false;
	public static boolean empirical = false;
	public static boolean userDefined = false;

	// Other parameters
	public static boolean subGraph = false;
	public static boolean usingDMA = false;
	public static double thresholdTurn;

	// Distance between Origin and Destination
	public static double minDistance = 1000;
	public static double maxDistance = 2500;

	// Use the enums for route choice models
	public static RouteChoice[] routeChoiceLandmarks = { RouteChoice.ROAD_DISTANCE, RouteChoice.LANDMARKS_DISTANCE,
			RouteChoice.ANGULAR_CHANGE, RouteChoice.LANDMARKS_ANGULAR, RouteChoice.LOCAL_LANDMARKS_DISTANCE,
			RouteChoice.DISTANT_LANDMARKS };

	public static RouteChoice[] routeChoiceSubdivisions = { RouteChoice.ANGULAR_CHANGE, RouteChoice.REGION_ANGULAR,
			RouteChoice.BARRIER_ANGULAR, RouteChoice.REGION_BARRIER_ANGULAR };

	public static RouteChoice[] routeChoiceUser;
	public static RouteChoice[] routeChoiceModels;

	public static Integer[] originsTmp = {};
	public static Integer[] destinationsTmp = {};

	// Landmark Integration
	public static double distanceNodeLandmark = 50.0;
	public static double distanceAnchors = 2000;
	public static double threshold3dVisibility = 300;
	public static double globalLandmarkThreshold = 0.30; //
	public static double localLandmarkThreshold = 0.30; //
	public static double salientNodesPercentile = 0.75; // Threshold Percentile to identify salient nodes
	public static int nrAnchors = 25; // to speed-up, it can be higher; it can be lower for more prototypical
	// weight Global Landmarkness in combination with edge costs (road distance)
	public static double globalLandmarknessWeightDistance = 0.85;
	// weight Global Landmarkness in combination with edge costs (angular change)
	public static double globalLandmarknessWeightAngular = 0.95;
	public static double regionBasedNavigationThreshold = 500; // Region-based navigation Threshold - meters

	// Wayfinding Easiness threshold
	public static double wayfindingEasinessThreshold = 0.95; // global navigation for local landmark identification
	public static double wayfindingEasinessThresholdRegions = 0.85; // within regions for local landmark identification

	// for development/testing purposes only
	public static boolean javaProject = true;
	public static boolean verboseMode = false;
	public static String localPath = null;

	/**
	 * Defines the simulation mode and sets simulation parameters based on the
	 * selected mode. Called at the beginning of the simulation to configure
	 * simulation settings.
	 */
	public static void defineMode() {

		if (stringMode.equals("Testing Landmarks")) {
			resetParameters();
			testingLandmarks = true;
			routeChoiceModels = routeChoiceLandmarks;
			numAgents = routeChoiceModels.length;
			numberTripsPerAgent = 255;
			jobs = 50;
		} else if (stringMode.equals("Testing Urban Subdivisions")) {
			resetParameters();
			testingSubdivisions = true;
			routeChoiceModels = routeChoiceSubdivisions;
			numAgents = routeChoiceModels.length;
			numberTripsPerAgent = 2000;
			jobs = 10;
		} else if (stringMode.equals("Empirical ABM")) {
			resetParameters();
			empirical = true;
			numAgents = 301;
			numberTripsPerAgent = 3;
			jobs = 10;
		} else if (stringMode.equals("Testing Specific Route Choice Models")) {
			resetParameters();
			testingModels = true;
			routeChoiceModels = routeChoiceUser;
			numAgents = routeChoiceModels.length;
		}
		if (testingSpecificOD)
			numberTripsPerAgent = originsTmp.length;

		isTestingTrue();
		moveRate = stepTimeUnit * pedestrianSpeed;
	}

	/**
	 * Defines the simulation mode and sets simulation parameters based on the
	 * selected mode. Called at the beginning of the simulation to configure
	 * simulation settings.
	 */
	private static void resetParameters() {
		testingLandmarks = false;
		testingSubdivisions = false;
		testingModels = false;
		empirical = false;
	}

	/**
	 * Checks if any testing mode (Landmarks, Subdivisions, or Specific Route Choice
	 * Models) is active. Updates the 'testing' flag accordingly.
	 */
	public static void isTestingTrue() {
		testing = testingLandmarks || testingSubdivisions || testingModels;
	}
}
