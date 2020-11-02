package sim.app.geo.pedSimCity;

/**
 * Some parameters set by the modeller
 *
 */
public class ResearchParameters {



	static String cityName = "Muenster";

	// testing elements
	static boolean testingRegions = false;
	static boolean testingLandmarks = false;
	static boolean visibility = false;
	static int jobs = 50;

	static boolean fiveElements = true;

	// Landmark Integration
	static double distanceNodeLandmark = 50.0;
	public static double distanceAnchors = 1000.0;
	static double globalLandmarkThreshold = 0.20; //
	static double localLandmarkThreshold = 0.30; //
	static double salientNodesPercentile = 0.75; // Threshold Percentile to identify salient nodes

	// Some researcher-defined parameter
	static double regionBasedNavigationThreshold = 600; //Region-based navigation Threshold - meters
	static double visibilityThreshold = 300; //2d Advance Visibility threshold; distanLandmarks usage threshold
	static double wayfindingEasinessThreshold = 0.95; //2d Visibility threshold; distanLandmarks usage threshold
	static double globalLandmarknessWeight = 0.80; //weight Global Landmarkness in combination with edge cost



	static double noobAgentThreshold = 0.25;
	static double expertAgentThreshold = 0.75;


	static int minutesPerStep = 10;
	static int startingHour = 7*minutesPerStep;
	static int endingHour = 24*minutesPerStep;
	static int numAgents = 2000;
	static int groups = 6;
	static Double[] composition = {0.30, 0.2, 0.2, 0.1, 0.1, 0.1};
}
