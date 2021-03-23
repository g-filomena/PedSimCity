package pedsimcity.main;

import java.util.ArrayList;
import java.util.List;

/**
 * Some parameters set by the modeller
 *
 */
public class UserParameters {

	// General parameters
	static String cityName = "Muenster";
	static int jobs = 5;
	static int numAgents = 365; // this is set to 1 agent per route choice model when one of testingLandmarks,
								// testingRegions, testingModels is True
	public static int numTrips = 1; // this is automatically set when one of testingLandmarks (255), testingRegions
									// (2000) is True

	// 1) Run the model to a) evaluate the introduction of landmarks or b) regions
	// and barriers.
	// set (only) one of the following ones as true, only when replicating the
	// analysis under the specific conditions of the papers (see github repository)
	static boolean testingLandmarks = false;
	static boolean testingRegions = false;
	// Testing Landmarks route choice models: DS = road distance, AC = angular
	// change, DL = road distance + landmarks, AL = angular change + landmarks
	// LL = road distance + local landmarks, GL = global landmarks
	static String routeChoicesLandmarks[] = { "DS", "AC", "DL", "AL", "LL", "GL" };
	// Testing regions route choice models:
	// AC = angular change, AR = angular change + region-based, AB = angular change
	// + barrier-based, ARB = angular change + region-barrier based
	static String routeChoicesRegions[] = { "AC", "AR", "AB", "ARB" };

	// In general heuristics: D = roadDistance, A = angularChange, T = turns
	// L = localLandmarks, G = globalLandmarks, R = regionBased, B = barrierBased, N
	// = nodeBasedNavigation
	// e.g. 'DLR' --> roadDistance + localLandmarks + regions. You can combine
	// different elements.

	// 2) Run the model with the groups of agents of your choice
	// specify the models in the string RouteChoice
	static boolean testingModels = false;
	// static String routeChoices[] = {"DS", "AC", "TS"};
	static String routeChoices[] = { "AB", "DB" };

	public static double minDistance = 300;
	public static double maxDistance = 2000;

	// 3) Test specificRoutes (1 agent per routeChoiceModel, specify them above in
	// routeChoices).
	static boolean testingSpecificRoutes = false;
	static List<Integer> OR = new ArrayList<>();
	static List<Integer> DE = new ArrayList<>();

	public static void setTestingMatrix() {
		// Integer[] or = {12232};
		// Integer[] de = {11628};
		final Integer[] or = { 2797, 1586, 4229 };
		final Integer[] de = { 5157, 11337, 11379 };
		OR.clear();
		DE.clear();
		for (final int i : or)
			OR.add(i);
		for (final int i : de)
			DE.add(i);
	}

	// 4) Empirical ABM - creating of groups stochastically from clusters of
	// individuals (in development)
	public static boolean empiricalABM = true;
	public static double noobAgentThreshold = 0.25;
	public static double expertAgentThreshold = 0.75;

	// Landmark Integration
	public static double distanceNodeLandmark = 50.0;
	public static double distanceAnchors = 1500;
	public static double threshold3dVisibility = 300;
	public static double globalLandmarkThreshold = 0.30; //
	public static double localLandmarkThreshold = 0.30; //
	public static double salientNodesPercentile = 0.75; // Threshold Percentile to identify salient nodes
	public static int nrAnchors = 25;

	// Some researcher-defined parameter
	public static double wayfindingEasinessThreshold = 0.95; // 2d Visibility threshold; distanLandmarks usage threshold
	public static double globalLandmarknessWeightDistance = 0.80; // weight Global Landmarkness in combination with edge
																	// cost
	public static double globalLandmarknessWeightAngular = 0.90; // weight Global Landmarkness in combination with edge
																	// cost
	public static double regionBasedNavigationThreshold = 600; // Region-based navigation Threshold - meters
	public static double thresholdTurn = 0.00;

	// Other parameters
	public static boolean socialInteraction = false;
	public static boolean subGraph = false;

	// directories
	public static String outputFolderDefault = "C:/Users/g_filo01/sciebo/Scripts/PedSimCity-Evaluation/Input/";
	public static String outputFolder;
	public static String outputFolderRoutes;

	public static void setOutputFolder() {
		if (testingSpecificRoutes || testingModels) {
			outputFolder = outputFolderDefault + "test/" + cityName + "_PedSim_test_";
			outputFolderRoutes = outputFolderDefault + "test/routes/" + cityName + "_PedSim_test_routes_";
		} else if (testingRegions) {
			outputFolder = outputFolderDefault + "landmarkNavigation/" + cityName + "_PedSim_regions_";
			outputFolderRoutes = outputFolderDefault + "landmarkNavigation/routes/" + cityName + "_PedSim_regions_";
		} else if (testingLandmarks) {
			outputFolder = outputFolderDefault + "landmarkNavigation/" + cityName + "_PedSim_landmarks_";
			outputFolderRoutes = outputFolderDefault + "landmarkNavigation/routes/" + cityName + "_PedSim_landmarks_";
		} else if (empiricalABM) {
			outputFolder = outputFolderDefault + "/empiricalABM/" + cityName + "_PedSim_empirical_";
			outputFolderRoutes = outputFolderDefault + "/empiricalABM/routes/" + cityName + "_PedSim_empirical_";
		}
	}
}
