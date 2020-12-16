package sim.app.geo.PedSimCity;

import java.util.ArrayList;
import java.util.List;

/**
 * Some parameters set by the modeller
 *
 */
public class UserParameters {

	//General parameters/components
	static String cityName = "Muenster";
	static int jobs = 1;

	// 1) Run the model to a) evaluate the introduction of landmarks or b) regions and barriers.
	// set (only) one of the following ones as true, only when replicating the analysis under the specific conditions of the papers (see github repository)
	static boolean testingLandmarks = false;
	static boolean testingRegions = false;
	// Testing Landmarks route choice models: DS = road distance, AC = angular change, DL = road distance + landmarks, AL = angular change + landmarks
	// LL = road distance + local landmarks, GL = global landmarks
	static String routeChoicesLandmarks[] = {"DS", "AC", "DL", "AL", "LL", "GL"};
	// Testing regions route choice models:
	// AC = angular change, AR = angular change + region-based, AB = angular change + barrier-based, ARB = angular change + region-barrier based
	static String routeChoicesRegions[] = {"AC", "AR", "AB","ARB"};

	// In general heuristics: D = roadDistance, A = angularChange, T = turns
	// L = localLandmarks, G = globalLandmarks, R = regionBased, B = barrierBased, N = nodeBasedNavigation
	// e.g. 'DLR' --> roadDistance + localLandmarks + regions. You can combine different elements.

	// 2) Run the model with the groups of agents of your choice
	//specify the models in the string RouteChoice
	static boolean testingModels = true;
	static String routeChoices[] = {"DS", "AC", "DL", "AL", "ALG", "DLG", "DG", "AG", "DR", "AR", "DRB", "ARB", "TS"};
	// 3) Test specificRoutes (1 agent per routeChoiceModel, specify them above in routeChoices).
	static boolean testingSpecificRoutes = true;
	static List<Integer> OR = new ArrayList<Integer>();
	static List<Integer> DE = new ArrayList<Integer>();

	// 4) Empirical ABM - creating of groups stochastically from clusters of individuals (in development)
	static boolean fiveElements = false;

	// Landmark Integration
	static double distanceNodeLandmark = 50.0;
	public static double distanceAnchors = 1500.0;
	static double threshold3dVisibility  = 300.0;
	static double globalLandmarkThreshold = 0.30; //
	static double localLandmarkThreshold = 0.30; //
	static double salientNodesPercentile = 0.75; // Threshold Percentile to identify salient nodes
	public static int nrAnchors = 25;

	// Some researcher-defined parameter
	static double wayfindingEasinessThreshold = 0.95; //2d Visibility threshold; distanLandmarks usage threshold
	static double globalLandmarknessWeightDistance = 0.80; //weight Global Landmarkness in combination with edge cost
	static double globalLandmarknessWeightAngular = 0.90; //weight Global Landmarkness in combination with edge cost
	static double regionBasedNavigationThreshold = 600; //Region-based navigation Threshold - meters
	static double thresholdTurn = 0.0;
	static boolean subGraph = false;

	// Agents/groups parameters
	static int numAgents = 2000;
	static int groups = 6;
	static Double[] composition = {0.30, 0.20, 0.20, 0.10, 0.10, 0.10};
	static double noobAgentThreshold = 0.25;
	static double expertAgentThreshold = 0.75;
	static boolean socialInteraction = false;


	// Time related parameters
	static int minutesPerStep = 10;
	static int startingHour = 7*minutesPerStep;
	static int endingHour = 24*minutesPerStep;

	//directories
	public static String outputFolderDefault = "C:/Users/g_filo01/sciebo/Scripts/PedSimCity-Evaluation/Input/";
	public static String outputFolder;
	public static String outputFolderRoutes;

	public static void setTestingMatrix() {
		Integer[] or = { 2797};
		Integer[] de = {5157};
		OR.clear();
		DE.clear();
		for (int i : or) OR.add(i);
		for (int i : de) DE.add(i);
	}

	public static void setOutputFolder() {
		if (testingSpecificRoutes) {
			outputFolder = outputFolderDefault+"test/"+cityName+"_PedSim_test_";
			outputFolderRoutes = outputFolderDefault+"test/routes/"+cityName+"_PedSim_test_";
		}
		else if (testingRegions) {
			outputFolder = outputFolderDefault+"landmarkNavigation/"+cityName+"_PedSim_regions_";
			outputFolderRoutes = outputFolderDefault+"landmarkNavigation/routes/"+cityName+"_PedSim_regions_";
		}
		else if (testingLandmarks) {
			outputFolder = outputFolderDefault+"landmarkNavigation/"+cityName+"_PedSim_landmarks_";
			outputFolderRoutes = outputFolderDefault+"landmarkNavigation/routes/"+cityName+"_PedSim_landmarks_";
		}
	}
}
