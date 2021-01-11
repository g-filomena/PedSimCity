package sim.app.geo.pedsimcity;

import java.util.ArrayList;
import java.util.List;

/**
 * Some parameters set by the modeller
 *
 */
public class UserParameters {

	//General parameters
	static String cityName = "London";
	static int jobs = 1;
	static int numAgents = 100; //this is set to 1 agent per route choice model when one of testingLandmarks, testingRegions, testingModels is True
	static int numTrips = 10; //this is automatically set when one of testingLandmarks (255), testingRegions (2000) is True

	// 1) Run the model to a) evaluate the introduction of landmarks or b) regions and barriers.
	// set (only) one of the following ones as true, only when replicating the analysis under the specific conditions of the papers (see github repository)
	static boolean testingLandmarks = true;
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
	static boolean testingModels = false;
	//	static String routeChoices[] = {"DS", "AC", "DL", "AL", "ALG", "DLG", "DG", "AG", "DR", "AR", "DRB", "ARB", "TS"};
	static String routeChoices[] = {"AL"};
	static double minDistance = 500;
	static double maxDistance = 3000;


	// 3) Test specificRoutes (1 agent per routeChoiceModel, specify them above in routeChoices).
	static boolean testingSpecificRoutes = false;
	static List<Integer> OR = new ArrayList<Integer>();
	static List<Integer> DE = new ArrayList<Integer>();

	public static void setTestingMatrix() {
		Integer[] or = { 1383};
		Integer[] de = {3234};
		OR.clear();
		DE.clear();
		for (int i : or) OR.add(i);
		for (int i : de) DE.add(i);
	}

	// 4) Empirical ABM - creating of groups stochastically from clusters of individuals (in development)
	static boolean empiricalABM = true;
	static boolean activityBased = false;
	static double noobAgentThreshold = 0.25;
	static double expertAgentThreshold = 0.75;

	// Time related parameters for activityBased simulation
	static int minutesPerStep = 10;
	static int startingHour = 7*minutesPerStep;
	static int endingHour = 24*minutesPerStep;

	// Landmark Integration
	static double distanceNodeLandmark = 50.0;
	public static double distanceAnchors = 1500.0;
	static double threshold3dVisibility  = 300.0;
	static double globalLandmarkThreshold = 0.30; //
	static double localLandmarkThreshold = 0.30; //
	static double salientNodesPercentile = 0.75; // Threshold Percentile to identify salient nodes
	public static int nrAnchors = 10;

	// Some researcher-defined parameter
	static double wayfindingEasinessThreshold = 0.95; //2d Visibility threshold; distanLandmarks usage threshold
	static double globalLandmarknessWeightDistance = 0.80; //weight Global Landmarkness in combination with edge cost
	static double globalLandmarknessWeightAngular = 0.90; //weight Global Landmarkness in combination with edge cost
	static double regionBasedNavigationThreshold = 600; //Region-based navigation Threshold - meters
	static double thresholdTurn = 0.0;

	// Other parameters
	static boolean socialInteraction = false;
	static boolean subGraph = false;

	//directories
	public static String outputFolderDefault = "C:/Users/g_filo01/sciebo/Scripts/PedSimCity-Evaluation/Input/";
	public static String outputFolder;
	public static String outputFolderRoutes;

	public static void setOutputFolder() {
		if (testingSpecificRoutes || testingModels) {
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
		else if (empiricalABM) {
			outputFolder = outputFolderDefault+"/empiricalABM/"+cityName+"_PedSim_";
			outputFolderRoutes = outputFolderDefault+"/empiricalABM/routes/"+cityName+"_PedSim_";
		}
	}
}
