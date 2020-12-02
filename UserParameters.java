package sim.app.geo.pedSimCity;

import java.util.ArrayList;
import java.util.List;

/**
 * Some parameters set by the modeller
 *
 */
public class UserParameters {

	//General parameters/components
	static String cityName = "London";
	static int jobs = 50;
	static boolean testingRegions = false;
	static boolean testingLandmarks = true;

	static boolean testingSpecificRoutes = false;
	static boolean readingFromPrevious =  false;
	static List<Integer> OR = new ArrayList<Integer>();
	static List<Integer> DE = new ArrayList<Integer>();

	static boolean subGraph = false;
	static String routeChoicesLandmarks[] = {"roadDistance", "angularChange", "roadDistanceLandmarks", "angularChangeLandmarks",
			"localLandmarks", "globalLandmarks"};
	static String routeChoicesRegions[] = {"angularChange", "angularChangeRegions", "angularChangeBarriers", "angularChangeRegionsBarriers"};
	static String routeChoices[] = {"roadDistance", "angularChange"};

	// Landmark Integration
	static double distanceNodeLandmark = 50.0;
	public static double distanceAnchors = 2000.0;
	static double globalLandmarkThreshold = 0.30; //
	static double localLandmarkThreshold = 0.30; //
	static double salientNodesPercentile = 0.75; // Threshold Percentile to identify salient nodes

	// Some researcher-defined parameter
	static double wayfindingEasinessThreshold = 0.95; //2d Visibility threshold; distanLandmarks usage threshold
	static double globalLandmarknessWeight = 0.80; //weight Global Landmarkness in combination with edge cost

	// Agents/groups parameters
	static int numAgents = 2000;

	//directories
	public static String outputFolderDefault = "C:/Users/g_filo01/sciebo/Scripts/PedSimCity_Analysis/Input/";
	public static String outputFolder;
	public static String outputFolderRoutes;

	public static void setTestingMatrix() {
		Integer[] or = {};
		Integer[] de = {};
		for (int i : or) OR.add(i);
		for (int i : de) DE.add(i);
	}

	public static void setOutputFolder() {
		if (testingSpecificRoutes) {
			outputFolder = outputFolderDefault+"test/"+cityName+"_PedSim_test_";
			outputFolderRoutes = outputFolderDefault+"test/routes/"+cityName+"_PedSim_test_";
		}
		if (testingLandmarks) {
			outputFolder = outputFolderDefault+"landmarkNavigation/"+cityName+"_PedSim_landmarks_";
			outputFolderRoutes = outputFolderDefault+"landmarkNavigation/routes/"+cityName+"_PedSim_landmarks_";
		}
		else if (testingRegions) {
			outputFolder = outputFolderDefault+"regionBasedNavigation/"+cityName+"_PedSim_regions_";
			outputFolderRoutes = outputFolderDefault+"regionBasedNavigation/routes/"+cityName+"_PedSim_regions_";
		}
	}


}
