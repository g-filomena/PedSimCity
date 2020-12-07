package sim.app.geo.PedSimCity;

import java.util.ArrayList;
import java.util.List;

/**
 * Some parameters set by the modeller
 *
 */
public class UserParameters {

	//General parameters/components
	static String cityName = "London";
	static int jobs = 5;

	static boolean testingRegions = true;
	static boolean testingSpecificRoutes = false;
	static List<Integer> OR = new ArrayList<Integer>();
	static List<Integer> DE = new ArrayList<Integer>();

	// AC = angular change
	// RB = angular change + region-based
	// BB = angular change + barrier-based
	// BRB = angular change + region-barrier based
	static String routeChoicesLandmarks[] = {"AC", "RB", "BB","BRB"};
	static String routeChoices[] = {"RD", "AC"};


	// Agents parameters
	static int numAgents = 2000;

	//directories
	public static String outputFolderDefault = "C:/Users/g_filo01/sciebo/Scripts/PedSimCity-Evaluation/Input/";
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
		if (testingRegions) {
			outputFolder = outputFolderDefault+"landmarkNavigation/"+cityName+"_PedSim_regions_";
			outputFolderRoutes = outputFolderDefault+"landmarkNavigation/routes/"+cityName+"_PedSim_regions_";
		}
	}
}
