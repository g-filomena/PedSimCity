package pedsim.cityimage.parameters;

import java.util.ArrayList;

import pedsim.cityimage.utilities.StringEnum.RouteChoice;
import pedsim.core.parameters.Pars;
import pedsim.core.parameters.TimePars;

/**
 * Parameters for the city-image testing module.
 *
 * This module covers:
 *
 * - landmark-based route-choice testing; - urban-subdivision / region / barrier
 * testing; - user-selected route-choice model testing; - optional specific OD
 * testing.
 *
 * Empirical ABM parameters belong to pedsim.empirical, not to cityimage.
 */
public final class TestPars extends Pars {

	private TestPars() {
	}

	public static final ArrayList<Float> distances = new ArrayList<>();

	public static String stringMode = "Testing Landmarks";

	public static int numberTripsPerAgent = 2;

	public static boolean testingLandmarks = false;
	public static boolean testingSubdivisions = false;
	public static boolean testingModels = false;
	public static boolean testingSpecificOD = false;
	public static boolean testing = false;

	public static boolean verboseMode = false;

	public static String localPath = "C:/Users/gfilo/OneDrive - The University of Liverpool/Scripts/pedsimcity/src/main/resources/";

	public static RouteChoice[] routeChoiceTestingLandmarks = { RouteChoice.ROAD_DISTANCE,
			RouteChoice.LANDMARKS_DISTANCE, RouteChoice.ANGULAR_CHANGE, RouteChoice.LANDMARKS_ANGULAR,
			RouteChoice.LOCAL_LANDMARKS_DISTANCE, RouteChoice.LOCAL_LANDMARKS_ANGULAR,
			RouteChoice.DISTANT_LANDMARKS_DISTANCE, RouteChoice.DISTANT_LANDMARKS_ANGULAR,
			RouteChoice.DISTANT_LANDMARKS };

	public static RouteChoice[] routeChoiceTestingSubdivisions = { RouteChoice.ANGULAR_CHANGE,
			RouteChoice.REGION_ANGULAR, RouteChoice.BARRIER_ANGULAR, RouteChoice.REGION_BARRIER_ANGULAR,
			RouteChoice.ROAD_DISTANCE, RouteChoice.REGION_DISTANCE, RouteChoice.BARRIER_DISTANCE,
			RouteChoice.REGION_BARRIER_DISTANCE };

	public static RouteChoice[] routeChoiceUser = { RouteChoice.ROAD_DISTANCE, RouteChoice.ANGULAR_CHANGE };

	public static RouteChoice[] routeChoiceModels = routeChoiceTestingLandmarks;

	public static Integer[] originsTmp = {};
	public static Integer[] destinationsTmp = {};

	public static void defineMode() {
		resetParameters();

		if ("Testing Landmarks".equals(stringMode)) {
			testingLandmarks = true;
			routeChoiceModels = routeChoiceTestingLandmarks;
			numberTripsPerAgent = 255;
			jobs = 50;

		} else if ("Testing Urban Subdivisions".equals(stringMode)) {
			testingSubdivisions = true;
			routeChoiceModels = routeChoiceTestingSubdivisions;
			numberTripsPerAgent = 2000;
			jobs = 10;

		} else if ("Testing Specific Route Choice Models".equals(stringMode)) {
			testingModels = true;
			routeChoiceModels = routeChoiceUser != null ? routeChoiceUser : new RouteChoice[0];

			if (routeChoiceModels.length == 0) {
				routeChoiceModels = new RouteChoice[] { RouteChoice.ROAD_DISTANCE, RouteChoice.ANGULAR_CHANGE };
			}
		}

		if (testingSpecificOD && originsTmp != null && originsTmp.length > 0) {
			numberTripsPerAgent = originsTmp.length;
		}

		numAgents = routeChoiceModels.length;
		testing = testingLandmarks || testingSubdivisions || testingModels;
		moveRate = TimePars.STEP_DURATION * pedestrianSpeed;
	}

	private static void resetParameters() {
		testingLandmarks = false;
		testingSubdivisions = false;
		testingModels = false;
		testing = false;
	}
}