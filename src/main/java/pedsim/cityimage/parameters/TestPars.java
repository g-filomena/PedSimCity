package pedsim.cityimage.parameters;

import pedsim.cityimage.utilities.StringEnum.RouteChoice;

/**
 * The Parameters class contains global parameters and settings for the PedSimCity simulation. These
 * parameters are used to configure various aspects of the simulation, including simulation mode,
 * agent behavior, and data import options.
 */
public class TestPars extends pedsim.core.parameters.Pars {

  public static String stringMode = "";
  public static int numberTripsPerAgent = 2;

  public static boolean testingLandmarks = false;
  public static boolean testingSubdivisions = false;
  public static boolean testingModels = false;
  public static boolean testingSpecificOD = false;

  public static boolean testing = false;
  // public static boolean userDefined = false;

  // for development/testing purposes only
  public static boolean javaProject = true;
  public static boolean verboseMode = false;
  public static String localPath =
      "C:/Users/gfilo/OneDrive - The University of Liverpool/Scripts/pedsimcity/src/main/resources/";

  /**
   * Defines the simulation mode and sets simulation parameters based on the selected mode. Called
   * at the beginning of the simulation to configure simulation settings.
   */
  public static void defineMode() {

    if (stringMode.equals("Testing Landmarks")) {
      resetParameters();
      testingLandmarks = true;
      routeChoiceModels = routeChoiceTestingLandmarks;
      numAgents = routeChoiceModels.length;
      numberTripsPerAgent = 255;
      jobs = 50;
    } else if (stringMode.equals("Testing Urban Subdivisions")) {
      resetParameters();
      testingSubdivisions = true;
      routeChoiceModels = routeChoiceTestingSubdivisions;
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
    moveRate = pedsim.core.parameters.TimePars.STEP_DURATION * pedestrianSpeed;
  }

  /**
   * Defines the simulation mode and sets simulation parameters based on the selected mode. Called
   * at the beginning of the simulation to configure simulation settings.
   */
  private static void resetParameters() {
    testingLandmarks = false;
    testingSubdivisions = false;
    testingModels = false;
    empirical = false;
  }

  /**
   * Checks if any testing mode (Landmarks, Subdivisions, or Specific Route Choice Models) is
   * active. Updates the 'testing' flag accordingly.
   */
  public static void isTestingTrue() {
    testing = testingLandmarks || testingSubdivisions || testingModels;
  }

  public static RouteChoice[] routeChoiceTestingLandmarks = {RouteChoice.ROAD_DISTANCE,
      RouteChoice.LANDMARKS_DISTANCE, RouteChoice.ANGULAR_CHANGE, RouteChoice.LANDMARKS_ANGULAR,
      RouteChoice.LOCAL_LANDMARKS_DISTANCE, RouteChoice.DISTANT_LANDMARKS};

  public static RouteChoice[] routeChoiceTestingSubdivisions = {RouteChoice.ANGULAR_CHANGE,
      RouteChoice.REGION_ANGULAR, RouteChoice.BARRIER_ANGULAR, RouteChoice.REGION_BARRIER_ANGULAR};

  public static RouteChoice[] routeChoiceUser;
  public static RouteChoice[] routeChoiceModels;


  public static Integer[] originsTmp = {};
  public static Integer[] destinationsTmp = {};
}
