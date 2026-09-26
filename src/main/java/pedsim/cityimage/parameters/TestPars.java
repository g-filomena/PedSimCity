package pedsim.cityimage.parameters;

import java.util.ArrayList;
import pedsim.cityimage.utilities.StringEnum.Scenario;
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

  private TestPars() {}

  public static final ArrayList<Float> distances = new ArrayList<>();

  public static String stringMode = "Testing Landmarks";

  public static int numberTripsPerAgent = 2;

  // The OD length range lives on Pars: minRouteLength / maxRouteLength, the desired walked length
  // of the route. It is shared with core's destination draw and with the empirical module on
  // purpose - all three are asking one question, how long a synthetic walking trip is in this city,
  // and all three had the same value. Separate copies would recreate the shadowed-static trap that
  // RouteChoicePars.{usingDMA, maxTripsPerDay, originsTmp, destinationsTmp} already were against
  // this class, made worse here because TestPars extends Pars: a minODdistance field would sit
  // beside an inherited minRouteLength holding the same number under another name.
  //
  // What must NOT come back is the commute reading this range. That is what capped every commute in
  // the model at 2,700 m - a discretionary walking range sizing a journey that is not
  // discretionary.
  // The commute has its own distance model: ActivityPars.workplaceDistanceDecay for where the
  // workplace goes, and ActivityAgent.walksToWork for whether it is walked.

  public static boolean testingLandmarks = false;
  public static boolean testingSubdivisions = false;
  public static boolean testingModels = false;
  public static boolean testingSpecificOD = false;
  public static boolean testing = false;

  public static boolean verboseMode = false;

  /**
   * Draw the generic OD matrix's origins and destinations in proportion to the floor area of the
   * buildings attached to each node, instead of uniformly over the network. Needs a buildings layer.
   */
  public static boolean weightODByFloorArea = false;

  public static Scenario[] landmarkScenarios = {
    Scenario.ROAD_DISTANCE,
    Scenario.LANDMARKS_DISTANCE,
    Scenario.ANGULAR_CHANGE,
    Scenario.LANDMARKS_ANGULAR,
    Scenario.LOCAL_LANDMARKS_DISTANCE,
    Scenario.LOCAL_LANDMARKS_ANGULAR,
    Scenario.DISTANT_LANDMARKS_DISTANCE,
    Scenario.DISTANT_LANDMARKS_ANGULAR,
    Scenario.DISTANT_LANDMARKS
  };

  public static Scenario[] subdivisionScenarios = {
    Scenario.ANGULAR_CHANGE,
    Scenario.REGION_ANGULAR,
    Scenario.BARRIER_ANGULAR,
    Scenario.REGION_BARRIER_ANGULAR,
    Scenario.ROAD_DISTANCE,
    Scenario.REGION_DISTANCE,
    Scenario.BARRIER_DISTANCE,
    Scenario.REGION_BARRIER_DISTANCE
  };

  public static Scenario[] userScenarios = {Scenario.ROAD_DISTANCE, Scenario.ANGULAR_CHANGE};

  public static Scenario[] scenarios = landmarkScenarios;

  public static Integer[] originsTmp = {};
  public static Integer[] destinationsTmp = {};

  public static void defineMode() {
    resetParameters();

    if ("Testing Landmarks".equals(stringMode)) {
      testingLandmarks = true;
      scenarios = landmarkScenarios;
      numberTripsPerAgent = 255;
      jobs = 50;

    } else if ("Testing Urban Subdivisions".equals(stringMode)) {
      testingSubdivisions = true;
      scenarios = subdivisionScenarios;
      numberTripsPerAgent = 2000;
      jobs = 10;

    } else if ("Testing Specific Route Choice Models".equals(stringMode)) {
      testingModels = true;
      scenarios = userScenarios != null ? userScenarios : new Scenario[0];

      if (scenarios.length == 0) {
        scenarios = new Scenario[] {Scenario.ROAD_DISTANCE, Scenario.ANGULAR_CHANGE};
      }
    }

    if (testingSpecificOD && originsTmp != null && originsTmp.length > 0) {
      numberTripsPerAgent = originsTmp.length;
    }

    numAgents = scenarios.length;
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
