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

  private TestPars() {}

  public static final ArrayList<Float> distances = new ArrayList<>();

  public static String stringMode = "Testing Landmarks";

  public static int numberTripsPerAgent = 2;

  // The OD length range lives on Pars: minTripDistance / maxTripDistance, derived from
  // avgTripDistance. It is shared with core's destination draw and with the empirical module on
  // purpose - all three are asking one question, how long a synthetic walking trip is in this city,
  // and all three had the same value. Separate copies would recreate the shadowed-static trap that
  // RouteChoicePars.{usingDMA, maxTripsPerDay, originsTmp, destinationsTmp} already were against
  // this class, made worse here because TestPars extends Pars: a minODdistance field would sit
  // beside an inherited minTripDistance holding the same number under another name.
  //
  // What must NOT come back is the commute reading this range. That is what capped every commute in
  // the model at 2,700 m - a discretionary walking range sizing a journey that is not discretionary.
  // The commute has its own distance model: ActivityPars.workplaceDistanceDecay for where the
  // workplace goes, and ActivityAgent.walksToWork for whether it is walked.

  public static boolean testingLandmarks = false;
  public static boolean testingSubdivisions = false;
  public static boolean testingModels = false;
  public static boolean testingSpecificOD = false;
  public static boolean testing = false;

  public static boolean verboseMode = false;

  public static RouteChoice[] routeChoiceTestingLandmarks = {
    RouteChoice.ROAD_DISTANCE,
    RouteChoice.LANDMARKS_DISTANCE,
    RouteChoice.ANGULAR_CHANGE,
    RouteChoice.LANDMARKS_ANGULAR,
    RouteChoice.LOCAL_LANDMARKS_DISTANCE,
    RouteChoice.LOCAL_LANDMARKS_ANGULAR,
    RouteChoice.DISTANT_LANDMARKS_DISTANCE,
    RouteChoice.DISTANT_LANDMARKS_ANGULAR,
    RouteChoice.DISTANT_LANDMARKS
  };

  public static RouteChoice[] routeChoiceTestingSubdivisions = {
    RouteChoice.ANGULAR_CHANGE,
    RouteChoice.REGION_ANGULAR,
    RouteChoice.BARRIER_ANGULAR,
    RouteChoice.REGION_BARRIER_ANGULAR,
    RouteChoice.ROAD_DISTANCE,
    RouteChoice.REGION_DISTANCE,
    RouteChoice.BARRIER_DISTANCE,
    RouteChoice.REGION_BARRIER_DISTANCE
  };

  public static RouteChoice[] routeChoiceUser = {
    RouteChoice.ROAD_DISTANCE, RouteChoice.ANGULAR_CHANGE
  };

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
        routeChoiceModels =
            new RouteChoice[] {RouteChoice.ROAD_DISTANCE, RouteChoice.ANGULAR_CHANGE};
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
