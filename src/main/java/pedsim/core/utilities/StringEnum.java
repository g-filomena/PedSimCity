package pedsim.core.utilities;

public class StringEnum {

  public enum MinimisationMode {
    NONE,
    DISTANCE,
    ANGULAR
  }

  public enum LocalHeuristicMode {
    NONE,
    DISTANCE,
    ANGULAR
  }

  public enum RouteChoiceElement {
    LOCAL_LANDMARKS,
    DISTANT_LANDMARKS,
    REGION_BASED_NAVIGATION,
    BARRIER_BASED_NAVIGATION
  }

  public enum RouteChoiceProperty {
    ROAD_DISTANCE,
    ANGULAR_CHANGE,
    ROAD_DISTANCE_LOCAL,
    ANGULAR_CHANGE_LOCAL,
    USING_ELEMENTS,
    NOT_USING_ELEMENTS,
    LOCAL_LANDMARKS,
    BARRIER_SUBGOALS,
    NO_SUBGOALS,
    REGION_BASED,
    NOT_REGION_BASED,
    USING_DISTANT,
    NOT_USING_DISTANT
  }

  public enum AgentStatus {
    WALKING_ALONE,
    WAITING,
    GOING_HOME,
    AT_DESTINATION,
  }

  public enum LandmarkNavigationMode {
    CITY_LEVEL,
    REGION_BASED
  }

  public enum RouteMeaningfulnessFactor {
    EASINESS,
    EXPOSURE,
    NOVELTY
  }

  public enum Learner {
    LEARNER,
    NOT_LEARNER
  }

  public enum LandmarkType {
    LOCAL,
    GLOBAL
  }

  public enum AgentBarrierType {
    ALL,
    POSITIVE,
    NEGATIVE,
    SEPARATING
  }

  public enum BarrierType {
    PARK,
    WATER,
    ROAD,
    RAILWAY,
    SECONDARY_ROAD
  }

  public enum RoadType {
    PRIMARY,
    SECONDARY,
    TERTIARY,
    NEIGHBOURHOOD,
    UNKNOWN
  }

  public enum Vulnerable {
    VULNERABLE,
    NON_VULNERABLE
  }

  /** Hour of the day, used as the scenario dimension for hourly pedestrian volumes. */
  public enum Hour {
    H01, H02, H03, H04, H05, H06, H07, H08, H09, H10, H11, H12,
    H13, H14, H15, H16, H17, H18, H19, H20, H21, H22, H23, H24;

    /** CSV label "h01".."h24"; h24 covers 23:00–00:00. */
    @Override
    public String toString() {
      return String.format("h%02d", ordinal() + 1);
    }

    /** The {@code Hour} bucket for a 0–23 clock hour (0 -> h01, 23 -> h24). */
    public static Hour of(int clockHour) {
      return values()[Math.floorMod(clockHour, 24)];
    }
  }

  /** Single default agent type, used when a module does not enforce agent sub-types. */
  public enum Default {
    DEFAULT
  }
}
