package pedsim.core.utilities;

public class StringEnum {

  public enum RouteChoiceProperty {
    ROAD_DISTANCE, ANGULAR_CHANGE, ROAD_DISTANCE_LOCAL, ANGULAR_CHANGE_LOCAL, USING_ELEMENTS, NOT_USING_ELEMENTS, LOCAL_LANDMARKS, BARRIER_SUBGOALS, NO_SUBGOALS, REGION_BASED, NOT_REGION_BASED, USING_DISTANT, NOT_USING_DISTANT
  }

  public enum AgentStatus {
    WALKING_ALONE, WAITING, GOING_HOME, AT_DESTINATION,
  }

  public enum LandmarkNavigationMode {
    CITY_LEVEL, REGION_BASED
  }

  public enum RouteMeaningfulnessFactor {
    EASINESS, EXPOSURE, NOVELTY
  }

  public enum Learner {
    LEARNER, NOT_LEARNER
  }

  public enum LandmarkType {
    LOCAL, GLOBAL
  }

  public enum AgentBarrierType {
    ALL, POSITIVE, NEGATIVE, SEPARATING
  }

  public enum BarrierType {
    PARK, WATER, ROAD, RAILWAY, SECONDARY_ROAD
  }

  public enum RoadType {
    PRIMARY, SECONDARY, TERTIARY, NEIGHBOURHOOD, UNKNOWN
  }

  public enum Vulnerable {
    VULNERABLE, NON_VULNERABLE
  }

  public enum TimeOfDay {
    DAY, NIGHT
  }
}
