package pedsim.cityimage.utilities;

public class StringEnum extends pedsim.core.utilities.StringEnum {

  public enum RouteChoice {
    ROAD_DISTANCE,
    ANGULAR_CHANGE,
    LANDMARKS_DISTANCE,
    LANDMARKS_ANGULAR,
    LOCAL_LANDMARKS_DISTANCE,
    LOCAL_LANDMARKS_ANGULAR,
    DISTANT_LANDMARKS_DISTANCE,
    DISTANT_LANDMARKS_ANGULAR,
    DISTANT_LANDMARKS,
    REGION_DISTANCE,
    REGION_BARRIER_DISTANCE,
    REGION_ANGULAR,
    REGION_BARRIER_ANGULAR,
    BARRIER_DISTANCE,
    BARRIER_ANGULAR,
  }

  public enum Groups {
    NULLGROUP,
    POPULATION,
    GROUP1,
    GROUP2,
    GROUP3,
    GROUP4,
    GROUP5,
    GROUP6,
  }

  public static String getAbbreviation(RouteChoice choice) {
    String[] parts = choice.toString().split("_");
    StringBuilder abbreviation = new StringBuilder();
    for (String part : parts) {
      abbreviation.append(part.charAt(0));
    }
    return abbreviation.toString();
  }
}
