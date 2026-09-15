package pedsim.cityimage.utilities;

public class StringEnum extends pedsim.core.utilities.StringEnum {

  /**
   * The route-choice models this module compares.
   *
   * <p>What each one means is stated in {@code CityImageAgent.modelFor}, an exhaustive switch: add a
   * constant here and the compiler refuses the switch until it is given a model. Names are labels
   * here, never parsed for their meaning.
   *
   * <p>How the names read: a bare {@code LANDMARKS_*} model uses local <i>and</i> distant landmarks,
   * {@code LOCAL_LANDMARKS_*} only local, {@code DISTANT_LANDMARKS_*} only distant. A trailing
   * {@code _DISTANCE} or {@code _ANGULAR} is the heuristic routing each leg between sub-goals;
   * {@code ROAD_DISTANCE} and {@code ANGULAR_CHANGE} have no sub-goals and minimise that cost end to
   * end. Bare {@code DISTANT_LANDMARKS} has neither: landmarkness alone decides the route.
   */
  public enum Scenario {
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
    REGION_ANGULAR,
    BARRIER_DISTANCE,
    BARRIER_ANGULAR,
    REGION_BARRIER_DISTANCE,
    REGION_BARRIER_ANGULAR
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

  public static String getAbbreviation(Scenario choice) {
    String[] parts = choice.toString().split("_");
    StringBuilder abbreviation = new StringBuilder();
    for (String part : parts) {
      abbreviation.append(part.charAt(0));
    }
    return abbreviation.toString();
  }
}
