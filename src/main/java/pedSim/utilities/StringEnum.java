package pedSim.utilities;

public class StringEnum {

	public enum RouteChoice {
		ROAD_DISTANCE, ANGULAR_CHANGE, LANDMARKS_DISTANCE, LANDMARKS_ANGULAR, LOCAL_LANDMARKS_DISTANCE,
		LOCAL_LANDMARKS_ANGULAR, DISTANT_LANDMARKS_DISTANCE, DISTANT_LANDMARKS_ANGULAR, DISTANT_LANDMARKS,
		REGION_DISTANCE, REGION_BARRIER_DISTANCE, REGION_ANGULAR, REGION_BARRIER_ANGULAR, BARRIER_DISTANCE,
		BARRIER_ANGULAR,
	}

	public enum RouteChoiceProperty {

		ROAD_DISTANCE, ANGULAR_CHANGE, ROAD_DISTANCE_LOCAL, ANGULAR_CHANGE_LOCAL, USING_ELEMENTS, NOT_USING_ELEMENTS,
		LOCAL_LANDMARKS, BARRIER_SUBGOALS, NO_SUBGOALS, REGION_BASED, NOT_REGION_BASED, USING_DISTANT, NOT_USING_DISTANT
	}

	public enum Groups {
		NULLGROUP, POPULATION, GROUP1, GROUP2, GROUP3, GROUP4, GROUP5, GROUP6,
	}

	public enum LandmarkType {
		LOCAL, GLOBAL
	}

	public enum BarrierType {
		ALL, POSITIVE, NEGATIVE, SEPARATING,
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
