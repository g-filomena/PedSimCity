package pedsim.empirical.agent;

/**
 * Calibration parameters for one empirical agent group.
 *
 * The expected CSV layout is:
 *
 * groupName, usingElements_mean, usingElements_std, noElements_mean,
 * noElements_std, onlyDistance_mean, onlyDistance_std, onlyAngular_mean,
 * onlyAngular_std, distanceHeuristic_mean, distanceHeuristic_std,
 * angularHeuristic_mean, angularHeuristic_std, regions_mean, regions_std,
 * routeMarks_mean, routeMarks_std, barriers_mean, barriers_std,
 * distantLandmarks_mean, distantLandmarks_std, preferenceNatural_mean,
 * preferenceNatural_std, aversionSevering_mean, aversionSevering_std, portion
 */
public class EmpiricalAgentsGroup {

	public EmpiricalGroup groupName;

	public double share = 0.0;

	// Only minimisation
	double probabilityRoadDistance = 0.0;
	double probabilityRoadDistanceSD = 0.0;
	double probabilityAngularChange = 0.0;
	double probabilityAngularChangeSD = 0.0;

	// Use or do not use meaningful elements
	double probabilityNotUsingElements = 0.0;
	double probabilityNotUsingElementsSD = 0.0;
	double probabilityUsingElements = 0.0;
	double probabilityUsingElementsSD = 0.0;

	// Local heuristics
	double probabilityLocalRoadDistance = 0.0;
	double probabilityLocalRoadDistanceSD = 0.0;
	double probabilityLocalAngularChange = 0.0;
	double probabilityLocalAngularChangeSD = 0.0;

	// Segmentation / coarse planning
	double probabilityRegionBasedNavigation = 0.0;
	double probabilityRegionBasedNavigationSD = 0.0;

	// Subgoals
	double probabilityLocalLandmarks = 0.0;
	double probabilityLocalLandmarksSD = 0.0;
	double probabilityBarrierSubGoals = 0.0;
	double probabilityBarrierSubGoalsSD = 0.0;

	// Distant landmarks
	double probabilityDistantLandmarks = 0.0;
	double probabilityDistantLandmarksSD = 0.0;

	// Other route properties
	double naturalBarriers = 0.0;
	double naturalBarriersSD = 0.0;
	double severingBarriers = 0.0;
	double severingBarriersSD = 0.0;

	public void setGroup(EmpiricalGroup groupName, String[] attributes) {
		this.groupName = groupName;

		if (groupName == EmpiricalGroup.NULLGROUP) {
			share = 1.0;
			return;
		}

		if (attributes == null || attributes.length < 25) {
			throw new IllegalArgumentException(
					"Invalid empirical group row for " + groupName + ": expected at least 25 columns");
		}

		probabilityUsingElements = parse(attributes[1]);
		probabilityUsingElementsSD = parse(attributes[2]);

		probabilityNotUsingElements = parse(attributes[3]);
		probabilityNotUsingElementsSD = parse(attributes[4]);

		probabilityRoadDistance = parse(attributes[5]);
		probabilityRoadDistanceSD = parse(attributes[6]);

		probabilityAngularChange = parse(attributes[7]);
		probabilityAngularChangeSD = parse(attributes[8]);

		probabilityLocalRoadDistance = parse(attributes[9]);
		probabilityLocalRoadDistanceSD = parse(attributes[10]);

		probabilityLocalAngularChange = parse(attributes[11]);
		probabilityLocalAngularChangeSD = parse(attributes[12]);

		probabilityRegionBasedNavigation = parse(attributes[13]);
		probabilityRegionBasedNavigationSD = parse(attributes[14]);

		probabilityLocalLandmarks = parse(attributes[15]);
		probabilityLocalLandmarksSD = parse(attributes[16]);

		probabilityBarrierSubGoals = parse(attributes[17]);
		probabilityBarrierSubGoalsSD = parse(attributes[18]);

		probabilityDistantLandmarks = parse(attributes[19]);
		probabilityDistantLandmarksSD = parse(attributes[20]);

		naturalBarriers = parse(attributes[21]);
		naturalBarriersSD = parse(attributes[22]);

		severingBarriers = parse(attributes[23]);
		severingBarriersSD = parse(attributes[24]);

		if (groupName == EmpiricalGroup.POPULATION) {
			share = 1.0;
		} else if (attributes.length > 25) {
			share = parse(attributes[25]);
		}
	}

	private static double parse(String value) {
		if (value == null || value.isBlank()) {
			return 0.0;
		}
		return Double.parseDouble(value.trim());
	}
}