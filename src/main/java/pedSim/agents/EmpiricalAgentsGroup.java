package pedSim.agents;

import pedSim.utilities.StringEnum.Groups;

/**
 * The `EmpiricalAgentsGroup` class represents a group of empirical agents in
 * the simulation. It encapsulates group-specific parameters and attributes that
 * influence the behaviour of agents within the group. These parameters include
 * probabilities for various route choice properties, such as using elements,
 * minimisation approaches, heuristics, region-based navigation, subgoals,
 * distant landmarks, and other route properties. Additionally, it stores
 * information about natural and severing barriers and the group's share in the
 * population.
 */
public class EmpiricalAgentsGroup {

	public Groups groupName;
	public double share = 0.0;

	// only minimisation
	double probabilityRoadDistance = 0.0;
	double probabilityRoadDistanceSD = 0.0;
	double probabilityAngularChange = 0.0;
	double probabilityAngularChangeSD = 0.0;

	double probabilityNotUsingElements = 0.0;
	double probabilityNotUsingElementsSD = 0.0;
	double probabilityUsingElements = 0.0;
	double probabilityUsingElementsSD = 0.0;

	// local heuristics
	double probabilityLocalRoadDistance = 0.0;
	double probabilityLocalRoadDistanceSD = 0.0;
	double probabilityLocalAngularChange = 0.0;
	double probabilityLocalAngularChangeSD = 0.0;

	// segmentation
	double probabilityRegionBasedNavigation = 0.0;
	double probabilityRegionBasedNavigationSD = 0.0;
	double probabilityLocalLandmarks = 0.0;
	double probabilityLocalLandmarksSD = 0.0;
	double probabilityBarrierSubGoals = 0.0;
	double probabilityBarrierSubGoalsSD = 0.0;

	// other route properties
	double probabilityDistantLandmarks = 0.0;
	double probabilityDistantLandmarksSD = 0.0;
	double naturalBarriers = 0.0;
	double naturalBarriersSD = 0.0;
	double severingBarriers = 0.0;
	double severingBarriersSD = 0.0;

	/**
	 * Sets the properties of the agent's group based on the provided group name and
	 * attributes. This method is responsible for configuring various route choice
	 * properties and other parameters for agents belonging to a specific group.
	 *
	 * @param groupName  The name of the agent's group.
	 * @param attributes An array of attributes containing group-specific
	 *                   parameters.
	 */
	public void setGroup(Groups groupName, String[] attributes) {

		this.groupName = groupName;
		if (groupName.equals(Groups.NULLGROUP))
			return;

		probabilityUsingElements = Float.parseFloat(attributes[1]);
		probabilityUsingElementsSD = Float.parseFloat(attributes[2]);
		probabilityNotUsingElements = Float.parseFloat(attributes[3]);
		probabilityNotUsingElementsSD = Float.parseFloat(attributes[4]);

		// onlyMinimisation
		probabilityRoadDistance = Float.parseFloat(attributes[5]);
		probabilityRoadDistanceSD = Float.parseFloat(attributes[6]);
		probabilityAngularChange = Float.parseFloat(attributes[7]);
		probabilityAngularChangeSD = Float.parseFloat(attributes[8]);

		// heuristics
		probabilityLocalRoadDistance = Float.parseFloat(attributes[9]);
		probabilityLocalRoadDistanceSD = Float.parseFloat(attributes[10]);
		probabilityLocalAngularChange = Float.parseFloat(attributes[11]);
		probabilityLocalAngularChangeSD = Float.parseFloat(attributes[12]);

		// Regions
		probabilityRegionBasedNavigation = Float.parseFloat(attributes[13]);
		probabilityRegionBasedNavigationSD = Float.parseFloat(attributes[14]);

		// subGoals
		probabilityLocalLandmarks = Float.parseFloat(attributes[15]);
		probabilityLocalLandmarksSD = Float.parseFloat(attributes[16]);
		probabilityBarrierSubGoals = Float.parseFloat(attributes[17]);
		probabilityBarrierSubGoalsSD = Float.parseFloat(attributes[18]);

		// other route properties
		probabilityDistantLandmarks = Float.parseFloat(attributes[19]);
		probabilityDistantLandmarksSD = Float.parseFloat(attributes[20]);

		naturalBarriers = Float.parseFloat(attributes[21]);
		naturalBarriersSD = Float.parseFloat(attributes[22]);
		severingBarriers = Float.parseFloat(attributes[23]);
		severingBarriersSD = Float.parseFloat(attributes[24]);

		if (this.groupName.equals(Groups.POPULATION))
			return;
		share = Float.parseFloat(attributes[25]);
	}
}
