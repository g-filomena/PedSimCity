package pedsimcity.agents;

public class Group {

	public String groupName;
	public double portion = 0.0;

	// only minimisation
	double pOnlyRoadDistance = 0.0;
	double pOnlyRoadDistanceSD = 0.0;
	double pOnlyAngularChange = 0.0;
	double pOnlyAngularChangeSD = 0.0;
	double pOnlyTurns = 0.0;
	double pOnlyTurnsSD = 0.0;

	// local heuristics
	double pRoadDistance = 0.0;
	double pRoadDistanceSD = 0.0;
	double pAngularChange = 0.0;
	double pAngularChangeSD = 0.0;

	// segmentation
	double pRegionBasedNavigation = 0.0;
	double pRegionBasedNavigationSD = 0.0;
	double pLocalLandmarks = 0.0;
	double pLocalLandmarksSD = 0.0;
	double pBarrierSubGoals = 0.0;
	double pBarrierSubGoalsSD = 0.0;

	// other route properties
	double pGlobalLandmarks = 0.0;
	double pGlobalLandmarksSD = 0.0;
	double meanNaturalBarriers = 0.0;
	double meanNaturalBarriersSD = 0.0;
	double meanSeveringBarriers = 0.0;
	double meanSeveringBarriersSD = 0.0;

	double agentKnowledge = 0.0;
	double agentKnowledgeSD = 1.0;

	public void setGroup(String groupName, String[] attributes) {

		this.groupName = groupName;
		if (this.groupName.equals("null"))
			return;

		// onlyMinimisation
		this.pOnlyRoadDistance = Float.parseFloat(attributes[1]);
		this.pOnlyRoadDistanceSD = Float.parseFloat(attributes[2]);
		this.pOnlyAngularChange = Float.parseFloat(attributes[3]);
		this.pOnlyAngularChangeSD = Float.parseFloat(attributes[4]);

		// heuristics
		this.pRoadDistance = Float.parseFloat(attributes[5]);
		this.pRoadDistanceSD = Float.parseFloat(attributes[6]);
		this.pAngularChange = Float.parseFloat(attributes[7]);
		this.pAngularChangeSD = Float.parseFloat(attributes[8]);

		// Regions
		this.pRegionBasedNavigation = Float.parseFloat(attributes[9]);
		this.pRegionBasedNavigationSD = Float.parseFloat(attributes[10]);

		// subGoals
		this.pLocalLandmarks = Float.parseFloat(attributes[11]);
		this.pLocalLandmarksSD = Float.parseFloat(attributes[12]);
		this.pBarrierSubGoals = Float.parseFloat(attributes[13]);
		this.pBarrierSubGoalsSD = Float.parseFloat(attributes[14]);

		// other route properties
		this.pGlobalLandmarks = Float.parseFloat(attributes[15]);
		this.pGlobalLandmarksSD = Float.parseFloat(attributes[16]);

		this.meanNaturalBarriers = Float.parseFloat(attributes[17]);
		this.meanNaturalBarriersSD = Float.parseFloat(attributes[18]);
		this.meanSeveringBarriers = Float.parseFloat(attributes[19]);
		this.meanSeveringBarriersSD = Float.parseFloat(attributes[20]);

		if (this.groupName.equals("population"))
			return;
		this.portion = Float.parseFloat(attributes[21]);
	}
}
