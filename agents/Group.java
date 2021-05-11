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

	double pNotUsingElements = 0.0;
	double pNotUsingElementsSD = 0.0;
	double pUsingElements = 0.0;
	double pUsingElementsSD = 0.0;

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
	double pDistantLandmarks = 0.0;
	double pDistantLandmarksSD = 0.0;
	double naturalBarriers = 0.0;
	double naturalBarriersSD = 0.0;
	double severingBarriers = 0.0;
	double severingBarriersSD = 0.0;

	double pNoRegionBasedNavigation = 0.0;
	double pNoRegionBasedNavigationSD = 0.0;
	double pNoSubGoals = 0.0;
	double pNoSubGoalsSD = 0.0;
	double pNoDistantLandmarks = 0.0;
	double pNoDistantLandmarksSD = 0.0;

	double agentKnowledge = 0.0;
	double agentKnowledgeSD = 1.0;

	public void setGroup(String groupName, String[] attributes) {

		this.groupName = groupName;
		if (this.groupName.equals("nullGroup"))
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
		this.pDistantLandmarks = Float.parseFloat(attributes[15]);
		this.pDistantLandmarksSD = Float.parseFloat(attributes[16]);

		this.pUsingElements = Float.parseFloat(attributes[37]);
		this.pUsingElementsSD = Float.parseFloat(attributes[38]);
		this.pNotUsingElements = Float.parseFloat(attributes[39]);
		this.pNotUsingElementsSD = Float.parseFloat(attributes[40]);

		this.pNoRegionBasedNavigation = Float.parseFloat(attributes[21]);
		this.pNoRegionBasedNavigationSD = Float.parseFloat(attributes[22]);
		this.pNoSubGoals = Float.parseFloat(attributes[23]);
		this.pNoSubGoalsSD = Float.parseFloat(attributes[24]);
		this.pNoDistantLandmarks = Float.parseFloat(attributes[25]);
		this.pNoDistantLandmarksSD = Float.parseFloat(attributes[26]);

		this.naturalBarriers = Float.parseFloat(attributes[33]);
		this.naturalBarriersSD = Float.parseFloat(attributes[34]);
		this.severingBarriers = Float.parseFloat(attributes[35]);
		this.severingBarriersSD = Float.parseFloat(attributes[36]);

		if (this.groupName.equals("population"))
			return;
		this.portion = Float.parseFloat(attributes[41]);
	}
}
