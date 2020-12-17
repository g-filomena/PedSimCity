package sim.app.geo.PedSimCity;

public class Group {


	String groupName;
	int groupID;
	double portion = 0.0;
	double agentKnowledgeMin = 0.0;
	double agentKnowledgeMax = 1.0;
	double pRoadDistance = 0.0;
	double pAngularChange = 0.0;
	double pTurns = 0.0;
	double pRegionBasedNavigation = 0.0;
	double pLocalLandmarks = 0.0;
	double pBarrierBasedNavigation = 0.0;
	double pNodeBasedNavigation = 0.0;
	double pGlobalLandmarks = 0.0;
	double pNaturalBarriers = 0.0;
	double pSeveringBarriers = 0.0;

	public void setGroup(String groupName, String [] attributes) {

		this.groupName = groupName;
		this.portion = Float.parseFloat(attributes[1]);

		// heuristics
		this.pRoadDistance = Float.parseFloat(attributes[2]);
		this.pAngularChange = Float.parseFloat(attributes[3]);
		this.pTurns = Float.parseFloat(attributes[4]);

		// Regions
		this.pRegionBasedNavigation = Float.parseFloat(attributes[5]);

		// subGoals
		this.pLocalLandmarks = Float.parseFloat(attributes[6]);
		this.pBarrierBasedNavigation = Float.parseFloat(attributes[7]);
		this.pNodeBasedNavigation = Float.parseFloat(attributes[8]);

		// other route properties
		this.pGlobalLandmarks = Float.parseFloat(attributes[9]);
		this.pNaturalBarriers = Float.parseFloat(attributes[10]);
		this.pSeveringBarriers = Float.parseFloat(attributes[11]);

	}
}







