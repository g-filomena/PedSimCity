package sim.app.geo.PedSimCity;

public class Group {


	String groupName;
	int groupID;
	double portion = 0.0;
	double pRoadDistanceMin = 0.0;
	double pRoadDistanceMax = 0.0;
	double pAngularChangeMin = 0.0;
	double pAngularChangeMax = 0.0;
	double pTurnsMin = 0.0;
	double pTurnsMax = 0.0;

	double pRegionBasedNavigationMin = 0.0;
	double pRegionBasedNavigationMax = 0.0;

	double pLocalLandmarksMin = 0.0;
	double pLocalLandmarksMax = 0.0;
	double pBarrierSubGoalsMin = 0.0;
	double pBarrierSubGoalsMax = 0.0;
	double pNodeMarksMin = 0.0;
	double pNodeMarksMax = 0.0;

	double pGlobalLandmarksMin = 0.0;
	double pGlobalLandmarksMax = 0.0;
	double pNaturalBarriersMin = 0.0;
	double pNaturalBarriersMax = 0.0;
	double pSeveringBarriersMin = 0.0;
	double pSeveringBarriersMax = 0.0;

	double agentKnowledgeMin = 0.0;
	double agentKnowledgeMax = 1.0;

	public void setGroup(String groupName, String [] attributes) {

		this.groupName = groupName;
		this.portion = Float.parseFloat(attributes[1]);

		// heuristics
		this.pRoadDistanceMin = Float.parseFloat(attributes[2]);
		this.pRoadDistanceMax = Float.parseFloat(attributes[3]);
		this.pAngularChangeMin = Float.parseFloat(attributes[4]);
		this.pAngularChangeMax = Float.parseFloat(attributes[5]);
		this.pTurnsMin = Float.parseFloat(attributes[6]);
		this.pTurnsMax = Float.parseFloat(attributes[7]);

		// Regions
		this.pRegionBasedNavigationMin = Float.parseFloat(attributes[8]);
		this.pRegionBasedNavigationMax = Float.parseFloat(attributes[9]);

		// subGoals
		this.pLocalLandmarksMin = Float.parseFloat(attributes[10]);
		this.pLocalLandmarksMax = Float.parseFloat(attributes[11]);
		this.pBarrierSubGoalsMin = Float.parseFloat(attributes[12]);
		this.pBarrierSubGoalsMax = Float.parseFloat(attributes[13]);
		this.pNodeMarksMin = Float.parseFloat(attributes[14]);
		this.pNodeMarksMax = Float.parseFloat(attributes[15]);

		// other route properties
		this.pGlobalLandmarksMin = Float.parseFloat(attributes[16]);
		this.pGlobalLandmarksMax = Float.parseFloat(attributes[17]);
		this.pNaturalBarriersMin = Float.parseFloat(attributes[18]);
		this.pNaturalBarriersMax = Float.parseFloat(attributes[19]);
		this.pSeveringBarriersMin = Float.parseFloat(attributes[20]);
		this.pSeveringBarriersMax = Float.parseFloat(attributes[21]);

		this.agentKnowledgeMin = Float.parseFloat(attributes[22]);
		this.agentKnowledgeMax = Float.parseFloat(attributes[23]);

	}
}







