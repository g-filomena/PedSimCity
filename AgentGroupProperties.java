package sim.app.geo.PedSimCity;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Random;

public class AgentGroupProperties extends AgentProperties {

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
	double sd_error = 0.10;
	ArrayList<Double> pHeuristics = new ArrayList<Double>();
	ArrayList<Double> pSubGoals = new ArrayList<Double>();
	HashMap<String, Double> mapHeuristics = new HashMap<String, Double>();
	HashMap<String, Double> mapSubGoals = new HashMap<String, Double>();

	public void setGroupProperties(Group group) {


		this.groupName = group.groupName;

		// heuristics
		this.pRoadDistance = group.pRoadDistance;
		this.pAngularChange = group.pAngularChange;
		this.pTurns = group.pTurns;

		// Regions
		this.pRegionBasedNavigation = group.pRegionBasedNavigation;

		// subGoals
		this.pLocalLandmarks = group.pLocalLandmarks;
		this.pBarrierBasedNavigation = group.pBarrierBasedNavigation;
		this.pNodeBasedNavigation = group.pNodeBasedNavigation;

		// other route properties
		this.pGlobalLandmarks = group.pGlobalLandmarks;
		this.pNaturalBarriers = group.pNaturalBarriers;
		this.pSeveringBarriers = group.pSeveringBarriers;

		pHeuristics.add(this.pRoadDistance);
		pHeuristics.add(this.pAngularChange);
		pHeuristics.add(this.pTurns);

		pSubGoals.add(this.pLocalLandmarks);
		pSubGoals.add(this.pBarrierBasedNavigation);
		pSubGoals.add(this.pNodeBasedNavigation);
		Collections.sort(pHeuristics);
		Collections.sort(pSubGoals);
		Collections.reverse(pHeuristics);
		Collections.reverse(pSubGoals);

	}
	public void set(Group group) {


		Random random = new Random();
		// heuristics
		for (double heuristic : pHeuristics)
		{
			if (random.nextFloat() <= p.RoadDistance)
				else if (random.nextFloat() <= p.RoadDistance))
this.pRoadDistance = group.pRoadDistance;
this.pAngularChange = group.pAngularChange;
this.pTurns = group.pTurns;

// Regions
this.pRegionBasedNavigation = group.pRegionBasedNavigation;

// subGoals
this.pLocalLandmarks = group.pLocalLandmarks;
this.pBarrierBasedNavigation = group.pBarrierBasedNavigation;
this.pNodeBasedNavigation = group.pNodeBasedNavigation;

// other route properties
this.pGlobalLandmarks = group.pGlobalLandmarks;
this.pNaturalBarriers = group.pNaturalBarriers;
this.pSeveringBarriers = group.pSeveringBarriers;

		}


	}
