package sim.app.geo.PedSimCity;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.javatuples.Pair;

import sim.app.geo.UrbanSim.NodeGraph;
import sim.util.Bag;
import sim.util.geo.MasonGeometry;


public class AgentProperties {

	public int agentID;
	public String criteria;
	double sd_error = 0.10;
	public int group;
	double agentKnowledge = 1.0;

	// for general routing
	String localHeuristic = "";
	public String routeChoice;
	ArrayList<Pair<NodeGraph, NodeGraph>> OD =  new ArrayList<Pair<NodeGraph, NodeGraph>>();
	ArrayList<ArrayList<NodeGraph>> listSequences = new ArrayList<ArrayList<NodeGraph>> ();

	//landmarkNavigation related parameters
	boolean landmarkBasedNavigation = false;
	boolean usingLocalLandmarks = false;
	boolean usingGlobalLandmarks = false;
	boolean onlyAnchors = true;
	// for computing the complexity of the environment ["local", "global"]
	String typeLandmarks = "";

	//region- and barrier-based parameters
	boolean regionBasedNavigation = false;
	boolean barrierBasedNavigation = false;
	// the ones possibly used as sub-goals ["all", "positive", "negative", "separating"]
	String typeBarriers = "";

	boolean nodeBasedNavigation = false;
	boolean activityBased = false;

	// for daily-routine
	boolean student;
	boolean worker;
	boolean flaneur;
	boolean homeBased;
	boolean atWork;
	boolean atHome;
	boolean atPlace;
	boolean away;
	double timeAtHome = 0.0;
	double timeAway = 0.0;
	double timeAtWork = 0.0;
	double thresholdAtHome = 0.0;
	double thresholdAway = 0.0;
	double thresholdWandering = 0.0;
	double totalTimeAway = 0.0;
	NodeGraph homePlace;
	NodeGraph workPlace;
	NodeGraph otherPlace;

	/**
	 * @param landmarkBasedNavigation using Landmarks y/n;
	 * @param onlyAnchors when computing global landmarkness, it considers only landmarks anchoring the destination as possible; if false,
	 * global landmark is considered as a possible distant landmark;
	 */
	public void setProperties(String routeChoice, String typeLandmarks, String typeBarriers) {

		this.routeChoice = routeChoice;
		if (routeChoice.contains("D")) localHeuristic = "roadDistance";
		if (routeChoice.contains("A")) localHeuristic = "angularChange";
		if (routeChoice.contains("T")) localHeuristic = "turns";

		if (routeChoice.contains("L")) {
			landmarkBasedNavigation = true;
			usingLocalLandmarks = true;
			typeLandmarks = "local"; //for measuring complexity when selecting on-route marks
		}
		if (routeChoice.contains("G")) {
			System.out.println(routeChoice);
			usingGlobalLandmarks = true;
			onlyAnchors = true;
		}
		if (routeChoice.contains("N")) {
			nodeBasedNavigation = true;
			usingLocalLandmarks = false;
			landmarkBasedNavigation = false;
		}
		if (routeChoice.contains("R")) regionBasedNavigation = true;
		if (routeChoice.contains("B")) barrierBasedNavigation = true;
		if (agentKnowledge <= UserParameters.noobAgentThreshold) onlyAnchors = false;
	}

	public void setOD(ArrayList<Pair<NodeGraph, NodeGraph>> OD, ArrayList<ArrayList<NodeGraph>> listSequences) {
		this.OD = OD;
		if (listSequences != null) this.listSequences = new ArrayList<ArrayList<NodeGraph>> (listSequences);
	}

	public void setAway() 	{
		this.timeAway = 0.0;
		this.atHome = false;
		this.atWork = false;
		this.atPlace = false;
	}

	public void setThresholdHome() {
		Random random = new Random();
		// from 1h to 3h
		if (this.student) thresholdAtHome = 60 + random.nextInt(3*60);
		// from 30 min to 2h
		if (this.flaneur) thresholdAtHome = 30 + random.nextInt(2*60);
		else thresholdAtHome = 60 + random.nextInt(3*60);
	}




	public void setThresholdAway(String type) {

		Random random = new Random();
		if (type.equals("leisure")) {
			// from 2h to 4h
			if (this.flaneur) thresholdAway = 2*60 + random.nextInt(4*60);
			// from 30 min to 4h
			if (this.student) thresholdAway = random.nextInt(4*60)+30;
			// from 30 min to 3h
			else thresholdAway = random.nextInt(3*60)+30;
		}
		else if (type.equals("work")) {
			if (this.student) thresholdAway = 4*60 + random.nextInt(30);
			// from 8 to 8h30
			else thresholdAway = 8*60 + random.nextInt(30);
		}
		else thresholdAway = random.nextInt(1*60)+30; // from 0 to 90 minutes (errands)
	}


	public void setLocations() {

		Bag buildingsFiltered = PedSimCity.buildings.filterFeatures("DMA", "live", true);
		Random random = new Random();
		MasonGeometry buildingGeometry = (MasonGeometry) buildingsFiltered.get(random.nextInt(buildingsFiltered.size()));
		int buildingID = (int) buildingGeometry.getUserData();
		this.homePlace = PedSimCity.buildingsMap.get(buildingID).node;

		List<String> education = new ArrayList<String>();
		education.add("general_education");
		education.add("education_research");
		if (this.student) buildingsFiltered = PedSimCity.buildings.filterFeatures("land_use", education, true);
		else if (this.worker) buildingsFiltered = PedSimCity.buildings.filterFeatures("DMA", "work", true);
		buildingGeometry = (MasonGeometry) buildingsFiltered.get(random.nextInt(buildingsFiltered.size()));
		buildingID = (int) buildingGeometry.getUserData();
		this.workPlace = PedSimCity.buildingsMap.get(buildingID).node;
	}

}
