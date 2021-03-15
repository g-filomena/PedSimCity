package pedsimcity.agents;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.javatuples.Pair;

import pedsimcity.main.PedSimCity;
import pedsimcity.main.UserParameters;
import sim.util.Bag;
import sim.util.geo.MasonGeometry;
import urbanmason.main.NodeGraph;


public class AgentProperties {

	public int agentID;
	public String criteria;
	public double agentKnowledge = 1.0;

	// for general routing
	public String localHeuristic = "";
	public String routeChoice;
	public ArrayList<Pair<NodeGraph, NodeGraph>> OD =  new ArrayList<Pair<NodeGraph, NodeGraph>>();
	public ArrayList<ArrayList<NodeGraph>> listSequences = new ArrayList<ArrayList<NodeGraph>> ();

	//landmarkNavigation related parameters
	public boolean landmarkBasedNavigation = false;
	public boolean usingGlobalLandmarks = false;
	public boolean onlyAnchors = true;
	// for computing the complexity of the environment ["local", "global"]
	public String typeLandmarks = "";

	//region- and barrier-based parameters
	public boolean regionBasedNavigation = false;
	public boolean barrierBasedNavigation = false;
	// the ones possibly used as sub-goals ["all", "positive", "negative", "separating"]
	public String typeBarriers = "";
	public boolean usingNaturalBarriers = false;
	public boolean avoidingSeveringBarriers = false;

	public boolean nodeBasedNavigation = false;

	/**
	 * @param landmarkBasedNavigation using Landmarks y/n;
	 * @param onlyAnchors when computing global landmarkness, it considers only landmarks anchoring the destination as possible; if false,
	 * global landmark is considered as a possible distant landmark;
	 */
	public void setRouteChoice(String routeChoice) {

		this.routeChoice = routeChoice;
		if (routeChoice.contains("D")) localHeuristic = "roadDistance";
		if (routeChoice.contains("A")) localHeuristic = "angularChange";
		if (routeChoice.contains("T")) localHeuristic = "turns";

		if (routeChoice.contains("L")) {
			landmarkBasedNavigation = true;
			typeLandmarks = "local"; //for measuring complexity when selecting on-route marks
		}
		if (routeChoice.contains("G")) {
			usingGlobalLandmarks = true;
			onlyAnchors = true;
		}
		if (routeChoice.contains("N")) {
			nodeBasedNavigation = true;
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


	public void setActivityProperties() {

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

		double p = random.nextFloat();
		if (p <= 0.45) worker = true;
		else if (p <= 0.70) student = true;
		else if (p <= 0.85 ) flaneur = true;
		else if (p <= 0.85 ) homeBased = true;
	}

}
