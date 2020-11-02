package sim.app.geo.pedSimCity;

import java.util.ArrayList;
import java.util.Random;

import org.javatuples.Pair;

import sim.app.geo.urbanSim.NodeGraph;
import sim.util.Bag;
import sim.util.geo.MasonGeometry;

/**
 * @param landmarkBasedNavigation using Landmarks y/n;
 * @param regionBasedNavigation using Regions y/n;
 * @param barrierBasedNavigation using Barriers y/n;
 * @param onlyAnchors when computing global landmarkness, it considers only landmarks anchoring the destination as possible; if false,
 * global landmark is considered as a possible distant landmark;
 */

public class AgentProperties
{
	public int agentID;
	public int group;
	public String criteria;
	double sd_error = 0.10;
	double agentKnowledge = 1.0;

	boolean landmarkBasedNavigation = false;
	boolean onlyAnchors = true;
	boolean regionBasedNavigation = false;
	boolean barrierBasedNavigation = false;
	boolean nodeBasedNavigation = false;

	String localHeuristic = "roadDistance";
	String algorithm = "dijkstra";
	String typeOfBarriers = "all";
	String typeLandmarks = "local";

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


	ArrayList<Pair<NodeGraph, NodeGraph>> OD =  new ArrayList<Pair<NodeGraph, NodeGraph>>();
	ArrayList<ArrayList<NodeGraph>> listSequences = new ArrayList<ArrayList<NodeGraph>> ();


	public void setProperties(String criteria) {
		this.criteria = criteria;
		if (criteria.contains("Landmarks")) landmarkBasedNavigation = true;
		if (criteria.contains("Barriers")) barrierBasedNavigation = true;
		if (criteria.contains("Regions")) regionBasedNavigation = true;
		if (criteria.contains("roadDistance")) localHeuristic = "roadDistance";
		else if (criteria.contains("angularChange")) localHeuristic = "angularChange";
		if (agentKnowledge <= ResearchParameters.noobAgentThreshold) onlyAnchors = false;

	}

	public void setOD(ArrayList<Pair<NodeGraph, NodeGraph>> OD, ArrayList<ArrayList<NodeGraph>> listSequences) {
		this.OD = OD;
		this.listSequences = new ArrayList<ArrayList<NodeGraph>> (listSequences);
	}

	public void setAway()
	{
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

		Bag buildingsFiltered = new Bag();
		if (this.flaneur) buildingsFiltered = PedSimCity.buildings.filter("landUse", "hospitality", "equals");
		else buildingsFiltered = PedSimCity.buildings.filter("landUse", "residential", "equals");

		Random random = new Random();
		MasonGeometry buildingGeometry = (MasonGeometry) buildingsFiltered.get(random.nextInt(buildingsFiltered.size()));
		int buildingID = buildingGeometry.getIntegerAttribute("buildingID");
		this.homePlace = PedSimCity.buildingsMap.get(buildingID).node;

		if (this.student) buildingsFiltered = PedSimCity.buildings.filter("landUse", "education", "equals");
		else if (this.worker) buildingsFiltered = PedSimCity.buildings.filter("landUse", "work", "equals");
		buildingGeometry = (MasonGeometry) buildingsFiltered.get(random.nextInt(buildingsFiltered.size()));
		buildingID = buildingGeometry.getIntegerAttribute("buildingID");
		this.workPlace = PedSimCity.buildingsMap.get(buildingID).node;
	}



}
