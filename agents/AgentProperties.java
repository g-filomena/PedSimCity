package pedsimcity.agents;

import java.util.ArrayList;

import org.javatuples.Pair;

import pedsimcity.main.UserParameters;
import urbanmason.main.NodeGraph;


public class AgentProperties {

	public int agentID;
	public String criteria;
	public double agentKnowledge = 1.0;

	// for general routing
	public String localHeuristic = null;
	public String onlyMinimising = null;
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
	public double meanNaturalBarriers = 0.0;
	public double meanSeveringBarriers = 0.0;
	public boolean preferenceNaturalBarriers = false;
	public boolean aversionSeveringBarriers = false;
	// the ones possibly used as sub-goals ["all", "positive", "negative", "separating"]
	public String typeBarriers = "";

	/**
	 * @param landmarkBasedNavigation using Landmarks y/n;
	 * @param onlyAnchors when computing global landmarkness, it considers only landmarks anchoring the destination as possible; if false,
	 * global landmark is considered as a possible distant landmark;
	 */
	public void setRouteChoice(String routeChoice) {

		this.routeChoice = routeChoice;

		if (routeChoice == "DS") onlyMinimising = "roadDistance";
		if (routeChoice == "AC") onlyMinimising = "angularChange";
		if (routeChoice == "TS") onlyMinimising = "turns";

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

		if (routeChoice.contains("R")) regionBasedNavigation = true;
		if (routeChoice.contains("B")) {
			barrierBasedNavigation = true;
			preferenceNaturalBarriers = true;
			aversionSeveringBarriers = true;
			meanNaturalBarriers = 0.70;
			meanSeveringBarriers = 1.30;
		}

		if (agentKnowledge <= UserParameters.noobAgentThreshold) onlyAnchors = false;
	}

	public void setOD(ArrayList<Pair<NodeGraph, NodeGraph>> OD, ArrayList<ArrayList<NodeGraph>> listSequences) {
		this.OD = OD;
		if (listSequences != null) this.listSequences = new ArrayList<ArrayList<NodeGraph>> (listSequences);
	}
}
