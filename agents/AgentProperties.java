package pedsimcity.agents;

import java.util.ArrayList;

import org.javatuples.Pair;

import pedsimcity.graph.NodeGraph;

public class AgentProperties {

	public int agentID;
	public String criteria;
	public double agentKnowledge = 1.0;

	// for general routing
	public String localHeuristic = "";
	public String onlyMinimising = "";
	public String routeChoice;
	public ArrayList<Pair<NodeGraph, NodeGraph>> OD = new ArrayList<>();

	// landmarkNavigation related parameters
	public boolean landmarkBasedNavigation = false;
	public boolean usingDistantLandmarks = false;
	public boolean onlyAnchors = true;
	// for computing the complexity of the environment ["local", "global"]
	public String typeLandmarks = "";

	// region- and barrier-based parameters
	public boolean regionBasedNavigation = false;
	public boolean barrierBasedNavigation = false;
	public double naturalBarriers = 0.0;
	public double naturalBarriersSD = 0.10;
	public double severingBarriers = 0.0;
	public double severingBarriersSD = 0.10;
	public boolean preferenceNaturalBarriers = false;
	public boolean aversionSeveringBarriers = false;
	// the ones possibly used as sub-goals ["all", "positive", "negative",
	// "separating"]
	public String typeBarriers = "";

	/**
	 * @param landmarkBasedNavigation using Landmarks y/n;
	 * @param onlyAnchors             when computing global landmarkness, it
	 *                                considers only landmarks anchoring the
	 *                                destination as possible; if false, global
	 *                                landmark is considered as a possible distant
	 *                                landmark;
	 */
	public void setRouteChoice(String routeChoice) {

		this.routeChoice = routeChoice;

		if (routeChoice == "DS")
			this.onlyMinimising = "roadDistance";
		if (routeChoice == "AC")
			this.onlyMinimising = "angularChange";
		if (routeChoice == "TS")
			this.onlyMinimising = "turns";

		if (routeChoice.contains("D"))
			this.localHeuristic = "roadDistance";
		if (routeChoice.contains("A"))
			this.localHeuristic = "angularChange";
		if (routeChoice.contains("T"))
			this.localHeuristic = "turns";

		if (routeChoice.contains("L")) {
			this.landmarkBasedNavigation = true;
			this.typeLandmarks = "local"; // for measuring complexity when selecting on-route marks
		}
		if (routeChoice.contains("G")) {
			this.usingDistantLandmarks = true;
			this.onlyAnchors = true;
		}

		if (routeChoice.contains("R"))
			this.regionBasedNavigation = true;

		if (routeChoice.contains("B")) {
			this.barrierBasedNavigation = true;
			this.preferenceNaturalBarriers = true;
			this.aversionSeveringBarriers = true;
			this.naturalBarriers = 0.70;
			this.severingBarriers = 1.30;
			this.typeBarriers = "all";
		}
	}

	public void setOD(ArrayList<Pair<NodeGraph, NodeGraph>> OD, ArrayList<ArrayList<NodeGraph>> listSequences) {
		this.OD = OD;

	}
}
