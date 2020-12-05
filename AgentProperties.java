package sim.app.geo.PedSimCity;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.javatuples.Pair;
import sim.app.geo.UrbanSim.NodeGraph;


/**
 * @param landmarkBasedNavigation using Landmarks y/n;
 * @param regionBasedNavigation using Regions y/n;
 * @param barrierBasedNavigation using Barriers y/n;
 * @param onlyAnchors when computing global landmarkness, it considers only landmarks anchoring the destination as possible; if false,
 * global landmark is considered as a possible distant landmark;
 */

public class AgentProperties {

	public int agentID;
	public String criteria;
	double sd_error = 0.10;

	// for general routing
	String localHeuristic = "";
	public String routeChoice;
	// only when testingLandmarks or testingRegions
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

	public void setProperties(String criteria) {
		this.criteria = criteria;
		if (criteria.contains("Landmarks")) landmarkBasedNavigation = true;
		if (criteria.contains("Barriers")) barrierBasedNavigation = true;
		if (criteria.contains("Regions")) regionBasedNavigation = true;
		if (criteria.contains("roadDistance")) localHeuristic = "roadDistance";
		else if (criteria.contains("angularChange")) localHeuristic = "angularChange";
		if (agentKnowledge <= ResearchParameters.noobAgentThreshold) onlyAnchors = false;
	}
	
	public void setProperties(String routeChoice) {
		this.routeChoice = routeChoice;
		if (routeChoice.contains("Landmarks")) {
			landmarkBasedNavigation = true;
			typeLandmarks = "local";
		}

		if (routeChoice.equals("localLandmarks")) usingLocalLandmarks = true;
		else if (routeChoice.equals("globalLandmarks")) usingGlobalLandmarks = true;
		else if (routeChoice.contains("Landmarks")) {
			usingLocalLandmarks = true;
			usingGlobalLandmarks = true;
		}
		if (routeChoice.contains("Barriers")) {
			typeBarriers = "all";
			barrierBasedNavigation = true;
		}
		if (routeChoice.contains("Regions")) regionBasedNavigation = true;
		if (routeChoice.contains("roadDistance")) localHeuristic = "roadDistance";
		else if (routeChoice.contains("angularChange")) localHeuristic = "angularChange";
	}

	public void setOD(ArrayList<Pair<NodeGraph, NodeGraph>> OD, ArrayList<ArrayList<NodeGraph>> listSequences) {
		this.OD = OD;
		this.listSequences = new ArrayList<ArrayList<NodeGraph>> (listSequences);
	}
}
