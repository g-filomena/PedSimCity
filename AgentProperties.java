package sim.app.geo.PedSimCity;

import java.util.ArrayList;

import org.javatuples.Pair;

import sim.app.geo.UrbanSim.NodeGraph;


/**
 * @param landmarkBasedNavigation using Landmarks y/n;
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
	ArrayList<Pair<NodeGraph, NodeGraph>> OD =  new ArrayList<Pair<NodeGraph, NodeGraph>>();
	ArrayList<ArrayList<NodeGraph>> listSequences = new ArrayList<ArrayList<NodeGraph>> ();

	//landmarkNavigation related parameters
	boolean landmarkBasedNavigation = false;
	boolean usingLocalLandmarks = false;
	boolean usingGlobalLandmarks = false;
	boolean onlyAnchors = true;

	// for computing the complexity of the environment ["local", "global"]
	String typeLandmarks = "";


	public void setProperties(String routeChoice) {
		this.routeChoice = routeChoice;
		if (routeChoice.equals("RL") || routeChoice.equals("AL") || routeChoice.equals("LL") ) {
			usingLocalLandmarks = true;
			landmarkBasedNavigation = true;
			typeLandmarks = "local";
		}

		if (routeChoice.equals("GL")) {
			usingGlobalLandmarks = true;
			onlyAnchors = true;
		}
		if (routeChoice.equals("RL") || routeChoice.equals("AL")) {
			usingGlobalLandmarks = true;
			onlyAnchors = true;
		}
		if (routeChoice.contains("R")) localHeuristic = "roadDistance";
		else if (routeChoice.contains("A")) localHeuristic = "angularChange";
	}

	public void setOD(ArrayList<Pair<NodeGraph, NodeGraph>> OD, ArrayList<ArrayList<NodeGraph>> listSequences) {
		this.OD = OD;
		this.listSequences = new ArrayList<ArrayList<NodeGraph>> (listSequences);
	}
}
