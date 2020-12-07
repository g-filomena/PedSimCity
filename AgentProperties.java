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


	//region- and barrier-based parameters
	boolean regionBasedNavigation = false;
	boolean barrierBasedNavigation = false;
	// the ones possibly used as sub-goals ["all", "positive", "negative", "separating"]
	String typeBarriers = "";


	public void setProperties(String routeChoice) {
		this.routeChoice = routeChoice;
		if (routeChoice.equals("BB") || routeChoice.equals("BRB") ) {
			typeBarriers = "all";
			barrierBasedNavigation = true;
		}
		if (routeChoice.equals("RB") || routeChoice.equals("BRB")) regionBasedNavigation = true;
		localHeuristic = "angularChange";
	}

	public void setOD(ArrayList<Pair<NodeGraph, NodeGraph>> OD, ArrayList<ArrayList<NodeGraph>> listSequences) {
		this.OD = OD;
		this.listSequences = new ArrayList<ArrayList<NodeGraph>> (listSequences);
	}
}
