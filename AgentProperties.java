package sim.app.geo.pedSimCity;

import java.util.ArrayList;

import org.javatuples.Pair;

import sim.app.geo.urbanSim.NodeGraph;

/**
 * @param landmarkBasedNavigation using Landmarks y/n;
 * @param regionBasedNavigation using Regions y/n;
 * @param barrierBasedNavigation using Barriers y/n;
 * @param onlyAnchors when computing global landmarkness, it considers only landmarks anchoring the destination as possible; if false,
 * global landmark is considered as a possible distant landmark;
 */

public class AgentProperties
{

	public String criteria;
	double sd_error = 0.10;
	double agentKnowledge = 1.0;
	boolean landmarkBasedNavigation = false;
	boolean onlyAnchors = true;
	boolean barrierBasedNavigation = false;
	boolean regionBasedNavigation = false;
	String localHeuristic = "roadDistance";
	String algorithm = "dijkstra";

	ArrayList<Pair<NodeGraph, NodeGraph>> OD =  new ArrayList<Pair<NodeGraph, NodeGraph>>();
	ArrayList<ArrayList<NodeGraph>> listSequences = new ArrayList<ArrayList<NodeGraph>> ();

	public void setProperties(String criteria) {
		this.criteria = criteria;
		if (criteria.contains("Landmarks")) landmarkBasedNavigation = true;
		if (criteria.contains("Barriers")) barrierBasedNavigation = true;
		if (criteria.contains("Regions")) regionBasedNavigation = true;
		if (criteria.contains("roadDistance")) localHeuristic = "roadDistance";
		else if (criteria.contains("angularChange")) localHeuristic = "angularChange";
		if (agentKnowledge <= researchParameters.noobAgentThreshold) onlyAnchors = false;
	}

	public void setOD(ArrayList<Pair<NodeGraph, NodeGraph>> OD, ArrayList<ArrayList<NodeGraph>> listSequences) {
		this.OD = OD;
		this.listSequences = new ArrayList<ArrayList<NodeGraph>> (listSequences);
	}

}
