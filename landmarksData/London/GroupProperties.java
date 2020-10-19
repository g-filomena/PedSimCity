package sim.app.geo.pedSimCity;

public class GroupProperties {

	double percentage;


	public String criteria;
	double sd_error = 0.10;
	double agentKnowledge = 1.0;
	boolean landmarkNavigation = false;
	boolean onlyAnchors = true;
	boolean barrierBasedNavigation = false;
	boolean regionBasedNavigation = false;
	String localHeuristic = "roadDistance";
	String algorithm = "dijkstra";

}
