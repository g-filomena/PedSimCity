package sim.app.geo.pedSimCity;

import java.util.Random;

public class Group {


	int groupID;
	double agentKnowledgeMin = 0.0;
	double agentKnowledgeMax = 1.0;
	boolean landmarkBasedNavigation = false;
	boolean regionBasedNavigation = false;
	boolean barrierBasedNavigation = false;
	boolean nodeBasedNavigation = false;
	String localHeuristic;


	private void group1(){
		// road distance only
		this.groupID = 1;
		this.localHeuristic = "roadDistance";
		this.agentKnowledgeMin = 0.75;
		this.agentKnowledgeMax = 1.0;
	}

	private void group2(){
		// road distance + landmarks only
		this.groupID = 2;
		this.localHeuristic = "roadDistance";
		this.agentKnowledgeMin = 0.50;
		this.agentKnowledgeMax = 0.75;
		this.landmarkBasedNavigation = true;
	}
	private void group3(){
		// road distance through nodes
		this.groupID = 3;
		this.localHeuristic = "roadDistance";
		this.agentKnowledgeMin = 0.50;
		this.agentKnowledgeMax = 0.75;
		this.nodeBasedNavigation = true;
	}
	private void group4(){
		// angularChange + landmarks nodes
		this.groupID = 4;
		this.localHeuristic = "angularChange";
		this.agentKnowledgeMin = 0.50;
		this.agentKnowledgeMax = 0.75;
		this.landmarkBasedNavigation = true;
	}
	private void group5(){
		// angularChange + regions and barriers
		this.groupID = 5;
		this.localHeuristic = "angularChange";
		this.agentKnowledgeMin = 0.25;
		this.agentKnowledgeMax = 0.50;
		this.regionBasedNavigation = true;
		this.barrierBasedNavigation = true;
	}
	private void group6(){
		// angularChange + regions, barriers and landmarks
		this.groupID = 6;
		this.localHeuristic = "angularChange";
		this.agentKnowledgeMin = 0.00;
		this.agentKnowledgeMax = 0.25;
		this.landmarkBasedNavigation = true;
		this.regionBasedNavigation = true;
		this.barrierBasedNavigation = true;
	}

	public void setAgentProperties(AgentProperties ap) {

		ap.group = this.groupID;
		ap.localHeuristic = this.localHeuristic = "angularChange";
		Random random = new Random();
		ap.agentKnowledge = this.agentKnowledgeMin + random.nextDouble() * (this.agentKnowledgeMax - this.agentKnowledgeMin);

		ap.landmarkBasedNavigation = this.landmarkBasedNavigation;
		ap.regionBasedNavigation = this.regionBasedNavigation;
		ap.barrierBasedNavigation = this.barrierBasedNavigation;
		ap.nodeBasedNavigation = this.nodeBasedNavigation;

	}

	public void configureGroup(int groupID) {
		if (groupID == 1) group1();
		else if (groupID == 2) group2();
		else if (groupID == 3) group3();
		else if (groupID == 4) group4();
		else if (groupID == 5) group5();
		else if (groupID == 6) group6();

	}
}


