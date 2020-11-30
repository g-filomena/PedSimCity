package sim.app.geo.pedSimCity;

import org.javatuples.Pair;

import sim.app.geo.urbanSim.NodeGraph;

public class Gateway {
	
	NodeGraph node;
	NodeGraph entry;
	Pair<NodeGraph, NodeGraph> gatewayID;
	Integer nodeID;
	Integer regionTo;
	Integer edgeID;
	Double distance;
	boolean cognitiveMap;
	Double entryAngle;
}
