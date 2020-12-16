package sim.app.geo.PedSimCity;

import org.javatuples.Pair;

import sim.app.geo.UrbanSim.NodeGraph;

public class Gateway {

	NodeGraph exit;
	NodeGraph entry;
	Pair<NodeGraph, NodeGraph> gatewayID;
	Integer nodeID;
	Integer regionTo;
	Integer edgeID;
	Double distance;
	boolean cognitiveMap;
	Double entryAngle;
}
