package pedsimcity.elements;

import org.javatuples.Pair;

import urbanmason.main.NodeGraph;

public class Gateway {

	public NodeGraph exit;
	public NodeGraph entry;
	public Pair<NodeGraph, NodeGraph> gatewayID;
	public Integer nodeID;
	public Integer regionTo;
	public Integer edgeID;
	public Double distance;
	public boolean cognitiveMap;
	public Double entryAngle;
}
