package pedSim.cognitiveMap;

import org.javatuples.Pair;

import sim.graph.NodeGraph;

/**
 * Represents a gateway connecting two nodes.
 */
public class Gateway {

	/**
	 * The exit node of the gateway.
	 */
	public NodeGraph exit;

	/**
	 * The entry node of the gateway.
	 */
	public NodeGraph entry;

	/**
	 * The unique identifier for the gateway.
	 */
	public Pair<NodeGraph, NodeGraph> gatewayID;

	/**
	 * The identifier of the node associated with this gateway.
	 */
	public Integer nodeID;

	/**
	 * The region identifier for the region the gateway leads to.
	 */
	public Integer regionTo;

	/**
	 * The identifier of the edge associated with this gateway.
	 */
	public Integer edgeID;

	/**
	 * The distance of the gateway.
	 */
	public Double distance;

	/**
	 * Indicates whether this gateway is part of a cognitive map.
	 */
	public boolean cognitiveMap;

	/**
	 * The entry angle of the gateway.
	 */
	public Double entryAngle;
}
