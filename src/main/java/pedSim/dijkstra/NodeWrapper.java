package pedSim.dijkstra;

import org.locationtech.jts.planargraph.DirectedEdge;

import sim.graph.NodeGraph;

/**
 * This class stores nodes' metainformation and supports Dikstra-based
 * algorithms.
 */
public class NodeWrapper {

	public NodeGraph node;
	public NodeGraph nodeFrom;
	public DirectedEdge directedEdgeFrom;
	public NodeGraph commonPrimalJunction;
	public double gx, hx, fx, landmarkness;
	public int nodesSoFar;
	public double pathCost, nodeLandmarkness, pathLandmarkness;
	public NodeWrapper previousWrapper;

	/**
	 * Constructs a new NodeWrapper for the corresponding NodeGraph.
	 *
	 * @param node The corresponding NodeGraph.
	 */
	public NodeWrapper(NodeGraph node) {
		this.node = node;
		gx = 0;
		hx = 0;
		fx = 0;
		nodeFrom = null;
		directedEdgeFrom = null;
		commonPrimalJunction = null;
		pathCost = 0.0;
		nodeLandmarkness = 0.0;
		pathLandmarkness = 0.0;
	}
}
