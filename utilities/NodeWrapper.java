package pedsimcity.utilities;

import pedsimcity.graph.NodeGraph;
import sim.util.geo.GeomPlanarGraphDirectedEdge;

/**
 * It stores nodes' metainformation and supports shortest-path algorithms.
 *
 */

public class NodeWrapper {

	public NodeGraph node;
	public NodeGraph nodeFrom;
	public GeomPlanarGraphDirectedEdge edgeFrom;
	public NodeGraph commonPrimalJunction;
	public double gx, hx, fx, landmarkness;
	public int nodesSoFar;
	public double pathCost, nodeLandmarkness, pathLandmarkness;


	/**
	 * It stores nodes' metainformation and supports shortest-path algorithms.
	 *
	 * @param node the corresponding NodeGraph;
	 */
	public NodeWrapper(NodeGraph node) {
		this.node = node;
		gx = 0;
		hx = 0;
		fx = 0;
		nodeFrom = null;
		edgeFrom = null;
		commonPrimalJunction = null;
		pathCost = 0.0;
		nodeLandmarkness = 0.0;
		pathLandmarkness = 0.0;
	}
}



