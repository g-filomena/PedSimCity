/**
 * AStarRoadDistance.java
 * This is built on AStar.java 2011 (by Sarah Wise, Mark Coletti, Andrew Crooks)
 *
 * It computes road (metric) distance shortest path by employing the A* shortest path algorithm.
 * It uses the primal graph of the street network.
 * It does not support navigation based on regions nor landmarks.
 * It does support barrier-based navigation.
 **/

package sim.app.geo.pedSimCity;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import sim.app.geo.urbanSim.EdgeGraph;
import sim.app.geo.urbanSim.NodeGraph;
import sim.app.geo.urbanSim.NodeWrapper;
import sim.app.geo.urbanSim.Utilities;
import sim.app.geo.urbanSim.Utilities.Path;
import sim.util.geo.GeomPlanarGraphDirectedEdge;

public class AStarRoadDistance
{
	NodeGraph originNode, destinationNode;
	ArrayList<NodeGraph> visitedNodes, unvisitedNodes;
	boolean barrierBasedNavigation;
	HashMap<NodeGraph, NodeWrapper> mapWrappers =  new HashMap<NodeGraph, NodeWrapper>();

	/**
	 * @param originNode the originNode;
	 * @param destinationNode the destinationNode;
	 * @param segmentsToAvoid street segments already traversed in previous iterations, if applicable;
	 * @param ap the set of the properties that describe the agent;
	 */

	public Path astarPath(NodeGraph originNode, NodeGraph destinationNode, ArrayList<GeomPlanarGraphDirectedEdge> segmentsToAvoid,
			AgentProperties ap) {
		// set up the containers for the sequenceEdges
		this.originNode = originNode;
		this.destinationNode = destinationNode;
		this.barrierBasedNavigation = ap.barrierBasedNavigation;

		ArrayList<GeomPlanarGraphDirectedEdge> sequenceEdges =  new ArrayList<GeomPlanarGraphDirectedEdge>();

		// NodeWrapper = container for the metainformation about a Node
		NodeWrapper originNodeWrapper = new NodeWrapper(originNode);
		NodeWrapper destinationNodeWrapper = new NodeWrapper(destinationNode);
		mapWrappers.put(originNode, originNodeWrapper);
		mapWrappers.put(destinationNode, destinationNodeWrapper);

		originNodeWrapper.gx = 0;
		originNodeWrapper.hx = 0;
		originNodeWrapper.fx = originNodeWrapper.gx + originNodeWrapper.hx;

		// A* containers: nodes to be investigated vs the ones that have been investigated
		ArrayList<NodeWrapper> closedSet = new ArrayList<NodeWrapper>();
		ArrayList<NodeWrapper> openSet = new ArrayList<NodeWrapper>();
		//add the originNode Wrapper
		openSet.add(originNodeWrapper);

		while (openSet.size() > 0) {
			// while there are reachable nodes to investigate
			// find the shortest path so far
			NodeWrapper currentNodeWrapper = findMin(openSet);
			NodeGraph currentNode = currentNodeWrapper.node;
			if (currentNode == destinationNode) return reconstructPath(destinationNodeWrapper); //found
			openSet.remove(currentNodeWrapper);
			closedSet.add(currentNodeWrapper);

			// check all the edges out from this Node
			ArrayList<GeomPlanarGraphDirectedEdge> outEdges = new ArrayList<GeomPlanarGraphDirectedEdge> (currentNode.getOutDirectedEdges());

			for (GeomPlanarGraphDirectedEdge outEdge : outEdges) {
				NodeGraph targetNode = null;
				EdgeGraph commonEdge = (EdgeGraph) outEdge.getEdge();

				// skip segments already traversed in previous iterations
				if (segmentsToAvoid == null);
				else if (segmentsToAvoid.contains(outEdge)) continue;
				targetNode = commonEdge.getOtherNode(currentNode);

				NodeWrapper nextNodeWrapper;
				if (mapWrappers.containsKey(targetNode)) nextNodeWrapper =  mapWrappers.get(targetNode);
				else {
					nextNodeWrapper = new NodeWrapper(targetNode);
					mapWrappers.put(targetNode, nextNodeWrapper);
				}
				// if it has already been considered
				if (closedSet.contains(nextNodeWrapper)) continue;

				// otherwise evaluate the cost of this node/edge combo and
				// compute errors in perception of road coasts with stochastic variables
				double error = 0.0;
				if (barrierBasedNavigation) {
					List<Integer> positiveBarriers = commonEdge.positiveBarriers;
					List<Integer> negativeBarriers = commonEdge.negativeBarriers;
					if (positiveBarriers != null) error = Utilities.fromNormalDistribution(0.70, 0.10, "left");
					else if (negativeBarriers != null) error = Utilities.fromNormalDistribution(1.30, 0.10, "right");
					else error = Utilities.fromNormalDistribution(1, 0.10, null);
				}
				else error = Utilities.fromNormalDistribution(1, 0.10, null);
				double tentativeCost = currentNodeWrapper.gx + commonEdge.getLength()*error;
				boolean better = false;

				if (!openSet.contains(nextNodeWrapper)) {
					openSet.add(nextNodeWrapper);
					nextNodeWrapper.hx = Utilities.nodesDistance(targetNode, destinationNode);
					better = true;
				}
				else if (tentativeCost < nextNodeWrapper.gx) better = true;

				// store A* information about this promising candidate node
				if (better) {
					nextNodeWrapper.nodeFrom = currentNodeWrapper.node;
					nextNodeWrapper.edgeFrom = outEdge;
					nextNodeWrapper.gx = tentativeCost;
					nextNodeWrapper.fx = nextNodeWrapper.gx + nextNodeWrapper.hx;
				}
			}
		}
		Path path = new Path();
		path.edges = sequenceEdges;
		path.mapWrappers = mapWrappers;
		return path;
	}

	// path reconstruction, given the last nodeWrapper
	Path reconstructPath(NodeWrapper nodeWrapper)
	{
		ArrayList<GeomPlanarGraphDirectedEdge> sequenceEdges =  new ArrayList<GeomPlanarGraphDirectedEdge>();
		NodeWrapper currentWrapper = nodeWrapper;

		while (currentWrapper.nodeFrom != null) {
			// add this edge to the front of the list
			sequenceEdges.add(0, currentWrapper.edgeFrom);
			currentWrapper = mapWrappers.get(currentWrapper.nodeFrom);
		}
		Path path = new Path();
		path.edges = sequenceEdges;
		path.mapWrappers = mapWrappers;
		return path;
	}


	NodeWrapper findMin(ArrayList<NodeWrapper> set) {
		double min = 100000;
		NodeWrapper minNode = null;

		for (NodeWrapper n : set) {
			if (n.fx < min) {
				min = n.fx;
				minNode = n;
			}
		}
		return minNode;
	}

}