/**
 * DijkstraAngularChange.java
 * It computes the cumulative angular change shortest path by employing the Dijkstra shortest-path algorithm
 * It uses the dual graph of the street network.
 *
 * It supports: landmark-, region-, barrier-based navigation.
 **/


package sim.app.geo.pedSimCity;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import sim.app.geo.urbanSim.EdgeGraph;
import sim.app.geo.urbanSim.NodeGraph;
import sim.app.geo.urbanSim.NodeWrapper;
import sim.app.geo.urbanSim.SubGraph;
import sim.app.geo.urbanSim.Utilities;
import sim.app.geo.urbanSim.Utilities.Path;
import sim.util.geo.GeomPlanarGraphDirectedEdge;

public class DijkstraAngularChange {

	NodeGraph originNode, destinationNode, primalDestinationNode, previousJunction;
	ArrayList<NodeGraph> visitedNodes, unvisitedNodes, centroidsToAvoid;
	HashMap<NodeGraph, NodeWrapper> mapWrappers =  new HashMap<NodeGraph, NodeWrapper>();
	SubGraph graph = new SubGraph();
	// it contemplates an attempt where navigation takes place by the convex-hull method (see below).
	boolean subGraph = true;

	AgentProperties ap = new AgentProperties();

	/**
	 * @param originNode the origin node (dual graph);
	 * @param destinationNode the destination node (dual graph);
	 * @param primalDestinationNode the actual final, primal destination Node;
	 * @param centroidsToAvoid the centroids (dual nodes, representing segments) already traversed in previous iterations, if applicable;
	 * @param previousJunction the previous primal junction, if any;
	 * @param ap the set of the properties that describe the agent;
	 */

	public Path dijkstraPath (NodeGraph originNode, NodeGraph destinationNode, NodeGraph primalDestinationNode,
			ArrayList<NodeGraph> centroidsToAvoid, NodeGraph previousJunction, AgentProperties ap) {

		this.ap = ap;
		this.originNode = originNode;
		this.destinationNode = destinationNode;
		this.primalDestinationNode = primalDestinationNode;
		if (centroidsToAvoid != null) this.centroidsToAvoid = new ArrayList<NodeGraph>(centroidsToAvoid);
		this.previousJunction = previousJunction;


		/**
		 * If region-based navigation, navigate only within the region subgraph, if origin and destination nodes belong to the same region.
		 * Otherwise, form a subgraph within a convex hull
		 **/

		if ((originNode.region == destinationNode.region) && (ap.regionBasedNavigation)) {
			graph = PedSimCity.regionsMap.get(originNode.region).dualGraph;
			originNode = graph.findNode(originNode.getCoordinate());
			destinationNode = graph.findNode(destinationNode.getCoordinate());
			if (centroidsToAvoid != null) centroidsToAvoid = graph.getChildNodes(centroidsToAvoid);
			// primalJunction is always the same;
		}

		// create graph from convex hull
		else if (subGraph == true) {
			ArrayList<EdgeGraph> containedEdges = PedSimCity.dualNetwork.edgesWithinSpace(originNode, destinationNode);
			graph = new SubGraph(PedSimCity.dualNetwork, containedEdges);
			originNode = graph.findNode(originNode.getCoordinate());
			destinationNode = graph.findNode(destinationNode.getCoordinate());
			if (centroidsToAvoid != null) centroidsToAvoid = graph.getChildNodes(centroidsToAvoid);
		}
		visitedNodes = new ArrayList<NodeGraph>();
		unvisitedNodes = new ArrayList<NodeGraph>();
		unvisitedNodes.add(originNode);

		// NodeWrapper = container for the metainformation about a Node
		NodeWrapper NodeWrapper = new NodeWrapper(originNode);
		NodeWrapper.gx = 0.0;
		if (previousJunction != null) NodeWrapper.commonPrimalJunction = previousJunction;
		mapWrappers.put(originNode, NodeWrapper);

		// add centroids to avoid in the visited set
		if (centroidsToAvoid != null) for (NodeGraph c : centroidsToAvoid) visitedNodes.add(c);

		while (unvisitedNodes.size() > 0) {
			// at the beginning it takes originNode
			NodeGraph currentNode = getClosest(unvisitedNodes);
			visitedNodes.add(currentNode);
			unvisitedNodes.remove(currentNode);
			findMinDistances(currentNode);
		}
		return reconstructPath(originNode, destinationNode);
	}

	private void findMinDistances(NodeGraph currentNode) {

		ArrayList<NodeGraph> adjacentNodes = currentNode.getAdjacentNodes();
		for (NodeGraph targetNode : adjacentNodes) {
			if (visitedNodes.contains(targetNode)) continue;

			/**
			 * Check if the current and the possible next centroid share in the primal graph the same junction as the current with
			 * its previous centroid --> if yes move on. This essential means that the in the primal graph you would go back to an
			 * already traversed node; but the dual graph wouldn't know.
			 */
			if (Utilities.commonPrimalJunction(targetNode, currentNode) == mapWrappers.get(currentNode).commonPrimalJunction)
				continue;

			EdgeGraph commonEdge = currentNode.getEdgeBetween(targetNode);

			// compute costs based on the navigation strategies.
			// compute errors in perception of road coasts with stochastic variables
			double error = 0.0;
			if (ap.barrierBasedNavigation) {
				List<Integer> positiveBarriers = targetNode.primalEdge.positiveBarriers;
				List<Integer> negativeBarriers = targetNode.primalEdge.negativeBarriers;
				if (positiveBarriers != null) error = Utilities.fromNormalDistribution(0.70, 0.10, "left");
				else if ((negativeBarriers != null) && (positiveBarriers == null)) error = Utilities.fromNormalDistribution(1.30, 0.10, "right");
				else error = Utilities.fromNormalDistribution(1, 0.10, null);
			}
			else error = Utilities.fromNormalDistribution(1, 0.10, null);
			double edgeCost = commonEdge.getDeflectionAngle() * error;
			if (edgeCost > 180) edgeCost = 180;
			if (edgeCost < 0) edgeCost = 0;

			GeomPlanarGraphDirectedEdge outEdge = currentNode.getDirectedEdgeBetween(targetNode);

			double tentativeCost;

			if (ap.usingGlobalLandmarks) {
				double globalLandmarkness = 0.0;
				if (ap.onlyAnchors) globalLandmarkness = LandmarkNavigation.globalLandmarknessDualNode(currentNode, targetNode,
						primalDestinationNode, true);
				else globalLandmarkness = LandmarkNavigation.globalLandmarknessDualNode(currentNode, targetNode, primalDestinationNode, false);
				double nodeLandmarkness = 1-globalLandmarkness*UserParameters.globalLandmarknessWeight;
				double nodeCost = nodeLandmarkness*edgeCost;
				tentativeCost = getBest(currentNode) + nodeCost;
			}

			else tentativeCost = getBest(currentNode) + edgeCost;

			if (getBest(targetNode) > tentativeCost) {
				NodeWrapper NodeWrapper = mapWrappers.get(targetNode);
				if (NodeWrapper == null) NodeWrapper = new NodeWrapper(targetNode);
				NodeWrapper.nodeFrom = currentNode;
				NodeWrapper.edgeFrom = outEdge;
				NodeWrapper.commonPrimalJunction = Utilities.commonPrimalJunction(currentNode, targetNode);
				NodeWrapper.gx = tentativeCost;
				mapWrappers.put(targetNode, NodeWrapper);
				unvisitedNodes.add(targetNode);
			}
		}
	}


	private NodeGraph getClosest(ArrayList<NodeGraph> nodes) {

		NodeGraph closest = null;
		for (NodeGraph node : nodes) {
			if (closest == null) closest = node;
			else if (getBest(node) < getBest(closest)) closest = node;
		}
		return closest;
	}

	Double getBest(NodeGraph target) {

		if (mapWrappers.get(target) == null) return Double.MAX_VALUE;
		else return mapWrappers.get(target).gx;
	}


	public Path reconstructPath(NodeGraph originNode, NodeGraph destinationNode) {

		Path path = new Path();
		path.edges = null;
		path.mapWrappers = null;

		HashMap<NodeGraph, NodeWrapper> mapTraversedWrappers =  new HashMap<NodeGraph, NodeWrapper>();
		ArrayList<GeomPlanarGraphDirectedEdge> sequenceEdges = new ArrayList<GeomPlanarGraphDirectedEdge>();
		NodeGraph step = destinationNode;
		mapTraversedWrappers.put(destinationNode, mapWrappers.get(destinationNode));

		// If the subgraph navigation hasn't worked, retry by using the full graph
		// --> it switches "subgraph" to false;

		if ((mapWrappers.get(destinationNode) == null) && (subGraph == true)) {
			subGraph = false;
			visitedNodes.clear();
			unvisitedNodes.clear();
			mapWrappers.clear();
			originNode = graph.getParentNode(originNode);
			destinationNode = graph.getParentNode(destinationNode);
			if (centroidsToAvoid != null) centroidsToAvoid = graph.getParentNodes(centroidsToAvoid);
			Path secondAttempt = this.dijkstraPath(originNode, destinationNode, primalDestinationNode, centroidsToAvoid, previousJunction, ap);
			return secondAttempt;
		}

		// check that the path has been formulated properly
		if (mapWrappers.size() == 1) return path;
		try {

			while (mapWrappers.get(step).nodeFrom != null) {
				GeomPlanarGraphDirectedEdge de = (GeomPlanarGraphDirectedEdge) step.primalEdge.getDirEdge(0);
				step = mapWrappers.get(step).nodeFrom;
				mapTraversedWrappers.put(step, mapWrappers.get(step));
				sequenceEdges.add(0, de);

				if (step == originNode) {
					GeomPlanarGraphDirectedEdge lastDe = (GeomPlanarGraphDirectedEdge) step.primalEdge.getDirEdge(0);
					sequenceEdges.add(0, lastDe);
					break;
				}
			}
		}
		catch(java.lang.NullPointerException e)  {return path;}

		path.edges = sequenceEdges;
		path.mapWrappers = mapTraversedWrappers;
		return path;
	}
}


