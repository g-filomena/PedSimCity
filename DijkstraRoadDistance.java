/**
 * DijkstraRoadDistance.java
 * It computes the road (metric) distance shortest path by employing the Dijkstra shortest-path algorithm.
 * It uses the primal graph of the street network.
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
import sim.app.geo.urbanSim.Path;
import sim.app.geo.urbanSim.SubGraph;
import sim.app.geo.urbanSim.Utilities;
import sim.util.geo.GeomPlanarGraphDirectedEdge;


public class DijkstraRoadDistance {

	NodeGraph originNode, destinationNode, finalDestinationNode;
	ArrayList<NodeGraph> visitedNodes, unvisitedNodes;
	HashMap<NodeGraph, NodeWrapper> mapWrappers =  new HashMap<NodeGraph, NodeWrapper>();
	ArrayList<GeomPlanarGraphDirectedEdge> segmentsToAvoid = new ArrayList<GeomPlanarGraphDirectedEdge> ();
	ArrayList<EdgeGraph> edgesToAvoid = new ArrayList<EdgeGraph> ();
	SubGraph graph = new SubGraph();

	// it contemplates an attempt where navigation takes place by the convex-hull method (see below).
	boolean subGraph = UserParameters.subGraph;
	AgentProperties ap = new AgentProperties();
	/**
	 * @param originNode the origin node (it may be a sequence intermediate origin node, e.g. in landmark navigation);
	 * @param destinationNode the destination node (it may be a sequence intermediate destination node, e.g. in landmark navigation);
	 * @param finalDestinationNode the actual final destination Node;
	 * @param segmentsToAvoid street segments already traversed in previous iterations, if applicable;
	 * @param ap the set of the properties that describe the agent;

	 */
	public Path dijkstraPath(NodeGraph originNode, NodeGraph destinationNode, NodeGraph finalDestinationNode,
			ArrayList<GeomPlanarGraphDirectedEdge> segmentsToAvoid, AgentProperties ap) {
		this.ap = ap;
		this.originNode = originNode;
		this.destinationNode = destinationNode;
		this.finalDestinationNode = finalDestinationNode;
		if (segmentsToAvoid != null) this.segmentsToAvoid = new ArrayList<GeomPlanarGraphDirectedEdge>(segmentsToAvoid);

		// from the GeomPlanarGraphDirectedEdge get the actual EdgeGraph (safer)
		if (segmentsToAvoid != null) for (GeomPlanarGraphDirectedEdge e : segmentsToAvoid) edgesToAvoid.add((EdgeGraph) e.getEdge());

		// If region-based navigation, navigate only within the region subgraph, if origin and destination nodes belong to the same region.
		// Otherwise, form a subgraph within a convex hull

		if ((originNode.region == destinationNode.region) && (ap.regionBasedNavigation)) {
			graph = PedSimCity.regionsMap.get(originNode.region).primalGraph;
			originNode = graph.findNode(originNode.getCoordinate());
			destinationNode = graph.findNode(destinationNode.getCoordinate());
			if (segmentsToAvoid != null) edgesToAvoid =  graph.getChildEdges(edgesToAvoid);
		}
		// create graph from convex hull
		else if (subGraph) {
			ArrayList<EdgeGraph> containedEdges = PedSimCity.network.edgesWithinSpace(originNode, destinationNode);
			graph = new SubGraph(PedSimCity.network, containedEdges);
			originNode = graph.findNode(originNode.getCoordinate());
			destinationNode = graph.findNode(destinationNode.getCoordinate());
			if (segmentsToAvoid != null) edgesToAvoid = graph.getChildEdges(edgesToAvoid);
		}

		visitedNodes = new ArrayList<NodeGraph>();
		unvisitedNodes = new ArrayList<NodeGraph>();
		unvisitedNodes.add(originNode);

		// NodeWrapper = container for the metainformation about a Node
		NodeWrapper nodeWrapper = new NodeWrapper(originNode);
		nodeWrapper.gx = 0.0;
		mapWrappers.put(originNode, nodeWrapper);

		while (unvisitedNodes.size() > 0) {
			// at the beginning it takes originNode
			NodeGraph currentNode = getClosest(unvisitedNodes);
			visitedNodes.add(currentNode);
			unvisitedNodes.remove(currentNode);
			findMinDistances(currentNode);
		}
		return reconstructPath(originNode, destinationNode);
	}

	void findMinDistances(NodeGraph currentNode) {
		ArrayList<NodeGraph> adjacentNodes = currentNode.adjacentNodes;
		for (NodeGraph targetNode : adjacentNodes) {

			if (visitedNodes.contains(targetNode)) continue;
			EdgeGraph commonEdge = currentNode.getEdgeWith(targetNode);
			GeomPlanarGraphDirectedEdge outEdge = (GeomPlanarGraphDirectedEdge) commonEdge.getDirEdge(0);

			if (segmentsToAvoid == null);
			else if (edgesToAvoid.contains(outEdge.getEdge()))	continue;
			double error = 0.0;

			// compute costs based on the navigation strategies.
			// compute errors in perception of road coasts with stochastic variables
			if (ap.barrierBasedNavigation) {
				List<Integer> positiveBarriers = commonEdge.positiveBarriers;
				List<Integer> negativeBarriers = commonEdge.negativeBarriers;
				if (positiveBarriers != null) error = Utilities.fromDistribution(0.70, 0.10, "left");
				else if (negativeBarriers != null) error = Utilities.fromDistribution(1.30, 0.10, "right");
				else error = Utilities.fromDistribution(1.0, 0.10, null);
			}
			else error = Utilities.fromDistribution(1.0, 0.10, null);
			double edgeCost = commonEdge.getLength()*error;

			double tentativeCost = 0.0;
			if (ap.usingGlobalLandmarks && NodeGraph.nodesDistance(targetNode, finalDestinationNode) > UserParameters.threshold3dVisibility) {
				double globalLandmarkness = LandmarkNavigation.globalLandmarknessNode(targetNode, finalDestinationNode, ap.onlyAnchors);
				double nodeLandmarkness = 1.0-globalLandmarkness*UserParameters.globalLandmarknessWeight;
				double nodeCost = edgeCost*nodeLandmarkness;
				tentativeCost = getBest(currentNode) + nodeCost;
			}
			else tentativeCost = getBest(currentNode) + edgeCost;
			if (getBest(targetNode) > tentativeCost) {
				NodeWrapper nodeWrapper = mapWrappers.get(targetNode);
				if (nodeWrapper == null) nodeWrapper = new NodeWrapper(targetNode);
				nodeWrapper.nodeFrom = currentNode;
				nodeWrapper.edgeFrom = outEdge;
				nodeWrapper.gx = tentativeCost;
				mapWrappers.put(targetNode, nodeWrapper);
				unvisitedNodes.add(targetNode);
			}
		}
	}

	// get the closest amongst unvisited nodes -- the ones that haven't been explored yet
	NodeGraph getClosest(ArrayList<NodeGraph> nodes) {
		NodeGraph closest = null;
		for (NodeGraph node : nodes) {
			if (closest == null) closest = node;
			else if (getBest(node) < getBest(closest)) closest = node;
		}
		return closest;
	}

	Double getBest(NodeGraph targetNode) {
		if (mapWrappers.get(targetNode) == null) return Double.MAX_VALUE;
		else return mapWrappers.get(targetNode).gx;
	}


	Path reconstructPath(NodeGraph originNode, NodeGraph destinationNode) {
		Path path = new Path();
		path.edges = null;
		path.mapWrappers = null;

		HashMap<NodeGraph, NodeWrapper> mapTraversedWrappers =  new HashMap<NodeGraph, NodeWrapper>();
		ArrayList<GeomPlanarGraphDirectedEdge> sequenceEdges = new ArrayList<GeomPlanarGraphDirectedEdge>();
		NodeGraph step = destinationNode;
		mapTraversedWrappers.put(destinationNode, mapWrappers.get(destinationNode));

		/**
		 * If the subgraph navigation hasn't worked, retry by using the full graph
		 * --> it switches "subgraph" to false;
		 */
		if ((mapWrappers.get(destinationNode) == null) && (subGraph == true)) {
			subGraph = false;
			visitedNodes.clear();
			unvisitedNodes.clear();
			mapWrappers.clear();
			originNode = graph.getParentNode(originNode);
			destinationNode = graph.getParentNode(destinationNode);
			Path secondAttempt = this.dijkstraPath(originNode, destinationNode, finalDestinationNode, segmentsToAvoid, ap);
			return secondAttempt;
		}

		// check that the path has been formulated properly
		// no path
		if (mapWrappers.size() == 1) return path;
		try {
			while (mapWrappers.get(step).nodeFrom != null) {
				GeomPlanarGraphDirectedEdge dd = mapWrappers.get(step).edgeFrom;
				step = mapWrappers.get(step).nodeFrom;
				sequenceEdges.add(0, dd);
				mapTraversedWrappers.put(step, mapWrappers.get(step));
			}
		}
		// no path
		catch(java.lang.NullPointerException e)	{return path;}
		path.edges = sequenceEdges;
		path.mapWrappers = mapTraversedWrappers;
		return path;
	}

}


