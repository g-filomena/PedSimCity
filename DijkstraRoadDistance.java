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
import sim.app.geo.urbanSim.SubGraph;
import sim.app.geo.urbanSim.Utilities;
import sim.app.geo.urbanSim.Utilities.Path;
import sim.util.geo.GeomPlanarGraphDirectedEdge;


public class DijkstraRoadDistance {

	NodeGraph originNode, destinationNode, finalDestinationNode;
	ArrayList<NodeGraph> visitedNodes, unvisitedNodes;
	HashMap<NodeGraph, NodeWrapper> mapWrappers =  new HashMap<NodeGraph, NodeWrapper>();
	ArrayList<GeomPlanarGraphDirectedEdge> segmentsToAvoid;
	ArrayList<EdgeGraph> edgesToAvoid = new ArrayList<EdgeGraph> ();
	boolean landmarkBasedNavigation, regionBasedNavigation, barrierBasedNavigation, onlyAnchors;
	SubGraph graph = new SubGraph();
	boolean subGraph = true; // it contemplates an attempt where navigation takes place by the convex-hull method (see below).

	AgentProperties ap = new AgentProperties();
	/**
	 * @param originNode the origin node (it may be a sequence intermediate origin node, e.g. in landmark navigation);
	 * @param destinationNode the destination node (it may be a sequence intermediate destination node, e.g. in landmark navigation);
	 * @param finalDestinationNode the actual final destination Node;
	 * @param segmentsToAvoid street segments already traversed in previous iterations, if applicable;
	 * @param ap the set of the properties that describe the agent;

	 */
	public Path dijkstraPath(NodeGraph originNode, NodeGraph destinationNode, NodeGraph finalDestinationNode,
			ArrayList<GeomPlanarGraphDirectedEdge> segmentsToAvoid, AgentProperties ap)
	{
		this.ap = ap;
		this.originNode = originNode;
		this.destinationNode = destinationNode;
		this.finalDestinationNode = finalDestinationNode;
		this.segmentsToAvoid = new ArrayList<GeomPlanarGraphDirectedEdge>(segmentsToAvoid);

		this.landmarkBasedNavigation = ap.landmarkBasedNavigation;
		this.regionBasedNavigation = ap.regionBasedNavigation;
		this.barrierBasedNavigation = ap.barrierBasedNavigation;
		this.onlyAnchors = ap.onlyAnchors;

		// from the GeomPlanarGraphDirectedEdge get the actual EdgeGraph (safer)
		if (segmentsToAvoid != null) for (GeomPlanarGraphDirectedEdge e : segmentsToAvoid) edgesToAvoid.add((EdgeGraph) e.getEdge());

		/**
		 * If region-based navigation, navigate only within the region subgraph, if origin and destination nodes belong to the same region.
		 * Otherwise, form a subgraph within a convex hull
		 **/

		if ((originNode.region == destinationNode.region) && (regionBasedNavigation))
		{
			graph = PedSimCity.regionsMap.get(originNode.region).dualGraph;
			originNode = graph.findNode(originNode.getCoordinate());
			destinationNode = graph.findNode(destinationNode.getCoordinate());
			if (segmentsToAvoid != null) edgesToAvoid =  graph.getChildEdges(edgesToAvoid);
		}
		else if (subGraph == true) // create graph from convex hull
		{
			ArrayList<EdgeGraph> containedEdges = PedSimCity.network.edgesWithinSpace(originNode, destinationNode);
			graph = new SubGraph(PedSimCity.network, containedEdges);
			originNode = graph.findNode(originNode.getCoordinate());
			destinationNode = graph.findNode(destinationNode.getCoordinate());
			if (segmentsToAvoid != null) edgesToAvoid =  graph.getChildEdges(edgesToAvoid);
		}

		visitedNodes = new ArrayList<NodeGraph>();
		unvisitedNodes = new ArrayList<NodeGraph>();
		unvisitedNodes.add(originNode);

		// NodeWrapper = container for the metainformation about a Node
		NodeWrapper nodeWrapper = new NodeWrapper(originNode);
		nodeWrapper.gx = 0.0;
		mapWrappers.put(originNode, nodeWrapper);

		while (unvisitedNodes.size() > 0)
		{
			NodeGraph currentNode = getClosest(unvisitedNodes); // at the beginning it takes originNode
			visitedNodes.add(currentNode);
			unvisitedNodes.remove(currentNode);
			findMinDistances(currentNode);
		}
		return reconstructPath(originNode, destinationNode);
	}

	void findMinDistances(NodeGraph currentNode)
	{
		ArrayList<NodeGraph> adjacentNodes = currentNode.getAdjacentNodes();
		for (NodeGraph targetNode : adjacentNodes)
		{
			if (visitedNodes.contains(targetNode)) continue;

			EdgeGraph commonEdge = currentNode.getEdgeBetween(targetNode);
			GeomPlanarGraphDirectedEdge outEdge = (GeomPlanarGraphDirectedEdge) commonEdge.getDirEdge(0);

			if (segmentsToAvoid == null);
			else if (edgesToAvoid.contains(outEdge.getEdge()))	continue;
			double error = 0.0;

			// computing costs based on the navigation strategies.
			// computing errors in perception of road coasts with stochastic variables
			if (barrierBasedNavigation)
			{
				List<Integer> positiveBarriers = commonEdge.positiveBarriers;
				List<Integer> negativeBarriers = commonEdge.negativeBarriers;
				if (positiveBarriers != null) error = Utilities.fromNormalDistribution(0.70, 0.10, "left");
				else if (negativeBarriers != null) error = Utilities.fromNormalDistribution(1.30, 0.10, "right");
				else error = Utilities.fromNormalDistribution(1, 0.10, null);
			}
			else error = Utilities.fromNormalDistribution(1, 0.10, null);
			double edgeCost = commonEdge.getLength()*error;

			double tentativeCost = 0.0;
			if (landmarkBasedNavigation)
			{
				double globalLandmarkness = 0.0;
				if (onlyAnchors) globalLandmarkness = LandmarkNavigation.globalLandmarknessNode(targetNode, finalDestinationNode, true);
				else globalLandmarkness = LandmarkNavigation.globalLandmarknessNode(targetNode, finalDestinationNode, false);

				double nodeLandmarkness = 1-globalLandmarkness*researchParameters.globalLandmarknessWeight;
				double nodeCost = edgeCost*nodeLandmarkness;
				tentativeCost = getBest(currentNode) + nodeCost;
			}
			else tentativeCost = getBest(currentNode) + edgeCost;

			if (getBest(targetNode) > tentativeCost)
			{
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
	NodeGraph getClosest(ArrayList<NodeGraph> nodes)
	{
		NodeGraph closest = null;
		for (NodeGraph node : nodes)
		{
			if (closest == null) closest = node;
			else if (getBest(node) < getBest(closest)) closest = node;
		}
		return closest;
	}

	Double getBest(NodeGraph targetNode)
	{
		if (mapWrappers.get(targetNode) == null) return Double.MAX_VALUE;
		else return mapWrappers.get(targetNode).gx;
	}


	Path reconstructPath(NodeGraph originNode, NodeGraph destinationNode)

	{
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
		if (step == null || mapWrappers.size() == 1) return path; //no path
		try
		{
			while (mapWrappers.get(step).nodeFrom != null)
			{
				GeomPlanarGraphDirectedEdge dd = mapWrappers.get(step).edgeFrom;
				step = mapWrappers.get(step).nodeFrom;
				sequenceEdges.add(0, dd);
				mapTraversedWrappers.put(step, mapWrappers.get(step));
			}
		}
		catch(java.lang.NullPointerException e)	{return path;} //no path
		path.edges = sequenceEdges;
		path.mapWrappers = mapTraversedWrappers;
		return path;
	}

}


