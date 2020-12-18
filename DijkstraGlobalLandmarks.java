/**
 * DijkstraGlobalLandmarks.java
 *
 * It computes the path that maximises global landmarkness between an origin and a destination.
 * It uses the primal graph of the street network.
 *
 */

package sim.app.geo.PedSimCity;
import java.util.ArrayList;
import java.util.HashMap;

import sim.app.geo.UrbanSim.EdgeGraph;
import sim.app.geo.UrbanSim.NodeGraph;
import sim.app.geo.UrbanSim.NodeWrapper;
import sim.app.geo.UrbanSim.Path;
import sim.app.geo.UrbanSim.SubGraph;
import sim.util.geo.GeomPlanarGraphDirectedEdge;

public class DijkstraGlobalLandmarks {

	NodeGraph originNode, destinationNode, finalDestinationNode;
	ArrayList<NodeGraph> visitedNodes, unvisitedNodes;
	HashMap<NodeGraph, NodeWrapper> mapWrappers =  new HashMap<NodeGraph, NodeWrapper>();
	ArrayList<GeomPlanarGraphDirectedEdge> segmentsToAvoid;
	ArrayList<EdgeGraph> edgesToAvoid = new ArrayList<EdgeGraph> ();
	AgentProperties ap = new AgentProperties();
	SubGraph graph = new SubGraph();

	/**
	 * @param originNode the origin node;
	 * @param destinationNode the final destination Node;
	 * @param segmentsToAvoid street segments already traversed in previous iterations, if applicable;
	 * @param ap the agent properties;
	 */
	public Path dijkstraPath (NodeGraph originNode, NodeGraph destinationNode, NodeGraph finalDestinationNode, ArrayList<GeomPlanarGraphDirectedEdge> segmentsToAvoid,
			AgentProperties ap) {

		this.ap = ap;
		this.originNode = originNode;
		this.destinationNode = destinationNode;
		if (segmentsToAvoid != null) this.segmentsToAvoid = new ArrayList<GeomPlanarGraphDirectedEdge>(segmentsToAvoid);

		System.out.println(originNode.region + "  nodeID "+originNode.getID());
		// If region-based navigation, navigate only within the region subgraph, if origin and destination nodes belong to the same region.
		if (originNode.region == destinationNode.region && ap.regionBasedNavigation) {
			graph = PedSimCity.regionsMap.get(originNode.region).primalGraph;
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

		// from the GeomPlanarGraphDirectedEdge get the actual EdgeGraph (safer)
		if (segmentsToAvoid != null) for (GeomPlanarGraphDirectedEdge e : segmentsToAvoid) edgesToAvoid.add((EdgeGraph) e.getEdge());

		while (unvisitedNodes.size() > 0) {
			// at the beginning it takes originNode
			NodeGraph node = getClosest(unvisitedNodes);
			visitedNodes.add(node);
			unvisitedNodes.remove(node);
			findMinDistances(node);
		}
		return reconstructPath(originNode, destinationNode);
	}

	void findMinDistances(NodeGraph currentNode)
	{
		ArrayList<NodeGraph> adjacentNodes = currentNode.adjacentNodes;
		for (NodeGraph targetNode : adjacentNodes) {
			if (visitedNodes.contains(targetNode)) continue;

			EdgeGraph commonEdge = currentNode.getEdgeWith(targetNode);
			GeomPlanarGraphDirectedEdge outEdge = (GeomPlanarGraphDirectedEdge) commonEdge.getDirEdge(0);

			if (segmentsToAvoid == null);
			else if (edgesToAvoid.contains(outEdge.getEdge()))	continue;

			double globalLandmarkness = LandmarkNavigation.globalLandmarknessNode(targetNode, finalDestinationNode, ap.onlyAnchors);

			// the global landmarkness from the node is divided by the segment's length so to avoid that the path is not affected
			// by network (topological) distance
			double nodeLandmarkness = (1.0-globalLandmarkness)/commonEdge.getLength();
			double tentativeCost = getBest(currentNode) + nodeLandmarkness;

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

	//amongst unvisited (they have to have been explored)
	NodeGraph getClosest(ArrayList<NodeGraph> nodes) {
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


	Path reconstructPath(NodeGraph originNode, NodeGraph destinationNode) {
		Path path = new Path();
		path.edges = null;
		path.mapWrappers = null;

		HashMap<NodeGraph, NodeWrapper> mapTraversedWrappers =  new HashMap<NodeGraph, NodeWrapper>();
		ArrayList<GeomPlanarGraphDirectedEdge> sequenceEdges = new ArrayList<GeomPlanarGraphDirectedEdge>();
		NodeGraph step = destinationNode;
		mapTraversedWrappers.put(destinationNode, mapWrappers.get(destinationNode));

		// check that the path has been formulated properly
		if ((step == null) || (mapWrappers.size() == 1))  return path;
		try {
			while (mapWrappers.get(step).nodeFrom != null) {
				GeomPlanarGraphDirectedEdge dd = mapWrappers.get(step).edgeFrom;
				step = mapWrappers.get(step).nodeFrom;
				sequenceEdges.add(0, dd);
				mapTraversedWrappers.put(step, mapWrappers.get(step));
			}
		}
		//no path
		catch(java.lang.NullPointerException e)	{return path;}

		path.edges = sequenceEdges;
		path.mapWrappers = mapTraversedWrappers;
		return path;
	}
}


