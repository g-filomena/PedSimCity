package pedSim.cognitiveMap;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

import sim.field.geo.VectorLayer;
import sim.graph.Building;
import sim.graph.Graph;
import sim.graph.NodeGraph;
import sim.graph.SubGraph;
import sim.util.geo.AttributeValue;
import sim.util.geo.MasonGeometry;

/**
 * Manages the integration of landmarks into a graph.
 */
public class LandmarkIntegration {

	private Graph graph;

	/**
	 * Constructs a LandmarkIntegration object for a given graph.
	 *
	 * @param graph the graph to which landmarks will be integrated.
	 */
	public LandmarkIntegration(Graph graph) {
		this.graph = graph;
	}

	/**
	 * It assigns to each node in the graph a list of local landmarks.
	 *
	 * @param localLandmarks the layer containing all the buildings possibly
	 *                       considered as local landmarks;
	 * @param buildingsMap   the map of buildings (buildingID, Building);
	 * @param radius         the maximum distance from a node to a building for the
	 *                       building to be considered a local landmark at the
	 *                       junction;
	 */
	public void setLocalLandmarkness(VectorLayer localLandmarks, HashMap<Integer, Building> buildingsMap,
			double radius) {

		final Collection<NodeGraph> nodes = this.graph.nodesMap.values();

		nodes.forEach((node) -> {
			ArrayList<MasonGeometry> containedLandmarks = localLandmarks
					.featuresWithinDistance(node.masonGeometry.geometry, radius);
			for (final MasonGeometry masonGeometry : containedLandmarks)
				node.adjacentBuildings.add(buildingsMap.get((int) masonGeometry.getUserData()));
		});
	}

	/**
	 * It assigns to each node in the graph a list of distant landmarks and their
	 * corresponding global landmarkness values.
	 *
	 * @param localLandmarks the layer containing all the buildings possibly
	 *                       considered as global landmarks;
	 * @param buildingsMap   the map of buildings (buildingID, Building);
	 * @param radiusAnchors  the distance radius within which a global landmark is
	 *                       considered to be an anchor of a node (when intended as
	 *                       destination node);
	 * @param sightLines     the layer containing the sight lines;
	 * @param nrAnchors      the max number of anchors per node, sorted by global
	 *                       landmarkness;
	 */
	public void setGlobalLandmarkness(VectorLayer globalLandmarks, HashMap<Integer, Building> buildingsMap,
			double radiusAnchors, VectorLayer sightLines, int nrAnchors) {

		final Collection<NodeGraph> nodes = this.graph.nodesMap.values();

		nodes.forEach((node) -> {
			final MasonGeometry nodeGeometry = node.masonGeometry;
			ArrayList<Building> anchors = new ArrayList<>(); //
			ArrayList<Double> distances = new ArrayList<>();

			ArrayList<MasonGeometry> containedLandmarks = globalLandmarks
					.featuresWithinDistance(node.masonGeometry.geometry, radiusAnchors);
			final List<Double> gScores = new ArrayList<>();

			if (nrAnchors != -1) {
				for (final MasonGeometry masonGeometry : containedLandmarks) {
					gScores.add(masonGeometry.getDoubleAttribute("gScore_sc"));

				}
				Collections.sort(gScores);
				Collections.reverse(gScores);
			}

			for (final Object landmark : containedLandmarks) {
				final MasonGeometry building = (MasonGeometry) landmark;
				if (nrAnchors != 999999 & building.getDoubleAttribute("gScore_sc") < gScores.get(nrAnchors - 1))
					continue;
				final int buildingID = (int) building.getUserData();
				anchors.add(buildingsMap.get(buildingID));
				distances.add(building.geometry.distance(nodeGeometry.geometry));
			}

			node.attributes.put("anchors", new AttributeValue(anchors));
			node.attributes.put("distances", new AttributeValue(distances));
		});

		final ArrayList<MasonGeometry> sightLinesGeometries = sightLines.getGeometries();
		for (final MasonGeometry sightLine : sightLinesGeometries) {
			final Building building = buildingsMap.get(sightLine.getIntegerAttribute("buildingID"));
			final NodeGraph node = this.graph.nodesMap.get(sightLine.getIntegerAttribute("nodeID"));
			if (node != null)
				node.visibleBuildings3d.add(building);
		}
	}

	/**
	 * Sets landmarks and visibility attributes for nodes within a given SubGraph
	 * subgraph. This method copies landmarks and visibility attributes from the
	 * corresponding parent graph's nodes to the nodes within the subgraph.
	 * 
	 * @param subGraph The SubGraph for which the landmark information is being set.
	 */
	public static void setSubGraphLandmarks(SubGraph subGraph) {
		final ArrayList<NodeGraph> childNodes = subGraph.getNodesList();

		for (final NodeGraph node : childNodes) {
			final NodeGraph parentNode = subGraph.getParentNode(node);
			node.visibleBuildings2d = parentNode.visibleBuildings2d;
			node.visibleBuildings3d = parentNode.visibleBuildings3d;
			node.adjacentBuildings = parentNode.adjacentBuildings;
			node.attributes.put("anchors", getAnchors(parentNode));
			node.attributes.put("distances", getDistances(parentNode));
		}
	}

	/**
	 * Retrieves the attribute value of the "anchors" for the given NodeGraph.
	 *
	 * @param node The NodeGraph from which to retrieve the attribute value.
	 * @return The attribute value associated with "anchors".
	 */
	public static AttributeValue getAnchors(NodeGraph node) {
		return node.attributes.get("anchors");
	}

	/**
	 * Retrieves the attribute value of the "distances" for the given NodeGraph.
	 *
	 * @param node The NodeGraph from which to retrieve the attribute value.
	 * @return The attribute value associated with "distances".
	 */
	public static AttributeValue getDistances(NodeGraph node) {
		return node.attributes.get("distances");
	}
}
