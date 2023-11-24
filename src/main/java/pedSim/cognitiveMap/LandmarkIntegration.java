package pedSim.cognitiveMap;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

import pedSim.engine.PedSimCity;
import sim.field.geo.VectorLayer;
import sim.graph.Building;
import sim.graph.NodeGraph;
import sim.graph.SubGraph;
import sim.util.geo.AttributeValue;
import sim.util.geo.MasonGeometry;

/**
 * Manages the integration of landmarks into a graph.
 */
public class LandmarkIntegration {

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
	public static void setLocalLandmarkness(VectorLayer localLandmarks, HashMap<Integer, Building> buildingsMap,
			double radius) {

		Collection<NodeGraph> nodes = PedSimCity.network.nodesMap.values();

		nodes.forEach((node) -> {
			ArrayList<MasonGeometry> containedLandmarks = localLandmarks
					.featuresWithinDistance(node.masonGeometry.geometry, radius);
			for (MasonGeometry masonGeometry : containedLandmarks)
				node.adjacentBuildings.add(buildingsMap.get((int) masonGeometry.getUserData()));
		});
	}

	/**
	 * It assigns to each node in the graph a list of distant landmarks and their
	 * corresponding global landmarkness values.
	 *
	 * @param globalLandmarks the layer containing all the buildings possibly
	 *                        considered as global landmarks;
	 * @param buildingsMap    the map of buildings (buildingID, Building);
	 * @param radiusAnchors   the distance radius within which a global landmark is
	 *                        considered to be an anchor of a node (when intended as
	 *                        destination node);
	 * @param sightLines      the layer containing the sight lines;
	 * @param nrAnchors       the max number of anchors per node, sorted by global
	 *                        landmarkness;
	 */
	public static void setGlobalLandmarkness(VectorLayer globalLandmarks, HashMap<Integer, Building> buildingsMap,
			double radiusAnchors, VectorLayer sightLines, int nrAnchors) {

		Collection<NodeGraph> nodes = PedSimCity.network.nodesMap.values();

		nodes.forEach((node) -> {
			MasonGeometry nodeGeometry = node.masonGeometry;
			ArrayList<Building> anchors = new ArrayList<>(); //
			ArrayList<Double> distances = new ArrayList<>();

			ArrayList<MasonGeometry> containedLandmarks = globalLandmarks
					.featuresWithinDistance(node.masonGeometry.geometry, radiusAnchors);
			List<Double> gScores = new ArrayList<>();

			if (nrAnchors != -1) {
				for (MasonGeometry masonGeometry : containedLandmarks) {
					gScores.add(masonGeometry.getDoubleAttribute("gScore_sc"));

				}
				Collections.sort(gScores);
				Collections.reverse(gScores);
			}

			for (Object landmark : containedLandmarks) {
				MasonGeometry building = (MasonGeometry) landmark;
				if (nrAnchors != 999999 & building.getDoubleAttribute("gScore_sc") < gScores.get(nrAnchors - 1))
					continue;
				int buildingID = (int) building.getUserData();
				anchors.add(buildingsMap.get(buildingID));
				distances.add(building.geometry.distance(nodeGeometry.geometry));
			}

			node.attributes.put("anchors", new AttributeValue(anchors));
			node.attributes.put("distances", new AttributeValue(distances));
		});

		ArrayList<MasonGeometry> sightLinesGeometries = sightLines.getGeometries();
		for (MasonGeometry sightLine : sightLinesGeometries) {
			Building building = buildingsMap.get(sightLine.getIntegerAttribute("buildingID"));
			NodeGraph node = PedSimCity.network.nodesMap.get(sightLine.getIntegerAttribute("nodeID"));
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
		ArrayList<NodeGraph> childNodes = subGraph.getNodesList();

		for (NodeGraph node : childNodes) {
			NodeGraph parentNode = subGraph.getParentNode(node);
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
