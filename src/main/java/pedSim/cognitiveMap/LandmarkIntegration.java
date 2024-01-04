package pedSim.cognitiveMap;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;

import pedSim.engine.PedSimCity;
import sim.field.geo.VectorLayer;
import sim.graph.Building;
import sim.graph.Graph;
import sim.graph.NodeGraph;
import sim.util.geo.AttributeValue;
import sim.util.geo.MasonGeometry;

/**
 * Manages the integration of landmarks into a graph.
 */
public class LandmarkIntegration {

	// TODO check which this graph thing
	private Graph graph;

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
	public void setLocalLandmarkness(VectorLayer localLandmarks, Map<Integer, Building> buildingsMap, double radius) {

		List<NodeGraph> nodes = graph.nodesGraph;
		nodes.forEach((node) -> {
			List<MasonGeometry> containedLandmarks = localLandmarks
					.featuresWithinDistance(node.getMasonGeometry().geometry, radius);
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
	public void setGlobalLandmarkness(VectorLayer globalLandmarks, Map<Integer, Building> buildingsMap,
			double radiusAnchors, VectorLayer sightLines, int nrAnchors) {

		List<NodeGraph> nodes = graph.nodesGraph;

		nodes.forEach((node) -> {
			ArrayList<Building> anchors = new ArrayList<>(); //
			ArrayList<Double> distances = new ArrayList<>();

			List<MasonGeometry> containedLandmarks = globalLandmarks
					.featuresWithinDistance(node.getMasonGeometry().geometry, radiusAnchors);
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
				distances.add(building.geometry.distance(node.getMasonGeometry().geometry));
			}

			node.attributes.put("anchors", new AttributeValue(anchors));
			node.attributes.put("distances", new AttributeValue(distances));
		});

		List<MasonGeometry> sightLinesGeometries = sightLines.getGeometries();
		for (MasonGeometry sightLine : sightLinesGeometries) {
			Building building = buildingsMap.get(sightLine.getIntegerAttribute("buildingID"));
			NodeGraph node = PedSimCity.nodesMap.get(sightLine.getIntegerAttribute("nodeID"));
			if (node != null)
				node.visibleBuildings3d.add(building);
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
