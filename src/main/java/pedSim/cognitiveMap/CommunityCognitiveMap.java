package pedSim.cognitiveMap;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import org.javatuples.Pair;
import org.locationtech.jts.geom.Geometry;

import pedSim.engine.Parameters;
import pedSim.engine.PedSimCity;
import sim.field.geo.VectorLayer;
import sim.graph.Building;
import sim.graph.Graph;
import sim.graph.GraphUtils;
import sim.graph.NodeGraph;
import sim.util.geo.MasonGeometry;

/**
 * This class represent a community share cognitive map (or Image of the City)
 * used for storing meaningful information about the environment and, in turn,
 * navigating.
 */
public class CommunityCognitiveMap {

	/**
	 * Stores local landmarks as a VectorLayer.
	 */
	public static VectorLayer localLandmarks = new VectorLayer();

	/**
	 * Stores global landmarks as a VectorLayer.
	 */
	public static VectorLayer globalLandmarks = new VectorLayer();

	/**
	 * Maps pairs of nodes to gateways.
	 */
	public Map<Pair<NodeGraph, NodeGraph>, Gateway> gatewaysMap = new HashMap<>();

	/**
	 * Stores barriers as a VectorLayer.
	 */
	protected static VectorLayer barriers;

	/**
	 * Graphs for navigation.
	 */
	static Graph communityNetwork;
	static Graph communityDualNetwork;

	/**
	 * Singleton instance of the CognitiveMap.
	 */
	private static final CommunityCognitiveMap instance = new CommunityCognitiveMap();

	Building buildingsHandler = new Building();

	/**
	 * Sets up the community cognitive map.
	 */
	public void setCommunityCognitiveMap() {

		if (!PedSimCity.buildings.getGeometries().isEmpty()) {
			identifyLandmarks();
			integrateLandmarks();
		}
		identifyRegionElements();

		barriers = PedSimCity.barriers;
		setCommunityNetwork(PedSimCity.network, PedSimCity.dualNetwork);
	}

	private void setCommunityNetwork(Graph network, Graph dualNetwork) {
		communityNetwork = network;
		communityDualNetwork = dualNetwork;

	}

	/**
	 * Gets the singleton instance of CognitiveMap.
	 *
	 * @return The CognitiveMap instance.
	 */
	public static CommunityCognitiveMap getInstance() {
		return instance;
	}

	/**
	 * Identifies and sets both local and global landmarks in the buildings dataset.
	 */
	private static void identifyLandmarks() {
		setLandmarks(PedSimCity.buildings);
	}

	/**
	 * Integrates landmarks into the street network, sets local landmarkness, and
	 * computes global landmarkness values for nodes.
	 */
	private static void integrateLandmarks() {
		// Integrate landmarks into the street network
		List<Integer> globalLandmarksID = globalLandmarks.getIntColumn("buildingID");
		VectorLayer sightLinesLight = PedSimCity.sightLines.selectFeatures("buildingID", globalLandmarksID, true);
		// free up memory
		PedSimCity.sightLines = null;
		LandmarkIntegration landmarkIntegration = new LandmarkIntegration(PedSimCity.network);
		landmarkIntegration.setLocalLandmarkness(localLandmarks, PedSimCity.buildingsMap,
				Parameters.distanceNodeLandmark);
		landmarkIntegration.setGlobalLandmarkness(globalLandmarks, PedSimCity.buildingsMap, Parameters.distanceAnchors,
				sightLinesLight, Parameters.nrAnchors);
	}

	/**
	 * Integrates landmarks into the street network, sets local landmarkness, and
	 * computes global landmarkness values for nodes.
	 */
	private void identifyRegionElements() {

		boolean integrateLandmarks = false;
		if (!PedSimCity.buildings.getGeometries().isEmpty())
			integrateLandmarks = true;

		for (final Entry<Integer, Region> entry : PedSimCity.regionsMap.entrySet()) {
			Region region = entry.getValue();
			if (integrateLandmarks) {
				region.buildings = getBuildingsWithinRegion(region);
				setRegionLandmarks(region);
			}
			BarrierIntegration.setSubGraphBarriers(region.primalGraph);
		}
		barriers = PedSimCity.barriers;
	}

	/**
	 * Sets landmarks (local or global) from a set of buildings (VectorLayer) based
	 * on a threshold set initially by the user.
	 *
	 * @param buildings The set of buildings.
	 */
	private static void setLandmarks(VectorLayer buildings) {

		List<MasonGeometry> buildingsGeometries = buildings.getGeometries();
		for (final MasonGeometry building : buildingsGeometries) {
			if (building.getDoubleAttribute("lScore_sc") >= Parameters.localLandmarkThreshold)
				localLandmarks.addGeometry(building);
			if (building.getDoubleAttribute("gScore_sc") >= Parameters.globalLandmarkThreshold)
				globalLandmarks.addGeometry(building);
		}
	}

	/**
	 * Sets region landmarks (local or global) from a set of buildings (ArrayList)
	 * based on a threshold set initially by the user.
	 *
	 * @param region The region for which to set landmarks.
	 */
	private static void setRegionLandmarks(Region region) {

		for (MasonGeometry building : region.buildings) {
			if (building.getDoubleAttribute("lScore_sc") >= Parameters.localLandmarkThreshold)
				region.localLandmarks.add(building);
			if (building.getDoubleAttribute("gScore_sc") >= Parameters.globalLandmarkThreshold)
				region.globalLandmarks.add(building);
		}
	}

	/**
	 * Gets local landmarks for a specific region.
	 *
	 * @param region The region for which to get local landmarks.
	 * @return A list of local landmarks.
	 */
	public List<MasonGeometry> getRegionLocalLandmarks(Region region) {
		return region.localLandmarks;
	}

	/**
	 * Gets global landmarks for a specific region.
	 *
	 * @param region The region for which to get global landmarks.
	 * @return A list of global landmarks.
	 */
	public List<MasonGeometry> getRegionGlobalLandmarks(Region region) {
		return region.globalLandmarks;
	}

	/**
	 * Returns all the buildings enclosed between two nodes.
	 *
	 * @param originNode      The first node.
	 * @param destinationNode The second node.
	 * @return A list of buildings.
	 */
	public List<MasonGeometry> getBuildings(NodeGraph originNode, NodeGraph destinationNode) {
		Geometry smallestCircle = GraphUtils.enclosingCircleBetweenNodes(originNode, destinationNode);
		return PedSimCity.buildings.containedFeatures(smallestCircle);
	}

	/**
	 * Get buildings within a specified region.
	 *
	 * @param region The region for which buildings are to be retrieved.
	 * @return An ArrayList of MasonGeometry objects representing buildings within
	 *         the region.
	 */
	public List<MasonGeometry> getBuildingsWithinRegion(Region region) {
		VectorLayer regionNetwork = region.regionNetwork;
		Geometry convexHull = regionNetwork.getConvexHull();
		return PedSimCity.buildings.containedFeatures(convexHull);
	}

	public Graph getKnownNetwork() {
		return communityNetwork;
	}

	public Graph getKnownDualNetwork() {
		return communityDualNetwork;
	}
}
