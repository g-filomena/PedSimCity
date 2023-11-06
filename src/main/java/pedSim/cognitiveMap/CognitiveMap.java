package pedSim.cognitiveMap;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;

import org.javatuples.Pair;

import pedSim.engine.Parameters;
import pedSim.engine.PedSimCity;
import sim.field.geo.VectorLayer;
import sim.graph.Building;
import sim.graph.NodeGraph;
import sim.util.geo.MasonGeometry;

/**
 * This class represent a community share cognitive map (or Image of the City)
 * used for storing meaningful information about the environment and, in turn,
 * navigating.
 */
public class CognitiveMap {

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
	public static HashMap<Pair<NodeGraph, NodeGraph>, Gateway> gatewaysMap = new HashMap<>();

	/**
	 * Stores barriers as a VectorLayer.
	 */
	protected static VectorLayer barriers;

	/**
	 * Singleton instance of the CognitiveMap.
	 */
	private static final CognitiveMap instance = new CognitiveMap();

	/**
	 * Sets up the community cognitive map.
	 */
	public static void setCommunityCognitiveMap() {

		if (!PedSimCity.buildings.getGeometries().isEmpty()) {
			identifyLandmarks();
			integrateLandmarks();
		}
		identifyRegionElements();
//		computeViewFields();
	}

	/**
	 * Gets the singleton instance of CognitiveMap.
	 *
	 * @return The CognitiveMap instance.
	 */
	public static CognitiveMap getInstance() {
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
	private static void identifyRegionElements() {

		boolean integrateLandmarks = false;
		if (!PedSimCity.buildings.getGeometries().isEmpty())
			integrateLandmarks = true;

		for (final Entry<Integer, Region> entry : PedSimCity.regionsMap.entrySet()) {
			Region region = entry.getValue();
			if (integrateLandmarks) {
				// set the landmarks of this region
				LandmarkIntegration.setSubGraphLandmarks(region.primalGraph);
				region.buildings = Building.getBuildingsWithinRegion(region);
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

		ArrayList<MasonGeometry> buildingsGeometries = buildings.getGeometries();
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
	public ArrayList<MasonGeometry> getRegionLocalLandmarks(Region region) {
		return region.localLandmarks;
	}

	/**
	 * Gets global landmarks for a specific region.
	 *
	 * @param region The region for which to get global landmarks.
	 * @return A list of global landmarks.
	 */
	public ArrayList<MasonGeometry> getRegionGlobalLandmarks(Region region) {
		return region.globalLandmarks;
	}
}