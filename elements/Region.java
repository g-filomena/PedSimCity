package pedsimcity.elements;

import java.util.ArrayList;

import pedsimcity.graph.EdgeGraph;
import pedsimcity.graph.NodeGraph;
import pedsimcity.graph.SubGraph;
import pedsimcity.main.UserParameters;
import pedsimcity.routeChoice.LandmarkNavigation;
import pedsimcity.utilities.VectorLayer;
import sim.util.geo.MasonGeometry;

/**
 * To store information about regions.
 *
 */
public class Region {

	public int regionID;
	public SubGraph primalGraph;
	public SubGraph dualGraph;
	public VectorLayer regionNetwork;

	public ArrayList<EdgeGraph> edges = new ArrayList<>();
	public ArrayList<Gateway> gateways = new ArrayList<>();
	public ArrayList<MasonGeometry> buildings = new ArrayList<>();
	public ArrayList<MasonGeometry> localLandmarks = new ArrayList<>();
	public ArrayList<MasonGeometry> globalLandmarks = new ArrayList<>();

	/**
	 * Assign landmarks to this region.
	 */

	public void assignLandmarks() {
		this.globalLandmarks = LandmarkNavigation.getLandmarks(this.buildings, UserParameters.globalLandmarkThreshold,
				"global");
		this.localLandmarks = LandmarkNavigation.getLandmarks(this.buildings, UserParameters.localLandmarkThreshold,
				"local");
	}

	/**
	 * It computes the building-based complexity of this region, given a type of
	 * landmarkness. The latter can be "local" or "global"; in the first case the
	 * legibility of the area, and in turn the complexity, is computed on the basis
	 * of local landmarks. Global landmarks are used when passing "global"
	 *
	 * @param typeLandmarkness the type of landmarks to use when computing the
	 *                         complexity.
	 */
	public double computeComplexity(NodeGraph node, NodeGraph destinationNode, String typeLandmarkness) {

		ArrayList<MasonGeometry> landmarks = new ArrayList<>();
		ArrayList<MasonGeometry> buildings = new ArrayList<>();
		buildings = LandmarkNavigation.getBuildings(node, destinationNode, this.regionID);
		if (this.buildings.size() == 0)
			return 0.0;

		landmarks = this.findDynamicLandmarks(buildings, typeLandmarkness);
		return LandmarkNavigation.buildingsComplexity(buildings, landmarks);

	}

	private ArrayList<MasonGeometry> findDynamicLandmarks(ArrayList<MasonGeometry> buildings, String type) {

		ArrayList<MasonGeometry> landmarks;

		if (type.equals("local")) {
			landmarks = new ArrayList<>(this.localLandmarks);
			landmarks.retainAll(buildings);
		} else {
			landmarks = new ArrayList<>(this.globalLandmarks);
			landmarks.retainAll(buildings);
		}
		return landmarks;
	}
}
