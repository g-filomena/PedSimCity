package sim.app.geo.pedSimCity;

import java.util.ArrayList;

import sim.app.geo.urbanSim.EdgeGraph;
import sim.app.geo.urbanSim.SubGraph;
import sim.app.geo.urbanSim.VectorLayer;
import sim.util.geo.MasonGeometry;


/**
 * To store information about regions.
 *
 */
public class RegionData {

	SubGraph primalGraph;
	SubGraph dualGraph;
	VectorLayer regionNetwork;
	double buildingsComplexity;

	ArrayList<EdgeGraph> edgesMap = new ArrayList<EdgeGraph>();
	ArrayList<GatewayData> gateways = new ArrayList<GatewayData>();
	ArrayList<MasonGeometry> buildings = new  ArrayList<MasonGeometry>();
	ArrayList<MasonGeometry> localLandmarks = new  ArrayList<MasonGeometry>();
	ArrayList<MasonGeometry> globalLandmarks = new ArrayList<MasonGeometry>();

	/**
	 * Assign landmarks to this region.
	 */

	public void assignLandmarks()
	{
		this.globalLandmarks = LandmarkNavigation.getLandmarks(this.buildings, ResearchParameters.globalLandmarkThreshold, "global");
		this.localLandmarks = LandmarkNavigation.getLandmarks(this.buildings, ResearchParameters.localLandmarkThreshold, "local");
	}

	/**
	 * It computes the building-based complexity of this region, given a type of landmarkness.
	 * The latter can be "local" or "global"; in the first case the legibility of the area, and in turn the complexity, is computed on the basis
	 * of local landmarks. Global landmarks are used when passing "global"
	 *
	 * @param typeLandmarkness the type of landmarks to use when computing the complexity.
	 */
	public double computeComplexity(String typeLandmarkness)
	{
		if (this.buildings.size() == 0 || this.buildings == null)  this.buildingsComplexity = 0.0;
		else if (typeLandmarkness == "global" && globalLandmarks.size() > 0 && globalLandmarks != null)
		{
			this.buildingsComplexity = LandmarkNavigation.buildingsComplexity(this.buildings, globalLandmarks);
		}
		else if (localLandmarks.size() > 0 && localLandmarks != null)
		{
			this.buildingsComplexity = LandmarkNavigation.buildingsComplexity(this.buildings, localLandmarks);
		}
		else this.buildingsComplexity = 1.0;

		return this.buildingsComplexity;
	}
}
