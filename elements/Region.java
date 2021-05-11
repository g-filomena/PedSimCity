package pedsimcity.elements;
import java.util.ArrayList;

import pedsimcity.graph.EdgeGraph;
import pedsimcity.graph.SubGraph;
import pedsimcity.main.UserParameters;
import pedsimcity.routeChoice.LandmarkNavigation;
import pedsimcity.utilities.VectorLayer;
import sim.util.Bag;


/**
 * To store information about regions.
 *
 */
public class Region {

	public SubGraph primalGraph;
	public SubGraph dualGraph;
	public VectorLayer regionNetwork;
	double buildingsComplexity;

	public ArrayList<EdgeGraph> edges = new ArrayList<EdgeGraph>();
	public ArrayList<Gateway> gateways = new ArrayList<Gateway>();
	public Bag buildings = new  Bag();
	public Bag localLandmarks = new  Bag();
	public Bag globalLandmarks = new Bag();

	/**
	 * Assign landmarks to this region.
	 */

	public void assignLandmarks()
	{
		this.globalLandmarks = LandmarkNavigation.getLandmarks(this.buildings, UserParameters.globalLandmarkThreshold, "global");
		this.localLandmarks = LandmarkNavigation.getLandmarks(this.buildings, UserParameters.localLandmarkThreshold, "local");
	}

	/**
	 * It computes the building-based complexity of this region, given a type of landmarkness.
	 * The latter can be "local" or "global"; in the first case the legibility of the area, and in turn the complexity, is computed on the basis
	 * of local landmarks. Global landmarks are used when passing "global"
	 *
	 * @param typeLandmarkness the type of landmarks to use when computing the complexity.
	 */
	public double computeComplexity(String typeLandmarkness) {
		if (this.buildings.size() == 0)  this.buildingsComplexity = 0.0;
		else if (typeLandmarkness == "global" && globalLandmarks.size() > 0)
			this.buildingsComplexity = LandmarkNavigation.buildingsComplexity(this.buildings, globalLandmarks);
		else if (localLandmarks.size() > 0)	this.buildingsComplexity = LandmarkNavigation.buildingsComplexity(this.buildings, localLandmarks);
		else this.buildingsComplexity = 1.0;
		return this.buildingsComplexity;
	}
}
