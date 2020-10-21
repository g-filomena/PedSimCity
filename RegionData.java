package sim.app.geo.pedSimCity;

import java.util.ArrayList;

import sim.app.geo.urbanSim.EdgeGraph;
import sim.app.geo.urbanSim.SubGraph;
import sim.app.geo.urbanSim.VectorLayer;
import sim.util.geo.MasonGeometry;

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


	public void assignLandmarks()
	{
		this.globalLandmarks = LandmarkNavigation.getLandmarks(this.buildings, ResearchParameters.globalLandmarkThreshold, "global");
		this.localLandmarks = LandmarkNavigation.getLandmarks(this.buildings, ResearchParameters.localLandmarkThreshold, "local");
	}
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
