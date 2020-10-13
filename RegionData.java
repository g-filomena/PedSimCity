package sim.app.geo.pedSimCity;

import java.util.ArrayList;
import java.util.HashMap;

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
    	this.globalLandmarks = LandmarkNavigation.getLandmarks(this.buildings, PedSimCity.globalLandmarkThreshold, "global");
    	if (globalLandmarks.size() > 0 && globalLandmarks != null) this.buildingsComplexity = LandmarkNavigation.buildingsComplexity(
    			this.buildings, globalLandmarks);
    	this.localLandmarks = LandmarkNavigation.getLandmarks(this.buildings, PedSimCity.localLandmarkThreshold, "local"); 
    }
}
