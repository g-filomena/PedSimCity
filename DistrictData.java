package sim.app.geo.pedestrianSimulation;

import java.util.ArrayList;
import java.util.HashMap;

public class DistrictData {
	
	SubGraph primalGraph;
	SubGraph dualGraph;
	Integer districtID;
    ArrayList<EdgeGraph> edgesMap = new ArrayList<EdgeGraph>();
    ArrayList<GatewayData> gateways = new ArrayList<GatewayData>();
}
