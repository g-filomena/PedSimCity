package sim.app.geo.pedestrianSimulation;

import java.util.List;

import com.vividsolutions.jts.planargraph.Node;

import sim.util.geo.GeomPlanarGraphEdge;

public class EdgeData {
	
	int district;
	int roadType;
	
	GeomPlanarGraphEdge planarEdge;
	double distanceScaled;
	
    int roadDistance;
	int angularChange;
    int topological;
    int roadDistanceLandmark;
	int angularChangeLandmark;
    int localLandmarks;
    int globalLandmarks;
    int regionalRoutingRoadDistance;
    int regionalRoutingAngularChange;
    
    int toNode;
    int fromNode;
    double bC;
    
	List<Integer> positiveBarriers;
	List<Integer> negativeBarriers;

}
