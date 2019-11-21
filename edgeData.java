package sim.app.geo.pedestrianSimulation;

import com.vividsolutions.jts.planargraph.Node;

import sim.util.geo.GeomPlanarGraphEdge;

public class EdgeData {
	
	int district;
	GeomPlanarGraphEdge planarEdge;
	double distanceScaled;

    int euclidean;
	int angular;
    int topological;
    int euclideanLand;
	int angularLand;
    int topologicalLand;
    int landmark;
    int landmarkL;
    int landmarkG;
    int districtRoutingEuclidean;
    int districtRoutingAngular;
    
    int toNode;
    int fromNode;
    double bC;

}
