package sim.app.geo.pedestrianSimulation;

import com.vividsolutions.jts.planargraph.Node;

import sim.util.geo.GeomPlanarGraphDirectedEdge;

public class DualNodeWrapper 
{
    Node node;
    Node nodeFrom;
    GeomPlanarGraphDirectedEdge edgeFrom;
    double gx, hx, fx;
    int commonPrimalJunction;
    int nodesSoFar;
    double pathCost, nodeLandmarkness, pathLandmarkness;

    public DualNodeWrapper(Node n)
    {
        node = n;
        gx = 0;
        hx = 0;
        fx = 0;
        nodeFrom = null;
        edgeFrom = null;
        commonPrimalJunction = Integer.MAX_VALUE;
        pathCost = 0.0;
        nodeLandmarkness = 0.0;
        pathLandmarkness = 0.0;
    }
    
}
