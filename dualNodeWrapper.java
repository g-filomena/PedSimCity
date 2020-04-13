package sim.app.geo.pedestrianSimulation;



import sim.util.geo.GeomPlanarGraphDirectedEdge;

public class DualNodeWrapper 
{
	NodeGraph node;
	NodeGraph nodeFrom;
    GeomPlanarGraphDirectedEdge edgeFrom;
    double gx, hx, fx;
    NodeGraph commonPrimalJunction;
    int nodesSoFar;
    double pathCost, nodeLandmarkness, pathLandmarkness;

    public DualNodeWrapper(NodeGraph n)
    {
        node = n;
        gx = 0;
        hx = 0;
        fx = 0;
        nodeFrom = null;
        edgeFrom = null;
        commonPrimalJunction = null;
        pathCost = 0.0;
        nodeLandmarkness = 0.0;
        pathLandmarkness = 0.0;
    }
    
}
