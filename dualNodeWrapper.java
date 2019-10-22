package sim.app.geo.pedestrianSimulation;

import com.vividsolutions.jts.planargraph.Node;

import sim.util.geo.GeomPlanarGraphDirectedEdge;

public class dualNodeWrapper 
{
    Node node;
    Node nodeFrom;
    GeomPlanarGraphDirectedEdge edgeFrom;
    double gx, hx, fx;
    int commonPrimalJunction;

    public dualNodeWrapper(Node n)
    {
        node = n;
        gx = 0;
        hx = 0;
        fx = 0;
        nodeFrom = null;
        edgeFrom = null;
        commonPrimalJunction = Integer.MAX_VALUE;
    }
    
}
