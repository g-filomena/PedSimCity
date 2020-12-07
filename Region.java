package sim.app.geo.PedSimCity;

import java.util.ArrayList;

import sim.app.geo.UrbanSim.EdgeGraph;
import sim.app.geo.UrbanSim.SubGraph;
import sim.app.geo.UrbanSim.VectorLayer;


/**
 * To store information about regions.
 *
 */
public class Region {

	public SubGraph primalGraph;
	public SubGraph dualGraph;
	public VectorLayer regionNetwork;

	public ArrayList<EdgeGraph> edges = new ArrayList<EdgeGraph>();
	public ArrayList<Gateway> gateways = new ArrayList<Gateway>();
}
