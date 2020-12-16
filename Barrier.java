package sim.app.geo.PedSimCity;

import java.util.ArrayList;

import sim.app.geo.UrbanSim.EdgeGraph;
import sim.util.geo.MasonGeometry;

public class Barrier {

	public int barrierID;
	public MasonGeometry masonGeometry;
	public ArrayList<EdgeGraph> edgesAlong = new ArrayList<EdgeGraph>();
	public String type;
}
