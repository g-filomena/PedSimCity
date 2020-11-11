package sim.app.geo.pedSimCity;

import java.util.ArrayList;

import sim.app.geo.urbanSim.EdgeGraph;
import sim.util.geo.MasonGeometry;

public class Barrier {

	public int barrierID;
	public MasonGeometry masonGeometry;
	public ArrayList<EdgeGraph> edgesAlong = new ArrayList<EdgeGraph>();
	public String type;
}
