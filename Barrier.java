package sim.app.geo.pedsimcity;

import java.util.ArrayList;

import sim.app.geo.urbanmason.EdgeGraph;
import sim.util.geo.MasonGeometry;

public class Barrier {

	public int barrierID;
	public MasonGeometry masonGeometry;
	public ArrayList<EdgeGraph> edgesAlong = new ArrayList<EdgeGraph>();
	public String type;
}
