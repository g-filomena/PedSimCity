package pedsimcity.elements;

import java.util.ArrayList;

import sim.util.geo.MasonGeometry;
import urbanmason.main.EdgeGraph;

public class Barrier {

	public int barrierID;
	public MasonGeometry masonGeometry;
	public ArrayList<EdgeGraph> edgesAlong = new ArrayList<EdgeGraph>();
	public String type;
}
