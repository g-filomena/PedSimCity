package pedsimcity.elements;

import java.util.ArrayList;

import pedsimcity.graph.EdgeGraph;
import sim.util.geo.MasonGeometry;

public class Barrier {

	public int barrierID;
	public MasonGeometry masonGeometry;
	public ArrayList<EdgeGraph> edgesAlong = new ArrayList<EdgeGraph>();
	public String type;
}
