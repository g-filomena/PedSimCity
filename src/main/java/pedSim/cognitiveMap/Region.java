package pedSim.cognitiveMap;

import java.util.ArrayList;

import sim.field.geo.VectorLayer;
import sim.graph.EdgeGraph;
import sim.graph.SubGraph;
import sim.util.geo.MasonGeometry;

/**
 * The class represents a geographic region in the cognitive map. This class
 * stores information about regions, including their unique identifier, graphs,
 * network data, edges, gateways, buildings, and landmarks.
 */
public class Region {

	/** The unique identifier of the region. */
	public int regionID;

	/** The primal graph associated with this region. */
	public SubGraph primalGraph;

	/** The dual graph associated with this region. */
	public SubGraph dualGraph;

	/** The vector layer representing the region's network. */
	public VectorLayer regionNetwork;

	/** A list of edges within this region. */
	public ArrayList<EdgeGraph> edges = new ArrayList<>();

	/** A list of gateways within this region. */
	public ArrayList<Gateway> gateways = new ArrayList<>();

	/** A list of buildings within this region. */
	public ArrayList<MasonGeometry> buildings = new ArrayList<>();

	/** A list of local landmarks within this region. */
	public ArrayList<MasonGeometry> localLandmarks = new ArrayList<>();

	/** A list of global landmarks within this region. */
	public ArrayList<MasonGeometry> globalLandmarks = new ArrayList<>();
}
