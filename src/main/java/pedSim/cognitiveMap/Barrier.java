package pedSim.cognitiveMap;

import java.util.ArrayList;

import sim.graph.EdgeGraph;
import sim.util.geo.MasonGeometry;

/**
 * Represents a barrier in the cognitive map, defined by a unique identifier,
 * geometry, edges it affects, and its type.
 */
public class Barrier {

	/**
	 * The unique identifier of the barrier.
	 */
	public int barrierID;

	/**
	 * The MasonGeometry representing the barrier's shape.
	 */
	public MasonGeometry masonGeometry;

	/**
	 * The list of edges along the barrier.
	 */
	public ArrayList<EdgeGraph> edgesAlong = new ArrayList<EdgeGraph>();

	/**
	 * The type of the barrier, such as "water," "park," etc.
	 */
	public String type;
}
