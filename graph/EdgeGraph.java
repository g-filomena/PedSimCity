package pedsimcity.graph;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.LineString;

import sim.util.geo.AttributeValue;
import sim.util.geo.GeomPlanarGraphEdge;
import sim.util.geo.MasonGeometry;

/**
 * An edge that extends the GeomPlanarGraphEdge (GeoMason) and Edge (Jts)
 * classes. This is one of the main components, along with NodeGraph, of graphs
 * belonging to the class Graph.
 *
 */
public class EdgeGraph extends GeomPlanarGraphEdge {

	public int region, edgeID;
	public double deflectionDegrees;
	public MasonGeometry masonGeometry;

	public int RD, AC, RL, AL, LL, GL;
	public int RB, BB, RBB;

	public NodeGraph u, v;
	public NodeGraph dualNode;

	public List<Integer> positiveBarriers = new ArrayList<>();
	public List<Integer> negativeBarriers = new ArrayList<>();
	public List<Integer> barriers = new ArrayList<>(); // all the barriers
	List<Integer> waterBodies = new ArrayList<>();
	List<Integer> parks = new ArrayList<>();

	public HashMap<String, Integer> volumes = new HashMap<>();
	public Map<String, AttributeValue> attributes;

	/**
	 * Get an integer attribute, given the name of the field;
	 *
	 * @param name the field's name;
	 * @note override as the attributes are private in GeomPlanarGraphEdge;
	 */
	@Override
	public void setAttributes(final Map<String, AttributeValue> attributes) {
		this.attributes = attributes;
	}

	/**
	 * The constructor.
	 *
	 */
	public EdgeGraph(LineString line) {
		super(line);
	}

	/**
	 * It sets the edge's nodes.
	 *
	 */
	public void setNodes(final NodeGraph u, final NodeGraph v) {
		this.u = u;
		this.v = v;
	}

	/**
	 * It returns the edge's ID.
	 *
	 */
	public void setID(int edgeID) {
		this.edgeID = edgeID;
	}

	/**
	 * It returns the edge's ID.
	 *
	 */
	public Integer getID() {
		return this.edgeID;
	}

	/**
	 * It returns the edge's corresponding dual node.
	 *
	 */
	public NodeGraph getDual() {
		return this.dualNode;
	}

	/**
	 * It returns the edge's length.
	 *
	 */
	public double getLength() {
		return this.getLine().getLength();
	}

	/**
	 * It returns the deflection angle if this edge is a dual edge and represents a
	 * link between two dual nodes (street segment).
	 *
	 */
	public double getDeflectionAngle() {
		return this.deflectionDegrees;
	}

	/**
	 * It returns the coordinate of the edge's centroid.
	 *
	 */
	public Coordinate getCoordsCentroid() {
		return this.getLine().getCentroid().getCoordinate();
	}

	/**
	 * It resets the densities when called during the simulation (e.g. new run).
	 *
	 */
	public void resetDensities() {

		this.RD = this.AC = this.RL = this.AL = this.LL = this.GL = 0;
		this.RB = this.BB = this.RBB = 0;
		for (final String key : this.volumes.keySet())
			this.volumes.replace(key, 0);
	}

	/**
	 * Set the barriers of an edge.
	 *
	 */
	public void setBarriers() {

		final String pBarriersString = this.getStringAttribute("p_barr");
		final String nBarriersString = this.getStringAttribute("n_barr");
		final String riversString = this.getStringAttribute("a_rivers");
		final String parksString = this.getStringAttribute("w_parks");

		if (!pBarriersString.equals("[]")) {
			final String p = pBarriersString.replaceAll("[^-?0-9]+", " ");
			for (final String t : Arrays.asList(p.trim().split(" ")))
				this.positiveBarriers.add(Integer.valueOf(t));
		}

		if (!nBarriersString.equals("[]")) {
			final String n = nBarriersString.replaceAll("[^-?0-9]+", " ");
			for (final String t : Arrays.asList(n.trim().split(" ")))
				this.negativeBarriers.add(Integer.valueOf(t));
		}

		if (!riversString.equals("[]")) {
			final String r = riversString.replaceAll("[^-?0-9]+", " ");
			for (final String t : Arrays.asList(r.trim().split(" ")))
				this.waterBodies.add(Integer.valueOf(t));
		}

		if (!parksString.equals("[]")) {
			final String p = parksString.replaceAll("[^-?0-9]+", " ");
			for (final String t : Arrays.asList(p.trim().split(" ")))
				this.parks.add(Integer.valueOf(t));
		}

		this.barriers.addAll(this.positiveBarriers);
		this.barriers.addAll(this.negativeBarriers);
		// this.attributes.clear();
	}

	/**
	 * Given one of the nodes of this segment, it returns the other one.
	 *
	 * @param node one of the nodes;
	 */
	public NodeGraph getOtherNode(NodeGraph node) {
		if (this.u == node)
			return this.v;
		else if (this.v == node)
			return this.u;
		else
			return null;
	}

	public NodeGraph getCommonNode(EdgeGraph edge) {

		if (this.u == edge.v || this.u == edge.u)
			return this.u;
		if (this.v == edge.v || this.v == edge.u)
			return this.v;
		else
			return null;
	}

//	public NodeGraph getCommonNode(EdgeGraph edge) {
//
//		if (this.u.nodeID == edge.v.nodeID || this.u.nodeID == edge.u.nodeID) return this.u;
//		if (this.v.nodeID == edge.v.nodeID || this.v.nodeID == edge.u.nodeID) return this.u;
//		else return null;
//	}

	/**
	 * Get an integer attribute, given the name of the field;
	 *
	 * @param the field's name;
	 * @note Override so to take it from the EdgeGraph attributes, not from the
	 *       Edge's class;
	 */
	@Override
	public Integer getIntegerAttribute(String name) {
		return this.attributes.get(name).getInteger();
	}

	/**
	 * Get a double attribute, given the name of the field;
	 *
	 * @param the field's name;
	 * @note Override so to take it from the EdgeGraph attributes, not from the
	 *       Edge's class;
	 */
	@Override
	public Double getDoubleAttribute(String name) {
		return this.attributes.get(name).getDouble();
	}

	/**
	 * Get a string attribute, given the name of the field;
	 *
	 * @param the field's name;
	 * @note Override so to take it from the EdgeGraph attributes, not from the
	 *       Edge's class;
	 */
	@Override
	public String getStringAttribute(String name) {
		return this.attributes.get(name).getString();
	}
}
