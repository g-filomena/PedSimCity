package pedSim.cognitiveMap;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.locationtech.jts.geom.Geometry;

import pedSim.agents.Agent;
import pedSim.utilities.StringEnum.BarrierType;
import sim.field.geo.VectorLayer;
import sim.graph.EdgeGraph;
import sim.graph.NodeGraph;
import sim.graph.SubGraph;
import sim.util.geo.Angles;
import sim.util.geo.AttributeValue;
import sim.util.geo.MasonGeometry;

public class BarrierIntegration {

	/**
	 * Returns a set of barriers in the direction of the destination node from a
	 * given location.
	 *
	 * @param currentLocation The current node.
	 * @param destinationNode The destination node.
	 * @param agent           The agent whose barrier type should be considered.
	 * @return A mapping of the viewField and the barrierIDs intersecting it.
	 */
	public Map<Geometry, Set<Integer>> intersectingBarriers(NodeGraph currentLocation, NodeGraph destinationNode,
			Agent agent) {

		Map<Geometry, Set<Integer>> viewFieldIntersectingBarriers = new HashMap<Geometry, Set<Integer>>();
		Set<Integer> intersectingBarrierIDs = new HashSet<>();
		BarrierType agentBarrierType = agent.getProperties().barrierType;
		Geometry viewField = Angles.viewField(currentLocation, destinationNode, 70.0);

		VectorLayer barriers = agent.cognitiveMap.getBarriers();
		if (barriers.getGeometries().isEmpty())
			return viewFieldIntersectingBarriers;

		List<MasonGeometry> intersectingGeometries = barriers.intersectingFeatures(viewField);
		if (intersectingGeometries.isEmpty())
			return viewFieldIntersectingBarriers;

		intersectingGeometries.stream().filter(barrierGeometry -> {
			String barrierType = barrierGeometry.getStringAttribute("type");
			return agentBarrierType.equals(BarrierType.ALL)
					|| (agentBarrierType.equals(BarrierType.POSITIVE)
							&& (barrierType.equals("park") || barrierType.equals("water")))
					|| (agentBarrierType.equals(BarrierType.NEGATIVE) && (barrierType.equals("railway")
							|| barrierType.equals("road") || barrierType.equals("secondary_road")))
					|| (agentBarrierType.equals(BarrierType.SEPARATING) && !barrierType.equals("park"));
		}).forEach(barrierGeometry -> intersectingBarrierIDs.add(barrierGeometry.getIntegerAttribute("barrierID")));

		viewFieldIntersectingBarriers.put(viewField, intersectingBarrierIDs);
		return viewFieldIntersectingBarriers;
	}

	/**
	 * Sets the barrier information for an EdgeGraph based on attribute values. This
	 * method parses attribute strings representing different types of barriers,
	 * such as positive barriers, negative barriers, rivers, and parks, and
	 * populates the corresponding lists in the EdgeGraph. The method retrieves
	 * attribute values for positive barriers ("p_barr"), negative barriers
	 * ("n_barr"), rivers ("a_rivers"), and parks ("w_parks") from the EdgeGraph's
	 * attributes. It parses these strings to extract barrier IDs and adds them to
	 * the appropriate lists: positiveBarriers, negativeBarriers, waterBodies, and
	 * parks. Additionally, it combines positive and negative barriers into the
	 * 'barriers' list for convenient access.
	 *
	 * @param edge The EdgeGraph for which barrier information is being set.
	 */
	public static void setEdgeGraphBarriers(EdgeGraph edge) {

		List<Integer> positiveBarriers = new ArrayList<>();
		List<Integer> negativeBarriers = new ArrayList<>();
		List<Integer> barriers = new ArrayList<>(); // all the barriers
		List<Integer> waterBodies = new ArrayList<>();
		List<Integer> parks = new ArrayList<>();
		final String pBarriersString = edge.attributes.get("p_barr").getString();
		final String nBarriersString = edge.attributes.get("n_barr").getString();
		final String riversString = edge.attributes.get("a_rivers").getString();
		final String parksString = edge.attributes.get("w_parks").getString();

		if (!pBarriersString.equals("[]")) {
			final String p = pBarriersString.replaceAll("[^-?0-9]+", " ");
			for (final String t : Arrays.asList(p.trim().split(" ")))
				positiveBarriers.add(Integer.valueOf(t));
		}
		edge.attributes.put("positiveBarriers", new AttributeValue(positiveBarriers));

		if (!nBarriersString.equals("[]")) {
			final String n = nBarriersString.replaceAll("[^-?0-9]+", " ");
			for (final String t : Arrays.asList(n.trim().split(" ")))
				negativeBarriers.add(Integer.valueOf(t));
		}
		edge.attributes.put("negativeBarriers", new AttributeValue(negativeBarriers));

		if (!riversString.equals("[]")) {
			final String r = riversString.replaceAll("[^-?0-9]+", " ");
			for (final String t : Arrays.asList(r.trim().split(" ")))
				waterBodies.add(Integer.valueOf(t));
		}
		edge.attributes.put("waterBodies", new AttributeValue(waterBodies));

		if (!parksString.equals("[]")) {
			final String p = parksString.replaceAll("[^-?0-9]+", " ");
			for (final String t : Arrays.asList(p.trim().split(" ")))
				parks.add(Integer.valueOf(t));
		}
		edge.attributes.put("parks", new AttributeValue(parks));

		barriers.addAll(positiveBarriers);
		barriers.addAll(negativeBarriers);
		edge.attributes.put("barriers", new AttributeValue(barriers));
	}

	/**
	 * It stores information about the barriers within a given SubGraph.
	 *
	 * @param subGraph The SubGraph for which the barrier information is being set.
	 */
	public static void setSubGraphBarriers(SubGraph subGraph) {

		List<Integer> graphBarriers = new ArrayList<>();
		for (EdgeGraph childEdge : subGraph.getEdges()) {
			graphBarriers.addAll(subGraph.getParentEdge(childEdge).attributes.get("barriers").getArray());
			// remove duplicates
			graphBarriers = new ArrayList<>(new HashSet<>(graphBarriers));
			subGraph.attributes.put("graphBarriers", new AttributeValue(graphBarriers));
		}
	}

	/**
	 * Returns the list of barrier IDs associated with the given subgraph. These
	 * barriers represent physical obstacles or features within the subgraph that
	 * shape movement and agent's cognitive maps.
	 *
	 * @param subGraph The SubGraph for which the barrier information is being
	 *                 requested.
	 *
	 * @return The list of barrier IDs within the subgraph.
	 */
	public static List<Integer> getSubGraphBarriers(SubGraph subGraph) {
		return subGraph.attributes.get("graphBarriers").getArray();
	}
}
