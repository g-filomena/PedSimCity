/**
 * It computes a sequence of sub-goals when barriers are used to orientate and to navigate across the city, between an origin and a destination.
 * This represent a barrier coarse plan that is then refined later on.
 *
 * */

package pedsimcity.routeChoice;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.Set;

import org.javatuples.Pair;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;

import pedsimcity.agents.AgentProperties;
import pedsimcity.elements.Barrier;
import pedsimcity.elements.Region;
import pedsimcity.graph.EdgeGraph;
import pedsimcity.graph.NodeGraph;
import pedsimcity.main.PedSimCity;
import pedsimcity.utilities.Angles;
import pedsimcity.utilities.Utilities;
import sim.util.Bag;
import sim.util.geo.MasonGeometry;

public class BarrierBasedNavigation {

	NodeGraph currentLocation;
	NodeGraph destinationNode;
	HashMap<Integer, EdgeGraph> edgesMap;
	final ArrayList<NodeGraph> sequence = new ArrayList<>();

	/**
	 * It returns a sequence of nodes, wherein, besides the origin and the
	 * destination nodes, the other nodes represent barrier subgoals. of the
	 * traversed regions. The traversed regions are identified as well within this
	 * function.
	 *
	 * @param originNode      the origin node;
	 * @param destinationNode the destination node;
	 * @param typeBarriers    the type of barriers to consider, it depends on the
	 *                        agent;
	 */
	public ArrayList<NodeGraph> sequenceBarriers(NodeGraph originNode, NodeGraph destinationNode, AgentProperties ap) {

		this.edgesMap = PedSimCity.edgesMap;
		this.currentLocation = originNode;

		// sub-goals
		this.sequence.add(originNode);
		this.currentLocation = originNode;
		this.destinationNode = destinationNode;
		// it stores the barriers that the agent has already been exposed to
		final ArrayList<Integer> adjacentBarriers = new ArrayList<>();

		while (true) {

			// check if there are good barriers in line of movement, all type of barriers
			final Set<Integer> intersectingBarriers = intersectingBarriers(this.currentLocation, destinationNode,
					ap.typeBarriers);
			// no barriers
			if (intersectingBarriers.size() == 0)
				break;

			// identify barriers around this location
			final ArrayList<EdgeGraph> incomingEdges = this.currentLocation.getEdges();
			for (final EdgeGraph edge : incomingEdges)
				if (edge.barriers != null)
					adjacentBarriers.addAll(edge.barriers);

//			// disregard barriers that have been already walked along
			final Set<Integer> visitedBarriers = new HashSet<>(adjacentBarriers);
			intersectingBarriers.removeAll(visitedBarriers);
			if (intersectingBarriers.size() == 0)
				break;

			NodeGraph subGoal = null;
			Region region = null;
			// given the intersecting barriers, identify the best one and the relative edge
			// close to it
			if (ap.regionBasedNavigation)
				region = PedSimCity.regionsMap.get(originNode.region);

			final Pair<EdgeGraph, Integer> barrierGoal = this.barrierGoal(intersectingBarriers, this.currentLocation,
					destinationNode, region);
			if (barrierGoal == null)
				break;
			final EdgeGraph edgeGoal = barrierGoal.getValue0();
			final int barrier = barrierGoal.getValue1();

			// pick the closest barrier sub-goal
			final NodeGraph u = edgeGoal.u;
			final NodeGraph v = edgeGoal.v;
			if (NodeGraph.nodesDistance(this.currentLocation, u) < NodeGraph.nodesDistance(this.currentLocation, v))
				subGoal = u;
			else
				subGoal = v;

			this.sequence.add(subGoal);
			this.currentLocation = subGoal;
			adjacentBarriers.add(barrier);
		}
		this.sequence.add(destinationNode);
		return this.sequence;
	}

	/**
	 * It returns a set of barriers in direction of the destination node from a
	 * given location.
	 *
	 * @param currentLocation the current Location node;
	 * @param destinationNode the destination node;
	 * @param typeBarriers    the type of barriers to consider, it depends on the
	 *                        agent;
	 */

	public static Set<Integer> intersectingBarriers(NodeGraph currentLocation, NodeGraph destinationNode,
			String typeBarriers) {

		final Geometry viewField = Angles.viewField(currentLocation, destinationNode, 70.0);
		final Bag intersecting = PedSimCity.barriers.intersectingFeatures(viewField);
		final Set<Integer> intersectingBarriers = new HashSet<>();
		final ArrayList<MasonGeometry> intersectingGeometries = new ArrayList<>();

		// check the found barriers and their type
		for (final Object iB : intersecting) {
			final MasonGeometry geoBarrier = (MasonGeometry) iB;
			final String barrierType = geoBarrier.getStringAttribute("type");

			if (typeBarriers.equals("all"))
				intersectingGeometries.add(geoBarrier);
			else if (typeBarriers.equals("positive") && (barrierType.equals("park") || barrierType.equals("water")))
				intersectingGeometries.add(geoBarrier);
			else if (typeBarriers.equals("negative") && (barrierType.equals("railway") || barrierType.equals("road")
					|| barrierType.equals("secondary_road")))
				intersectingGeometries.add(geoBarrier);
			else if (typeBarriers.equals("separating") && !barrierType.equals("park"))
				intersectingGeometries.add(geoBarrier);
			else if (typeBarriers.equals(barrierType))
				intersectingGeometries.add(geoBarrier);
		}
		for (final MasonGeometry i : intersectingGeometries)
			intersectingBarriers.add(i.getIntegerAttribute("barrierID"));
		return intersectingBarriers;
	}

	/**
	 * Given a set of barriers between a location and a destination nodes, it
	 * identifies barriers that are actually complying with certain criteria in
	 * terms of distance, location and direction and it identifies, if any, the
	 * closest edge to it.
	 *
	 * @param intersectingBarriers the set of barriers that are in the search space
	 *                             between the location and the destination;
	 * @param currentLocation      the current location;
	 * @param destinationNode      the destination node;
	 * @param region               the metainformation about the region when the
	 *                             agent is navigateing through regions;
	 */

	public Pair<EdgeGraph, Integer> barrierGoal(Set<Integer> intersectingBarriers, NodeGraph currentLocation,
			NodeGraph destinationNode, Region region) {

		this.currentLocation = currentLocation;
		this.destinationNode = destinationNode;
		final HashMap<Integer, Double> possibleBarriers = new HashMap<>();
		// create search-space
		final Geometry viewField = Angles.viewField(currentLocation, destinationNode, 70.0);

		// for each barrier, check whether they are within the region/area considered
		// and within the search-space, and if
		// it complies with the criteria

		for (final int barrierID : intersectingBarriers) {

			final MasonGeometry barrierGeometry = PedSimCity.barriersMap.get(barrierID).masonGeometry;
			final Coordinate[] intersections = viewField.intersection(barrierGeometry.geometry).getCoordinates();
			double distanceIntersection = Double.MAX_VALUE;

			for (final Coordinate c : intersections) {
				final double tmpDistance = Utilities.euclideanDistance(currentLocation.getCoordinate(), c);
				if (tmpDistance < distanceIntersection)
					distanceIntersection = tmpDistance;
			}

			if (distanceIntersection > Utilities.euclideanDistance(currentLocation.getCoordinate(),
					destinationNode.getCoordinate()))
				continue;
			// it is acceptable
			possibleBarriers.put(barrierID, distanceIntersection);
		}
		// no barriers found
		if (possibleBarriers.size() == 0 || possibleBarriers == null)
			return null;

		boolean regionBasedNavigation = false;
		EdgeGraph edgeGoal = null;
		if (region != null)
			regionBasedNavigation = true;

		// sorted by distance (further away first)
		LinkedHashMap<Integer, Double> validSorted = (LinkedHashMap<Integer, Double>) Utilities
				.sortByValue(possibleBarriers, true);
		if (regionBasedNavigation)
			validSorted = (LinkedHashMap<Integer, Double>) Utilities.sortByValue(possibleBarriers, true);

		ArrayList<EdgeGraph> regionEdges = null;
		// the edges of the current region
		if (regionBasedNavigation)
			regionEdges = region.primalGraph.getParentEdges(region.primalGraph.getEdges());

		final ArrayList<Integer> withinBarriers = new ArrayList<>();
		final ArrayList<EdgeGraph> possibleEdgeGoals = new ArrayList<>();

		int waterCounter = 0;
		int parkCounter = 0;

		for (final int barrierID : validSorted.keySet()) {
			final Barrier barrier = PedSimCity.barriersMap.get(barrierID);
			final String type = barrier.type;

			// identify edges that are along the identified barrier
			final ArrayList<EdgeGraph> edgesAlong = barrier.edgesAlong;
			HashMap<EdgeGraph, Double> thisBarrierEdgeGoals = new HashMap<>();

			// keep only edges, along the identified barrier, within the the current region
			if (regionBasedNavigation) {
				edgesAlong.retainAll(regionEdges);
				if (edgesAlong.size() == 0)
					continue;
			}

			// verify if also the edge meets the criterion
			thisBarrierEdgeGoals = this.checkRequirementsSubGoal(edgesAlong);
			// if the barrier doensn't have decent edges around
			if (thisBarrierEdgeGoals.size() == 0)
				continue;

			// this is considered a good Edge, sort by distance and takes the closest to the
			// current location.
			final LinkedHashMap<EdgeGraph, Double> thisBarrierSubGoalSorted = (LinkedHashMap<EdgeGraph, Double>) Utilities
					.sortByValue(thisBarrierEdgeGoals, false);
			final EdgeGraph possibleEdgeGoal = thisBarrierSubGoalSorted.keySet().iterator().next();

			// compare it with the previous barrier-edges pairs, on the basis of the type.
			// Positive barriers are preferred.
			if (type.equals("water")) {
				withinBarriers.add(waterCounter, barrierID);
				possibleEdgeGoals.add(waterCounter, possibleEdgeGoal);
				waterCounter += 1;
				parkCounter += 1;
			} else if (type.equals("park")) {
				withinBarriers.add(parkCounter, barrierID);
				possibleEdgeGoals.add(parkCounter, possibleEdgeGoal);
				parkCounter += 1;
			} else {
				withinBarriers.add(barrierID);
				possibleEdgeGoals.add(possibleEdgeGoal);
			}
		}

		if (possibleEdgeGoals.size() == 0 || possibleEdgeGoals.get(0) == null)
			return null;

		edgeGoal = possibleEdgeGoals.get(0);
		final int barrier = withinBarriers.get(0);
		final Pair<EdgeGraph, Integer> pair = new Pair<>(edgeGoal, barrier);
		return pair;
	}

	private HashMap<EdgeGraph, Double> checkRequirementsSubGoal(ArrayList<EdgeGraph> edgesAlong) {

		final HashMap<EdgeGraph, Double> thisBarrierEdgeGoals = new HashMap<>();

		for (final EdgeGraph edge : edgesAlong) {

			final double distanceToEdge = Utilities.euclideanDistance(this.currentLocation.getCoordinate(),
					edge.getCoordsCentroid());
			if (distanceToEdge > Utilities.euclideanDistance(this.currentLocation.getCoordinate(),
					this.destinationNode.getCoordinate()))
				continue;
			for (final NodeGraph n : this.sequence)
				if (n.getEdges().contains(edge))
					continue;
			if (this.currentLocation.getEdges().contains(edge))
				continue;
			thisBarrierEdgeGoals.put(edge, distanceToEdge);
		}
		return thisBarrierEdgeGoals;
	}

}
