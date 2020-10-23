/**
 * It computes a sequence of sub-goals when barriers are used to orientate and to navigate across the city, between an origin and a destination.
 * This represent a barrier coarse plan that is then refined later on.
 *
 * */

package sim.app.geo.pedSimCity;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.Set;

import org.javatuples.Pair;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;

import sim.app.geo.urbanSim.EdgeGraph;
import sim.app.geo.urbanSim.NodeGraph;
import sim.app.geo.urbanSim.Utilities;
import sim.util.Bag;
import sim.util.geo.MasonGeometry;

public class BarrierBasedNavigation {

	NodeGraph currentLocation;
	HashMap<Integer, EdgeGraph> edgesMap;

	/**
	 * It returns a sequence of nodes, wherein, besides the origin and the destination nodes, the other nodes represent barrier subgoals.
	 * of the traversed regions. The traversed regions are identified as well within this function.
	 *
	 * @param originNode the origin node;
	 * @param destinationNode the destination node;
	 * @param typeOfBarriers the type of barriers to consider, it depends on the agent;
	 */

	public ArrayList<NodeGraph> sequenceBarriers(NodeGraph originNode, NodeGraph destinationNode, String typeOfBarriers) {

		this.edgesMap = PedSimCity.edgesMap;
		this.currentLocation = originNode;

		// sub-goals
		ArrayList<NodeGraph> sequence = new ArrayList<NodeGraph>();
		sequence.add(originNode);
		currentLocation = originNode;

		// it stores the barriers that the agent has already been exposed to
		ArrayList<Integer> adjacentBarriers = new ArrayList<Integer>();

		while (true) {

			// check if there are good barriers in line of movement, all type of barriers
			Set<Integer> intersectingBarriers = intersectingBarriers(currentLocation, destinationNode, typeOfBarriers);
			// no barriers
			if (intersectingBarriers.size() == 0) break;

			// identify barriers around this location
			ArrayList<EdgeGraph> incomingEdges = currentLocation.getEdges();
			for (EdgeGraph edge : incomingEdges) if (edge.barriers != null) adjacentBarriers.addAll(edge.barriers);

			// disregard barriers that have been already walked along
			Set<Integer> visitedBarriers = new HashSet<Integer>(adjacentBarriers);
			intersectingBarriers.removeAll(visitedBarriers);
			if (intersectingBarriers.size() == 0) break;

			NodeGraph subGoal = null;
			// given the intersecting barriers, identify the best one and the relative edge close to it
			Pair<EdgeGraph, Integer> barrierGoal = barrierGoal(intersectingBarriers, destinationNode, currentLocation, null);
			if (barrierGoal == null) break;
			EdgeGraph edgeGoal = barrierGoal.getValue0();
			int barrier = barrierGoal.getValue1();

			// pick the closest barrier sub-goal
			NodeGraph u = edgeGoal.u;
			NodeGraph v = edgeGoal.v;
			if ((Utilities.nodesDistance(currentLocation, u)) < (Utilities.nodesDistance(currentLocation, v))) subGoal = u;
			else subGoal = v;

			sequence.add(subGoal);
			currentLocation = subGoal;
			adjacentBarriers.add(barrier);
		}
		sequence.add(destinationNode);
		return sequence;
	}


	/**
	 * It returns a set of barriers in direction of the destination node from a given location.
	 *
	 * @param currentLocation the current Location node;
	 * @param destinationNode the destination node;
	 * @param typeOfBarriers the type of barriers to consider, it depends on the agent;
	 */

	public static Set<Integer> intersectingBarriers(NodeGraph currentLocation, NodeGraph destinationNode, String typeOfBarriers) {

		Geometry viewField = Utilities.viewField(currentLocation, destinationNode);
		Bag intersecting = PedSimCity.barriers.getIntersecting(viewField);
		Set<Integer> intersectingBarriers = new HashSet<Integer>();
		ArrayList<MasonGeometry> intersectingGeometries = new ArrayList<MasonGeometry>();

		// check the found barriers and their type
		for (Object iB : intersecting) {
			MasonGeometry geoBarrier = (MasonGeometry) iB;
			String barrierType = geoBarrier.getStringAttribute("type");

			if (typeOfBarriers.equals("all")) intersectingGeometries.add(geoBarrier);
			else if ((typeOfBarriers.equals("positive")) & (barrierType.equals("park")) || (barrierType.equals("water")))
				intersectingGeometries.add(geoBarrier);
			else if ((typeOfBarriers.equals("negative")) && (barrierType.equals("railway")) || (barrierType.equals("road")))
				intersectingGeometries.add(geoBarrier);
			else if ((typeOfBarriers.equals("separating")) && (!barrierType.equals("parks"))) intersectingGeometries.add(geoBarrier);
			else if (typeOfBarriers == barrierType) intersectingGeometries.add(geoBarrier);
		}
		for (MasonGeometry i: intersectingGeometries) intersectingBarriers.add(i.getIntegerAttribute("barrierID"));
		return intersectingBarriers;
	}

	/**
	 * Given a set of barriers between a location and a destination nodes, it identifies barriers that are actually complying with certain criteria
	 * in terms of distance, location and direction and it identifies, if any, the closest edge to it.
	 *
	 * @param intersectingBarriers the set of barriers that are in the search space between the location and the destination;
	 * @param currentLocation the current location;
	 * @param destinationNode the destination node;
	 * @param region the metainformation about the region when the agent is navigateing through regions;
	 */

	public static Pair<EdgeGraph, Integer> barrierGoal (Set<Integer> intersectingBarriers, NodeGraph currentLocation, NodeGraph destinationNode,
			RegionData region) {

		HashMap<Integer, Double> possibleBarriers = new HashMap<Integer, Double> ();
		// create search-space
		Geometry viewField = Utilities.viewField(currentLocation, destinationNode);

		// for each barrier, check whether they are within the region/area considered and within the search-space, and if
		// it complies with the criteria
		for (int barrierID : intersectingBarriers)
		{
			MasonGeometry barrierGeometry = PedSimCity.barriersMap.get(barrierID);
			Coordinate intersection = viewField.intersection(barrierGeometry.getGeometry()).getCoordinate();
			double distanceIntersection = Utilities.euclideanDistance(currentLocation.getCoordinate(), intersection);
			if (distanceIntersection > Utilities.euclideanDistance(currentLocation.getCoordinate(),	destinationNode.getCoordinate())) continue;
			// it is acceptable
			possibleBarriers.put(barrierID, distanceIntersection);
		}
		// no barriers found
		if ((possibleBarriers.size() == 0) || (possibleBarriers == null)) return null;

		boolean regionBasedNavigation = false;
		EdgeGraph edgeGoal = null;
		if (region != null) regionBasedNavigation = true;

		// sorted by distance (further away first)
		LinkedHashMap<Integer, Double> validSorted = (LinkedHashMap<Integer, Double>) Utilities.sortByValue(possibleBarriers, "descending");
		ArrayList<EdgeGraph> regionEdges = null;
		// the edges of the current region
		if (regionBasedNavigation) regionEdges = region.primalGraph.getParentEdges(region.primalGraph.getEdges());

		ArrayList<Integer> withinBarriers = new ArrayList<Integer>();
		ArrayList<EdgeGraph> possibleEdgeGoals = new ArrayList<EdgeGraph>();

		for (int barrierID : validSorted.keySet()) {
			MasonGeometry barrierGeometry = PedSimCity.barriersMap.get(barrierID);
			String type = barrierGeometry.getStringAttribute("type");

			// identify edges that are along the identified barrier
			ArrayList<EdgeGraph> alongEdges = PedSimCity.barriersEdgesMap.get(barrierID);
			HashMap<EdgeGraph, Double> thisBarrierEdgeGoals = new HashMap<EdgeGraph, Double>();

			// keep only edges, along the identified barrier, within the the current region
			if (regionBasedNavigation) alongEdges.retainAll(regionEdges);

			// verify if also the edge meets the criterion
			for (EdgeGraph edge: alongEdges) {
				double distanceEdge = Utilities.euclideanDistance(currentLocation.getCoordinate(), edge.getCoordsCentroid());
				if (distanceEdge > Utilities.euclideanDistance(currentLocation.getCoordinate(), destinationNode.getCoordinate())) continue;
				thisBarrierEdgeGoals.put(edge, distanceEdge);
			}
			// if the barrier doensn't have decent edges around
			if (thisBarrierEdgeGoals.size() == 0) continue;

			// this is considered a good Edge, sort by distance and takes the closest to the current location.
			LinkedHashMap<EdgeGraph, Double> thisBarrierSubGoalSorted = (LinkedHashMap<EdgeGraph, Double>)
					Utilities.sortByValue(thisBarrierEdgeGoals, "ascending");
			EdgeGraph possibleEdgeGoal = thisBarrierSubGoalSorted.keySet().iterator().next();

			// compare it with the previous barrier-edges pairs, on the basis of the type. Positive barriers are preferred.
			int waterCounter = 0;
			int parkCounter = 0;

			if (type.equals("water")) {
				withinBarriers.add(waterCounter, barrierID);
				possibleEdgeGoals.add(waterCounter, possibleEdgeGoal);
				waterCounter += 1;
				parkCounter = waterCounter +1;
			}
			else if (type.equals("park")) {
				withinBarriers.add(parkCounter, barrierID);
				possibleEdgeGoals.add(parkCounter, possibleEdgeGoal);
				parkCounter += 1;
			}
			else {
				withinBarriers.add(barrierID);
				possibleEdgeGoals.add(possibleEdgeGoal);
			}
		}

		if ((possibleEdgeGoals.size() == 0) || (possibleEdgeGoals.get(0) == null )) return null;

		edgeGoal = possibleEdgeGoals.get(0);
		int barrier = withinBarriers.get(0);
		Pair<EdgeGraph, Integer> pair = new Pair<EdgeGraph, Integer> (edgeGoal, barrier);
		return pair;
	}
}

