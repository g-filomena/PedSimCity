package pedSim.routeChoice;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.javatuples.Pair;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;

import pedSim.agents.Agent;
import pedSim.cognitiveMap.Barrier;
import pedSim.cognitiveMap.BarrierIntegration;
import pedSim.cognitiveMap.Region;
import pedSim.engine.PedSimCity;
import sim.graph.EdgeGraph;
import sim.graph.GraphUtils;
import sim.graph.NodeGraph;
import sim.util.geo.MasonGeometry;
import sim.util.geo.Utilities;

/**
 * Series of functions for computing a sequence of barrier sub-goals in the
 * space between an origin and a destination.
 */
public class BarrierBasedNavigation {

	Map<Integer, EdgeGraph> edgesMap = new HashMap<Integer, EdgeGraph>();
	List<NodeGraph> sequence = new ArrayList<>();
	private NodeGraph originNode;
	private NodeGraph currentLocation;
	private NodeGraph destinationNode;
	private Agent agent;
	boolean regionBasedNavigation = false;
	// it stores the barriers that the agent has already been exposed to
	Set<Integer> visitedBarriers = new HashSet<>();

	/**
	 * Initialises a new instance of the BarrierBasedNavigation class.
	 *
	 * @param originNode      the origin node;
	 * @param destinationNode the destination node;
	 * @param agent           the agent for navigation.
	 */
	public BarrierBasedNavigation(NodeGraph originNode, NodeGraph destinationNode, Agent agent,
			boolean regionBasedNavigation) {
		this.originNode = originNode;
		this.destinationNode = destinationNode;
		this.agent = agent;
		this.regionBasedNavigation = regionBasedNavigation;
	}

	/**
	 * Computes a sequence of nodes, including origin, destination, and barrier
	 * sub-goals, and identifies traversed regions.
	 *
	 * @return an ArrayList of NodeGraph representing the sequence of sub-goals.
	 * @throws Exception
	 */
	public List<NodeGraph> sequenceBarriers() throws Exception {

		edgesMap = new HashMap<Integer, EdgeGraph>(PedSimCity.edgesMap);
		currentLocation = originNode;

		// sub-goals
		sequence.add(originNode);

		while (true) {
			if (currentLocation.equals(destinationNode))
				throw new Exception("destinationNode and currentLocation are the same");

			Map<Integer, Double> validBarriers = findValidBarriers(currentLocation, null);
			if (validBarriers.isEmpty())
				break;

			Pair<EdgeGraph, Integer> barrierGoal = identifyBarrierSubGoal(validBarriers, null);
			if (barrierGoal == null)
				break;

			EdgeGraph edgeGoal = barrierGoal.getValue0();
			int barrier = barrierGoal.getValue1();
			NodeGraph subGoal = GraphUtils.getCachedNodesDistance(currentLocation, edgeGoal.getFromNode()) < GraphUtils
					.getCachedNodesDistance(currentLocation, edgeGoal.getToNode()) ? edgeGoal.getFromNode()
							: edgeGoal.getToNode();

			sequence.add(subGoal);
			currentLocation = subGoal;
			visitedBarriers.add(barrier); // to avoid visiting them again
			if (subGoal.equals(destinationNode)) {
				break;
			}
		}

		if (!sequence.contains(destinationNode)) {
			sequence.add(destinationNode);
		}
		return sequence;
	}

	/**
	 * Finds the valid barriers along the line of movement towards the destination
	 * within the given region.
	 * 
	 * @param currentLocation The current node location (gateway when region-based
	 *                        navigation).
	 * @param region          the region of the currentLocation, only for
	 *                        region-based navigation.
	 * @return A HashMap containing valid barrier IDs and their respective distances
	 *         from the currentLocation.
	 */
	protected Map<Integer, Double> findValidBarriers(NodeGraph currentLocation, Region region) {

		this.currentLocation = currentLocation;
		Map<Integer, Double> validBarriers = new HashMap<>();
		BarrierIntegration barrierIntegration = new BarrierIntegration();
		// check if there are good barriers in line of movement towards the destination
		Map<Geometry, Set<Integer>> viewFieldIntersectingBarriers = barrierIntegration
				.intersectingBarriers(currentLocation, destinationNode, agent);

		// no barriers
		if (viewFieldIntersectingBarriers.isEmpty())
			return validBarriers;

		Geometry viewField = (new ArrayList<Geometry>(viewFieldIntersectingBarriers.keySet())).get(0);
		Set<Integer> intersectingBarriers = viewFieldIntersectingBarriers.get(viewField);

		// when region-based, only keep barriers that are actually within the region
		// boundaries
		if (regionBasedNavigation)
			intersectingBarriers.retainAll(new HashSet<>(BarrierIntegration.getSubGraphBarriers(region.primalGraph)));

		// assuming that these have been "visited" already
		identifyAdjacentBarriers();
		if (intersectingBarriers.isEmpty())
			return validBarriers;

		Coordinate currentCoordinate = currentLocation.getCoordinate();
		Coordinate destinationCoordinate = destinationNode.getCoordinate();

		// for each candidate barrier, check whether it complies with the criteria
		for (int barrierID : intersectingBarriers) {
			if (visitedBarriers.contains(barrierID))
				continue;
			MasonGeometry barrierGeometry = PedSimCity.barriersMap.get(barrierID).masonGeometry;
			Coordinate[] intersections = viewField.intersection(barrierGeometry.geometry).getCoordinates();

			double minDistance = Arrays.stream(intersections).parallel()
					.mapToDouble(intersection -> GraphUtils.euclideanDistance(currentCoordinate, intersection)).min()
					.orElse(Double.MAX_VALUE);

			// barriers that are more distant than the destinationNode are disregarded
			if (minDistance > GraphUtils.euclideanDistance(currentCoordinate, destinationCoordinate))
				continue;

			validBarriers.put(barrierID, minDistance);
		}

		return validBarriers;
	}

	/**
	 * Identifies the barriers surrounding the current location by examining the
	 * incoming edges of the node. Adds the identified barriers to the list of
	 * adjacent barriers.
	 */
	private void identifyAdjacentBarriers() {
		// identify barriers around this currentLocation
		List<EdgeGraph> incomingEdges = currentLocation.getEdges();
		for (EdgeGraph edge : incomingEdges) {
			List<Integer> edgeBarriers = edge.attributes.get("barriers").getArray();
			if (!edgeBarriers.isEmpty())
				visitedBarriers.addAll(edgeBarriers);
		}
	}

	/**
	 * Identifies an edge-subGoals associated with the most attractive barrier,
	 * complying with certain criteria, and finds the closest edge to them.
	 *
	 * @param validBarriers a set of valid, intersecting barriers towards the
	 *                      destination.
	 * @param region        the region of the currentLocation, only for region-based
	 *                      navigation.
	 * @return a Pair of EdgeGraph and Integer representing the closest edge to the
	 *         barrier and the barrierID.
	 */
	protected Pair<EdgeGraph, Integer> identifyBarrierSubGoal(Map<Integer, Double> validBarriers, Region region) {

		List<EdgeGraph> regionEdges = new ArrayList<>();
		if (regionBasedNavigation)
			regionEdges = region.edges;

		EdgeGraph edgeGoal = null;

		// sorted by distance (further away first, as it leads your further away)
		Map<Integer, Double> validSorted = (LinkedHashMap<Integer, Double>) Utilities.sortByValue(validBarriers, true);

		List<Integer> barrierIDs = new ArrayList<>();
		List<EdgeGraph> possibleEdgeGoals = new ArrayList<>();

		int waterCounter = 0;
		int parkCounter = 0;

		// When more than one barrier is identified, the farthest water body barrier is
		// chosen; if no water bodies are identified, the agent picks the farthest park
		// barrier, if any, or, otherwise, the farthest viable severing barrier.
		for (int barrierID : validSorted.keySet()) {
			Barrier barrier = PedSimCity.barriersMap.get(barrierID);
			String type = barrier.type;
			List<EdgeGraph> edgesAlong = new ArrayList<>(barrier.edgesAlong);

			// for region-based, only consider edges in the region
			if (regionBasedNavigation) {
				edgesAlong.retainAll(regionEdges);
				if (edgesAlong.isEmpty())
					continue;
			}

			Map<EdgeGraph, Double> thisBarrierEdgeGoals = keepValidSubGoals(edgesAlong);
			if (thisBarrierEdgeGoals.isEmpty())
				continue;

			// this is considered a good Edge, sort by distance and takes the closest to the
			// current location.
			Map<EdgeGraph, Double> thisBarrierSubGoalSorted = (LinkedHashMap<EdgeGraph, Double>) Utilities
					.sortByValue(thisBarrierEdgeGoals, false);
			EdgeGraph possibleEdgeGoal = thisBarrierSubGoalSorted.keySet().iterator().next();

			switch (type) {
			case "water" -> {
				barrierIDs.add(waterCounter, barrierID);
				possibleEdgeGoals.add(waterCounter, possibleEdgeGoal);
				waterCounter++;
				parkCounter++;
			}
			case "park" -> {
				barrierIDs.add(parkCounter);
				possibleEdgeGoals.add(parkCounter, possibleEdgeGoal);
				parkCounter++;
			}
			default -> {
				barrierIDs.add(barrierID);
				possibleEdgeGoals.add(possibleEdgeGoal);
			}
			}
		}

		if (possibleEdgeGoals.isEmpty() || possibleEdgeGoals.get(0) == null)
			return null;

		edgeGoal = possibleEdgeGoals.get(0);
		int barrierID = barrierIDs.get(0);
		Pair<EdgeGraph, Integer> pair = new Pair<>(edgeGoal, barrierID);
		return pair;
	}

	/**
	 * Checks requirements for selecting barrier sub-goals from a list of edges
	 * along a barrier.
	 *
	 * @param edgesAlong a list of edges along a barrier;
	 * @return a HashMap of EdgeGraph and Double representing eligible barrier
	 *         sub-goals and their distances.
	 */
	private Map<EdgeGraph, Double> keepValidSubGoals(List<EdgeGraph> edgesAlong) {

		final Map<EdgeGraph, Double> thisBarrierEdgeGoals = new HashMap<>();

		for (EdgeGraph edge : edgesAlong) {
			double distanceToEdge = GraphUtils.euclideanDistance(currentLocation.getCoordinate(),
					edge.getCoordsCentroid());
			double distanceToDestination = GraphUtils.getCachedNodesDistance(currentLocation, destinationNode);

			if (distanceToEdge > distanceToDestination) {
				continue;
			}

			boolean containsInSequence = sequence.stream().anyMatch(node -> node.getEdges().contains(edge));
			boolean containsInCurrentLocation = currentLocation.getEdges().contains(edge);

			if (containsInSequence || containsInCurrentLocation)
				continue;

			thisBarrierEdgeGoals.put(edge, distanceToEdge);
		}
		return thisBarrierEdgeGoals;
	}
}
