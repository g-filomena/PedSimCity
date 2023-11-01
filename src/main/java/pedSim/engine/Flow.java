package pedSim.engine;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.planargraph.DirectedEdge;

import pedSim.agents.Agent;
import pedSim.agents.AgentProperties;
import pedSim.agents.EmpiricalAgentProperties;
import pedSim.utilities.RouteData;
import sim.graph.EdgeGraph;
import sim.graph.NodeGraph;

/**
 * The Flow class provides methods for updating various data related to agent
 * movement and route storing in the simulation.
 */
public class Flow {

	/**
	 * Updates the edge data on the basis of the passed agent's route and its edges
	 * sequence.
	 *
	 * @param agent                 The agent for which edge data is updated.
	 * @param directedEdgesSequence The sequence of directed edges traveled by the
	 *                              agent.
	 */
	public static void updateEdgeData(Agent agent, ArrayList<DirectedEdge> directedEdgesSequence) {

		AgentProperties agentProperties = agent.getProperties();
		String attributeName = Parameters.empirical ? ((EmpiricalAgentProperties) agentProperties).groupName
				: agentProperties.routeChoice.toString();

		for (DirectedEdge directedEdge : directedEdgesSequence) {
			EdgeGraph edge = (EdgeGraph) directedEdge.getEdge();
			EdgeGraph originalEdge = PedSimCity.edgesMap.get(edge.getID());
			originalEdge.volumes.replace(attributeName, originalEdge.volumes.get(attributeName) + 1);
		}
	}

	/**
	 * Stores route data for an agent based on the specified edge IDs sequence.
	 *
	 * @param agent           The agent for which route data is stored.
	 * @param edgeIDsSequence The sequence of edge IDs traveled by the agent.
	 */
	public static void storeRouteData(Agent agent, ArrayList<Integer> edgeIDsSequence) {
		RouteData route = createRouteData(agent);
		List<Coordinate> allCoords = computeAllCoordinates(edgeIDsSequence, agent);
		route.edgeIDsSequence = edgeIDsSequence;
		route.lineGeometry = createLineGeometry(allCoords);
		PedSimCity.routesData.add(route);
	}

	/**
	 * Creates and initialises a new RouteData object for the given agent.
	 *
	 * @param agent The agent for which route data is created.
	 * @return A RouteData object containing route information.
	 */
	private static RouteData createRouteData(Agent agent) {
		RouteData route = new RouteData();
		route.origin = agent.originNode.getID();
		route.destination = agent.destinationNode.getID();

		if (Parameters.empirical)
			route = getDataFromEmpiricalAgent(agent, route);
		else
			route.routeChoice = agent.getProperties().routeChoice.toString();

		return route;
	}

	/**
	 * Extracts data from an empirical agent's properties and populates the route
	 * data.
	 *
	 * @param agent The empirical agent for which route data is extracted.
	 * @param route The RouteData object to be populated with data.
	 * @return A RouteData object containing route information.
	 */
	private static RouteData getDataFromEmpiricalAgent(Agent agent, RouteData route) {
		EmpiricalAgentProperties agentProperties = (EmpiricalAgentProperties) agent.getProperties();
		route.group = agentProperties.groupName;
		route.routeID = agent.originNode.getID() + "-" + agent.destinationNode.getID();
		route.minimisingDistance = agentProperties.minimisingDistance ? true : false;
		route.minimisingAngular = agentProperties.minimisingAngular ? true : false;
		route.localHeuristicDistance = agentProperties.localHeuristicDistance ? true : false;
		route.localHeuristicAngular = agentProperties.localHeuristicAngular ? true : false;
		route.barrierSubGoals = agentProperties.barrierBasedNavigation ? true : false;
		route.distantLandmarks = agentProperties.usingDistantLandmarks ? true : false;
		route.regionBased = agentProperties.regionBasedNavigation ? true : false;
		route.onRouteMarks = agentProperties.usingLocalLandmarks ? true : false;
		route.naturalBarriers = agentProperties.naturalBarriers;
		route.severingBarriers = agentProperties.severingBarriers;
		return route;
	}

	/**
	 * Computes all coordinates along the sequence of edges for an agent's route.
	 *
	 * @param sequenceEdges The sequence of edge IDs traveled by the agent.
	 * @param agent         The agent for which coordinates are computed.
	 * @return A list of coordinates representing the agent's route.
	 */
	private static List<Coordinate> computeAllCoordinates(List<Integer> sequenceEdges, Agent agent) {
		List<Coordinate> allCoords = new ArrayList<>();
		NodeGraph lastNode = agent.originNode;

		for (int i : sequenceEdges) {
			EdgeGraph edge = PedSimCity.edgesMap.get(i);
			LineString geometry = (LineString) edge.masonGeometry.geometry;
			Coordinate[] coords = geometry.getCoordinates();
			List<Coordinate> coordsCollection = new ArrayList<>(Arrays.asList(coords));

			if (coords[0].distance(lastNode.getCoordinate()) > coords[coords.length - 1]
					.distance(lastNode.getCoordinate())) {
				Collections.reverse(coordsCollection);
			}

			coordsCollection.set(0, lastNode.getCoordinate());

			if (lastNode.equals(edge.fromNode))
				lastNode = edge.toNode;
			else if (lastNode.equals(edge.toNode))
				lastNode = edge.fromNode;
			else
				System.out.println("Something is wrong with the sequence in this agent");

			coordsCollection.set(coordsCollection.size() - 1, lastNode.getCoordinate());
			allCoords.addAll(coordsCollection);
		}
		return allCoords;
	}

	/**
	 * Creates a LineString geometry from a list of coordinates.
	 *
	 * @param coordinates The list of coordinates to create a LineString from.
	 * @return A LineString geometry representing the route.
	 */
	private static LineString createLineGeometry(List<Coordinate> coordinates) {
		GeometryFactory factory = new GeometryFactory();
		Coordinate[] coordsArray = coordinates.toArray(new Coordinate[0]);
		return factory.createLineString(coordsArray);
	}
}
