package pedSim.agents;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import org.javatuples.Pair;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.linearref.LengthIndexedLine;
import org.locationtech.jts.planargraph.DirectedEdge;

import pedSim.cognitiveMap.AgentCognitiveMap;
import pedSim.engine.Parameters;
import pedSim.engine.PedSimCity;
import pedSim.routeChoice.Route;
import pedSim.routeChoice.RoutePlanner;
import sim.engine.SimState;
import sim.engine.Steppable;
import sim.engine.Stoppable;
import sim.graph.EdgeGraph;
import sim.graph.NodeGraph;
import sim.util.geo.MasonGeometry;
import sim.util.geo.PointMoveTo;

/**
 * This class represents an agent in the pedestrian simulation. Agents move
 * along paths between origin and destination nodes.
 */
public final class Agent implements Steppable {

	private static final long serialVersionUID = 1L;
	PedSimCity state;
	public Integer agentID;

	// Initial Attributes
	public List<Pair<NodeGraph, NodeGraph>> OD = new LinkedList<>();
	public NodeGraph originNode = null;
	public NodeGraph destinationNode = null;
	public List<DirectedEdge> directedEdgesSequence = new ArrayList<>();
	private List<Integer> edgeIDsSequence;
	boolean reachedDestination = false;
	public Integer tripsDone = 0;

	// point that denotes agent's position
	// private Point location;
	private AgentProperties agentProperties;
	public AgentCognitiveMap cognitiveMap;

	Stoppable killAgent;
	protected MasonGeometry agentLocation;
	// How much to move the agent by in each step()

	double speed = 0.0;

	// start, current, end position along current line
	EdgeGraph currentEdge = null;
	double startIndex = 0.0;
	double currentIndex = 0.0;
	double endIndex = 0.0;

	// used by agent to walk along line segment
	int linkDirection = 1;
	int indexOnEdgesSequence = 0;
	int pathDirection = 1;
	protected LengthIndexedLine indexedSegment = null;
	public Route route = new Route();

	/**
	 * Constructor Function. Creates a new agent with the specified agent
	 * properties.
	 *
	 * @param state the PedSimCity simulation state.
	 */
	public Agent(PedSimCity state) {

		this.state = state;
		final GeometryFactory fact = new GeometryFactory();
		agentLocation = new MasonGeometry(fact.createPoint(new Coordinate(10, 10)));
		getGeometry().isMovable = true;

		if (!OD.isEmpty()) {
			originNode = (NodeGraph) OD.get(tripsDone).getValue(0);
			Coordinate startCoord = null;
			startCoord = originNode.getCoordinate();
			updateAgentPosition(startCoord);
		}
	}

	/**
	 * Initialises the agent properties.
	 */
	public void initialiseAgentProperties() {
		cognitiveMap = new AgentCognitiveMap();
		agentProperties = new AgentProperties();
	}

	/**
	 * Initialises the agent properties with an empirical group.
	 *
	 * @param empiricalGroup the empirical group for agent properties.
	 */
	public void initialiseAgentProperties(EmpiricalAgentsGroup empiricalGroup) {
		cognitiveMap = new AgentCognitiveMap();
		agentProperties = new EmpiricalAgentProperties(this, empiricalGroup);
	}

	/**
	 * Moves the agent to the given coordinates.
	 *
	 * @param c the coordinates.
	 */
	public void updateAgentPosition(Coordinate c) {
		PointMoveTo pointMoveTo = new PointMoveTo();
		pointMoveTo.setCoordinate(c);
		state.agents.setGeometryLocation(agentLocation, pointMoveTo);
	}

	/**
	 * Performs agent's stepping action in the simulation.
	 *
	 * @param state The simulation state.
	 */
	@Override
	public void step(SimState state) {

		if (reachedDestination || destinationNode == null)
			try {
				handleReachedDestination();
			} catch (Exception e) {
				e.printStackTrace();
			}
		else
			keepWalking();
	}

	/**
	 * Handles the agent's behaviour when reaching its destination.
	 *
	 * @throws Exception
	 */
	protected void handleReachedDestination() throws Exception {

		reachedDestination = false;
		if (tripsDone == OD.size()) {
			removeAgent();
			return;
		} else
			selectNodesFromOD();
		updateAgentPosition(originNode.getCoordinate());
		planRoute();
		initialisePath();
		return;
	}

	/**
	 * Removes the agent from the simulation.
	 *
	 * @param stateSchedule the simulation state.
	 */
	private void removeAgent() {
		state.agentsList.remove(this);
		killAgent.stop();
		if (state.agentsList.isEmpty())
			state.finish();
	}

	/**
	 * Selects origin and destination nodes for the agent.
	 */
	private void selectNodesFromOD() {
		originNode = (NodeGraph) OD.get(tripsDone).getValue(0);
		destinationNode = (NodeGraph) OD.get(tripsDone).getValue(1);
	}

	/**
	 * Initialises the directedEdgesSequence (the path) for the agent.
	 */
	public void initialisePath() {
		this.directedEdgesSequence = route.directedEdgesSequence;
		// set up how to traverse this first link
		indexOnEdgesSequence = 0;
		EdgeGraph firstEdge = (EdgeGraph) directedEdgesSequence.get(0).getEdge();
		// Sets the Agent up to proceed along an Edge
		setupEdge(firstEdge);
		// update the current position for this link
		updateAgentPosition(indexedSegment.extractPoint(currentIndex));
		updateData();
		tripsDone += 1;
	}

	/**
	 * Updates data related to the volumes on the segments traversed.
	 */
	public void updateData() {
		getSequenceEdges();
		state.flowHandler.updateEdgeData(this, directedEdgesSequence);
		state.flowHandler.storeRouteData(this, edgeIDsSequence);
	}

	/**
	 * Plans the route for the agent.
	 * 
	 * @throws Exception
	 */
	protected void planRoute() throws Exception {
		if (Parameters.empirical)
			((EmpiricalAgentProperties) agentProperties).randomizeRouteChoiceParameters();
		if (Parameters.verboseMode) {
			if (agentProperties.routeChoice != null)
				System.out.println("Agent " + agentProperties.routeChoice);
			else
				System.out.println(((EmpiricalAgentProperties) agentProperties).groupName);
			System.out.println(" - origin  " + originNode.getID() + " destination " + destinationNode.getID());
		}
		final RoutePlanner planner = new RoutePlanner(originNode, destinationNode, this);
		route = planner.definePath();
	}

	/**
	 * Gets the sequence of edge IDs for the agent's path.
	 */
	protected void getSequenceEdges() {

		edgeIDsSequence = new ArrayList<>();
		for (DirectedEdge directedEdge : directedEdgesSequence) {

			int edgeID = ((EdgeGraph) directedEdge.getEdge()).getID();
			edgeIDsSequence.add(edgeID);
		}
	}

	/**
	 * Moves the agent along the computed route.
	 */
	protected void keepWalking() {
		// move along the current segment
		speed = updateSpeed();
		currentIndex += speed;

		// check to see if the progress has taken the current index beyond its goal
		// given the direction of movement. If so, proceed to the next edge
		if (linkDirection == 1 && currentIndex > endIndex) {
			final Coordinate currentPos = indexedSegment.extractPoint(endIndex);
			updateAgentPosition(currentPos);
			transitionToNextEdge(currentIndex - endIndex);
		} else if (linkDirection == -1 && currentIndex < startIndex) {
			final Coordinate currentPos = indexedSegment.extractPoint(startIndex);
			updateAgentPosition(currentPos);
			transitionToNextEdge(startIndex - currentIndex);
		} else {
			// just update the position!
			final Coordinate currentPos = indexedSegment.extractPoint(currentIndex);
			updateAgentPosition(currentPos);
		}
	}

	/**
	 * Updates the agent's speed based on the move rate and link direction.
	 *
	 * @return The updated speed of the agent.
	 */
	private double updateSpeed() {
		speed = progress(Parameters.moveRate);
		return speed;
	}

	/**
	 * Transitions to the next edge in the {@code directedEdgesSequence}.
	 *
	 * @param residualMove The amount of distance the agent can still travel this
	 *                     step.
	 */
	void transitionToNextEdge(double residualMove) {

		// update the counter for where the index on the directedEdgesSequence is
		indexOnEdgesSequence += pathDirection;

		// check to make sure the Agent has not reached the end of the
		// directedEdgesSequence already
		// depends on where you're going!
		if (pathDirection > 0 && indexOnEdgesSequence >= directedEdgesSequence.size()
				|| pathDirection < 0 && indexOnEdgesSequence < 0) {
			reachedDestination = true;
			indexOnEdgesSequence -= pathDirection; // make sure index is correct
			return;
		}

		// move to the next edge in the directedEdgesSequence
		final EdgeGraph edge = (EdgeGraph) directedEdgesSequence.get(indexOnEdgesSequence).getEdge();
		setupEdge(edge);
		speed = updateSpeed();
		currentIndex += speed;

		// check to see if the progress has taken the current index beyond its goal
		// given the direction of movement. If so, proceed to the next edge
		if (linkDirection == 1 && currentIndex > endIndex)
			transitionToNextEdge(currentIndex - endIndex);
		else if (linkDirection == -1 && currentIndex < startIndex)
			transitionToNextEdge(startIndex - currentIndex);
	}

	/**
	 * Sets the agent up to proceed along an edge.
	 *
	 * @param edge The EdgeGraph to traverse next.
	 */
	void setupEdge(EdgeGraph edge) {

		currentEdge = edge;
		// transform GeomPlanarGraphEdge in Linestring
		final LineString line = edge.getLine();
		// index the Linestring
		indexedSegment = new LengthIndexedLine(line);
		startIndex = indexedSegment.getStartIndex();
		endIndex = indexedSegment.getEndIndex();
		linkDirection = 1;

		// check to ensure that Agent is moving in the right direction (direction)
		final double distanceToStart = line.getStartPoint().distance(agentLocation.geometry);
		final double distanceToEnd = line.getEndPoint().distance(agentLocation.geometry);

		if (distanceToStart <= distanceToEnd) {
			// closer to start
			currentIndex = startIndex;
			linkDirection = 1;
		} else if (distanceToEnd < distanceToStart) {
			// closer to end
			currentIndex = endIndex;
			linkDirection = -1;
		}
	}

	/**
	 * Computes the agent's speed.
	 *
	 * @param val The value used for computing speed.
	 * @return The computed speed.
	 */
	private double progress(double val) {
		return val * linkDirection;
	}

	/**
	 * Sets the stoppable reference for the agent.
	 *
	 * @param a The stoppable reference.
	 */
	public void setStoppable(Stoppable a) {
		this.killAgent = a;
	}

	/**
	 * Gets the geometry representing the agent's location.
	 *
	 * @return The geometry representing the agent's location.
	 */
	public MasonGeometry getGeometry() {
		return agentLocation;
	}

	public AgentProperties getProperties() {
		return agentProperties;
	}

	/**
	 * Gets the cognitive map.
	 *
	 * @return The cognitive map.
	 */
	synchronized public AgentCognitiveMap getCognitiveMap() {
		return cognitiveMap;
	}

}
