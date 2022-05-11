package pedsimcity.agents;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.linearref.LengthIndexedLine;

import pedsimcity.graph.EdgeGraph;
import pedsimcity.graph.NodeGraph;
import pedsimcity.main.PedSimCity;
import pedsimcity.main.UserParameters;
import pedsimcity.routeChoice.CombinedNavigation;
import pedsimcity.utilities.NodesLookup;
import pedsimcity.utilities.RouteData;
import sim.engine.SimState;
import sim.engine.Steppable;
import sim.engine.Stoppable;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.MasonGeometry;
import sim.util.geo.PointMoveTo;

public final class Pedestrian implements Steppable {

	private static final long serialVersionUID = -1113018274619047013L;
	PedSimCity state;
	public Integer agentID;

	// Initial Attributes
	NodeGraph originNode = null;
	NodeGraph destinationNode = null;
	ArrayList<GeomPlanarGraphDirectedEdge> path = new ArrayList<>();

	// point that denotes agent's position
	// private Point location;
	private final MasonGeometry agentLocation;

	// How much to move the agent by in each step()
	// One step == 10 minutes, average speed 1 meter/sec., --> moveRate = 60*10
	// meters per step
	private final double moveRate = 600.00;
	PointMoveTo pointMoveTo = new PointMoveTo();

	// used by agent to walk along line segment
	private LengthIndexedLine segment = null;
	// start position of current line
	double startIndex = 0.0;
	// end position of current line
	double endIndex = 0.0;
	// current location along line
	double currentIndex = 0.0;
	double speed = 0.0;
	int linkDirection = 1;
	int indexOnPath = 0;
	int pathDirection = 1;
	ArrayList<GeomPlanarGraphDirectedEdge> newPath = null;

	EdgeGraph currentEdge = null;
	boolean reachedDestination = false;
	Integer numTrips = 0;
	AgentProperties ap = new AgentProperties();
	AgentGroupProperties agp;
	Stoppable killAgent;
	// time
	int minutesSoFar = 0;

	/** Constructor Function */
	public Pedestrian(PedSimCity state, AgentProperties ap) {

		this.state = state;
		this.ap = ap;
		if (UserParameters.empiricalABM)
			this.agp = (AgentGroupProperties) ap;
		final GeometryFactory fact = new GeometryFactory();
		this.agentLocation = new MasonGeometry(fact.createPoint(new Coordinate(10, 10)));

		if (ap.OD.size() > 0) {
			this.originNode = (NodeGraph) ap.OD.get(this.numTrips).getValue(0);
			Coordinate startCoord = null;
			startCoord = this.originNode.getCoordinate();
			this.updatePosition(startCoord);
		}
	}

	/**
	 * It formulates a new path when the agent is done with its previous one.
	 *
	 * @param state the simulation state;
	 */
	public void findNewAStarPath(PedSimCity state) {

		if (UserParameters.empiricalABM) {

			this.agp.defineRouteChoiceParameters();

			System.out.println(
					"   Agent nr. " + this.agentID + " group " + this.agp.groupName + " OD " + this.originNode.getID()
							+ "  " + this.destinationNode.getID() + " only minimising " + this.agp.onlyMinimising
							+ " heuristic " + this.agp.localHeuristic + " regions " + this.agp.regionBasedNavigation
							+ " local landmarks " + this.agp.landmarkBasedNavigation + " barrier sub-goals "
							+ this.agp.barrierBasedNavigation + " distant landmarks " + this.agp.usingDistantLandmarks
							+ " barriers " + this.agp.naturalBarriers + "  " + this.agp.aversionSeveringBarriers);
			final CombinedNavigation combinedNavigation = new CombinedNavigation();
			this.newPath = combinedNavigation.path(this.originNode, this.destinationNode, this.agp);
		} else
			this.selectRouteChoice();

		final List<Integer> sequenceEdges = new ArrayList<>();
		for (final GeomPlanarGraphDirectedEdge o : this.newPath) {
			// update edge data
			this.updateEdgeData((EdgeGraph) o.getEdge());
			final int edgeID = ((EdgeGraph) o.getEdge()).getID();
			sequenceEdges.add(edgeID);
		}
		this.storeRouteData(sequenceEdges);

		this.indexOnPath = 0;
		this.path = this.newPath;

		// set up how to traverse this first link
		final EdgeGraph firstEdge = (EdgeGraph) this.newPath.get(0).getEdge();
		this.setupEdge(firstEdge); // Sets the Agent up to proceed along an Edge
		// update the current position for this link
		this.updatePosition(this.segment.extractPoint(this.currentIndex));
		this.numTrips += 1;
	}

	/**
	 * This is called every tick by the scheduler. It moves the agent along the
	 * path.
	 *
	 * @param state the simulation state;
	 */

	@Override
	public void step(SimState state) {
		final PedSimCity stateSchedule = (PedSimCity) state;

		if (this.reachedDestination || this.destinationNode == null) {

			if (this.reachedDestination)
				this.reachedDestination = false;
			if (this.numTrips == this.ap.OD.size() && this.ap.OD.size() > 0
					|| this.ap.OD.size() == 0 && this.numTrips == UserParameters.numTrips) {
				stateSchedule.agentsList.remove(this);
				if (stateSchedule.agentsList.size() == 0)
					stateSchedule.finish();
				this.killAgent.stop();
				return;
			} else if (this.ap.OD.size() == 0) {
				this.originNode = this.destinationNode = null;
				while (this.originNode == null)
					this.originNode = NodesLookup.randomNode(PedSimCity.network);
				while (this.destinationNode == null)
					this.destinationNode = NodesLookup.randomNodeBetweenLimits(PedSimCity.network, this.originNode,
							UserParameters.minDistance, UserParameters.maxDistance);
			} else {
				this.originNode = (NodeGraph) this.ap.OD.get(this.numTrips).getValue(0);
				this.destinationNode = (NodeGraph) this.ap.OD.get(this.numTrips).getValue(1);
			}
			this.updatePosition(this.originNode.getCoordinate());
			this.findNewAStarPath(stateSchedule);
			return;
		} else
			this.keepWalking();
	}

	/**
	 * Transition to the next edge in the path
	 *
	 * @param residualMove the amount of distance the agent can still travel this
	 *                     step
	 */
	void transitionToNextEdge(double residualMove) {

		// update the counter for where the index on the path is
		this.indexOnPath += this.pathDirection;

		// check to make sure the Agent has not reached the end of the path already
		// depends on where you're going!
		if (this.pathDirection > 0 && this.indexOnPath >= this.path.size()
				|| this.pathDirection < 0 && this.indexOnPath < 0) {
			this.reachedDestination = true;
			this.indexOnPath -= this.pathDirection; // make sure index is correct
			return;
		}

		// move to the next edge in the path
		final EdgeGraph edge = (EdgeGraph) this.path.get(this.indexOnPath).getEdge();
		this.setupEdge(edge);
		this.speed = this.progress(residualMove);
		this.currentIndex += this.speed;

		// check to see if the progress has taken the current index beyond its goal
		// given the direction of movement. If so, proceed to the next edge
		if (this.linkDirection == 1 && this.currentIndex > this.endIndex)
			this.transitionToNextEdge(this.currentIndex - this.endIndex);
		else if (this.linkDirection == -1 && this.currentIndex < this.startIndex)
			this.transitionToNextEdge(this.startIndex - this.currentIndex);
	}

	/**
	 * Sets the Agent up to proceed along an Edge.
	 *
	 * @param edge the EdgeGraph to traverse next;
	 */
	void setupEdge(EdgeGraph edge) {

		if (UserParameters.socialInteraction)
			this.updateCrowdness(edge);
		this.currentEdge = edge;
		// transform GeomPlanarGraphEdge in Linestring
		final LineString line = edge.getLine();
		// index the Linestring
		this.segment = new LengthIndexedLine(line);
		this.startIndex = this.segment.getStartIndex();
		this.endIndex = this.segment.getEndIndex();
		this.linkDirection = 1;

		// check to ensure that Agent is moving in the right direction (direction)
		final double distanceToStart = line.getStartPoint().distance(this.agentLocation.geometry);
		final double distanceToEnd = line.getEndPoint().distance(this.agentLocation.geometry);

		if (distanceToStart <= distanceToEnd) {
			// closer to start
			this.currentIndex = this.startIndex;
			this.linkDirection = 1;
		} else if (distanceToEnd < distanceToStart) {
			// closer to end
			this.currentIndex = this.endIndex;
			this.linkDirection = -1;
		}

	}

	/**
	 * It moves the agent to the given coordinates.
	 *
	 * @param c the coordinates;
	 **/
	public void updatePosition(Coordinate c) {
		this.pointMoveTo.setCoordinate(c);
		PedSimCity.agents.setGeometryLocation(this.agentLocation, this.pointMoveTo);
	}

	/**
	 * It updates the volumes on a given edge, on the basis of the agent's route
	 * choice model.
	 *
	 * @param EdgeGraph the edge;
	 **/
	void updateEdgeData(EdgeGraph edge) {
		final EdgeGraph originalEdge = PedSimCity.edgesMap.get(edge.getID()); // in case it was a subgraph edge
		if (UserParameters.empiricalABM)
			originalEdge.volumes.replace(this.agp.groupName, originalEdge.volumes.get(this.agp.groupName) + 1);
		else
			originalEdge.volumes.replace(this.ap.routeChoice, originalEdge.volumes.get(this.ap.routeChoice) + 1);
	}

	public void setStoppable(Stoppable a) {
		this.killAgent = a;
	}

	/** It returns the geometry representing agent location */
	public MasonGeometry getGeometry() {
		return this.agentLocation;
	}

	/**
	 * It select the route choice model and it calls the path formulation algorithm
	 */
	public void selectRouteChoice() {

		final CombinedNavigation combinedNavigation = new CombinedNavigation();
		this.newPath = combinedNavigation.path(this.originNode, this.destinationNode, this.ap);
	}

	/** It computes the agents' speed */
	double progress(double val) {
		return val * this.linkDirection;
	}

	double progressSocial(double val) {
		final double edgeLength = this.currentEdge.getLine().getLength();
		final double crowdness = PedSimCity.edgesVolume.get(this.currentEdge).size();
		double factor = 1000 * edgeLength / (crowdness * 5);
		factor = Math.min(1, factor);
		return val * this.linkDirection;
	}

	/** It makes the agent move along the computed route */
	public void keepWalking() {
		// move along the current segment
		if (UserParameters.socialInteraction)
			this.speed = this.progressSocial(this.moveRate);
		else
			this.speed = this.progress(this.moveRate);
		this.currentIndex += this.speed;
		// check to see if the progress has taken the current index beyond its goal
		// given the direction of movement. If so, proceed to the next edge
		if (this.linkDirection == 1 && this.currentIndex > this.endIndex) {
			final Coordinate currentPos = this.segment.extractPoint(this.endIndex);
			this.updatePosition(currentPos);
			this.transitionToNextEdge(this.currentIndex - this.endIndex);
		} else if (this.linkDirection == -1 && this.currentIndex < this.startIndex) {
			final Coordinate currentPos = this.segment.extractPoint(this.startIndex);
			this.updatePosition(currentPos);
			this.transitionToNextEdge(this.startIndex - this.currentIndex);
		} else {
			// just update the position!
			final Coordinate currentPos = this.segment.extractPoint(this.currentIndex);
			this.updatePosition(currentPos);
		}
	}

	public void updateCrowdness(EdgeGraph newEdge) {
		// clean up on old edge
		if (this.currentEdge != null) {
			final ArrayList<Pedestrian> currentVolume = PedSimCity.edgesVolume.get(this.currentEdge);
			currentVolume.remove(this);
		}

		// update new edge traffic
		if (PedSimCity.edgesVolume.get(newEdge) == null)
			PedSimCity.edgesVolume.put(newEdge, new ArrayList<Pedestrian>());
		PedSimCity.edgesVolume.get(newEdge).add(this);
	}

	public void storeRouteData(List<Integer> sequenceEdges) {

		final RouteData route = new RouteData();
		route.origin = this.originNode.getID();
		route.destination = this.destinationNode.getID();

		if (UserParameters.empiricalABM) {
			route.group = this.agp.groupName;
			route.routeID = this.originNode.getID().toString() + "-" + this.destinationNode.getID().toString();
			route.barrierSubGoals = this.ap.barrierBasedNavigation ? 1 : 0;
			route.distantLandmarks = this.ap.usingDistantLandmarks ? 1 : 0;
			route.regionBased = this.ap.regionBasedNavigation ? 1 : 0;
			route.routeMarks = this.ap.landmarkBasedNavigation ? 1 : 0;
			route.onlyMinimisation = this.ap.onlyMinimising;
			route.localHeuristic = this.ap.localHeuristic;
			route.naturalBarriers = this.ap.naturalBarriers;
			route.severingBarriers = this.ap.severingBarriers;
		} else
			route.routeChoice = this.ap.routeChoice;

		final List<Coordinate> allCoords = new ArrayList<>();
		NodeGraph lastNode = this.originNode;
		for (final int i : sequenceEdges) {
			final EdgeGraph edge = PedSimCity.edgesMap.get(i);
			final LineString geometry = (LineString) edge.masonGeometry.geometry;
			final Coordinate[] coords = geometry.getCoordinates();
			final List<Coordinate> coordsCollection = Arrays.asList(coords);

			if (coords[0].distance(lastNode.getCoordinate()) > coords[coords.length - 1]
					.distance(lastNode.getCoordinate()))
				Collections.reverse(coordsCollection);
			coordsCollection.set(0, lastNode.getCoordinate());
			if (lastNode.equals(edge.u))
				lastNode = edge.v;
			else if (lastNode.equals(edge.v))
				lastNode = edge.u;
			else
				System.out.println("Something is wrong with the sequence in this agent");
			coordsCollection.set(coordsCollection.size() - 1, lastNode.getCoordinate());
			allCoords.addAll(coordsCollection);
		}

		route.sequenceEdges = sequenceEdges;
		PedSimCity.routesData.add(route);

		final GeometryFactory factory = new GeometryFactory();
		final Coordinate[] allCoordsArray = new Coordinate[allCoords.size()];
		for (int i = 0; i <= allCoords.size() - 1; i++)
			allCoordsArray[i] = allCoords.get(i);
		final LineString lineGeometry = factory.createLineString(allCoordsArray);
		route.lineGeometry = lineGeometry;
	}

}
