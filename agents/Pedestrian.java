package pedsimcity.agents;

import java.util.ArrayList;
import java.util.List;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.linearref.LengthIndexedLine;

import pedsimcity.main.PedSimCity;
import pedsimcity.main.RouteData;
import pedsimcity.main.UserParameters;
import pedsimcity.routeChoice.CombinedNavigation;
import pedsimcity.routeChoice.RoutePlanner;
import sim.engine.SimState;
import sim.engine.Steppable;
import sim.engine.Stoppable;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.MasonGeometry;
import sim.util.geo.PointMoveTo;
import urbanmason.main.EdgeGraph;
import urbanmason.main.NodeGraph;
import urbanmason.main.NodesLookup;

public final class Pedestrian implements Steppable {

	private static final long serialVersionUID = -1113018274619047013L;
	PedSimCity state;
	public Integer agentID;

	// Initial Attributes
	NodeGraph originNode = null;
	public NodeGraph destinationNode = null;
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
	ArrayList<NodeGraph> sequence = new ArrayList<>();
	boolean reachedDestination = false;
	Integer numTrips = 0;
	AgentProperties ap = new AgentProperties();
	AgentGroupProperties agp = new AgentGroupProperties();
	Stoppable killAgent;

	// time
	int minutesSoFar = 0;

	/** Constructor Function */
	public Pedestrian(PedSimCity state, AgentProperties ap) {

		this.state = state;
		final GeometryFactory fact = new GeometryFactory();
		this.agentLocation = new MasonGeometry(fact.createPoint(new Coordinate(10, 10)));
		if (UserParameters.empiricalABM)
			this.agp = (AgentGroupProperties) ap;
		else
			this.ap = ap;
		if (!UserParameters.empiricalABM) {
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

		final RouteData route = new RouteData();
		route.origin = this.originNode.getID();
		route.destination = this.destinationNode.getID();

		if (UserParameters.empiricalABM) {
			System.out.println("   Agent nr. " + this.agentID + " group " + this.agp.groupName + " OD "
					+ this.originNode.getID() + "  " + this.destinationNode.getID());
			this.agp.defineRouteChoiceParameters();
			final CombinedNavigation combinedNavigation = new CombinedNavigation();
			this.newPath = combinedNavigation.path(this.originNode, this.destinationNode, this.agp);
			route.group = this.agp.groupName;
			route.localH = this.agp.localHeuristic;
			route.routeID = this.agentID.toString() + "-" + this.originNode.getID().toString() + "-"
					+ this.destinationNode.getID().toString();
		} else {
			System.out
					.println(this.originNode.getID() + "  " + this.destinationNode.getID() + " " + this.ap.routeChoice);
			this.selectRouteChoice();
			route.routeChoice = this.ap.routeChoice;
			// route.routeID = numTrips;
		}

		final List<Integer> sequenceEdges = new ArrayList<>();

		for (final GeomPlanarGraphDirectedEdge o : this.newPath) {
			// update edge data
			this.updateEdgeData((EdgeGraph) o.getEdge());
			final int edgeID = ((EdgeGraph) o.getEdge()).getID();
			sequenceEdges.add(edgeID);
		}
		route.sequenceEdges = sequenceEdges;
		PedSimCity.routesData.add(route);
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
			if (this.numTrips == this.ap.OD.size() && !UserParameters.empiricalABM
					|| UserParameters.empiricalABM && this.numTrips == UserParameters.numTrips) {
				stateSchedule.agentsList.remove(this);
				if (stateSchedule.agentsList.size() == 0) {
					System.out.println("calling finish");
					stateSchedule.finish();
				}
				this.killAgent.stop();
				return;
			} else if (UserParameters.empiricalABM) {
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
		edge = PedSimCity.edgesMap.get(edge.getID()); // in case it was a subgraph edge
		if (UserParameters.empiricalABM)
			edge.volumes.replace(this.agp.groupName, edge.volumes.get(this.agp.groupName) + 1);
		else
			edge.volumes.replace(this.ap.routeChoice, edge.volumes.get(this.ap.routeChoice) + 1);
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
		final RoutePlanner planner = new RoutePlanner();
		this.sequence = this.ap.listSequences.get(this.numTrips);
		// only minimisation
		if (this.ap.routeChoice.equals("DS"))
			this.newPath = planner.roadDistance(this.originNode, this.destinationNode, this.ap);
		else if (this.ap.routeChoice.equals("AC"))
			this.newPath = planner.angularChangeBased(this.originNode, this.destinationNode, this.ap);
		else if (this.ap.routeChoice.equals("TS"))
			this.newPath = planner.angularChangeBased(this.originNode, this.destinationNode, this.ap);
		// minimisation plus only global landmarks
		else if (this.ap.routeChoice.equals("DG"))
			this.newPath = planner.roadDistance(this.originNode, this.destinationNode, this.ap);
		else if (this.ap.routeChoice.equals("AG"))
			this.newPath = planner.angularChangeBased(this.originNode, this.destinationNode, this.ap);
		// minimisation plus local and (optionally) global landmarks
		else if (this.ap.routeChoice.contains("D") && this.ap.routeChoice.contains("L"))
			this.newPath = planner.roadDistanceSequence(this.sequence, this.ap);
		else if (this.ap.routeChoice.contains("A") && this.ap.routeChoice.contains("L"))
			this.newPath = planner.angularChangeBasedSequence(this.sequence, this.ap);
		// anything with regions and/or barriers, or just barriers
		else if (this.ap.routeChoice.contains("R"))
			this.newPath = planner.regionBarrierBasedPath(this.originNode, this.destinationNode, this.ap);
		else if (this.ap.routeChoice.contains("B"))
			this.newPath = planner.barrierBasedPath(this.originNode, this.destinationNode, this.ap);
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

}
