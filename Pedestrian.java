package sim.app.geo.pedSimCity;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.linearref.LengthIndexedLine;

import sim.app.geo.urbanSim.EdgeGraph;
import sim.app.geo.urbanSim.NodeGraph;
import sim.engine.SimState;
import sim.engine.Steppable;
import sim.engine.Stoppable;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.MasonGeometry;
import sim.util.geo.PointMoveTo;

public final class Pedestrian implements Steppable {

	private static final long serialVersionUID = -1113018274619047013L;
	PedSimCity state;

	// Initial Attributes
	NodeGraph originNode = null;
	NodeGraph destinationNode = null;
	ArrayList<GeomPlanarGraphDirectedEdge> path =  new ArrayList<GeomPlanarGraphDirectedEdge>();

	// point that denotes agent's position
	// private Point location;
	private MasonGeometry agentLocation;

	// How much to move the agent by in each step()
	// One step ==  10 minutes, average speed 1 meter/sec., --> moveRate = 60*10 meters per step
	private double moveRate = 600.00;
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
	ArrayList<NodeGraph> sequence = new ArrayList<NodeGraph>();
	boolean reachedDestination = false;
	int numTrips = 0;
	AgentProperties ap = new AgentProperties();
	Stoppable killAgent;

	//time
	int minutesSoFar = 0;
	boolean activityBased = false;

	/** Constructor Function */
	public Pedestrian(PedSimCity state, AgentProperties ap) {
		this.ap = ap;
		this.state = state;

		if (UserParameters.fiveElements) {
			this.activityBased = true;
			minutesSoFar = UserParameters.startingHour;
		}
		else originNode = (NodeGraph) ap.OD.get(numTrips).getValue(0);

		GeometryFactory fact = new GeometryFactory();
		agentLocation = new MasonGeometry(fact.createPoint(new Coordinate(10, 10)));
		Coordinate startCoord = null;
		startCoord = originNode.getCoordinate();
		updatePosition(startCoord);
	}


	public void findNewAStarPath(PedSimCity state) {
		System.out.println(originNode.getID() + "  "+ destinationNode.getID()+ " "+ap.routeChoice);
		if (activityBased) {
			CombinedNavigation combinedNavigation = new CombinedNavigation();
			newPath = combinedNavigation.path(originNode, destinationNode, this.ap);
		}
		else selectrouteChoice();


		RouteData route = new RouteData();
		route.origin = originNode.getID();
		route.destination = destinationNode.getID();
		route.routeChoice = ap.routeChoice;
		List<Integer> sequenceEdges = new ArrayList<Integer>();
		if (!this.activityBased) route.routeID = numTrips;

		for (GeomPlanarGraphDirectedEdge o : newPath) {
			// update edge data
			updateEdgeData((EdgeGraph) o.getEdge());
			int edgeID = ((EdgeGraph) o.getEdge()).getID();
			sequenceEdges.add(edgeID);
		}
		route.sequenceEdges = sequenceEdges;
		PedSimCity.routesData.add(route);
		indexOnPath = 0;
		path = newPath;

		// set up how to traverse this first link
		EdgeGraph firstEdge = (EdgeGraph) newPath.get(0).getEdge();
		setupEdge(firstEdge); //Sets the Agent up to proceed along an Edge

		// update the current position for this link
		updatePosition(segment.extractPoint(currentIndex));
		numTrips += 1;
	}

	/**
	 * This is called every tick by the scheduler.
	 * It moves the agent along the path.
	 *
	 * @param state the simulation state;
	 */

	@Override
	public void step(SimState state)
	{
		PedSimCity stateSchedule = (PedSimCity) state;
		// check that we've been placed on an Edge  //check that we haven't already reached our destination
		minutesSoFar += UserParameters.minutesPerStep;

		if (activityBased)
		{
			if (minutesSoFar == UserParameters.endingHour) {
				System.out.println("End of the day - calling finish");
				stateSchedule.finish();
			}
			checkRoutine(state);
			if (ap.atPlace) return;
		}
		else {
			if (reachedDestination || destinationNode == null) {
				if (reachedDestination)	reachedDestination = false;
				if (numTrips == ap.OD.size()) {
					stateSchedule.agentsList.remove(this);
					if (stateSchedule.agentsList.size() == 0) {
						System.out.println("calling finish");
						stateSchedule.finish();
					}
					killAgent.stop();
					return;
				}
				originNode = (NodeGraph) ap.OD.get(numTrips).getValue(0);
				updatePosition(originNode.getCoordinate());
				destinationNode = (NodeGraph) ap.OD.get(numTrips).getValue(1);
				findNewAStarPath(stateSchedule);
				return;
			}
		}
		keepWalking();
	}

	/**
	 * Transition to the next edge in the path
	 *
	 * @param residualMove the amount of distance the agent can still travel this step
	 */
	void transitionToNextEdge(double residualMove) {

		// update the counter for where the index on the path is
		indexOnPath += pathDirection;

		// check to make sure the Agent has not reached the end of the path already
		// depends on where you're going!
		if ((pathDirection > 0 && indexOnPath >= path.size()) || (pathDirection < 0 && indexOnPath < 0)) {
			reachedDestination = true;
			indexOnPath -= pathDirection; // make sure index is correct
			return;
		}

		// move to the next edge in the path
		EdgeGraph edge = (EdgeGraph) path.get(indexOnPath).getEdge();
		setupEdge(edge);
		speed = progress(residualMove);
		currentIndex += speed;

		//	speed = socialBasedProgress(residualMove);
		//		currentIndex += residualMove;
		// check to see if the progress has taken the current index beyond its goal
		// given the direction of movement. If so, proceed to the next edge
		if (linkDirection == 1 && currentIndex > endIndex) transitionToNextEdge(currentIndex - endIndex);
		else if (linkDirection == -1 && currentIndex < startIndex) transitionToNextEdge(startIndex - currentIndex);
	}


	/**
	 * Sets the Agent up to proceed along an Edge.
	 *
	 * @param edge the GeomPlanarGraphEdge to traverse next;
	 * */
	void setupEdge(EdgeGraph edge) {

		currentEdge = edge;

		//transform GeomPlanarGraphEdge in Linestring
		LineString line = edge.getLine();
		//index the Linestring
		segment = new LengthIndexedLine(line);
		startIndex = segment.getStartIndex();
		endIndex = segment.getEndIndex();
		linkDirection = 1;

		// check to ensure that Agent is moving in the right direction (direction)
		double distanceToStart = line.getStartPoint().distance(agentLocation.geometry);
		double distanceToEnd = line.getEndPoint().distance(agentLocation.geometry);

		if (distanceToStart <= distanceToEnd) {
			// closer to start
			currentIndex = startIndex;
			linkDirection = 1;
		}
		else if (distanceToEnd < distanceToStart) {
			// closer to end
			currentIndex = endIndex;
			linkDirection = -1;
		}

	}

	/** move the agent to the given coordinates */
	public void updatePosition(Coordinate c) {
		pointMoveTo.setCoordinate(c);
		PedSimCity.agents.setGeometryLocation(agentLocation, pointMoveTo);
	}

	void updateEdgeData(EdgeGraph edge) {
		edge = PedSimCity.edgesMap.get(edge.getID()); //in case it was a subgraph edge
		if (ap.routeChoice.equals("roadDistance")) edge.roadDistance += 1;
		else if (ap.routeChoice.equals("angularChange")) edge.angularChangeLandmarks += 1;
		else if (ap.routeChoice.equals("roadDistanceLandmarks")) edge.roadDistanceLandmarks += 1;
		else if (ap.routeChoice.equals("angularChangeLandmarks")) edge.angularChangeLandmarks += 1;
		else if (ap.routeChoice.equals("localLandmarks")) edge.localLandmarks += 1;
		else if (ap.routeChoice.equals("globalLandmarks")) edge.globalLandmarks += 1;
		else if (ap.routeChoice.contains("roadDistanceRegions")) edge.roadDistanceRegions += 1;
		else if (ap.routeChoice.contains("angularChangeRegions")) edge.angularChangeRegions += 1;
		else if (ap.routeChoice.contains("roadDistanceBarriers")) edge.roadDistanceBarriers += 1;
		else if (ap.routeChoice.contains("angularChangeBarriers")) edge.angularChangeBarriers += 1;
		else if (ap.routeChoice.contains("roadDistanceRegionsBarriers")) edge.roadDistanceRegionsBarriers += 1;
		else if (ap.routeChoice.contains("angularChangeRegionsBarriers")) edge.angularChangeRegionsBarriers += 1;
	}

	public void setStoppable(Stoppable a) {killAgent = a;}


	/** return geometry representing agent location */
	public MasonGeometry getGeometry() {return agentLocation;}

	public void selectrouteChoice()
	{
		if (UserParameters.testingLandmarks) this.sequence = ap.listSequences.get(numTrips);
		RoutePlanner planner = new RoutePlanner();

		if (ap.routeChoice.equals("roadDistance"))	newPath = planner.roadDistance(originNode, destinationNode, ap);
		else if (ap.routeChoice.equals("angularChange")) newPath = planner.angularChangeBased(originNode, destinationNode, ap);
		else if (ap.routeChoice.equals("roadDistanceLandmarks")) newPath = planner.roadDistanceSequence(sequence, ap);
		else if (ap.routeChoice.equals("angularChangeLandmarks")) newPath = planner.angularChangeBasedSequence(sequence, ap);
		else if (ap.routeChoice.equals("localLandmarks")) newPath = planner.roadDistanceSequence(sequence, ap);
		else if (ap.routeChoice.equals("globalLandmarks")) newPath = planner.globalLandmarksPath(originNode, destinationNode, ap);
		else if (ap.routeChoice.contains("Regions")) newPath = planner.regionBarrierBasedPath(originNode, destinationNode, ap);
		else if (ap.routeChoice.contains("Barriers")) newPath= planner.barrierBasedPath(originNode, destinationNode, ap);
		else if (ap.routeChoice.contains("turns")) newPath= planner.angularChangeBased(originNode, destinationNode, ap);
	}


	double progress(double val)
	{
		//        double traffic = world.edgeTraffic.get(currentEdge).size();
		//        double factor = 1000 * edgeLength / (traffic * 5);
		double edgeLength = currentEdge.getLine().getLength();
		double factor = 1000 * edgeLength;
		factor = Math.min(1, factor);

		return val * linkDirection * factor;
	}


	public void keepWalking() {

		// move along the current segment
		speed = progress(moveRate);
		// speed = socialBasedProgress(moveRate);
		currentIndex += speed;
		//		currentIndex += moveRate;
		// check to see if the progress has taken the current index beyond its goal
		// given the direction of movement. If so, proceed to the next edge
		if (linkDirection == 1 && currentIndex > endIndex) {
			Coordinate currentPos = segment.extractPoint(endIndex);
			updatePosition(currentPos);
			transitionToNextEdge(currentIndex - endIndex);
		}
		else if (linkDirection == -1 && currentIndex < startIndex) {
			Coordinate currentPos = segment.extractPoint(startIndex);
			updatePosition(currentPos);
			transitionToNextEdge(startIndex - currentIndex);
		}
		else {
			// just update the position!
			Coordinate currentPos = segment.extractPoint(currentIndex);
			updatePosition(currentPos);
		}
	}

	public void checkRoutine(SimState state) {

		ap.totalTimeAway += UserParameters.minutesPerStep;
		if (reachedDestination) {
			if (destinationNode == ap.workPlace) ap.atWork = true;
			if (destinationNode == ap.homePlace) ap.atHome = true;
			if (destinationNode == ap.otherPlace) ap.away = true;
			ap.atPlace = true;
			reachedDestination = false;
		}
		if (!reachedDestination & !ap.atPlace) return;

		if (ap.atWork | ap.away) ap.timeAway += UserParameters.minutesPerStep;
		else if (ap.atHome) ap.timeAtHome += UserParameters.minutesPerStep;
		if ((ap.timeAtWork < ap.thresholdAway) | (ap.timeAtHome < ap.thresholdAtHome) | (ap.timeAway < ap.thresholdAway)) return;

		// new activity
		double probNightOut = 0.40;
		double probErrands = 0.35;
		Random random = new Random();
		ActivityPlanner activity = new ActivityPlanner();
		// 20.00 pm (approx)
		int evening = 20*60+random.nextInt(30);
		int afternoon = 15*60+random.nextInt(30);
		int morning = 8*60+random.nextInt(30);

		if (ap.student || ap.worker)
		{
			if (ap.atWork) {
				// either go home or for errands (25% of chances if it's earlier than 20pm
				if (random.nextFloat() <= probErrands & minutesSoFar < evening) activity.goForErrands(this);
				else activity.goHome(this);
			}
			else if (ap.atHome & ap.timeAtWork == 0.0) activity.goToWork(this);
			else if (ap.atHome & ap.timeAtWork > 0.0) {
				if (ap.student) probNightOut = 0.60;
				// go out or for other errands
				if (minutesSoFar < evening & random.nextFloat() <= probErrands) activity.goForErrands(this);
				else if (minutesSoFar > evening & random.nextFloat() <= probNightOut) activity.goForLeisure(this);
			}
			else if (ap.away) activity.goHome(this);
		}

		else if (ap.flaneur)
		{
			if ((ap.atHome && minutesSoFar > morning) || (ap.totalTimeAway < ap.thresholdWandering)) {
				if (minutesSoFar < evening & random.nextFloat() <= probErrands) activity.goForErrands(this);
				activity.goForLeisure(this);
			}
			else if (ap.totalTimeAway >= ap.thresholdWandering) {
				activity.goHome(this);
			}
		}

		else if (ap.homeBased)
		{
			if (ap.atHome && minutesSoFar < evening) {
				// either go home or for errands (45% of chances if it's earlier than 20pm
				if (minutesSoFar > morning & random.nextFloat() <= probErrands) activity.goForErrands(this);
				else if (minutesSoFar > afternoon) activity.goForLeisure(this);
			}
			// either go home or for errands (25% of chances if it's earlier than 20pm
			else if (ap.atHome && minutesSoFar > evening) {
				// either go home or for errands (25% of chances if it's earlier than 20pm
				if (random.nextFloat() <= probNightOut) activity.goForLeisure(this);
			}
			else if (ap.away) activity.goHome(this);
		}
	}
}

