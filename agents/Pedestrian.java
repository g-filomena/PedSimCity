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
	Integer numTrips = 0;
	AgentProperties ap = new AgentProperties();
	AgentGroupProperties agp = new AgentGroupProperties();
	Stoppable killAgent;

	//time
	int minutesSoFar = 0;

	/** Constructor Function */
	public Pedestrian(PedSimCity state, AgentProperties ap) {

		this.state = state;
		GeometryFactory fact = new GeometryFactory();
		agentLocation = new MasonGeometry(fact.createPoint(new Coordinate(10, 10)));

		if (UserParameters.activityBased) minutesSoFar = UserParameters.startingHour;
		if (UserParameters.empiricalABM) this.agp = (AgentGroupProperties) ap;
		else this.ap = ap;

		if (!UserParameters.empiricalABM) {
			originNode = (NodeGraph) ap.OD.get(numTrips).getValue(0);
			Coordinate startCoord = null;
			startCoord = originNode.getCoordinate();
			updatePosition(startCoord);
		}
	}

	/**
	 * It formulates a new path when the agent is done with its previous one.
	 *
	 * @param state the simulation state;
	 */
	public void findNewAStarPath(PedSimCity state) {

		RouteData route = new RouteData();
		route.origin = originNode.getID();
		route.destination = destinationNode.getID();
		//		originNode = PedSimCity.nodesMap.get(9406);
		//		destinationNode = PedSimCity.nodesMap.get(4456);

		if (UserParameters.empiricalABM) {
			System.out.println("   Agent nr. "+this.agentID + " group " + this.agp.groupName + " OD " + originNode.getID()+"  " +destinationNode.getID());
			agp.defineRouteChoiceParameters();
			CombinedNavigation combinedNavigation = new CombinedNavigation();
			newPath = combinedNavigation.path(originNode, destinationNode, agp);
			route.group = this.agp.groupID;
			route.localH = this.agp.localHeuristic;
			route.routeID = this.agentID.toString()+"-"+originNode.getID().toString()+"-"+destinationNode.getID().toString();
		}
		else {
			System.out.println(originNode.getID() + "  "+ destinationNode.getID()+ " "+ap.routeChoice);
			selectRouteChoice();
			route.routeChoice = ap.routeChoice;
			//			route.routeID = numTrips;
		}

		List<Integer> sequenceEdges = new ArrayList<Integer>();

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
		minutesSoFar += UserParameters.minutesPerStep;

		if (UserParameters.activityBased && minutesSoFar == UserParameters.endingHour) {
			System.out.println("End of the day - calling finish");
			stateSchedule.finish();
		}
		else if ((reachedDestination || destinationNode == null) && !UserParameters.activityBased) {

			if (reachedDestination)	reachedDestination = false;
			if ((numTrips == ap.OD.size() && !UserParameters.empiricalABM)  ||
					(UserParameters.empiricalABM && this.numTrips == UserParameters.numTrips)) {
				stateSchedule.agentsList.remove(this);
				if (stateSchedule.agentsList.size() == 0) {
					System.out.println("calling finish");
					stateSchedule.finish();
				}
				killAgent.stop();
				return;
			}
			else if (UserParameters.empiricalABM) {
				originNode = destinationNode = null;
				while (originNode == null) originNode = NodesLookup.randomNode(PedSimCity.network);
				while (destinationNode == null)	destinationNode = NodesLookup.randomNodeBetweenLimits(PedSimCity.network, originNode,
						UserParameters.minDistance, UserParameters.maxDistance);
			}
			else {
				originNode = (NodeGraph) ap.OD.get(numTrips).getValue(0);
				destinationNode = (NodeGraph) ap.OD.get(numTrips).getValue(1);
			}
			updatePosition(originNode.getCoordinate());
			findNewAStarPath(stateSchedule);
			return;
		}
		else if (UserParameters.activityBased)  {
			ap.totalTimeAway += UserParameters.minutesPerStep;
			if (!reachedDestination & !ap.atPlace) keepWalking();
			else {
				ActivityPlanner activityPlanner = new ActivityPlanner();
				activityPlanner.checkRoutine(state, this);
				if (ap.atPlace) return;
			}
		}
		else keepWalking();
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

		// check to see if the progress has taken the current index beyond its goal
		// given the direction of movement. If so, proceed to the next edge
		if (linkDirection == 1 && currentIndex > endIndex) transitionToNextEdge(currentIndex - endIndex);
		else if (linkDirection == -1 && currentIndex < startIndex) transitionToNextEdge(startIndex - currentIndex);
	}


	/**
	 * Sets the Agent up to proceed along an Edge.
	 *
	 * @param edge the EdgeGraph to traverse next;
	 * */
	void setupEdge(EdgeGraph edge) {

		if (UserParameters.socialInteraction) updateCrowdness(edge);
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

	/**
	 * It moves the agent to the given coordinates.
	 *
	 * @param c the coordinates;
	 **/
	public void updatePosition(Coordinate c) {
		pointMoveTo.setCoordinate(c);
		PedSimCity.agents.setGeometryLocation(agentLocation, pointMoveTo);
	}

	/**
	 * It updates the volumes on a given edge, on the basis of the agent's route choice model.
	 *
	 * @param EdgeGraph the edge;
	 **/
	void updateEdgeData(EdgeGraph edge) {
		edge = PedSimCity.edgesMap.get(edge.getID()); //in case it was a subgraph edge
		if (UserParameters.empiricalABM) edge.densities.replace(agp.groupName, edge.densities.get(agp.groupName)+1);
		else edge.densities.replace(ap.routeChoice, edge.densities.get(ap.routeChoice)+1);
	}

	public void setStoppable(Stoppable a) {killAgent = a;}

	/** It returns the geometry representing agent location */
	public MasonGeometry getGeometry() {return agentLocation;}

	/** It select the route choice model and it calls the path formulation algorithm  */
	public void selectRouteChoice()
	{
		RoutePlanner planner = new RoutePlanner();
		this.sequence = ap.listSequences.get(numTrips);
		// only minimisation
		if (ap.routeChoice.equals("DS")) newPath = planner.roadDistance(originNode, destinationNode, ap);
		else if (ap.routeChoice.equals("AC")) newPath = planner.angularChangeBased(originNode, destinationNode, ap);
		else if (ap.routeChoice.equals("TS")) newPath = planner.angularChangeBased(originNode, destinationNode, ap);
		// minimisation plus only global landmarks
		else if (ap.routeChoice.equals("DG")) newPath = planner.roadDistance(originNode, destinationNode, ap);
		else if (ap.routeChoice.equals("AG")) newPath = planner.angularChangeBased(originNode, destinationNode, ap);
		// minimisation plus local and (optionally) global landmarks
		else if (ap.routeChoice.contains("D") && ap.routeChoice.contains("L")) newPath = planner.roadDistanceSequence(sequence, ap);
		else if (ap.routeChoice.contains("A") && ap.routeChoice.contains("L")) newPath = planner.angularChangeBasedSequence(sequence, ap);
		// anything with regions and/or barriers, or just barriers
		else if (ap.routeChoice.contains("R")) newPath = planner.regionBarrierBasedPath(originNode, destinationNode, ap);
		else if (ap.routeChoice.contains("B")) newPath = planner.barrierBasedPath(originNode, destinationNode, ap);
	}

	/** It computes the agents' speed  */
	double progress(double val)
	{
		return val * linkDirection;
	}

	double progressSocial(double val)
	{
		double edgeLength = currentEdge.getLine().getLength();
		double crowdness = PedSimCity.edgesVolume.get(currentEdge).size();
		double factor = 1000 * edgeLength / (crowdness * 5);
		factor = Math.min(1, factor);
		return val * linkDirection;
	}


	/** It makes the agent move along the computed route  */
	public void keepWalking() {
		// move along the current segment
		if (UserParameters.socialInteraction) speed = progressSocial(moveRate);
		else speed = progress(moveRate);
		currentIndex += speed;
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

	public void updateCrowdness(EdgeGraph newEdge) {
		// clean up on old edge
		if (currentEdge != null) {
			ArrayList<Pedestrian> currentVolume = PedSimCity.edgesVolume.get(currentEdge);
			currentVolume.remove(this);
		}

		// update new edge traffic
		if (PedSimCity.edgesVolume.get(newEdge) == null) PedSimCity.edgesVolume.put(newEdge, new ArrayList<Pedestrian>());
		PedSimCity.edgesVolume.get(newEdge).add(this);
	}


}

