package sim.app.geo.pedSimCity;

import java.util.ArrayList;
import java.util.List;

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

public final class Pedestrian implements Steppable
{

	private static final long serialVersionUID = -1113018274619047013L;
	PedSimCity state;

	// Initial Attributes
	NodeGraph originNode = null;
	NodeGraph destinationNode = null;
	public Integer maxDistance;
	ArrayList<GeomPlanarGraphDirectedEdge> path =  new ArrayList<GeomPlanarGraphDirectedEdge>();

	// point that denotes agent's position
	// private Point location;
	private MasonGeometry agentLocation;

	// How much to move the agent by in each step()
	private double moveRate = 240.00;
	PointMoveTo pointMoveTo = new PointMoveTo();

	// Used by agent to walk along line segment
	private LengthIndexedLine segment = null;
	double startIndex = 0.0; // start position of current line
	double endIndex = 0.0; // end position of current line
	double currentIndex = 0.0; // current location along line
	EdgeGraph currentEdge = null;
	int linkDirection = 1;
	double speed = 0;
	int indexOnPath = 0;
	int pathDirection = 1;



	ArrayList<NodeGraph> sequence = new ArrayList<NodeGraph>();
	boolean reachedDestination = false;
	int numTrips = 0;

	AgentProperties ap = new AgentProperties();
	Stoppable killAgent;

	/** Constructor Function */
	public Pedestrian(PedSimCity state, AgentProperties ap)
	{
		this.ap = ap;
		this.state = state;

		NodeGraph originNode = (NodeGraph) ap.OD.get(numTrips).getValue(0);

		GeometryFactory fact = new GeometryFactory();
		agentLocation = new MasonGeometry(fact.createPoint(new Coordinate(10, 10)));
		Coordinate startCoord = null;
		startCoord = originNode.getCoordinate();
		updatePosition(startCoord);
	}


	public void findNewAStarPath(PedSimCity state)
	{
		//    	Node currentJunction;
		//    	if (state.dynamicRouting)
		//    	{
		//	        // get the starting and goal Nodes with which this Agent is associated
		//	    	currentJunction = state.network.findNode(agentLocation.geometry.getCoordinate());
		//
		//	    	if (currentJunction == null)
		//		   	{
		//		   		repositionAgent();
		//		   		currentJunction = state.network.findNode(agentLocation.geometry.getCoordinate());
		//		   	}
		//	    	while (currentJunction == destinationNode) destinationNode = nodesLookup.searchRandomNode(state.geometriesNodes, state);
		//    	}
		ArrayList<GeomPlanarGraphDirectedEdge> newPath = null;

		this.sequence = ap.listSequences.get(numTrips);
		//        System.out.println("trip "+criteria+" OD "+originNode.getID()+" "+destinationNode.getID());

		RoutePlanner planner = new RoutePlanner();

		if (ap.criteria == "roadDistance") newPath = planner.roadDistance(originNode, destinationNode, null, ap);
		else if (ap.criteria == "angularChange") newPath = planner.angularChange(originNode, destinationNode, null, null, ap);
		else if (ap.criteria == "roadDistanceLandmarks") newPath = planner.roadDistanceLandmarks(sequence, ap);
		else if (ap.criteria == "angularChangeLandmarks") newPath = planner.angularChangeLandmarks(sequence, ap);
		else if (ap.criteria == "localLandmarks") newPath = planner.roadDistanceSequence(sequence, ap);
		else if (ap.criteria == "globalLandmarks") newPath = planner.globalLandmarksPath(originNode, destinationNode, null, ap);
		else if (ap.criteria == "roadDistanceBarriers" || ap.criteria == "angularChangeBarriers")
			newPath = planner.barrierBasedPath(originNode, destinationNode, ap);
		else newPath = planner.regionBarrierBasedPath(originNode, destinationNode, ap);

		RouteData route = new RouteData();
		route.origin = originNode.getID();
		route.destination = destinationNode.getID();
		route.criteria = ap.criteria;
		List<Integer> sequenceEdges = new ArrayList<Integer>();


		for (GeomPlanarGraphDirectedEdge o : newPath)
		{
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

	double progress(double val)
	{
		//        double traffic = world.edgeTraffic.get(currentEdge).size();
		//        double factor = 1000 * edgeLength / (traffic * 5);
		double edgeLength = currentEdge.getLine().getLength();
		double factor = 1000 * edgeLength;
		factor = Math.min(1, factor);

		return val * linkDirection * factor;
	}


	/** Called every tick by the scheduler */
	/** moves the agent along the path */

	@Override
	public void step(SimState state)
	{
		PedSimCity stateSchedule = (PedSimCity) state;
		// check that we've been placed on an Edge  //check that we haven't already reached our destination
		if (reachedDestination || destinationNode == null)
		{
			if (reachedDestination)	reachedDestination = false;
			if (numTrips == ap.OD.size())
			{
				stateSchedule.agentsList.remove(this);
				if (stateSchedule.agentsList.size() == 0)
				{
					System.out.println("calling finish");
					stateSchedule.finish();
				}
				killAgent.stop();
				return;
			}
			//        	repositionAgent();
			originNode = (NodeGraph) ap.OD.get(numTrips).getValue(0);
			updatePosition(originNode.getCoordinate());
			destinationNode = (NodeGraph) ap.OD.get(numTrips).getValue(1);
			findNewAStarPath(stateSchedule);
			return;
		}
		// move along the current segment
		speed = progress(moveRate);
		currentIndex += speed;

		// check to see if the progress has taken the current index beyond its goal
		// given the direction of movement. If so, proceed to the next edge
		if (linkDirection == 1 && currentIndex > endIndex)
		{
			Coordinate currentPos = segment.extractPoint(endIndex);
			updatePosition(currentPos);
			transitionToNextEdge(currentIndex - endIndex);
		}
		else if (linkDirection == -1 && currentIndex < startIndex)
		{
			Coordinate currentPos = segment.extractPoint(startIndex);
			updatePosition(currentPos);
			transitionToNextEdge(startIndex - currentIndex);
		} else
		{ // just update the position!
			Coordinate currentPos = segment.extractPoint(currentIndex);
			updatePosition(currentPos);
		}
	}

	/**
	 * Transition to the next edge in the path
	 * @param residualMove the amount of distance the agent can still travel
	 * this turn
	 */
	void transitionToNextEdge(double residualMove)
	{
		// update the counter for where the index on the path is
		indexOnPath += pathDirection;

		// check to make sure the Agent has not reached the end
		// of the path already
		if ((pathDirection > 0 && indexOnPath >= path.size()) || (pathDirection < 0 && indexOnPath < 0))// depends on where you're going!
		{
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


	/** Sets the Agent up to proceed along an Edge
	 * @param edge the GeomPlanarGraphEdge to traverse next
	 * */
	void setupEdge(EdgeGraph edge)
	{
		// storing data about number of pedestrians

		//        // clean up on old edge
		//        if (currentEdge != null)
		//        {
		//            ArrayList<PedestrianLondon> traffic = world.edgeTraffic.get(currentEdge);
		//            traffic.remove(this);
		//        }

		currentEdge = edge;

		// update new edge traffic
		//        if (world.edgeTraffic.get(currentEdge) == null)
		//        {
		//            world.edgeTraffic.put(currentEdge, new ArrayList<PedestrianLondon>());
		//        }
		//        world.edgeTraffic.get(currentEdge).add(this);

		// set up the new segment and index info

		LineString line = edge.getLine(); //transforming GeomPlanarGraphEdge in Linestring
		segment = new LengthIndexedLine(line); //indexing the Linestring
		startIndex = segment.getStartIndex();
		endIndex = segment.getEndIndex();
		linkDirection = 1;

		// check to ensure that Agent is moving in the right direction (direction)
		double distanceToStart = line.getStartPoint().distance(agentLocation.geometry);
		double distanceToEnd = line.getEndPoint().distance(agentLocation.geometry);

		if (distanceToStart <= distanceToEnd)
		{ // closer to start
			currentIndex = startIndex;
			linkDirection = 1;
		} else if (distanceToEnd < distanceToStart)
		{ // closer to end
			currentIndex = endIndex;
			linkDirection = -1;
		}

	}


	/** move the agent to the given coordinates */
	public void updatePosition(Coordinate c)
	{
		pointMoveTo.setCoordinate(c);
		PedSimCity.agents.setGeometryLocation(agentLocation, pointMoveTo);
	}

	void updateEdgeData(EdgeGraph edge)
	{

		if (ap.criteria == "roadDistance") edge.roadDistance += 1;
		else if (ap.criteria == "angularChange") edge.angularChange += 1;
		else if (ap.criteria == "topological") edge.topological += 1;
		else if (ap.criteria == "roadDistanceLandmarks") edge.roadDistanceLandmarks += 1;
		else if (ap.criteria == "angularChangeLandmarks") edge.angularChangeLandmarks += 1;
		else if (ap.criteria == "localLandmarks") edge.localLandmarks += 1;
		else if (ap.criteria == "globalLandmarks") edge.globalLandmarks += 1;
		else if (ap.criteria == "roadDistanceRegions") edge.roadDistanceRegions += 1;
		else if (ap.criteria == "angularChangeRegions") edge.angularChangeRegions += 1;
		else if (ap.criteria == "roadDistanceBarriers") edge.roadDistanceBarriers += 1;
		else if (ap.criteria == "angularChangeBarriers") edge.angularChangeBarriers += 1;
		else if (ap.criteria == "roadDistanceRegionsBarriers") edge.roadDistanceRegionsBarriers += 1;
		else if (ap.criteria == "angularChangeRegionsBarriers") edge.angularChangeRegionsBarriers += 1;
		else System.out.println(ap.criteria);
	}

	public void setStoppable(Stoppable a) {killAgent = a;}


	//    public void repositionAgent()
	//    {
	//        NodeGraph new_start = nodesLookup.randomNode(PedestrianSimulation.nodesGeometries, state);
	//        Coordinate startCoord = new_start.getCoordinate();
	//        updatePosition(startCoord);
	//    }

	/** return geometry representing agent location */
	public MasonGeometry getGeometry() {return agentLocation;}


}
