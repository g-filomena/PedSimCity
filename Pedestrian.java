package sim.app.geo.pedestrianSimulation;

import com.vividsolutions.jts.geom.*;
import com.vividsolutions.jts.linearref.LengthIndexedLine;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import org.javatuples.Pair;

import sim.app.geo.urbanSim.*;
import sim.engine.SimState;
import sim.engine.Steppable;
import sim.engine.Stoppable;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.MasonGeometry;
import sim.util.geo.PointMoveTo;

public final class Pedestrian implements Steppable
{

    private static final long serialVersionUID = -1113018274619047013L;
    PedestrianSimulation state;
   
    // Initial Attributes
    NodeGraph originNode = null;
    NodeGraph destinationNode = null;
    public Integer maxDistance;
    ArrayList<GeomPlanarGraphDirectedEdge> path =  new ArrayList<GeomPlanarGraphDirectedEdge>();
	ArrayList<Pair<NodeGraph, NodeGraph>> OD =  new ArrayList<Pair<NodeGraph, NodeGraph>>();
    
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
    
    public String criteria;
    boolean landmarksRouting = false;
    boolean barriersRouting = false;
    boolean regionalRouting = true;

    boolean reachedDestination = false;
    int numTrips = 0;
    
	HashMap<Integer, NodeGraph> nodesMap;
	HashMap<Integer, EdgeGraph> edgesMap;
	HashMap<Integer, NodeGraph> centroidsMap;
    Stoppable killAgent;
   
    /** Constructor Function */
    public Pedestrian(PedestrianSimulation state, String criteria, ArrayList<Pair<NodeGraph, NodeGraph>> OD)
    {
    	this.criteria = criteria;
    	this.OD = OD;
    	this.state = state;
//    	this.maxDistance = maxDistance;
    	    	
    	if (criteria.contains("Landmarks")) landmarksRouting = true;
    	if (criteria.contains("Barriers")) barriersRouting = true; 
    	if (criteria.contains("Regions")) barriersRouting = true; 
    	
    	NodeGraph originNode = (NodeGraph) OD.get(numTrips).getValue(0);
        GeometryFactory fact = new GeometryFactory();
        agentLocation = new MasonGeometry(fact.createPoint(new Coordinate(10, 10)));
        Coordinate startCoord = null;
        startCoord = originNode.getCoordinate();
        updatePosition(startCoord);   
    }

    
    public void findNewAStarPath(PedestrianSimulation state)
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
    	int barrierID = 999999;
    	ArrayList<GeomPlanarGraphDirectedEdge> newPath = null;
        ArrayList<NodeGraph> sequence = new ArrayList<NodeGraph>();
        System.out.println("trip "+criteria+" OD "+originNode.getID()+" "+destinationNode.getID());
         
        if (criteria == "roadDistance")
        {
        	newPath = routePlanning.roadDistance(originNode, destinationNode, null, 
        			regionalRouting, barriersRouting, barrierID, "dijkstra").edges;
        }
        
        else if (criteria == "angularChange")
        {
        	newPath = routePlanning.angularChange(originNode, destinationNode, null, null,
        			regionalRouting, barriersRouting, barrierID, "dijkstra").edges;
        }
        
//        else if (criteria == "topological")
//        {
//        	newPath = routePlanning.topological(originNode, destinationNode, null, 
//        			regionalRouting, barrierID, state).edges;
//        }

        else if (criteria == "roadDistanceLandmark")
        {
        	sequence = LandmarksNavigation.findSequenceSubGoals(originNode, destinationNode);
        	if (sequence.size() == 0) newPath = routePlanning.roadDistance(originNode, 
        			destinationNode, null, regionalRouting, barriersRouting, barrierID, "astar").edges;
        	else newPath = routePlanning.roadDistanceLandmarksPath(originNode, destinationNode, sequence);
        }
        
        else if (criteria == "angularChangeLandmark")
        {
        	sequence = LandmarksNavigation.findSequenceSubGoals(originNode, destinationNode);
        	if (sequence.size() == 0) newPath = routePlanning.angularChange(originNode, destinationNode, null,null, 
        			regionalRouting, barriersRouting, barrierID, "dijkstra").edges;
        	else newPath = routePlanning.AngularChangeLandmarksPath(originNode,destinationNode, sequence);
        }
        
        else if (criteria == "localLandmarks")
        {
        	sequence = LandmarksNavigation.findSequenceSubGoals(originNode, destinationNode);
        	if (sequence.size() == 0) newPath = routePlanning.roadDistance(originNode, destinationNode, null, 
        			regionalRouting, barriersRouting, barrierID, "dijkstra").edges;
        	else newPath = routePlanning.roadDistanceLocalLandmarks(originNode, destinationNode, sequence);
        }
        
        else if (criteria == "globalLandmarks")
        {
        	newPath = routePlanning.globalLandmarksPath(originNode,destinationNode, null, "road_distance").edges;
        }
        else if (criteria == "roadDistanceBarriers" || criteria == "angularChangeBarriers") //regional approaches
    	{
        	newPath = routePlanning.barriersPath(originNode, destinationNode, criteria);
    	}	
        
        else //regional approaches
    	{
        	newPath = routePlanning.regionsBarriersPath(originNode,destinationNode, criteria);
    	}	
             
    	RouteData route = new RouteData();
    	route.origin = originNode.getID();
    	route.destination = destinationNode.getID();
    	route.criteria = criteria;
    	List<Integer> sequenceEdges = new ArrayList<Integer>();

    	for (GeomPlanarGraphDirectedEdge o : newPath)
        {
        	// update edge data
        	updateEdgeData((EdgeGraph) o.getEdge());
        	int edgeID = ((EdgeGraph) o.getEdge()).getID();
        	sequenceEdges.add(edgeID);
        }

    	route.sequenceEdges = sequenceEdges; 
    	PedestrianSimulation.routesData.add(route);
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
    
    public void step(SimState state)
    {
    	PedestrianSimulation stateSchedule = (PedestrianSimulation) state;
        // check that we've been placed on an Edge  //check that we haven't already reached our destination
        if (reachedDestination || destinationNode == null)
        {
        	if (reachedDestination)	reachedDestination = false;
        	if (numTrips == OD.size()) 
        	{
        		stateSchedule.agentList.remove(this);
            	if (stateSchedule.agentList.size() == 0) 
            		{
	            		System.out.println("calling finish");
	            		stateSchedule.finish();
            		}
        		killAgent.stop();
            	return;
        	}
//        	repositionAgent();
        	originNode = (NodeGraph) OD.get(numTrips).getValue(0);	
        	updatePosition(originNode.getCoordinate());
        	destinationNode = (NodeGraph) OD.get(numTrips).getValue(1);		
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
        PedestrianSimulation.agents.setGeometryLocation(agentLocation, pointMoveTo);
    }
     
    void updateEdgeData(EdgeGraph edge)
    {

		if (criteria == "roadDistance") edge.roadDistance += 1;
		else if (criteria == "angularChange") edge.angularChange += 1;
		else if (criteria == "topological") edge.topological += 1;
		else if (criteria == "roadDistanceLandmarks") edge.roadDistanceLandmarks += 1;
		else if (criteria == "angularChangeLandmarks") edge.angularChangeLandmarks += 1;
		else if (criteria == "localLandmarks") edge.localLandmarks += 1;
		else if (criteria == "globalLandmarks") edge.globalLandmarks += 1;
		else if (criteria == "roadDistanceRegions") edge.roadDistanceRegions += 1;
		else if (criteria == "angularChangeRegions") edge.angularChangeRegions += 1;
		else if (criteria == "roadDistanceBarriers") edge.roadDistanceBarriers += 1;
		else if (criteria == "angularChangeBarriers") edge.angularChangeBarriers += 1;
		else if (criteria == "roadDistanceRegionsBarriers") edge.roadDistanceRegionsBarriers += 1;
		else if (criteria == "angularChangeRegionsBarriers") edge.angularChangeRegionsBarriers += 1;
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
