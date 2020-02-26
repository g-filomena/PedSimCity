package sim.app.geo.pedestrianSimulation;
import com.vividsolutions.jts.geom.*;
import com.vividsolutions.jts.linearref.LengthIndexedLine;
import com.vividsolutions.jts.planargraph.Node;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import org.javatuples.Pair;

import sim.engine.SimState;
import sim.engine.Steppable;
import sim.engine.Stoppable;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.GeomPlanarGraphEdge;
import sim.util.geo.MasonGeometry;
import sim.util.geo.PointMoveTo;

public final class Pedestrian implements Steppable
{

    private static final long serialVersionUID = -1113018274619047013L;
    PedestrianSimulation state;
   
    // Initial Attributes
    Node originNode = null;
    Node destinationNode = null;
    public String criteria;
    public Integer maxDistance;
    
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
    GeomPlanarGraphEdge currentEdge = null;
    GeomPlanarGraphEdge destinationEdge;
    int linkDirection = 1;
    
    double speed = 0; // useful for graph
    ArrayList<GeomPlanarGraphDirectedEdge> path =  new ArrayList<GeomPlanarGraphDirectedEdge>();
	ArrayList<Pair<Node, Node>> OD =  new ArrayList<Pair<Node, Node>>();
    int indexOnPath = 0;
    int pathDirection = 1;
    boolean reachedDestination = false;
    boolean landmarkRouting = false;
    boolean controlMode = false;
    boolean gl = false;
    
	HashMap<Integer, CentroidData> centroidsMap;
	HashMap<Integer, EdgeData> edgesMap;
	HashMap<Integer, NodeData> nodesMap;
    int numTrips = 0;

    ArrayList<GeomPlanarGraphDirectedEdge> oldPath;
    Stoppable killAgent;
   
    
    /** Constructor Function */
    public Pedestrian(PedestrianSimulation state, String criteria, ArrayList<Pair<Node, Node>> OD)
    {
    	this.criteria = criteria;
    	this.OD = OD;
    	this.state = state;
//    	this.maxDistance = maxDistance;
    	centroidsMap = state.centroidsMap;
    	edgesMap = state.edgesMap;
    	nodesMap = state.nodesMap;
    	
    	if (criteria == "roadDistanceLandmark" || criteria == "angularChangeLandmark" || 
    	criteria == "landmark_G" || criteria == "landmark_L") landmarkRouting = true;
    	     
        Node originNode = (Node) OD.get(numTrips).getValue(0);
        GeometryFactory fact = new GeometryFactory();
        agentLocation = new MasonGeometry(fact.createPoint(new Coordinate(10, 10)));
        Coordinate startCoord = null;
        startCoord = originNode.getCoordinate();
        updatePosition(startCoord);   
    }


    /** Initialization of an Agent: find an A* path to work!
     * @param state
     * @return whether or not the agent successfully found a path to work
     */

    /** Plots a path between the Agent's home Node and its work Node */
    
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

    	ArrayList<GeomPlanarGraphDirectedEdge> newPath = null;
        ArrayList<Node> sequence = new ArrayList<Node>();
        
         
        if (criteria == "roadDistance")
        {
        	newPath = routePlanning.routeDistanceShortestPath(originNode, destinationNode, null, state, "dijkstra").edges;
        }
        
        else if (criteria == "angularChange")
        {
        	newPath = routePlanning.angularChangeShortestPath(originNode, destinationNode, null, state, "dijkstra").edges;
        }
        
        else if (criteria == "topological")
        {
        	newPath = routePlanning.topologicalShortestPath(originNode, destinationNode, null, state).edges;
        }

        else if (criteria == "roadDistanceLandmark")
        {
        	sequence = routePlanning.findSequenceIntermediateNodes(originNode, destinationNode, state);
        	if (sequence.size() == 0) newPath = routePlanning.routeDistanceShortestPath(originNode, destinationNode, null, state, "astar").edges;
        	else newPath = routePlanning.RoadDistanceLandmarksPath(originNode,destinationNode, sequence, state);
        }
        
        else if (criteria == "angularChangeLandmark")
        {
        	sequence = routePlanning.findSequenceIntermediateNodes(originNode, destinationNode, state);
        	if (sequence.size() == 0) newPath = routePlanning.angularChangeShortestPath(originNode, destinationNode, null, state, "dijkstra").edges;
        	else newPath = routePlanning.AngularChangeLandmarksPath(originNode,destinationNode, sequence, state);
        }
        
        else if (criteria == "localLandmarks")
        {
        	sequence = routePlanning.findSequenceIntermediateNodes(originNode, destinationNode, state);
        	if (sequence.size() == 0) newPath = routePlanning.routeDistanceShortestPath(originNode, destinationNode, null, state, "dijkstra").edges;
        	else newPath = routePlanning.RoadDistanceLocalLandmarksPath(originNode,destinationNode, sequence, state);
        }
        
        else if (criteria == "globalLandmarks")
        {
        	newPath = routePlanning.globalLandmarksPath(originNode,destinationNode, null, "road_distance", state).edges;
        }
             
    	RouteData route = new RouteData();
    	route.origin = (int) originNode.getData();
    	route.destination = (int) destinationNode.getData();
    	route.criteria = criteria;
    	List<Integer> sequenceEdges = new ArrayList<Integer>();

    	for (GeomPlanarGraphDirectedEdge o  : newPath)
        {
        	// update edge data
        	updateEdgeData((GeomPlanarGraphEdge) o.getEdge());
        	int edge = ((GeomPlanarGraphEdge) o.getEdge()).getIntegerAttribute("edgeID");
        	sequenceEdges.add(edge);
        }

    	route.sequenceEdges = sequenceEdges; 
    	state.routesData.add(route);
    	indexOnPath = 0;
    	path = newPath;

        // set up how to traverse this first link
        GeomPlanarGraphEdge firstEdge = (GeomPlanarGraphEdge) newPath.get(0).getEdge();
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
        	originNode = (Node) OD.get(numTrips).getValue(0);	
        	updatePosition(originNode.getCoordinate());
        	destinationNode = (Node) OD.get(numTrips).getValue(1);		
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
        GeomPlanarGraphEdge edge = (GeomPlanarGraphEdge) path.get(indexOnPath).getEdge();
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
    void setupEdge(GeomPlanarGraphEdge edge)
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
        state.agents.setGeometryLocation(agentLocation, pointMoveTo);
    }
     
    void updateEdgeData(GeomPlanarGraphEdge edge)
    {
		EdgeData ed = state.edgesMap.get(edge.getIntegerAttribute("edgeID"));
		if (criteria == "roadDistance") ed.roadDistance += 1;
		else if (criteria == "angularChange") ed.angularChange += 1;
		else if (criteria == "topological") ed.topological += 1;
		else if (criteria == "roadDistanceLandmark") ed.roadDistanceLandmark += 1;
		else if (criteria == "angularChangeLandmark") ed.angularChangeLandmark += 1;
		else if (criteria == "localLandmarks") ed.localLandmarks += 1;
		else if (criteria == "globalLandmarks") ed.globalLandmarks += 1;
    }
    
    public void setStoppable(Stoppable a) {killAgent = a;}

    
    public void repositionAgent() 
    {
        Node new_start = nodesLookup.searchRandomNode(PedestrianSimulation.nodesGeometries, state);
        Coordinate startCoord = new_start.getCoordinate();
        updatePosition(startCoord); 
    }

    /** return geometry representing agent location */
    public MasonGeometry getGeometry() {return agentLocation;}  
    
    
}
