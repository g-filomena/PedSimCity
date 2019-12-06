package sim.app.geo.pedestrianSimulation;
import com.vividsolutions.jts.geom.*;
import com.vividsolutions.jts.linearref.LengthIndexedLine;
import com.vividsolutions.jts.planargraph.Node;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;

import org.javatuples.Pair;

import sim.app.geo.pedestrianSimulation.utilities.Path;
import sim.engine.SimState;
import sim.engine.Steppable;
import sim.engine.Stoppable;
import sim.field.geo.GeomVectorField;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.GeomPlanarGraphEdge;
import sim.util.geo.MasonGeometry;
import sim.util.geo.PointMoveTo;

public final class Pedestrian implements Steppable
{

    private static final long serialVersionUID = -1113018274619047013L;
    pedestrianSimulation state;
   
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
    @SuppressWarnings("rawtypes")
	ArrayList<Pair> OD =  new ArrayList<Pair>();
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
    public Pedestrian(pedestrianSimulation state, String criteria, ArrayList<Pair> OD)
    {
    	this.criteria = criteria;
    	this.OD = OD;
    	this.state = state;
//    	this.maxDistance = maxDistance;
    	centroidsMap = state.centroidsMap;
    	edgesMap = state.edgesMap;
    	nodesMap = state.nodesMap;
    	
    	if (criteria == "euclideanLand" || criteria == "angularLand" || 
    			criteria == "landmark" || criteria == "landmark_G" || criteria == "landmark_L") landmarkRouting = true;
    	     
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
    
    public void findNewAStarPath(pedestrianSimulation state)
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
        if (landmarkRouting)
    	{
	    	GeomVectorField knownJunctions = landmarkFunctions.relevantNodes(originNode, destinationNode, state);
	        double wayfindingComplexity = landmarkFunctions.easinessNavigation(originNode, destinationNode, state);
	        double searchRange = utilities.nodesDistance(originNode, destinationNode) * (wayfindingComplexity);
	        Node currentNode = originNode;
//	        ArrayList<Integer> segmentsToAvoid = new ArrayList<Integer>();

		    List<Integer> badNodes = new ArrayList<Integer>();
        	while (searchRange >  state.t)
	        {

	        	Node bestNode = null;
//	        	Node bestDualNode = null;
//	        	ArrayList<GeomPlanarGraphDirectedEdge> bestPath = null;
//	        	ArrayList<GeomPlanarGraphDirectedEdge> tmpPath = null;
	        	double attractivness = 0.0;
	        		        	
	        	for (Object o : knownJunctions.getGeometries())
		        {
			    	MasonGeometry geoNode = (MasonGeometry) o;
			    	int nodeID = state.nodesGeometryMap.get(geoNode);
			    	Node tmpNode = state.nodesMap.get(nodeID).node;
			    	
			    	if (tmpNode == originNode) continue;
			    	if (Node.getEdgesBetween(tmpNode, currentNode).size() > 0) continue;
			    	if (utilities.nodesDistance(currentNode, tmpNode) > searchRange)
			    	{
			    		badNodes.add(nodeID);
			    		continue; //only nodes in range	
			    	}
	        		double localScore = 0.0;
	        		localScore = landmarkFunctions.localLandmarkness(tmpNode, false, null, state);
//	        		double globalScore = 0.0;
//	        		Node dualTmpDestination = null;

//	        		if (criteria == "angularLand") 
//	        		{
////	        			Node dualOrigin = utilities.getDualNode(currentNode, pedestrianSimulation.dualNetwork);
////	        			dualTmpDestination = utilities.getDualNode(tmpNode, pedestrianSimulation.dualNetwork);
////
////	        			AStarAngular pathfinderAngular = new AStarAngular(); 
////	        			Path path = pathfinderAngular.astarPath(dualOrigin, dualTmpDestination, state, segmentsToAvoid, gl);
////	        			tmpPath = path.edges;
////	        			HashMap<Node, DualNodeWrapper> nodesAhead = path.dualMapWrappers;
//	        			try 
//	        			{
//		        			
////		        			localScore = landmarkFunctions.localLandmarknessDualGraph(tmpNode, dualTmpDestination, state);
////		        			globalScore = landmarkFunctions.globalLandmarknessDualGraph(destinationNode, tmpNode, nodesAhead, state);
//	        			}
//	        			catch(java.lang.NullPointerException e) {continue;} //flawed path
//	        		}
//			        else
//			        {
////			        	AStarEuclidean pathfinderEuclidean = new AStarEuclidean();   
////			        	Path pathVis = pathfinderEuclidean.astarPath(currentNode, tmpNode, state, segmentsToAvoid, gl);
////			        	
////			        	DijkstraGlobalLand globalPath = new DijkstraGlobalLand();   
////			        	ArrayList<GeomPlanarGraphDirectedEdge> pathG = globalPath.dijkstraLandmarkPath(
////			        				currentNode, tmpNode, state);
////			        	
//////			        	tmpPath = pathVis.edges;
////			        	tmpPath = pathG;
////			        	HashMap<Node, NodeWrapper> nodesAhead = pathVis.mapWrappers; 
//			        	
//			        	try 
//			        	{
//				        	localScore = landmarkFunctions.localLandmarkness(tmpNode,false, null, state);
////				        	globalScore = landmarkFunctions.globalLandmarkness(destinationNode, tmpNode, nodesAhead, state);
//			        	}
//			        	catch(java.lang.NullPointerException e) {continue;} //flawed path
//			        }

			    	double gain = ((utilities.nodesDistance(currentNode, destinationNode) - 
			    			utilities.nodesDistance(tmpNode, destinationNode))/utilities.nodesDistance(currentNode, destinationNode));
			    	
			    	double landmarkness = localScore*0.60 + gain*0.40;
			    	if (landmarkness > attractivness) 
	    			{
			    		attractivness = landmarkness;
			    		bestNode = tmpNode;
//			    		bestPath = tmpPath;

//			    		if (criteria == "angularLand") bestDualNode = dualTmpDestination;
	    			}
		        }
	        		        	
	        	if ((bestNode == null) & (sequence.size() == 0))
	        	{
	        		System.out.println("here's the problem...");
	        		System.out.println(currentNode.getData()+ " "+destinationNode.getData());
	        		System.out.println(badNodes.size()+" "+ knownJunctions.getGeometries().size());
	        		System.out.println("att: "+attractivness+" wayfinding complex "+ wayfindingComplexity+ " search"+ searchRange );
	        	}

	        	if (bestNode == null) 
	        	{

//	        		// reconstruct path till the destination
//	        		if (criteria == "angularLand") 
//	        		{
//	        			Node dualOrigin = utilities.getDualNode(currentNode, pedestrianSimulation.dualNetwork);
//	        			Node dualTmpDestination = utilities.getDualNode(destinationNode, pedestrianSimulation.dualNetwork);
//	        			
//	        			AStarAngular pathfinderAngular = new AStarAngular(); 
//	        			Path path = pathfinderAngular.astarPath(dualOrigin, dualTmpDestination, state, segmentsToAvoid, gl);
//	        			tmpPath = path.edges;
//	        		}
//			        else
//			        {
//			        	AStarEuclidean pathfinderEuclidean = new AStarEuclidean();   
//			        	Path path = pathfinderEuclidean.astarPath(currentNode, destinationNode, state, segmentsToAvoid, gl);
//			        	tmpPath = path.edges;
//			        }
	        		//len sequence == 0
//		    		if (currentNode == originNode) newPath = tmpPath;
		    		//add to the rest
//		    		else newPath.addAll(tmpPath);
	        		break;
	        	}
	        	
//	    		if (currentNode == originNode) newPath = bestPath;
//	    		//add to the rest
//	    		else newPath.addAll(bestPath);
//	    		Iterator<GeomPlanarGraphDirectedEdge> it = bestPath.iterator();
//	    		while (it.hasNext()) segmentsToAvoid.add((int) ((GeomPlanarGraphEdge) 
//    					it.next().getEdge()).getIntegerAttribute("edgeID"));
	    		
//    			if (criteria == "angularLand") segmentsToAvoid.remove(bestDualNode.getData());
    			
	        	sequence.add(bestNode);
	        	knownJunctions = landmarkFunctions.relevantNodes(bestNode, destinationNode, state);
	        	wayfindingComplexity = landmarkFunctions.easinessNavigation(bestNode, destinationNode, state);
	        	searchRange = utilities.nodesDistance(bestNode, destinationNode) * wayfindingComplexity;
	            currentNode = bestNode;
	        }
	    
//		    System.out.println(" OD "+(int) originNode.getData()+ " "+ (int) destinationNode.getData()+" "+ criteria +
//		    		" len seq "+sequence.size() + " distance "+  utilities.nodesDistance(originNode, destinationNode) );

		}
        
        if ((criteria == "euclidean") || ((criteria == "euclideanLand") && (sequence.size() == 0)))
        {

    		AStarEuclidean pathfinderEuclidean = new AStarEuclidean();
            Path path = pathfinderEuclidean.astarPath(originNode, destinationNode, state, null, true);    
            newPath = path.edges;
        }
        else if (criteria == "euclideanLand")
        {
        	{   
        		DijkstraEucLand pathFinderEucLand = new DijkstraEucLand();  
        		Node tmpNode = originNode;
        		ArrayList<GeomPlanarGraphDirectedEdge> tmpPath = null;
        		            	
        		for (Node intermediateNode : sequence)
            	{
        			if (sequence.indexOf(intermediateNode) == 0) newPath = pathFinderEucLand.dijkstraPath(tmpNode, intermediateNode, state);
            		else
            		{
            			tmpPath = pathFinderEucLand.dijkstraPath(tmpNode, intermediateNode, state); 
            			newPath.addAll(tmpPath); 
            		}
                	tmpNode = intermediateNode;
            	}
            	tmpPath = pathFinderEucLand.dijkstraPath(tmpNode, destinationNode, state);  
            	newPath.addAll(tmpPath); 
            }
        	
        	
        }
        
        else if ((criteria == "angular") || ((criteria == "angularLand") && (sequence.size() == 0)))
        {
        	Node dualOrigin = utilities.getDualNode(originNode, pedestrianSimulation.dualNetwork);
        	Node dualDestination = null;
        	while (dualDestination == dualOrigin || dualDestination == null) dualDestination = utilities.getDualNode(
        			destinationNode, pedestrianSimulation.dualNetwork);

    		AStarAngular pathfinderAngular = new AStarAngular();
            Path path = pathfinderAngular.astarPath(dualOrigin, dualDestination, state, null, true);
            newPath = path.edges;
        }
        
        else if (criteria == "angularLand")
        {
        	{
        		Node dualOrigin = utilities.getDualNode(originNode, pedestrianSimulation.dualNetwork);
        		Node dualDestination = utilities.getDualNode(destinationNode, pedestrianSimulation.dualNetwork);
        		
        		DijkstraAngLand pathFinderAngLand = new DijkstraAngLand();  
        		Node tmpNode = dualOrigin;
        		ArrayList<GeomPlanarGraphDirectedEdge> tmpPath = null;
//            	List centroidsToAvoid = new ArrayList<GeomPlanarGraphDirectedEdge>();

            	for (Node intermediateNode : sequence)
            	{
//            		System.out.println("originNode" + originNode.getData()+ "interm"+ intermediateNode.getData()+ " destn "+ destinationNode.getData());
            		Node node = utilities.getDualNode(intermediateNode, pedestrianSimulation.dualNetwork);
            		if (sequence.indexOf(intermediateNode) == 0) 
            		{
            			newPath = pathFinderAngLand.dijkstraPath(tmpNode, node, destinationNode, state); 
             		}
            		else
            		{
            			tmpPath = pathFinderAngLand.dijkstraPath(tmpNode, node, destinationNode, state);  
            			newPath.addAll(tmpPath); 
            		}
                	tmpNode = node;
//                	centroidsToAvoid.remove(node.getData());
            	}
            	tmpPath = pathFinderAngLand.dijkstraPath(tmpNode, dualDestination, destinationNode, state);  
            	newPath.addAll(tmpPath); 
        	}
        }
        

//        else if (criteria == "topological")
//        {
//        	AStarTopological pathfinderToplogical = new AStarTopological();
//        	newPath = pathfinderToplogical.astarPath(originNode, destinationNode, state);
//
//        }
        
//        else if (criteria == "landmarkL")
//        {
//        	DijkstraLand pathfinderLandmarkness = new DijkstraLand();
//            newPath = pathfinderLandmarkness.dijkstraLandmarkPath(originNode, destinationNode, 1.0, 0.0, state);
//
//        }
//        else if (criteria == "landmarkG")
//        {
//        	DijkstraLand pathfinderLandmarkness = new DijkstraLand();
//            newPath = pathfinderLandmarkness.dijkstraLandmarkPath(originNode, destinationNode, 0.0, 1.0, state);
//
//        }
//        else
//        {
//        	DijkstraLand pathfinderLandmarkness = new DijkstraLand();
//            newPath = pathfinderLandmarkness.dijkstraLandmarkPath(originNode, destinationNode, 1, 1, state);
//        }
              
        // if the path works, lay it in
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
    	pedestrianSimulation stateSchedule = (pedestrianSimulation) state;
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
    
    
//    public void updateStatistics(ArrayList<GeomPlanarGraphDirectedEdge> path, Node originNode, Node destinationNode)
//    {
//    	
//    	CriteriaData cd = state.criteriaMap.get(criteria);
//    	double distance = utilities.finalDistance(path);
//    	cd.trips += 1;
//    	cd.totalDistance += distance;
//    }
      
    void updateEdgeData(GeomPlanarGraphEdge edge)
    {
		EdgeData ed = state.edgesMap.get(edge.getIntegerAttribute("edgeID"));
		if (criteria == "euclidean") ed.euclidean += 1;
		else if (criteria == "angular") ed.angular += 1;
		else if (criteria == "topological") ed.topological += 1;
		else if (criteria == "euclideanLand") ed.euclideanLand += 1;
		else if (criteria == "angularLand") ed.angularLand += 1;
//		else if (criteria == "landmark") ed.landmark += 1;
    }
    
    public void setStoppable(Stoppable a) {killAgent = a;}

    
    public void repositionAgent() 
    {
        Node new_start = nodesLookup.searchRandomNode(pedestrianSimulation.nodesGeometries, state);
        Coordinate startCoord = new_start.getCoordinate();
        updatePosition(startCoord); 
    }

    /** return geometry representing agent location */
    public MasonGeometry getGeometry() {return agentLocation;}  
    
    
}
