package sim.app.geo.pedestrianSimulation;
import com.vividsolutions.jts.geom.*;
import com.vividsolutions.jts.linearref.LengthIndexedLine;
import com.vividsolutions.jts.planargraph.Node;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

import org.javatuples.Pair;
import sim.engine.SimState;
import sim.engine.Steppable;
import sim.engine.Stoppable;
import sim.field.geo.GeomVectorField;
import sim.util.geo.GeomPlanarGraph;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.GeomPlanarGraphEdge;
import sim.util.geo.MasonGeometry;
import sim.util.geo.PointMoveTo;

@SuppressWarnings("restriction")


public final class Pedestrian implements Steppable
{

    private static final long serialVersionUID = -1113018274619047013L;
    pedestrianSimulation state;
   
    // Initial Attributes
    Node originNode = null;
    Node destinationNode = null;
	Node dualDestination = null;
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
    ArrayList<Pair> OD =  new ArrayList<Pair>();
    int indexOnPath = 0;
    int pathDirection = 1;
    boolean reachedDestination = false;
    boolean landmarkRouting = false;
    boolean controlMode = false;
    
	HashMap<Integer, centroidData> centroidsMap;
	HashMap<Integer, edgeData> edgesMap;
	HashMap<Integer, nodeData> nodesMap;
    int numTrips = 0;
    double wL = 0.5;
    double wG = 0.5;
    ArrayList<GeomPlanarGraphDirectedEdge> oldPath;
    Stoppable killAgent;
   
    
    /** Constructor Function */
    public Pedestrian(pedestrianSimulation state, String criteria, ArrayList<Pair> OD)
    {
        this.state = state;
    	this.criteria = criteria;
    	this.OD = OD;
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
    
    @SuppressWarnings("unchecked")
	public void findNewAStarPath(pedestrianSimulation state)
    {
//    	Node currentJunction;
//    	if (state.dynamicRouting == true)
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
        ArrayList<Node> sequence =  new ArrayList<Node>();   
        
        if (landmarkRouting == true)
    	{
	        
	        nodeData dd = nodesMap.get((int) destinationNode.getData());
//	        List<Integer> anchors = new ArrayList<Integer>();
//	        anchors = dd.anchors;
//	        if (anchors == null) 
//	    	{
//	        	wG = 0.0;
//	        	wL = 1.0;
//	    	}
	    	
	    	GeomVectorField knownJunctions = landmarkFunctions.relevantNodes(originNode, destinationNode, state);
	        double wayfindingComplexity = landmarkFunctions.easinessNavigation(originNode, destinationNode, state);
	        double searchRange = utilities.nodesDistance(originNode, destinationNode) * (wayfindingComplexity);
	    	double destinationAngle = utilities.angle(originNode.getCoordinate(), destinationNode.getCoordinate());
	        Node currentNode = originNode;

		    while (searchRange >  state.t)
	        {
//		    	System.out.println("search range  "+ searchRange + " Initial Distance  " + utilities.nodesDistance(originNode, destinationNode) + "  current distance " +
//		    			utilities.nodesDistance(currentNode, destinationNode));
	        	Node bestNode = null;
	        	double attractivness = 0.0;
	        	if (knownJunctions.getGeometries().size() == 0)
	        	{
		        	System.out.println(" attention " + knownJunctions.getGeometries().size());
		        	System.out.println("distance ... "+ utilities.nodesDistance(originNode, destinationNode));
	        	}
//	        	System.out.println("number of junctions " + knownJunctions.getGeometries().size());
	        	
	        	for (Object o : knownJunctions.getGeometries())
		        {
	        		
			    	MasonGeometry geoNode = (MasonGeometry) o;
			    	int nodeID = state.nodesGeometryMap.get(geoNode);
			    	Node tmpNode = state.nodesMap.get(nodeID).node;
			    	if (tmpNode == originNode) continue;
			    	if (Node.getEdgesBetween(tmpNode, currentNode).size() > 0) continue;
			    	if (utilities.nodesDistance(currentNode, tmpNode) > searchRange) continue; //only nodes in range	
	        		double localScore = 0.0;
	        		
	        		if (criteria == "angularLand") 
	        		{
	        			Node dualTmpDestination = utilities.getDualNode(tmpNode, state.dualNetwork);
	        			Node dualOrigin = utilities.getDualNode(currentNode, state.dualNetwork);
	        			AStarAngularNode nodesFinder = new AStarAngularNode();
	        			

	        			
	        			HashMap<Node, dualNodeWrapper> nodesAhead = nodesFinder.astarPathNodes(dualOrigin, dualTmpDestination, state);
	        			localScore = landmarkFunctions.localLandmarknessDualGraph(tmpNode, dualTmpDestination, nodesAhead, state);
	        		}
			        else
			        {
			        	AStarEuclideanNode nodesFinder = new AStarEuclideanNode();   
			        	HashMap<Node, nodeWrapper> nodesAhead = nodesFinder.astarPathNodes(currentNode, tmpNode, state);
			        	localScore = landmarkFunctions.localLandmarkness(tmpNode, nodesAhead, state);
			        }
//			        System.out.println("localScore "+localScore);
//			    	double globalScore = landmarkFunctions.globalLandmarkness(tmpNode, destinationNode, state); 
			    	double landmarkness = localScore;
			    	double targetAngle = utilities.angle(currentNode.getCoordinate(), tmpNode.getCoordinate());
			    	
//			    	System.out.println("difference angle "+ utilities.angleDiff(targetAngle, destinationAngle));
//			    	double deviation = 1 - utilities.angleDiff(targetAngle, destinationAngle)/180;
//			    	double distance = 1 - utilities.nodesDistance(currentNode, tmpNode)/range;
			    	double gain = ((utilities.nodesDistance(currentNode, destinationNode) - 
			    			utilities.nodesDistance(tmpNode, destinationNode))/utilities.nodesDistance(currentNode, destinationNode));
			    	
//			    	double gain = (utilities.nodesDistance(originNode, destinationNode)-utilities.nodesDistance(tmpNode, destinationNode))/
//			    			utilities.nodesDistance(originNode, destinationNode);
			    	
//			    	System.out.println("first distance" + utilities.nodesDistance(currentNode, destinationNode) + " good one  "+ utilities.nodesDistance(tmpNode, destinationNode));
//			    	System.out.println("difference angle "+ utilities.angleDiff(targetAngle, destinationAngle));
//			    	System.out.println("deviation" + deviation + " gain  "+  gain);
//			    	double cognitiveCost = (deviation + gain)/2;
			    	
			    	
			    	if (landmarkness * gain > attractivness) 
	    			{
//			    		System.out.println("node "+ nodeID+ " landmarknes: "+ landmarkness + " distance destination: "+ distanceDestination);
			    		attractivness = landmarkness * gain;
			    		bestNode = tmpNode;
	    			}
			    	
		        }
	        	if (bestNode == null) break;
	        	if (bestNode == destinationNode) break;
	        	
	        	sequence.add(bestNode);
	        	knownJunctions = landmarkFunctions.relevantNodes(bestNode, destinationNode, state);
	        	wayfindingComplexity = landmarkFunctions.easinessNavigation(bestNode, destinationNode, state);
	        	searchRange = utilities.nodesDistance(bestNode, destinationNode) * wayfindingComplexity;
	            currentNode = bestNode;

	        }
//		    System.out.println(" distance " + utilities.nodesDistance(originNode, destinationNode)+ " len seq "+sequence.size());
		    System.out.println(" OD "+(int) originNode.getData()+ " "+ (int) destinationNode.getData()+" "+ criteria +
		    		" len seq "+sequence.size() + " distance "+  utilities.nodesDistance(originNode, destinationNode) );
    	}
        
        if (criteria == "euclidean" || criteria  == "euclideanLand" )
        {
//        	System.out.println("sequence"+ sequence);
        	if (criteria == "euclidean" || sequence.size() == 0)
        	{
//            	DijkstraEuclidean pathfinderEuclidean = new DijkstraEuclidean();
        		AStarEuclidean pathfinderEuclidean = new AStarEuclidean();
                newPath = pathfinderEuclidean.astarPath(originNode, destinationNode, state, null);      
        	}
        	else
            {   
       		
        		AStarEuclidean pathfinderEuclidean = new AStarEuclidean();
            	Node tmpNode = originNode;
            	ArrayList<GeomPlanarGraphDirectedEdge> tmpPath = null;
            	List segmentsToAvoid = new ArrayList<GeomPlanarGraphDirectedEdge>();
            	
            	
            	for (int i = 0; i < sequence.size(); i++)
            	{
            		if (i == 0) 
            		{
            			newPath = pathfinderEuclidean.astarPath(tmpNode, sequence.get(i), state, null);
            			Iterator it = newPath.iterator();
            			while (it.hasNext()) segmentsToAvoid.add((int) ((GeomPlanarGraphEdge) 
            					((GeomPlanarGraphDirectedEdge) it.next()).getEdge()).getIntegerAttribute("streetID"));
            		}
            		else
            		{
            			tmpPath = pathfinderEuclidean.astarPath(tmpNode, sequence.get(i), state, segmentsToAvoid); 
            			Iterator it = tmpPath.iterator();
            			while (it.hasNext()) segmentsToAvoid.add((int) ((GeomPlanarGraphEdge) 
            					((GeomPlanarGraphDirectedEdge) it.next()).getEdge()).getIntegerAttribute("streetID"));
            			newPath.addAll(tmpPath); 
            		}
                	tmpNode = sequence.get(i);
            	}
            	tmpPath = pathfinderEuclidean.astarPath(tmpNode, destinationNode, state, segmentsToAvoid);  
            	newPath.addAll(tmpPath); 
            }
        }
        
        else if (criteria == "angular" || criteria == "angularLand")
        {

        	Node dualOrigin = utilities.getDualNode(originNode, state.dualNetwork);
        	dualDestination = null;
        	while (dualDestination == dualOrigin || dualDestination == null) dualDestination = utilities.getDualNode(
        			destinationNode, state.dualNetwork);

        	if (criteria == "angular" || sequence.size() == 0)
        	{
//            	DijkstraAngular pathfinderAngular = new DijkstraAngular();
//                newPath = pathfinderAngular.dijkstraPath(dualOrigin, dualDestination, state);
        		AStarAngular pathfinderAngular = new AStarAngular();
                newPath = pathfinderAngular.astarPath(dualOrigin, dualDestination, state, null);
           
        	}
        	else 
        	{
        		AStarAngular pathfinderAngular = new AStarAngular();
            	Node tmpNode = dualOrigin;
            	ArrayList<GeomPlanarGraphDirectedEdge> tmpPath = null;
            	List centroidsToAvoid = new ArrayList<GeomPlanarGraphDirectedEdge>();
            	
            	for (int i = 0; i < sequence.size(); i++)
            	{
            		
            		Node node = utilities.getDualNode(sequence.get(i), state.dualNetwork);
            		if (i == 0) 
            		{
            			newPath = pathfinderAngular.astarPath(tmpNode, node, state, centroidsToAvoid); 
            			Iterator it = newPath.iterator();
            			while (it.hasNext()) centroidsToAvoid.add((int) ((GeomPlanarGraphEdge) 
            					((GeomPlanarGraphDirectedEdge) it.next()).getEdge()).getIntegerAttribute("streetID"));
            		}
            		else
            		{
            			tmpPath = pathfinderAngular.astarPath(tmpNode, node, state, centroidsToAvoid); 
            			Iterator it = tmpPath.iterator();
            			while (it.hasNext()) centroidsToAvoid.add((int) ((GeomPlanarGraphEdge) 
            					((GeomPlanarGraphDirectedEdge) it.next()).getEdge()).getIntegerAttribute("streetID"));
            			newPath.addAll(tmpPath); 
            		}
                	tmpNode = node;
                	centroidsToAvoid.remove(node.getData());
            	}
            	tmpPath = pathfinderAngular.astarPath(tmpNode, dualDestination, state, centroidsToAvoid);  
            	newPath.addAll(tmpPath); 
        	}

        }
        
        else if (criteria == "topological")
        {
        	AStarTopological pathfinderToplogical = new AStarTopological();
        	newPath = pathfinderToplogical.astarPath(originNode, destinationNode, state);

        }
        
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
        else
        {
        	DijkstraLand pathfinderLandmarkness = new DijkstraLand();
            newPath = pathfinderLandmarkness.dijkstraLandmarkPath(originNode, destinationNode, wL, wG, state);
        }
              
        // if the path works, lay it in
        if (newPath != null && newPath.size() > 0) 
        {
        	indexOnPath = 0;
        	path = newPath;

            // set up how to traverse this first link
            GeomPlanarGraphEdge firstEdge = (GeomPlanarGraphEdge) newPath.get(0).getEdge();
            setupEdge(firstEdge); //Sets the Agent up to proceed along an Edge

            // update the current position for this link
            updatePosition(segment.extractPoint(currentIndex));

        }
        
    	// update edge data
        for (GeomPlanarGraphDirectedEdge o  : newPath) updateEdgeData((GeomPlanarGraphEdge) o.getEdge());
    	numTrips += 1;
    }

    double progress(double val)
    {
    	try 
    	{
        double edgeLength = currentEdge.getLine().getLength();
    	}
    	catch(NullPointerException i)
    	{
    		System.out.println(criteria);
    	}
    	
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
        if (reachedDestination == true || destinationNode == null)
        {
        	if (reachedDestination == true)	reachedDestination = false;
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
    
    
    public void updateStatistics(ArrayList<GeomPlanarGraphDirectedEdge> path, Node originNode, Node destinationNode)
    {
    	
    	criteriaData cd = state.criteriaMap.get(criteria);
    	double distance = utilities.finalDistance(path);
    	cd.trips += 1;
    	cd.totalDistance += distance;
    }
      
    void updateEdgeData(GeomPlanarGraphEdge edge)
    {
		edgeData ed = state.edgesMap.get(edge.getIntegerAttribute("streetID"));
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
        Node new_start = nodesLookup.searchRandomNode(state.geometriesNodes, state);
        Coordinate startCoord = new_start.getCoordinate();
        updatePosition(startCoord); 
    }

    /** return geometry representing agent location */
    public MasonGeometry getGeometry() {return agentLocation;}  
    
    
}
