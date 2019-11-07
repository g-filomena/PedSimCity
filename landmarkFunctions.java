package sim.app.geo.pedestrianSimulation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.planargraph.Node;
import sim.field.geo.GeomVectorField;
import sim.util.Bag;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.GeomPlanarGraphEdge;
import sim.util.geo.MasonGeometry;


public class landmarkFunctions{	
	

	
    
    static double localLandmarkness(Node targetNode, HashMap<Node, nodeWrapper> mapWrappers, pedestrianSimulation state)
    {   	
        Integer nodeID = (Integer) targetNode.getData();
        nodeData nd = state.nodesMap.get(nodeID);
        List<Integer> localLandmarks = new ArrayList<Integer>();
        localLandmarks = nd.localLandmarks;
        nodeWrapper previous = mapWrappers.get(mapWrappers.get(targetNode).nodeFrom);
        double localScore = 0.0;
        if (localLandmarks == null) return 0.0;
        else
        {
        	for (int i = 0; i < localLandmarks.size(); i++)
	        {
	            Node nodeTo =  targetNode;
	            Node nodeFrom = null;
	            try {nodeFrom = previous.node;}
	            catch (java.lang.NullPointerException e) {System.out.println("problem  "+ targetNode.getData());}

	            double distanceTravelled = 0;
	            double cumulativeAdvanceVis = 0;
	            while ((nodeFrom != null) & (distanceTravelled <= state.t))
	            {
	            	Integer nodeIDLoop = (Integer) nodeFrom.getData();
	                nodeData ndLoop = state.nodesMap.get(nodeIDLoop);
	                List<Integer> visible = new ArrayList<Integer>();
	                visible = ndLoop.visible2d;
                	nodeWrapper nt = mapWrappers.get(nodeTo);
                	GeomPlanarGraphEdge segment = (GeomPlanarGraphEdge) nt.edgeFrom.getEdge();

	                distanceTravelled += segment.getDoubleAttribute("length");	                
	                if (visible.contains(localLandmarks.get(i))) cumulativeAdvanceVis += segment.getDoubleAttribute("length");

	                nodeTo = nodeFrom;
	                nodeWrapper nf = mapWrappers.get(nodeFrom);
	                try {nodeFrom = nf.nodeFrom;}
	                catch (java.lang.NullPointerException e) {nodeFrom = null;}
	            }
	              
	          double aV = cumulativeAdvanceVis/distanceTravelled;
	          if (aV > 1.0) aV = 1.0;
	          double tmp = nd.localScores.get(i) * aV;
	          if (tmp > localScore) 
	          {
//	              System.out.println("best so far "+ localLandmarks.get(i) +  "  visibility  " + cumulativeAdvanceVis);
	        	  localScore = tmp;
	        	  
	          }
	        }
        	return localScore;
        }
    }
        
        
    static double localLandmarknessDualGraph(Node targetNode, Node targetCentroid, HashMap<Node, dualNodeWrapper> mapWrappers, pedestrianSimulation state)
    {
	    	
    	// current real segment
    	Integer streetID = (Integer) targetCentroid.getData();
    	GeomPlanarGraphDirectedEdge streetSegment = (GeomPlanarGraphDirectedEdge) state.edgesMap.get(streetID).planarEdge.getDirEdge(0); 
    	
    	//previous segment
    	Node previousCentroid = mapWrappers.get(targetCentroid).nodeFrom;
    	Integer streetIDp = (Integer) previousCentroid.getData();   	
    	GeomPlanarGraphDirectedEdge edgeDirP = (GeomPlanarGraphDirectedEdge) state.edgesMap.get(streetIDp).planarEdge.getDirEdge(0); 
    	Node nToP = edgeDirP.getToNode();
    	Node nFromP = edgeDirP.getFromNode();
    	
    	// as the direction is not known  
    	if (targetNode == nToP || targetNode == nFromP) targetNode = streetSegment.getFromNode();
    	
    	//real node
    	Integer nodeID = (Integer) targetNode.getData();
        nodeData nd = state.nodesMap.get(nodeID);	
    	
        List<Integer> localLandmarks = new ArrayList<Integer>();
        double localScore = 0.0;
        localLandmarks = nd.localLandmarks;
        if (localLandmarks == null) return 0.0;
        else
        {
        	for (int i = 0; i < localLandmarks.size(); i++)
	        {
	            Node nodeTo =  targetNode;
	            Node centroidFrom = previousCentroid;
	            Node nodeFrom;
	            double distanceTravelled = 0;
	            double cumulativeAdvanceVis = 0;
	            GeomPlanarGraphDirectedEdge currentSegment = streetSegment;
	            
	            while ((centroidFrom != null) & (distanceTravelled <= state.t))
	            {
	            	if (currentSegment.getFromNode() == nodeTo) nodeFrom = currentSegment.getToNode();
	            	else nodeFrom = currentSegment.getToNode();
	            	
	            	Integer nodeIDLoop = (Integer) nodeFrom.getData();
	                nodeData ndLoop = state.nodesMap.get(nodeIDLoop);
	                List<Integer> visible = new ArrayList<Integer>();
	                visible = ndLoop.visible2d;

	                GeomPlanarGraphEdge segment = (GeomPlanarGraphEdge) currentSegment.getEdge();
	                distanceTravelled = distanceTravelled + segment.getLine().getLength();
	                       
	                if (visible.contains(localLandmarks.get(i))) cumulativeAdvanceVis += segment.getDoubleAttribute("length");
	                nodeTo = nodeFrom;
	                Integer streetIDLoop = (Integer) centroidFrom.getData();
	            	currentSegment = (GeomPlanarGraphDirectedEdge) state.edgesMap.get(streetIDLoop).planarEdge.getDirEdge(0);
	                dualNodeWrapper cf = mapWrappers.get(centroidFrom);
	                try {centroidFrom = cf.nodeFrom;}
	                catch (java.lang.NullPointerException e) {centroidFrom = null;}
	            } 
	            
	          double aV = cumulativeAdvanceVis/distanceTravelled;
	          if (aV > 1.0) aV = 1.0;
	          double tmp = nd.localScores.get(i) * aV;
	          if (tmp > localScore) localScore = tmp;
	        }
        	
        	return localScore;
        }
    }
                        
//        static double globalLandmarknessDualGraph(Node targetCentroid, Node destination, pedestrianSimulation state)
//        {   	
//        	// current real segment: identifying node
//        	Integer streetID = (Integer) targetCentroid.getData();
//        	GeomPlanarGraphDirectedEdge streetSegment = (GeomPlanarGraphDirectedEdge) state.edgesMap.get(streetID).planarEdge.getDirEdge(0); 
//        	Node targetNode = streetSegment.getToNode(); // targetNode            
//            	    	
//            Integer nodeID = (Integer) targetNode.getData();
//            nodeData nd = state.nodesMap.get(nodeID);
//            List<Integer> distantLandmarks = new ArrayList<Integer>();
//            	
//            // destination segment: identifying node
//        	Node destNode = destination;       
//            Integer nodeIDdest = (Integer) destNode.getData();
//        	nodeData dd = state.nodesMap.get(nodeIDdest);
//        	List<Integer> anchors = new ArrayList<Integer>();
//        	anchors = dd.anchors;
//        	if (anchors == null) return 0.0;
//        	    
//        	double globalScore = 0.0;
//        	for (int i = 0; i < distantLandmarks.size(); i++)
//        	{
//        		double tmp = 0.0;
//        		if (anchors.contains(distantLandmarks.get(i)))
//        		{
//    				tmp = nd.distantScores.get(i);
//    				double distance = dd.distances.get(anchors.lastIndexOf(distantLandmarks.get(i)));
//    				double distanceWeight = utilities.nodesDistance(targetCentroid, destination)/distance;
//    				if (distanceWeight > 1.0) distanceWeight = 1.0;
//    				tmp = tmp*distanceWeight;
//    			}
//    			if (tmp > globalScore) globalScore = tmp;
//    		}
//    	    return globalScore;
//        }    
        
        
        public static double easinessNavigation(Node originNode, Node destinationNode, pedestrianSimulation state)
        {
            Geometry smallestCircle = utilities.smallestEnclosingCircle(originNode, destinationNode);
            double distanceComplexity = utilities.nodesDistance(originNode, destinationNode)/4000;
            Bag filterBuildings = state.buildings.getContainedObjects(smallestCircle);
            HashMap<MasonGeometry, Double> buildingsMap =  utilities.filterMap(state.buildingsLS, filterBuildings);
            
            int count = 0;
            Iterator it = buildingsMap.entrySet().iterator();
            while (it.hasNext()) 
            {
            	Map.Entry<MasonGeometry, Double> entry = (Map.Entry<MasonGeometry, Double>)it.next();
            	if (entry.getValue() > 0.30) count += 1;
            }
            
            double buildingsComplexity = 0.0;
            if (filterBuildings.size() == 0) buildingsComplexity = 0.0;
            else buildingsComplexity =  (filterBuildings.size() - count) /  filterBuildings.size();
            double environmentalComplexity = (distanceComplexity + buildingsComplexity)/2;
            double easiness = 1 - environmentalComplexity;
            return easiness;
        }
	
    	public static GeomVectorField relevantNodes(Node originNode, Node destinationNode, pedestrianSimulation state)
    	{
    	    Geometry smallestCircle = utilities.smallestEnclosingCircle(originNode, destinationNode);
    	    Bag filter = state.junctions.getContainedObjects(smallestCircle);      	    
    	    HashMap<MasonGeometry, Double> nodesMap =  utilities.filterMap(state.nodesBc600, filter);

    	    Map<MasonGeometry, Double> orderedMap = utilities.sortByValue(nodesMap);
    	    // quantile
    	    int percentile = (int) (orderedMap.size()*0.75);
    	    int count = 0;
    	    double boundary = 0.0;
    	    
    	    Iterator it = orderedMap.entrySet().iterator();
            while (it.hasNext()) 
            {
            	count += 1;
            	Map.Entry<MasonGeometry, Double> entry = (Entry<MasonGeometry, Double>) it.next();
            	
            	if (count > percentile) 
            		{
    	        		boundary  = orderedMap.get(entry.getKey());
    	        		break;
            		}
            }
    	    
    	    GeomVectorField knownJunctions = new GeomVectorField();
    	    for (Object o : filter)
    	    {    
    	    	MasonGeometry geoNode = (MasonGeometry) o;
    	    	if (orderedMap.get(geoNode) < boundary) continue;
    		   	knownJunctions.addGeometry(geoNode);
    	    }
    	    return knownJunctions;
    	    		
    	}
    	
    	
        static double globalLandmarkness(Node targetNode, Node tmpNode, HashMap<Node, nodeWrapper> mapWrappers, pedestrianSimulation state)
        {   	
            Integer destinationID = (Integer) targetNode.getData();
            nodeData dd = state.nodesMap.get(destinationID);
            List<Integer> anchors = new ArrayList<Integer>();
            anchors = dd.anchors;

            double globalScore = 0.0;
            Iterator it = mapWrappers.entrySet().iterator();
            while (it.hasNext()) 
            {
                Map.Entry pair = (Map.Entry)it.next();
                Node node = (Node) pair.getKey();
                Integer nodeID = (Integer) node.getData();
                nodeData nd = state.nodesMap.get(nodeID);
                List<Integer> distantLandmarks = new ArrayList<Integer>();
                distantLandmarks = nd.distantLandmarks;
                if (distantLandmarks == null) continue;
                else
                {
                    for (int i = 0; i < distantLandmarks.size(); i++)
                    {
                 	   double tmp = 0.0;
                 	   if (anchors.contains(distantLandmarks.get(i)))
                 	   {
                 		   tmp = nd.distantScores.get(i);
                 		   double distanceLandmark = dd.distances.get(anchors.lastIndexOf(distantLandmarks.get(i)));
                 		   double distanceWeight = utilities.nodesDistance(tmpNode, targetNode)/distanceLandmark;
                 		   if (distanceWeight > 1.0) distanceWeight = 1.0;
                 		   tmp = tmp*distanceWeight;   
                 	   }
                 	  if (tmp > globalScore) globalScore = tmp;
                    }
                }
            }
            return globalScore; 
        }
            
            
            
        static double globalLandmarknessDualGraph(Node targetNode, Node tmpNode, HashMap<Node, dualNodeWrapper> mapWrappers, pedestrianSimulation state)
        {   
            Integer destinationID = (Integer) targetNode.getData();
            nodeData dd = state.nodesMap.get(destinationID);
            List<Integer> anchors = new ArrayList<Integer>();
            anchors = dd.anchors;
        	
            double globalScore = 0.0;
            Iterator it = mapWrappers.entrySet().iterator();
            Node previous = targetNode;
            Node throughNode;
            while (it.hasNext()) 
            {
                Map.Entry pair = (Map.Entry)it.next();
                Node centroid = (Node) pair.getKey();
                Integer streetIDp = (Integer) centroid.getData();
            	GeomPlanarGraphDirectedEdge edgeDirP = (GeomPlanarGraphDirectedEdge) state.edgesMap.get(streetIDp).planarEdge.getDirEdge(0); 
            	Node nToP = edgeDirP.getToNode();
            	Node nFromP = edgeDirP.getFromNode();
            	if (previous == nToP)  throughNode = nFromP;
            	else throughNode = nToP;

                Integer nodeID = (Integer) throughNode.getData();
                nodeData nd = state.nodesMap.get(nodeID);
                List<Integer> distantLandmarks = new ArrayList<Integer>();
                distantLandmarks = nd.distantLandmarks;
                if (distantLandmarks == null) continue;
                else
                {
                    for (int i = 0; i < distantLandmarks.size(); i++)
                    {
                 	   double tmp = 0.0;
                 	   if (anchors.contains(distantLandmarks.get(i)))
                 	   {
                 		   tmp = nd.distantScores.get(i);
                 		   double distanceLandmark = dd.distances.get(anchors.lastIndexOf(distantLandmarks.get(i)));
                 		   double distanceWeight = utilities.nodesDistance(tmpNode, targetNode)/distanceLandmark;
                 		   if (distanceWeight > 1.0) distanceWeight = 1.0;
                 		   tmp = tmp*distanceWeight;   
                 	   }
                 	  if (tmp > globalScore) globalScore = tmp;
                    }
                }
                previous = throughNode;
            }
            return globalScore; 
        }


}
    