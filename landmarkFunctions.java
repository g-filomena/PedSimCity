package sim.app.geo.pedestrianSimulation;

import java.util.*;
import java.util.Map.Entry;

import com.vividsolutions.jts.geom.*;
import sim.field.geo.GeomVectorField;
import sim.util.Bag;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.MasonGeometry;

public class landmarkFunctions{	
	

    static double localLandmarkness(NodeGraph node, boolean advanceVis, HashMap<NodeGraph, NodeWrapper> 
    			mapWrappers, PedestrianSimulation state)
    {   	
        List<Integer> localLandmarks = new ArrayList<Integer>();
        localLandmarks = node.localLandmarks;
        double localScore = 0.0;
        if (localLandmarks == null) return 0.0;
        
        if (!advanceVis) return Collections.max(node.localScores);
        else
        {
        	NodeWrapper previous = mapWrappers.get(mapWrappers.get(node).nodeFrom);
        	for (int lL : localLandmarks)
	        {
        		NodeGraph nodeTo =  node;
        		NodeGraph nodeFrom = null;
	            nodeFrom = previous.node;
	            double distanceTravelled = 0;
	            double cumulativeAdvanceVis = 0;
	            
	            while ((nodeFrom != null) & (distanceTravelled <= state.t))
	            {
	                List<Integer> visible = new ArrayList<Integer>();
	                visible = nodeFrom.visible2d;
                	NodeWrapper nt = mapWrappers.get(nodeTo);
                	EdgeGraph segment = (EdgeGraph) nt.edgeFrom.getEdge();
	                distanceTravelled += segment.getLine().getLength();	                
	                if (visible.contains(lL)) cumulativeAdvanceVis += segment.getLine().getLength();

	                nodeTo = nodeFrom;
	                NodeWrapper nf = mapWrappers.get(nodeFrom);
	                try {nodeFrom = nf.nodeFrom;}
	                catch (java.lang.NullPointerException e) {nodeFrom = null;}
	            }
	              
	          double aV = cumulativeAdvanceVis/distanceTravelled;
	          if (aV > 1.0) aV = 1.0;
	          double tmp = node.localScores.get(localLandmarks.indexOf(lL)) * aV;
	          if (tmp > localScore) localScore = tmp;
	        }
        	return localScore;
        }
    }
        

    static double localLandmarknessDualGraph(NodeGraph centroid, NodeGraph node, boolean advanceVis, 
    		HashMap<NodeGraph, DualNodeWrapper> mapWrappers,
    		PedestrianSimulation state)
    {
    	// current real segment
    	GeomPlanarGraphDirectedEdge streetSegment = (GeomPlanarGraphDirectedEdge) 
    			centroid.primalEdge.getDirEdge(0); 
    	
    	//real node
	
        List<Integer> localLandmarks = new ArrayList<Integer>();
        double localScore = 0.0;
        localLandmarks = node.localLandmarks;
        if (localLandmarks == null) return 0.0;
        if (!advanceVis) return Collections.max(node.localScores);
        else
        {
        	//previous segment
        	NodeGraph previousCentroid = mapWrappers.get(centroid).nodeFrom;
        	for (int lL : localLandmarks)
	        {
        		NodeGraph nodeTo =  node;
        		NodeGraph centroidFrom = previousCentroid;
        		NodeGraph nodeFrom;
	            double distanceTravelled = 0;
	            double cumulativeAdvanceVis = 0;
	            GeomPlanarGraphDirectedEdge currentSegment = streetSegment;
	            
	            while ((centroidFrom != null) & (distanceTravelled <= state.t))
	            {
	            	if (currentSegment.getFromNode() == nodeTo) nodeFrom = (NodeGraph) currentSegment.getToNode();
	            	else nodeFrom = (NodeGraph) currentSegment.getToNode();
	                List<Integer> visible = new ArrayList<Integer>();
	                visible = nodeFrom.visible2d;

	                EdgeGraph segment = (EdgeGraph) currentSegment.getEdge();
	                distanceTravelled = distanceTravelled + segment.getLine().getLength();
	                       
	                if (visible.contains(lL)) cumulativeAdvanceVis += segment.getLine().getLength();
	                nodeTo = nodeFrom;
	            	currentSegment = (GeomPlanarGraphDirectedEdge) centroidFrom.primalEdge.getDirEdge(0);
	                DualNodeWrapper cf = mapWrappers.get(centroidFrom);
	                try {centroidFrom = cf.nodeFrom;}
	                catch (java.lang.NullPointerException e) {centroidFrom = null;}
	            } 
	          double aV = cumulativeAdvanceVis/distanceTravelled;
	          if (aV > 1.0) aV = 1.0;
	          double tmp = node.localScores.get(localLandmarks.indexOf(lL)) * aV;
	          if (tmp > localScore) localScore = tmp;
	        }
        	return localScore;
        }

    }
    
    static double globalLandmarknessNode(NodeGraph node, NodeGraph destinationNode, boolean useAnchors, 
    		PedestrianSimulation state) 
    {   	
            
       List<Integer> distantLandmarks = new ArrayList<Integer>();
       distantLandmarks = node.distantLandmarks;
       if (distantLandmarks == null) return 1.0;
       if (!useAnchors) return Collections.max(node.distantScores);	
       
       List<Integer> anchors = new ArrayList<Integer>();
       anchors = destinationNode.anchors;
       if (useAnchors & anchors == null) return 0.0;
       double nodeGlobalScore = 0.0;
       for (int dL : distantLandmarks)
       {
   			double tmp = 0.0;
    	   	if (anchors.contains(dL))
    	   	{
				tmp = node.distantScores.get(distantLandmarks.indexOf(dL));
				double distanceLandmark = destinationNode.distances.get(anchors.indexOf(dL));
				double distanceWeight = utilities.nodesDistance(node, destinationNode)/distanceLandmark;
				if (distanceWeight > 1.0) distanceWeight = 1.0;
				tmp = tmp*distanceWeight;   
    	   }
    	  if (tmp > nodeGlobalScore) nodeGlobalScore = tmp;
       }
       return nodeGlobalScore;
    }
       
    static double globalLandmarknessDualNode(NodeGraph centroid, NodeGraph destinationNode, 
    		boolean useAnchors, PedestrianSimulation state) 
    {   	
		// current real segment: identifying node
		GeomPlanarGraphDirectedEdge streetSegment = (GeomPlanarGraphDirectedEdge) centroid.primalEdge.getDirEdge(0); 
		NodeGraph targetNode = (NodeGraph) streetSegment.getToNode(); // targetNode            
		    	
		List<Integer> distantLandmarks = new ArrayList<Integer>();
		distantLandmarks = targetNode.distantLandmarks;
		if (distantLandmarks == null) return 0.0;
		if (!useAnchors) return Collections.max(targetNode.distantScores);	
		
		// destination segment: identifying node
		List<Integer> anchors = new ArrayList<Integer>();
		anchors = destinationNode.anchors;
		if (useAnchors & anchors == null) return 0.0;
		double nodeGlobalScore = 0.0;

		for (int dL : distantLandmarks)
		{
			double tmp = 0.0;
			if (anchors.contains(dL))
			{
				tmp = targetNode.distantScores.get(distantLandmarks.indexOf(dL));
				double distanceLandmark = destinationNode.distances.get(anchors.indexOf(dL));
				double distanceWeight = utilities.nodesDistance(centroid, destinationNode)/distanceLandmark;
				if (distanceWeight > 1.0) distanceWeight = 1.0;
				tmp = tmp*distanceWeight;
			}
			if (tmp > nodeGlobalScore) nodeGlobalScore = tmp;
		}
		return nodeGlobalScore;
	}
    
                        
    static double globalLandmarknessPaths(NodeGraph destinationNode, NodeGraph tmpNode, HashMap<NodeGraph, 
    		NodeWrapper> mapWrappers, boolean useAnchors, String method, PedestrianSimulation state)
    {   	
        List<Integer> anchors = new ArrayList<Integer>();
        anchors = destinationNode.anchors;
        if (useAnchors & anchors == null) return 0.0;

        List<Double> nodeGlobalScores = new ArrayList<Double>();
        Iterator<Entry<NodeGraph, NodeWrapper>> it = mapWrappers.entrySet().iterator();
        
        if (!useAnchors)
        {
            while (it.hasNext()) 
            {
                Map.Entry pair = (Map.Entry)it.next();
                NodeGraph node = (NodeGraph) pair.getKey();
                List<Integer> distantLandmarks = new ArrayList<Integer>();
                distantLandmarks = node.distantLandmarks;
                
                double nodeGlobalScore = 0.0;
                if (distantLandmarks == null) nodeGlobalScore = 0.0;
                nodeGlobalScore = Collections.max(node.distantScores);
                nodeGlobalScores.add(nodeGlobalScore);
            }
        	if (method == "max") return Collections.max(nodeGlobalScores);
        	if (method == "mean")return nodeGlobalScores.stream().mapToDouble(i -> i).average().orElse(0.0);
        }
        else
        {
        	while (it.hasNext()) 
            {
                Map.Entry pair = (Map.Entry)it.next();
                NodeGraph node = (NodeGraph) pair.getKey();
                List<Integer> distantLandmarks = new ArrayList<Integer>();
                distantLandmarks = node.distantLandmarks;
                double nodeGlobalScore = 0.0;
                if (distantLandmarks == null) 
                {
                	nodeGlobalScores.add(nodeGlobalScore);
                	continue;
                }
                else
                {
                	for (int dL : distantLandmarks)
                    {
                		double tmp = 0.0;
                 	   	if (anchors.contains(dL))
                 	   	{
							tmp = node.distantScores.get(distantLandmarks.indexOf(dL));
							double distanceLandmark = destinationNode.distances.get(anchors.indexOf(dL));
							double distanceWeight = utilities.nodesDistance(tmpNode, destinationNode)/
									distanceLandmark;
							if (distanceWeight > 1.0) distanceWeight = 1.0;
							tmp = tmp*distanceWeight;   
                 	   }
                 	  if (tmp > nodeGlobalScore) nodeGlobalScore = tmp;
                    }
                    nodeGlobalScores.add(nodeGlobalScore);
                }
            }
            if (method == "max") return Collections.max(nodeGlobalScores);
            if (method == "mean") return nodeGlobalScores.stream().mapToDouble(i -> i).average().orElse(0.0);
        }
        return 0.0;
    }
        
        
        
    static double globalLandmarknessDualPath(NodeGraph dualDestinationNode, NodeGraph tmpNode, 
    		NodeGraph destinationNode, HashMap<NodeGraph, DualNodeWrapper> mapWrappers,
    		boolean useAnchors, String method, PedestrianSimulation state)
    {   

        List<Integer> anchors = new ArrayList<Integer>();
        anchors = destinationNode.anchors;
        if (useAnchors & anchors == null) return 0.0;
    	
        List<Double> nodeGlobalScores = new ArrayList<Double>();
        Iterator<Entry<NodeGraph, DualNodeWrapper>> it = mapWrappers.entrySet().iterator();
        NodeGraph previous = dualDestinationNode;
        NodeGraph throughNode;
        
        if (!useAnchors)
        {
            while (it.hasNext()) 
            {
                Map.Entry pair = (Map.Entry)it.next();
                NodeGraph centroid = (NodeGraph) pair.getKey();
            	GeomPlanarGraphDirectedEdge edgeDirP = (GeomPlanarGraphDirectedEdge) 
            			centroid.primalEdge.getDirEdge(0); 
            	NodeGraph nToP = (NodeGraph) edgeDirP.getToNode();
            	NodeGraph nFromP = (NodeGraph) edgeDirP.getFromNode();
            	if (previous == nToP)  throughNode = nFromP;
            	else throughNode = nToP;

                List<Integer> distantLandmarks = new ArrayList<Integer>();
                distantLandmarks = throughNode.distantLandmarks;
                double nodeGlobalScore = 0.0;
                
                if (distantLandmarks == null) nodeGlobalScore = 0.0;
                nodeGlobalScore = Collections.max(throughNode.distantScores);
                nodeGlobalScores.add(nodeGlobalScore);
	            previous = throughNode;
            }
        	if (method == "max") return Collections.max(nodeGlobalScores);
        	if (method == "mean") return nodeGlobalScores.stream().mapToDouble(i -> i).average().orElse(0.0);
        }
        else
        {
	        while (it.hasNext()) 
	        {
	            Map.Entry pair = (Map.Entry)it.next();
	            NodeGraph centroid = (NodeGraph) pair.getKey();
	        	GeomPlanarGraphDirectedEdge edgeDirP = (GeomPlanarGraphDirectedEdge) 
	        			centroid.primalEdge.getDirEdge(0); 
	        	NodeGraph nToP = (NodeGraph) edgeDirP.getToNode();
	        	NodeGraph nFromP = (NodeGraph) edgeDirP.getFromNode();
	        	if (previous == nToP)  throughNode = nFromP;
	        	else throughNode = nToP;
	
	            List<Integer> distantLandmarks = new ArrayList<Integer>();
	            distantLandmarks = throughNode.distantLandmarks;
	            double nodeGlobalScore = 0.0;
	            if (distantLandmarks == null) 
                {
                	nodeGlobalScores.add(nodeGlobalScore);
                	continue;
                }
	            
	            else
                {
                	for (int dL : distantLandmarks)
                    {
                		double tmp = 0.0;
                 	   	if (anchors.contains(dL))
                 	   	{
							tmp = throughNode.distantScores.get(distantLandmarks.indexOf(dL));
							double distanceLandmark = destinationNode.distances.get(anchors.indexOf(dL));
							double distanceWeight = utilities.nodesDistance(tmpNode, destinationNode)/distanceLandmark;
							if (distanceWeight > 1.0) distanceWeight = 1.0;
							tmp = tmp*distanceWeight;   
                 	   }
                 	  if (tmp > nodeGlobalScore) nodeGlobalScore = tmp;
                    }
                    nodeGlobalScores.add(nodeGlobalScore);
                }
	            previous = throughNode;
	        }
            if (method == "max") return Collections.max(nodeGlobalScores);
            if (method == "mean") return nodeGlobalScores.stream().mapToDouble(i -> i).average().orElse(0.0);
        }
    return 0.0;
    }

        
    public static double easinessNavigation(NodeGraph originNode, NodeGraph destinationNode, PedestrianSimulation state)
    {
        Geometry smallestCircle = utilities.smallestEnclosingCircle(originNode, destinationNode);
        double distanceComplexity = utilities.nodesDistance(originNode, destinationNode)/Collections.max(PedestrianSimulation.distances);
        Bag filterBuildings = PedestrianSimulation.buildings.getContainedObjects(smallestCircle);
        HashMap<MasonGeometry, Double> buildingsMap =  utilities.filterMap(state.buildingsLS, filterBuildings);
        
        int count = 0;
        Iterator<Entry<MasonGeometry, Double>> it = buildingsMap.entrySet().iterator();
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
	
	public static GeomVectorField relevantNodes(NodeGraph originNode, NodeGraph destinationNode, PedestrianSimulation state)
	{
	    Geometry smallestCircle = utilities.smallestEnclosingCircle(originNode, destinationNode);
	    Bag filter = PedestrianSimulation.junctions.getContainedObjects(smallestCircle);      	    
	    HashMap<MasonGeometry, Double> nodesMap =  utilities.filterMap(state.nodesBc, filter);

	    Map<MasonGeometry, Double> orderedMap = utilities.sortByValue(nodesMap);
	    // quantile
	    int percentile = (int) (orderedMap.size()*0.8);
	    int count = 0;
	    double boundary = 0.0;
	    
	    Iterator<Entry<MasonGeometry, Double>> it = orderedMap.entrySet().iterator();

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
	
	public static Bag intersectingBarriers(NodeGraph originNode, NodeGraph destinationNode, String type)
	{
			
		LineString l = utilities.LineStringBetweenNodes(originNode, destinationNode);
		Bag intersecting = PedestrianSimulation.barriers.getTouchingObjects(l);
    	GeomVectorField intersectingBarriers = new GeomVectorField();

		for (Object iB:intersecting)
		{
			MasonGeometry geoBarrier = (MasonGeometry) iB;
			String barrierType = geoBarrier.getStringAttribute("type");
			if (type == "all") intersectingBarriers.addGeometry(geoBarrier);
			else if (type == "positive" & (barrierType == "park" || barrierType == "river"))
					intersectingBarriers.addGeometry(geoBarrier);
			else if (type == "negative" & (barrierType == "railway" || barrierType == "road"))
				intersectingBarriers.addGeometry(geoBarrier);
			else if (type == "separating" & barrierType != "parks")	intersectingBarriers.addGeometry(geoBarrier);
			else if (type == barrierType) intersectingBarriers.addGeometry(geoBarrier);
		} 
			
		return intersectingBarriers.getGeometries();
		
	}
}
    