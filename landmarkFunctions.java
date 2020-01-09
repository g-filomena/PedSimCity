package sim.app.geo.pedestrianSimulation;

import java.util.ArrayList;
import java.util.Collections;
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
	

    static double localLandmarkness(Node node, boolean advanceVis, HashMap<Node, NodeWrapper> mapWrappers, PedestrianSimulation state)
    {   	
        Integer nodeID = (Integer) node.getData();
        NodeData nd = state.nodesMap.get(nodeID);
        List<Integer> localLandmarks = new ArrayList<Integer>();
        localLandmarks = nd.localLandmarks;
        double localScore = 0.0;
        if (localLandmarks == null) return 0.0;
        
        if (!advanceVis) return Collections.max(nd.localScores);
        else
        {
        	NodeWrapper previous = mapWrappers.get(mapWrappers.get(node).nodeFrom);
        	for (int lL : localLandmarks)
	        {
	            Node nodeTo =  node;
	            Node nodeFrom = null;
	            nodeFrom = previous.node;
	            double distanceTravelled = 0;
	            double cumulativeAdvanceVis = 0;
	            
	            while ((nodeFrom != null) & (distanceTravelled <= state.t))
	            {
	            	Integer nodeIDLoop = (Integer) nodeFrom.getData();
	                NodeData ndLoop = state.nodesMap.get(nodeIDLoop);
	                List<Integer> visible = new ArrayList<Integer>();
	                visible = ndLoop.visible2d;
                	NodeWrapper nt = mapWrappers.get(nodeTo);
                	GeomPlanarGraphEdge segment = (GeomPlanarGraphEdge) nt.edgeFrom.getEdge();

	                distanceTravelled += segment.getDoubleAttribute("length");	                
	                if (visible.contains(lL)) cumulativeAdvanceVis += segment.getDoubleAttribute("length");

	                nodeTo = nodeFrom;
	                NodeWrapper nf = mapWrappers.get(nodeFrom);
	                try {nodeFrom = nf.nodeFrom;}
	                catch (java.lang.NullPointerException e) {nodeFrom = null;}
	            }
	              
	          double aV = cumulativeAdvanceVis/distanceTravelled;
	          if (aV > 1.0) aV = 1.0;
	          double tmp = nd.localScores.get(localLandmarks.indexOf(lL)) * aV;
	          if (tmp > localScore) localScore = tmp;
	        }
        	return localScore;
        }
    }
        

    static double localLandmarknessDualGraph(Node centroid, Node node, boolean advanceVis, HashMap<Node, DualNodeWrapper> mapWrappers,
    		PedestrianSimulation state)
    {
    	// current real segment
    	Integer edgeID = (Integer) centroid.getData();
    	GeomPlanarGraphDirectedEdge streetSegment = (GeomPlanarGraphDirectedEdge) 
    			state.edgesMap.get(edgeID).planarEdge.getDirEdge(0); 
    	
    	//real node
    	Integer nodeID = (Integer) node.getData();
        NodeData nd = state.nodesMap.get(nodeID);	
	
        List<Integer> localLandmarks = new ArrayList<Integer>();
        double localScore = 0.0;
        localLandmarks = nd.localLandmarks;
        if (localLandmarks == null) return 0.0;
        if (!advanceVis) return Collections.max(nd.localScores);
        else
        {
        	//previous segment
        	Node previousCentroid = mapWrappers.get(centroid).nodeFrom;
        	for (int lL : localLandmarks)
	        {
	            Node nodeTo =  node;
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
	                NodeData ndLoop = state.nodesMap.get(nodeIDLoop);
	                List<Integer> visible = new ArrayList<Integer>();
	                visible = ndLoop.visible2d;

	                GeomPlanarGraphEdge segment = (GeomPlanarGraphEdge) currentSegment.getEdge();
	                distanceTravelled = distanceTravelled + segment.getLine().getLength();
	                       
	                if (visible.contains(lL)) cumulativeAdvanceVis += segment.getDoubleAttribute("length");
	                nodeTo = nodeFrom;
	                Integer edgeIDLoop = (Integer) centroidFrom.getData();
	            	currentSegment = (GeomPlanarGraphDirectedEdge) state.edgesMap.get(edgeIDLoop).planarEdge.getDirEdge(0);
	                DualNodeWrapper cf = mapWrappers.get(centroidFrom);
	                try {centroidFrom = cf.nodeFrom;}
	                catch (java.lang.NullPointerException e) {centroidFrom = null;}
	            } 
	          double aV = cumulativeAdvanceVis/distanceTravelled;
	          if (aV > 1.0) aV = 1.0;
	          double tmp = nd.localScores.get(localLandmarks.indexOf(lL)) * aV;
	          if (tmp > localScore) localScore = tmp;
	        }
        	return localScore;
        }

    }
    
    static double globalLandmarknessNode(Node node, Node destinationNode, boolean useAnchors, PedestrianSimulation state) 
    {   	
            
       Integer nodeID = (Integer) node.getData();
       NodeData nd = state.nodesMap.get(nodeID);
       List<Integer> distantLandmarks = new ArrayList<Integer>();
       distantLandmarks = nd.distantLandmarks;
       if (distantLandmarks == null) return 1.0;
       if (!useAnchors) return Collections.max(nd.distantScores);	
       
       Integer destinationID = (Integer) destinationNode.getData();
       NodeData dd = state.nodesMap.get(destinationID);
       List<Integer> anchors = new ArrayList<Integer>();
       anchors = dd.anchors;
       if (useAnchors & anchors == null) return 0.0;
       double nodeGlobalScore = 0.0;
       for (int dL : distantLandmarks)
       {
   			double tmp = 0.0;
    	   	if (anchors.contains(dL))
    	   	{
				tmp = nd.distantScores.get(distantLandmarks.indexOf(dL));
				double distanceLandmark = dd.distances.get(anchors.indexOf(dL));
				double distanceWeight = utilities.nodesDistance(node, destinationNode)/distanceLandmark;
				if (distanceWeight > 1.0) distanceWeight = 1.0;
				tmp = tmp*distanceWeight;   
    	   }
    	  if (tmp > nodeGlobalScore) nodeGlobalScore = tmp;
       }
       return nodeGlobalScore;
    }
       

    static double globalLandmarknessDualNode(Node centroid, Node primalDestinationNode, boolean useAnchors, PedestrianSimulation state) 
    {   	
		// current real segment: identifying node
		Integer edgeID = (Integer) centroid.getData();
		GeomPlanarGraphDirectedEdge streetSegment = (GeomPlanarGraphDirectedEdge) state.edgesMap.get(edgeID).planarEdge.getDirEdge(0); 
		Node targetNode = streetSegment.getToNode(); // targetNode            
		    	
		Integer nodeID = (Integer) targetNode.getData();
		NodeData nd = state.nodesMap.get(nodeID);
		List<Integer> distantLandmarks = new ArrayList<Integer>();
		distantLandmarks = nd.distantLandmarks;
		if (distantLandmarks == null) return 0.0;
		if (!useAnchors) return Collections.max(nd.distantScores);	
		
		// destination segment: identifying node
		Integer destinationID = (Integer) primalDestinationNode.getData();
		NodeData dd = state.nodesMap.get(destinationID);
		List<Integer> anchors = new ArrayList<Integer>();
		anchors = dd.anchors;
		if (useAnchors & anchors == null) return 0.0;
		double nodeGlobalScore = 0.0;

		for (int dL : distantLandmarks)
		{
			double tmp = 0.0;
			if (anchors.contains(dL))
			{
				tmp = nd.distantScores.get(distantLandmarks.indexOf(dL));
				double distanceLandmark = dd.distances.get(anchors.indexOf(dL));
				double distanceWeight = utilities.nodesDistance(centroid, primalDestinationNode)/distanceLandmark;
				if (distanceWeight > 1.0) distanceWeight = 1.0;
				tmp = tmp*distanceWeight;
			}
			if (tmp > nodeGlobalScore) nodeGlobalScore = tmp;
		}
		return nodeGlobalScore;
	}
    
                        
    static double globalLandmarknessPaths(Node destinationNode, Node tmpNode, HashMap<Node, NodeWrapper> mapWrappers, boolean useAnchors,
    		String method, PedestrianSimulation state)
    {   	
        Integer destinationID = (Integer) destinationNode.getData();
        NodeData dd = state.nodesMap.get(destinationID);
        List<Integer> anchors = new ArrayList<Integer>();
        anchors = dd.anchors;
        if (useAnchors & anchors == null) return 0.0;

        List<Double> nodeGlobalScores = new ArrayList<Double>();
        Iterator<Entry<Node, NodeWrapper>> it = mapWrappers.entrySet().iterator();
        
        if (!useAnchors)
        {
            while (it.hasNext()) 
            {
                Map.Entry pair = (Map.Entry)it.next();
                Node node = (Node) pair.getKey();
                Integer nodeID = (Integer) node.getData();
                NodeData nd = state.nodesMap.get(nodeID);
                List<Integer> distantLandmarks = new ArrayList<Integer>();
                distantLandmarks = nd.distantLandmarks;
                
                double nodeGlobalScore = 0.0;
                if (distantLandmarks == null) nodeGlobalScore = 0.0;
                nodeGlobalScore = Collections.max(nd.distantScores);
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
                Node node = (Node) pair.getKey();
                Integer nodeID = (Integer) node.getData();
                NodeData nd = state.nodesMap.get(nodeID);
                List<Integer> distantLandmarks = new ArrayList<Integer>();
                distantLandmarks = nd.distantLandmarks;
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
							tmp = nd.distantScores.get(distantLandmarks.indexOf(dL));
							double distanceLandmark = dd.distances.get(anchors.indexOf(dL));
							double distanceWeight = utilities.nodesDistance(tmpNode, destinationNode)/distanceLandmark;
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
        
        
        
    static double globalLandmarknessDualPath(Node dualDestinationNode, Node tmpNode, Node destinationNode, HashMap<Node, DualNodeWrapper> mapWrappers,
    		boolean useAnchors, String method, PedestrianSimulation state)
    {   
        Integer destinationID = (Integer) destinationNode.getData();
        NodeData dd = state.nodesMap.get(destinationID);
        List<Integer> anchors = new ArrayList<Integer>();
        anchors = dd.anchors;
        if (useAnchors & anchors == null) return 0.0;
    	
        List<Double> nodeGlobalScores = new ArrayList<Double>();
        Iterator<Entry<Node, DualNodeWrapper>> it = mapWrappers.entrySet().iterator();
        Node previous = dualDestinationNode;
        Node throughNode;
        
        if (!useAnchors)
        {
            while (it.hasNext()) 
            {
                Map.Entry pair = (Map.Entry)it.next();
                Node centroid = (Node) pair.getKey();
                Integer edgeIDp = (Integer) centroid.getData();
            	GeomPlanarGraphDirectedEdge edgeDirP = (GeomPlanarGraphDirectedEdge) 
            			state.edgesMap.get(edgeIDp).planarEdge.getDirEdge(0); 
            	Node nToP = edgeDirP.getToNode();
            	Node nFromP = edgeDirP.getFromNode();
            	if (previous == nToP)  throughNode = nFromP;
            	else throughNode = nToP;

                Integer nodeID = (Integer) throughNode.getData();
                NodeData nd = state.nodesMap.get(nodeID);
                List<Integer> distantLandmarks = new ArrayList<Integer>();
                distantLandmarks = nd.distantLandmarks;
                double nodeGlobalScore = 0.0;
                
                if (distantLandmarks == null) nodeGlobalScore = 0.0;
                nodeGlobalScore = Collections.max(nd.distantScores);
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
	            Node centroid = (Node) pair.getKey();
	            Integer edgeIDp = (Integer) centroid.getData();
	        	GeomPlanarGraphDirectedEdge edgeDirP = (GeomPlanarGraphDirectedEdge) 
	        			state.edgesMap.get(edgeIDp).planarEdge.getDirEdge(0); 
	        	Node nToP = edgeDirP.getToNode();
	        	Node nFromP = edgeDirP.getFromNode();
	        	if (previous == nToP)  throughNode = nFromP;
	        	else throughNode = nToP;
	
	            Integer nodeID = (Integer) throughNode.getData();
	            NodeData nd = state.nodesMap.get(nodeID);
	            List<Integer> distantLandmarks = new ArrayList<Integer>();
	            distantLandmarks = nd.distantLandmarks;
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
							tmp = nd.distantScores.get(distantLandmarks.indexOf(dL));
							double distanceLandmark = dd.distances.get(anchors.indexOf(dL));
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

        
    public static double easinessNavigation(Node originNode, Node destinationNode, PedestrianSimulation state)
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
	
	public static GeomVectorField relevantNodes(Node originNode, Node destinationNode, PedestrianSimulation state)
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
    	

}
    