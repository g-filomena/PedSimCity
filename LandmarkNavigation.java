package sim.app.geo.pedSimCity;

import java.util.*;
import java.util.Map.Entry;

import com.vividsolutions.jts.geom.*;

import sim.app.geo.urbanSim.*;
import sim.util.Bag;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.MasonGeometry;

public class LandmarkNavigation
{	
	/** 
	* Sequence intermediate points based on local-landmarkness (identification of decision points).
	* If regionBasedNavigation is true, look within the region, regardless originNode and destinatioNode
	* Percentile indicates the percentile above which a node is considered salient, on the basis of local or global betweenness centrality.
	* typeLandmarkness can be "global" or "local", depending on whether global landmarks or local landmarks are used to compute
	* the wayfinding complexity of the environment.
	*/

	public static ArrayList<NodeGraph> findSequenceSubGoals(NodeGraph originNode, NodeGraph destinationNode, boolean regionBasedNavigation,
			String typeLandmarkness)
	{
		double percentile = PedSimCity.salientNodesPercentile;
		ArrayList<NodeGraph> knownJunctions;
		ArrayList<NodeGraph> sequence = new ArrayList<NodeGraph>();
	    List<Integer> badCandidates = new ArrayList<Integer>();
		
		if (!regionBasedNavigation) knownJunctions = PedSimCity.network.salientNodesBewteenSpace(originNode, destinationNode, 
					0,0, percentile, "local");
		else 
		{
			RegionData region = PedSimCity.regionsMap.get(originNode.region);
			knownJunctions = region.primalGraph.salientNodesBewteenSpace(originNode, destinationNode, 0,0, percentile,"local");
		}
    	
		/**  
		 * If no salient junctions are found, the tolerance increases till the 0.50 percentile;
		 * if still no salient junctions are found, the agent continues without landmarks
		 */
    	while (knownJunctions == null) 
    	{
    		percentile -= 0.05;
    		if (percentile < 0.50)
    		{
		    	sequence.add(originNode);
		    	sequence.add(destinationNode);
		    	return sequence;
    		}
    		knownJunctions = PedSimCity.network.salientNodesBewteenSpace(originNode, destinationNode, 0,0, percentile, "local");
    	}

        double wayfindingEasiness = wayfindingEasiness(originNode, destinationNode, typeLandmarkness);
        double searchDistance = Utilities.nodesDistance(originNode, destinationNode) * (wayfindingEasiness);
        NodeGraph currentNode = originNode;
            
	    
		/**  
		 * While the wayfindingEasiness is lower than the threshold the agent looks for intermediate-points.
	
		 */
    	while (wayfindingEasiness < PedSimCity.wayfindingEasinessThreshold)
        {

    		NodeGraph bestNode = null;
        	double attractivness = 0.0;
        		        	
        	for (NodeGraph tmpNode : knownJunctions)
	        {	    	
        		
        		// bad candidates (candidate is destination, or origin, already visited, etc)
		    	if (sequence.contains(tmpNode) || tmpNode == originNode || tmpNode.getEdgeBetween(currentNode) != null || 
		    			tmpNode.getEdgeBetween(destinationNode)!= null || tmpNode.getEdgeBetween(originNode)!= null) continue;

		    	if (Utilities.nodesDistance(currentNode, tmpNode) > searchDistance)
		    	{
		    		badCandidates.add(tmpNode.getID());
		    		continue; //only nodes in range	
		    	}
        		double localScore = 0.0;
        		localScore = localLandmarkness(tmpNode, false, null);
        		
        		double currentDistance = Utilities.nodesDistance(currentNode, destinationNode);
		    	double gain = (currentDistance - Utilities.nodesDistance(tmpNode, destinationNode))/currentDistance;
		    	
		    	double landmarkness = localScore*0.60 + gain*0.40;
		    	if (landmarkness > attractivness) 
    			{
		    		attractivness = landmarkness;
		    		bestNode = tmpNode;
    			}
	        }
        		        	
        	if (bestNode == null) break;
			if (bestNode == destinationNode) break;
        	sequence.add(bestNode);
        	
    		/** 
    		 * While the wayfindingEasiness is lower than the threshold the agent looks for intermediate-points.
    		 * Second and third parameter not necessary here (i.e. set to 0,0)
    		 * "local" rescales betweenness centrality within the search space; otherwise uses "global" for the global centrality value.
    		 */
        	percentile = PedSimCity.salientNodesPercentile;
        	knownJunctions = PedSimCity.network.salientNodesBewteenSpace(bestNode, destinationNode, 0, 0,  percentile, "local");
        	while (knownJunctions == null) 
        	{
        		percentile -= 0.05;
        		if (percentile < 0.50) 
            	{
                	sequence.add(0, originNode);
                	sequence.add(destinationNode);
                	return sequence;
                }
        		knownJunctions = PedSimCity.network.salientNodesBewteenSpace(bestNode, destinationNode, 0,0, percentile, "local");
        	}
        	wayfindingEasiness = wayfindingEasiness(bestNode, destinationNode, typeLandmarkness);
        	searchDistance = Utilities.nodesDistance(bestNode, destinationNode) * wayfindingEasiness;
            currentNode = bestNode;
            bestNode = null;
        }
    	sequence.add(0, originNode);
    	sequence.add(destinationNode);
    	return sequence;
	}

    static double localLandmarkness(NodeGraph node, boolean advanceVis, HashMap<NodeGraph, NodeWrapper>	mapWrappers)
    {   
		/** 
		 * While the wayfindingEasiness is lower than the threshold the agent looks for intermediate-points.
		 * Second and third parameter not necessary here (i.e. set to 0,0)
		 * "local" rescales betweenness centrality within the search space; otherwise uses "global" for the global centrality value.
		 */
    	
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
	            
	            while ((nodeFrom != null) & (distanceTravelled <= PedSimCity.visibilityThreshold))
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
        
   
    static double globalLandmarknessNode(NodeGraph targetNode, NodeGraph destinationNode, boolean useAnchors) 
    {   	
            
       List<Integer> distantLandmarks = new ArrayList<Integer>();
       distantLandmarks = targetNode.distantLandmarks;
       if (distantLandmarks == null) return 1.0;
       if (!useAnchors) return Collections.max(targetNode.distantScores);	
       
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
				double distanceWeight = Utilities.nodesDistance(targetNode, destinationNode)/distanceLandmark;
				if (distanceWeight > 1.0) distanceWeight = 1.0;
				tmp = tmp*distanceWeight;   
    	   }
    	  if (tmp > nodeGlobalScore) nodeGlobalScore = tmp;
       }
       return nodeGlobalScore;
    }
       
    static double globalLandmarknessDualNode(NodeGraph centroid, NodeGraph targetCentroid, NodeGraph destinationNode, boolean useAnchors) 
    {   	
		// current real segment: identifying node
		GeomPlanarGraphDirectedEdge streetSegment = (GeomPlanarGraphDirectedEdge) targetCentroid.primalEdge.getDirEdge(0); 
		NodeGraph targetNode = (NodeGraph) streetSegment.getToNode(); // targetNode  
		if (Utilities.commonPrimalJunction(centroid, targetCentroid) == targetNode) targetNode = (NodeGraph) streetSegment.getFromNode();
		    	
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
				double distanceWeight = Utilities.nodesDistance(targetNode, destinationNode)/distanceLandmark;
				if (distanceWeight > 1.0) distanceWeight = 1.0;
				tmp = tmp*distanceWeight;
			}
			if (tmp > nodeGlobalScore) nodeGlobalScore = tmp;
		}
		return nodeGlobalScore;
	}
    
                        
    static double globalLandmarknessPaths(NodeGraph destinationNode, NodeGraph tmpNode, HashMap<NodeGraph, 
    		NodeWrapper> mapWrappers, boolean useAnchors, String method)
    {   	
        List<Integer> anchors = new ArrayList<Integer>();
        anchors = destinationNode.anchors;
        if (useAnchors & anchors == null) return 0.0;

        List<Double> nodeGlobalScores = new ArrayList<Double>();
        Set<Entry<NodeGraph, NodeWrapper>> entries = mapWrappers.entrySet();
        
        if (!useAnchors)
        {
            for (Entry<NodeGraph, NodeWrapper> pair : entries)
            {
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
        	for (Entry<NodeGraph, NodeWrapper> pair : entries) 
            {
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
							double distanceWeight = Utilities.nodesDistance(tmpNode, destinationNode)/
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
    		NodeGraph destinationNode, HashMap<NodeGraph, NodeWrapper> mapWrappers,
    		boolean useAnchors, String method)
    {   

        List<Integer> anchors = new ArrayList<Integer>();
        anchors = destinationNode.anchors;
        if (useAnchors & anchors == null) return 0.0;
    	
        List<Double> nodeGlobalScores = new ArrayList<Double>();
        Set<Entry<NodeGraph, NodeWrapper>> entries = mapWrappers.entrySet();
        NodeGraph previous = dualDestinationNode;
        NodeGraph throughNode;
        
        if (!useAnchors)
        {
        	for (Entry<NodeGraph, NodeWrapper> pair : entries) 
            {
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
        	for (Entry<NodeGraph, NodeWrapper> pair : entries) 
            {
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
							double distanceWeight = Utilities.nodesDistance(tmpNode, destinationNode)/distanceLandmark;
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
        
    public static double wayfindingEasiness(NodeGraph originNode, NodeGraph destinationNode, String typeLandmarkness)
    {
    	double distanceComplexity = Utilities.nodesDistance(originNode, destinationNode)/Math.max(PedSimCity.roads.MBR.getHeight(),
    				PedSimCity.roads.MBR.getWidth());
               
    	ArrayList<MasonGeometry> buildings = getBuildings(originNode, destinationNode, originNode.region);
    	ArrayList<MasonGeometry> landmarks = new ArrayList<MasonGeometry>();
    	if (typeLandmarkness == "global")  landmarks = getLandmarks(buildings, PedSimCity.globalLandmarkThreshold, "global");
    	else landmarks = getLandmarks(buildings, PedSimCity.localLandmarkThreshold, "local");
    	
        double buildingsComplexity = 1.0;
        if (buildings.size() == 0 || buildings == null) buildingsComplexity = 0.0;
        else buildingsComplexity = buildingsComplexity(buildings, landmarks);
        double wayfindingComplexity = (distanceComplexity + buildingsComplexity)/2.0;
        double easiness = 1.0 - wayfindingComplexity;
        return easiness;
    }
    
    public static double wayfindingEasinessRegion(NodeGraph originNode, NodeGraph destinationNode, NodeGraph tmpOrigin, NodeGraph tmpDestination)
    {
		double intraRegionDistance = Utilities.nodesDistance(tmpOrigin, tmpDestination);
		double distance = Utilities.nodesDistance(originNode, destinationNode);
		if (intraRegionDistance/distance < 0.10) return 1;
		
		double distanceComplexity = intraRegionDistance/distance;
		double buildingsComplexity = PedSimCity.regionsMap.get(tmpOrigin.region).buildingsComplexity;
        double wayfindingComplexity = (distanceComplexity + buildingsComplexity)/2.0;
        double easiness = 1.0 - wayfindingComplexity;
        return easiness;
    }
    
    
    public static double buildingsComplexity(ArrayList<MasonGeometry> buildings, ArrayList<MasonGeometry> landmarks)
    {   	
    	return ((double) buildings.size()-landmarks.size())/(double) buildings.size();
    }
    
    public static ArrayList<MasonGeometry> getBuildings(NodeGraph originNode, NodeGraph destinationNode, int region)
    {
      	ArrayList<MasonGeometry> buildings = new ArrayList<MasonGeometry>();
    	
    	if (originNode != null)
    	{
    		Geometry smallestCircle = Utilities.smallestEnclosingCircle(originNode, destinationNode);
    		Bag filterBuildings = PedSimCity.buildings.getContainedObjects(smallestCircle);
    		for (Object o: filterBuildings) buildings.add((MasonGeometry) o);
    	}
    	
    	else
    	{
    		VectorLayer regionNetwork = PedSimCity.regionsMap.get(region).regionNetwork;
    		Geometry convexHull = regionNetwork.getConvexHull().getGeometry();
    		Bag filterBuildings = PedSimCity.buildings.getContainedObjects(convexHull);
    		for (Object o: filterBuildings) buildings.add((MasonGeometry) o);
    	}
    	return buildings;
    }
    
    public static ArrayList<MasonGeometry> getLandmarks(ArrayList<MasonGeometry> buildings, double threshold, String type)
	{
		ArrayList<MasonGeometry> landmarks = new ArrayList<MasonGeometry>();
		String attribute;
		if (type == "local") attribute = "lScore_sc";
		else attribute = "gScore_sc";
		
		for (MasonGeometry b: buildings)
		{
			 if (b.getDoubleAttribute(attribute) >= threshold) landmarks.add(b);
		}
				
		return landmarks;

	}
	
}
    