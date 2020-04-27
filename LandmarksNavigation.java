package sim.app.geo.pedestrianSimulation;

import java.util.*;
import java.util.Map.Entry;

import com.vividsolutions.jts.geom.*;

import sim.app.geo.urbanSim.*;
import sim.util.Bag;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.MasonGeometry;

public class LandmarksNavigation
{	
	
	public static ArrayList<NodeGraph> findSequenceSubGoals(NodeGraph originNode, NodeGraph destinationNode)
	{

    	ArrayList<NodeGraph> knownJunctions = PedestrianSimulation.network.salientNodes(originNode, destinationNode, 0, 0.80, "local");
        double wayfindingComplexity = easinessNavigation(originNode, destinationNode);
        double searchRange = utilities.nodesDistance(originNode, destinationNode) * (wayfindingComplexity);
        NodeGraph currentNode = originNode;
        
	    List<Integer> badCandidates = new ArrayList<Integer>();
	    ArrayList<NodeGraph> sequence = new ArrayList<NodeGraph>();
    	while (searchRange >  PedestrianSimulation.t)
        {
    		NodeGraph bestNode = null;
        	double attractivness = 0.0;
        		        	
        	for (NodeGraph tmpNode : knownJunctions)
	        {	    	
		    	if (sequence.contains(tmpNode)) continue;
		    	if (tmpNode == originNode) continue;
		    	if (tmpNode.getEdgeBetween(currentNode) != null) continue;
		    	if (tmpNode.getEdgeBetween(destinationNode)!= null) continue;
		    	if (tmpNode.getEdgeBetween(originNode)!= null) continue;
		    	
		    	if (utilities.nodesDistance(currentNode, tmpNode) > searchRange)
		    	{
		    		badCandidates.add(tmpNode.getID());
		    		continue; //only nodes in range	
		    	}
        		double localScore = 0.0;
        		localScore = localLandmarkness(tmpNode, false, null);

		    	double gain = ((utilities.nodesDistance(currentNode, destinationNode) - 
		    			utilities.nodesDistance(tmpNode, destinationNode))/utilities.nodesDistance(currentNode, destinationNode));
		    	
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
        	knownJunctions = PedestrianSimulation.network.salientNodes(bestNode, destinationNode,0, 0.80, "local");
        	wayfindingComplexity = easinessNavigation(bestNode, destinationNode);
        	searchRange = utilities.nodesDistance(bestNode, destinationNode) * wayfindingComplexity;
            currentNode = bestNode;
            bestNode = null;
        }
    	return sequence;
	}

    static double localLandmarkness(NodeGraph node, boolean advanceVis, HashMap<NodeGraph, NodeWrapper>	mapWrappers)
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
	            
	            while ((nodeFrom != null) & (distanceTravelled <= PedestrianSimulation.t))
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
    		HashMap<NodeGraph, NodeWrapper> mapWrappers)
    {
    	// current real segment
    	GeomPlanarGraphDirectedEdge streetSegment = (GeomPlanarGraphDirectedEdge) centroid.primalEdge.getDirEdge(0); 
    	
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
	            
	            while ((centroidFrom != null) & (distanceTravelled <= PedestrianSimulation.t))
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
	                NodeWrapper cf = mapWrappers.get(centroidFrom);
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
    
    static double globalLandmarknessNode(NodeGraph node, NodeGraph destinationNode, boolean useAnchors) 
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
       
    static double globalLandmarknessDualNode(NodeGraph centroid, NodeGraph destinationNode, boolean useAnchors) 
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
        
    public static double easinessNavigation(NodeGraph originNode, NodeGraph destinationNode)
    {
        Geometry smallestCircle = utilities.smallestEnclosingCircle(originNode, destinationNode);
        double distanceComplexity = utilities.nodesDistance(originNode, destinationNode)/Collections.max(PedestrianSimulation.distances);
        Bag filterBuildings = PedestrianSimulation.buildings.getContainedObjects(smallestCircle);
        HashMap<MasonGeometry, Double> buildingsMap =  utilities.filterMap(PedestrianSimulation.buildingsLS, filterBuildings);
        
        int count = 0;
        Iterator<Entry<MasonGeometry, Double>> it = buildingsMap.entrySet().iterator();
        while (it.hasNext()) 
        {
        	Map.Entry<MasonGeometry, Double> entry = it.next();
        	if (entry.getValue() > 0.30) count += 1;
        }
        
        double buildingsComplexity = 0.0;
        if (filterBuildings.size() == 0) buildingsComplexity = 0.0;
        else buildingsComplexity =  (filterBuildings.size() - count) /  filterBuildings.size();
        double environmentalComplexity = (distanceComplexity + buildingsComplexity)/2;
        double easiness = 1 - environmentalComplexity;
        return easiness;
    }
	
}
    