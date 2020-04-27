package sim.app.geo.pedestrianSimulation;

import java.util.*;
import com.vividsolutions.jts.geom.*;

import sim.util.Bag;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.MasonGeometry;
import sim.app.geo.urbanSim.*;

public class BarriersRouting {

	NodeGraph currentLocation;
	HashMap<Integer, EdgeGraph> edgesMap;
	

	public ArrayList<GeomPlanarGraphDirectedEdge> pathFinderBarriers(NodeGraph originNode, 
			NodeGraph destinationNode, String localHeuristic)
	{	
		this.edgesMap = PedestrianSimulation.edgesMap;
		this.currentLocation = originNode;

		ArrayList<GeomPlanarGraphDirectedEdge> completePath =  new ArrayList<GeomPlanarGraphDirectedEdge>();
		//sub-goals and barriers - navigation
		ArrayList<NodeGraph> sequence = new ArrayList<NodeGraph>();
		HashMap<NodeGraph, Integer> barrierSubGoals = new HashMap<NodeGraph, Integer>();
		sequence.add(originNode);
		boolean completed = false;
		currentLocation = originNode;
		ArrayList<Integer> pBarriers = new ArrayList<Integer>();
		
		while (!completed)
		{
			//check if there are good barriers in line of movement
			ArrayList<MasonGeometry> intersectingBarriers = intersectingBarriers(currentLocation, destinationNode, "positive");
			MasonGeometry farthest = null;
			NodeGraph subGoal = null;
			EdgeGraph edgeGoal = null;
			
			ArrayList<EdgeGraph> incomingEdges = currentLocation.getEdgesNode();
			for (EdgeGraph edge : incomingEdges) if (edge.positiveBarriers != null) pBarriers.addAll(edge.positiveBarriers);
			Set<Integer> adjacentBarriers = new HashSet<Integer>(pBarriers);
			if (intersectingBarriers.size() >= 1)
			{
				//look for orienting barriers
				double distance = 0.0;
				for (Object i:intersectingBarriers)
				{
					MasonGeometry geoBarrier = (MasonGeometry) i;
					if (adjacentBarriers.contains(geoBarrier.getIntegerAttribute("barrierID"))) continue;
					Geometry viewField = utilities.viewField(currentLocation, destinationNode);
					
					Coordinate intersection = viewField.intersection(geoBarrier.getGeometry()).getCoordinate();
					double distanceIntersection = utilities.euclideanDistance(currentLocation.getCoordinate(), intersection);
					if (distanceIntersection > utilities.euclideanDistance(currentLocation.getCoordinate(),	destinationNode.getCoordinate())) continue;
					else if(distanceIntersection > distance)
					{
						farthest = geoBarrier;	
						distance = distanceIntersection;
					}
				}
			}	

			if (farthest != null)
			{
				int barrierID = farthest.getIntegerAttribute("barrierID");
				double distance = Double.MAX_VALUE;
				for (EdgeGraph edge : edgesMap.values())
				{
					List<Integer> positiveBarriers = edge.positiveBarriers;
					if ((positiveBarriers == null) || (!positiveBarriers.contains(barrierID))) continue;
					double distanceEdge = utilities.euclideanDistance(currentLocation.getCoordinate(), edge.getCoordsCentroid());
					if (distanceEdge < distance) edgeGoal = edge;
				}
			}
			if (edgeGoal == null) 
			{
				completed = true;
				break;
			}
			else
			{
				NodeGraph u = edgeGoal.u;
				NodeGraph v = edgeGoal.v;
				
				if ((utilities.nodesDistance(currentLocation, u)) < (utilities.nodesDistance(currentLocation, v))) subGoal = u;
				else subGoal = v;
			}
			sequence.add(subGoal);
			currentLocation = subGoal;
			pBarriers.add(farthest.getIntegerAttribute("barrierID"));
			barrierSubGoals.put(subGoal, farthest.getIntegerAttribute("barrierID"));
		}
		
		sequence.add(destinationNode);
		NodeGraph tmpOrigin = null;
		NodeGraph tmpDestination = null;
		
		for (NodeGraph subGoal: sequence)
		{
			int indexOf = sequence.indexOf(subGoal);	
			ArrayList<GeomPlanarGraphDirectedEdge> resultPartial =  new ArrayList<GeomPlanarGraphDirectedEdge>();
			
			if (indexOf == 0) //start
			{
				tmpOrigin = subGoal;
				continue;
			}
			else tmpDestination = subGoal; // actual final destination
			EdgeGraph edge = tmpOrigin.getEdgeBetween(tmpDestination);
			if (edge != null)
			{
				if (!completePath.contains(((GeomPlanarGraphDirectedEdge) edge.getDirEdge(0))))
					completePath.add((GeomPlanarGraphDirectedEdge) edge.getDirEdge(0));
				tmpOrigin = tmpDestination;
				continue;
			}	
			
			int barrierID = 999999;
			if (barrierSubGoals.get(tmpDestination)!= null) barrierID = barrierSubGoals.get(subGoal);
			if (localHeuristic == "roadDistance") 
				{resultPartial = routePlanning.roadDistance(tmpOrigin, tmpDestination, completePath, false, true, 
						barrierID, "dijkstra").edges;}
			else 
			{
				NodeGraph pJ = null;
				ArrayList<NodeGraph> centroidsToAvoid = utilities.centroidsFromPath(completePath);
				if (completePath.size() >1) pJ = utilities.previousJunction(completePath);
				resultPartial = routePlanning.angularChange(tmpOrigin, tmpDestination, centroidsToAvoid, pJ, false, true,
					barrierID, "dijkstra").edges;
			}

			tmpOrigin = tmpDestination;
			completePath.addAll(resultPartial);
		}
		return completePath;	
	}
	
	public static ArrayList<MasonGeometry> intersectingBarriers(NodeGraph originNode, NodeGraph destinationNode, String type)
	{
		Geometry viewField = utilities.viewField(originNode, destinationNode);
		Bag intersecting = PedestrianSimulation.barriers.getIntersecting(viewField);
    	ArrayList<MasonGeometry> intersectingBarriers = new ArrayList<MasonGeometry>();

		for (Object iB:intersecting)
		{
			MasonGeometry geoBarrier = (MasonGeometry) iB;
			String barrierType = geoBarrier.getStringAttribute("type");

			if (type.equals("all")) intersectingBarriers.add(geoBarrier);
			else if ((type.equals("positive")) & (barrierType.equals("park")) || (barrierType.equals("water")))
					intersectingBarriers.add(geoBarrier);
			else if ((type.equals("negative")) && (barrierType.equals("railway")) || (barrierType.equals("road")))
				intersectingBarriers.add(geoBarrier);
			else if ((type.equals("separating")) && (!barrierType.equals("parks"))) intersectingBarriers.add(geoBarrier);
			else if (type == barrierType) intersectingBarriers.add(geoBarrier);
		} 
		return intersectingBarriers;
	}
}

