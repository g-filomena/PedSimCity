package sim.app.geo.pedestrianSimulation;

import java.util.*;
import com.vividsolutions.jts.geom.*;
import sim.util.Bag;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.MasonGeometry;

public class BarriersRouting {

	NodeGraph currentLocation;
	HashMap<Integer, EdgeGraph> edgesMap;
	

	public ArrayList<GeomPlanarGraphDirectedEdge> pathFinderBarriers(NodeGraph originNode, 
			NodeGraph destinationNode, String localHeuristic, PedestrianSimulation state)
	{	
		this.edgesMap = state.edgesMap;
		this.currentLocation = originNode;

		ArrayList<GeomPlanarGraphDirectedEdge> completePath =  new ArrayList<GeomPlanarGraphDirectedEdge>();
		//sub-goals and barriers - navigation
		ArrayList<NodeGraph> sequence = new ArrayList<NodeGraph>();
		HashMap<NodeGraph, Integer> barrierSubGoals = new HashMap<NodeGraph, Integer>();
		sequence.add(originNode);
		boolean completed = false;
		currentLocation = originNode;
		
		while (!completed)
		{
			//check if there are good barriers in line of movement
			Bag intersectingBarriers = landmarkFunctions.intersectingBarriers(currentLocation,
					destinationNode, "positive");
			
			MasonGeometry farthest = null;
			NodeGraph subGoal = null;
			EdgeGraph edgeGoal = null;
			
			if (intersectingBarriers.size() >= 1)
			{
				//look for orienting barriers
				double distance = 0.0;
				for (Object i:intersectingBarriers)
				{
					MasonGeometry geoBarrier = (MasonGeometry) i;
					LineString l = utilities.LineStringBetweenNodes(originNode, destinationNode);
					Coordinate intersection = l.intersection(geoBarrier.getGeometry()).getCoordinate();
					double distanceIntersection = utilities.euclideanDistance(currentLocation.getCoordinate(),
							intersection);
					if (distanceIntersection > utilities.euclideanDistance(currentLocation.getCoordinate(), 
							destinationNode.getCoordinate())) continue;
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
					if ((positiveBarriers == null) || (positiveBarriers.contains(barrierID))) continue;
					double distanceEdge = utilities.euclideanDistance(currentLocation.getCoordinate(), 
							edge.getDual().getCoordinate());
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
				if ((utilities.nodesDistance(currentLocation, u)) < (utilities.nodesDistance(currentLocation, v)))
					subGoal = u;
				else subGoal = v;
			}
			sequence.add(subGoal);
			currentLocation = subGoal;
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
			EdgeGraph edge = Graph.getEdgeBetween(tmpOrigin, tmpDestination);
			if (edge != null)
			{
				if (!completePath.contains(((GeomPlanarGraphDirectedEdge) edge.getDirEdge(0))))
					completePath.add((GeomPlanarGraphDirectedEdge) edge.getDirEdge(0));
				tmpOrigin = tmpDestination;
				continue;
			}	
			
			int barrierID = 999999;
			if (barrierSubGoals.get(subGoal)!= null) barrierID = barrierSubGoals.get(subGoal);
			
			if (localHeuristic == "roadDistance") 
				{resultPartial = routePlanning.roadDistance(tmpOrigin, tmpDestination, completePath, false, 
						barrierID, "dijkstra", state).edges;}
			else 
			{
				NodeGraph pJ = null;
				ArrayList<NodeGraph> centroidsToAvoid = utilities.centroidsFromPath(completePath);
				if (completePath.size() >1) pJ = utilities.previousJunction(completePath, state);
				resultPartial = routePlanning.angularChange(tmpOrigin, tmpDestination, centroidsToAvoid, pJ, false,
					barrierID, "dijkstra", state).edges;
			}

			tmpOrigin = tmpDestination;
			completePath.addAll(resultPartial);
		}
		return completePath;	
	}
}

