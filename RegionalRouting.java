package sim.app.geo.pedestrianSimulation;

import java.util.*;
import java.util.Map.Entry;

import com.vividsolutions.jts.geom.*;
import sim.field.geo.GeomVectorField;
import sim.util.Bag;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.MasonGeometry;

public class RegionalRouting {
	
	ArrayList<Integer> visited = new ArrayList<Integer>();
	ArrayList<NodeGraph> sequence = new ArrayList<NodeGraph>();
	ArrayList<Integer> sequenceGateways = new ArrayList<Integer>();
	ArrayList<Integer> badExits = new ArrayList<Integer>();
	NodeGraph originNode;
	NodeGraph destinationNode;
	NodeGraph currentLocation;
	NodeGraph previousLocation;
	int currentRegion;
	int targetRegion;
	boolean found = false;
	
	HashMap<Integer,EdgeGraph> edgesMap;
	HashMap<Integer, GatewayData> gatewaysMap;
	

	public ArrayList<GeomPlanarGraphDirectedEdge> pathFinderRegions(NodeGraph originNode, 
			NodeGraph destinationNode, String localHeuristic, boolean usingBarriers, PedestrianSimulation state)
	{	
		
		this.edgesMap = state.edgesMap;
		this.gatewaysMap = state.gatewaysMap;
		
		this.originNode = originNode;
	    this.destinationNode = destinationNode;
	    
		currentLocation = originNode;
		currentRegion = currentLocation.district;
		targetRegion = destinationNode.district;
		visited.add(currentRegion);
		sequence.add(currentLocation);
		sequenceGateways.add(currentLocation.getID());
		badExits.clear();
		previousLocation = null;
		
		// rough plan
		while (found == false)
		{
			ArrayList<GatewayData> possibleGates = state.districtsMap.get(currentRegion).gateways;
			HashMap<Integer, Double> validGates = new HashMap<Integer, Double> ();
			HashMap<Integer, Double> otherGates = new HashMap<Integer, Double> ();
			
			double destinationAngle = utilities.angle(currentLocation.getCoordinate(), destinationNode.getCoordinate());
			double distanceTarget = utilities.nodesDistance(currentLocation, destinationNode);
			if (usingBarriers & landmarkFunctions.intersectingBarriers(currentLocation, destinationNode, 
					"separating").size()>0) distanceTarget = distanceTarget*1.10;
			
			for (GatewayData gd : possibleGates)
			{	
				if (gd.node == currentLocation) continue;
				if (badExits.contains(gd.gatewayID)) continue;
				if (visited.contains(gd.regionTo)) continue;
				double gateAngle = utilities.angle(currentLocation.getCoordinate(), gd.node.getCoordinate());
				double distanceFromGate = utilities.nodesDistance(gd.node, destinationNode);
				if (usingBarriers & landmarkFunctions.intersectingBarriers(gd.node, destinationNode, "separating").size()>0)
					distanceFromGate = distanceFromGate*1.10;
				
				if ((distanceFromGate > distanceTarget) ||
					(utilities.isInDirection(destinationAngle, gateAngle, 140) == false) || 
					(utilities.isInDirection(destinationAngle, gd.entryAngle, 140) == false))
					{
						double cost = utilities.differenceAngles(gateAngle, destinationAngle); 
						if (cost > 95) continue;
						cost += utilities.differenceAngles(gateAngle, gd.entryAngle);
						otherGates.put(gd.gatewayID, cost);
						continue;
					}
				double cost = utilities.differenceAngles(gateAngle, destinationAngle);
				cost += utilities.differenceAngles(gateAngle, gd.entryAngle);   
				validGates.put(gd.gatewayID, cost);
			}
			
			if (validGates.size() == 0) validGates = otherGates;
			if (validGates.size() == 0) 
			{
				if (previousLocation != null)
					{
						badExits.add(sequenceGateways.get(sequenceGateways.size()-2));
						currentLocation = previousLocation;
						currentRegion = previousLocation.district;
						sequence.remove(sequence.size() - 1);
						sequence.remove(sequence.size() - 1);
						sequenceGateways.remove(sequenceGateways.size()-1);
						sequenceGateways.remove(sequenceGateways.size()-1);
						visited.remove(visited.size()-1);
						previousLocation = null;
						continue;
					}
					
		        if (localHeuristic == "roadDistance")
		        {
		        	return routePlanning.roadDistance(originNode, destinationNode, null, 
		        			false, 999999, "dijkstra", state).edges;
		        }
			        
		        else if (localHeuristic == "angularChange")
		        {
		        	ArrayList<GeomPlanarGraphDirectedEdge> completePath =  new ArrayList<GeomPlanarGraphDirectedEdge>();
		        	completePath = routePlanning.angularChange(originNode, destinationNode, null, null, 
		        			false, 999999, "dijkstra", state).edges;
		        	return completePath;
		        }
					
			}			
			Map<Integer, Double> validSorted = utilities.sortByValue(validGates); 
			Iterator<Entry<Integer, Double>> it = validSorted.entrySet().iterator();
			// sequence formulation
			
			while (it.hasNext())
			{
				Map.Entry<Integer, Double> pair = (Map.Entry<Integer, Double>)it.next();
				Integer exitID = pair.getKey();
				NodeGraph exitNode = gatewaysMap.get(exitID).node;
			  	NodeGraph entryNode = gatewaysMap.get(exitID).entry;
			  	sequenceGateways.add(exitID);
			  	sequenceGateways.add(entryNode.getID());
			  	
	  			visited.add(entryNode.district);
	  			previousLocation = currentLocation;
	  			currentLocation = entryNode;
	  			currentRegion = entryNode.district;
	  			sequence.add(exitNode);
	  			sequence.add(entryNode);
	  			break;
			}
			
			if (currentRegion == targetRegion)
			{
				found = true;
				sequence.add(destinationNode);
				sequenceGateways.add(null);
				break;
			}		
		}

		ArrayList<GeomPlanarGraphDirectedEdge> completePath =  new ArrayList<GeomPlanarGraphDirectedEdge>();
		//sub-goals and barriers - navigation
		ArrayList<NodeGraph> newSequence = new ArrayList<NodeGraph>();
		HashMap<NodeGraph, Integer> barrierSubGoals = new HashMap<NodeGraph, Integer>();

		if (usingBarriers)
		{
			for (NodeGraph gateway: sequence)
			{
				int indexOf = sequence.indexOf(gateway);
				newSequence.add(gateway);
				if (indexOf == sequence.size()-1) break;
				if ((indexOf > 0) & (indexOf%2 != 0)) continue; //entry gateway,
				
				//check if there are good barriers in line of movement
				Bag intersectingBarriers = landmarkFunctions.intersectingBarriers(gateway,
						destinationNode, "positive");
				
				MasonGeometry farthest = null;
				NodeGraph subGoal = null;
				EdgeGraph edgeGoal = null;
				
				if (intersectingBarriers.size() >= 1)
				{
					//look for orienting barriers
					NodeGraph nextExit = sequence.get(indexOf+1);
					double distance = 0.0;
					
					for (Object i:intersectingBarriers)
					{
						MasonGeometry geoBarrier = (MasonGeometry) i;
						LineString l = utilities.LineStringBetweenNodes(originNode, destinationNode);
						Coordinate intersection = l.intersection(geoBarrier.getGeometry()).getCoordinate();
						double distanceIntersection = utilities.euclideanDistance(gateway.getCoordinate(), 
								intersection);
						if (distanceIntersection > utilities.euclideanDistance(gateway.getCoordinate(), 
								nextExit.getCoordinate())) continue;
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
					GeomVectorField filteryByDistricts = utilities.filterGeomVectorField(
							PedestrianSimulation.roads, "district",	gateway.district, "equal");
					
					Bag possibleRoads = filteryByDistricts.getGeometries();
					double distance = Double.MAX_VALUE;
					for (Object road: possibleRoads)
					{
						MasonGeometry eg = (MasonGeometry) road;
						Integer edgeID = eg.getIntegerAttribute("edgeID");
						List<Integer> positiveBarriers = edgesMap.get(edgeID).positiveBarriers;
						if ((positiveBarriers == null) || (positiveBarriers.contains(barrierID))) continue;
						double distanceEdge = utilities.euclideanDistance(gateway.getCoordinate(), 
								eg.geometry.getCentroid().getCoordinate());
						if (distanceEdge < distance) edgeGoal = edgesMap.get(edgeID);
					}
				}
				if (edgeGoal == null) continue;
				else
				{
					NodeGraph u = edgeGoal.u;
					NodeGraph v = edgeGoal.v;
					if ((utilities.nodesDistance(gateway, u)) < (utilities.nodesDistance(gateway, v))) 
						subGoal = u;
					else subGoal = v;
				}
				newSequence.add(subGoal);
				barrierSubGoals.put(subGoal, farthest.getIntegerAttribute("barrierID"));
			}
		}
		else newSequence = sequence;
		
		NodeGraph tmpOrigin = null;
		NodeGraph tmpDestination = null;
		
		for (NodeGraph subGoal : newSequence)
		{
			int indexOf = newSequence.indexOf(subGoal);	
			ArrayList<GeomPlanarGraphDirectedEdge> resultPartial =  new ArrayList<GeomPlanarGraphDirectedEdge>();
			
			if (indexOf == 0) //start
			{
				tmpOrigin = subGoal;
				continue;
			}
			else tmpDestination = subGoal; // actual tmp destination
			EdgeGraph edge = Graph.getEdgeBetween(tmpOrigin, tmpDestination);
			if (edge != null)
			{
				if (!completePath.contains(((GeomPlanarGraphDirectedEdge) edge.getDirEdge(0))))
					completePath.add((GeomPlanarGraphDirectedEdge) edge.getDirEdge(0));
				tmpOrigin = tmpDestination;
				continue;
			}	
			int barrierID = 999999;
			if (barrierSubGoals.get(subGoal)!= null)
			{
				barrierID = barrierSubGoals.get(subGoal);
			}
			
			
			if (localHeuristic == "roadDistance") 
				{resultPartial = routePlanning.roadDistance(tmpOrigin, tmpDestination, null, 
						true, barrierID, "dijkstra", state).edges;}
			else 
			{
				ArrayList<NodeGraph> centroidsToAvoid = utilities.centroidsFromPath(completePath);
				NodeGraph pJ = null;
				if (completePath.size() >1) pJ = utilities.previousJunction(completePath, state); 
				resultPartial = routePlanning.angularChange(tmpOrigin, tmpDestination, centroidsToAvoid, pJ, true,
						barrierID, "dijkstra", state).edges;
			}

			tmpOrigin = tmpDestination;
			completePath.addAll(resultPartial);
		}
		return completePath;	
	}
}

