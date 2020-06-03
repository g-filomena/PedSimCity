package sim.app.geo.pedestrianSimulation;

import java.util.*;
import java.util.Map.Entry;

import org.javatuples.Pair;

import com.vividsolutions.jts.geom.*;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.MasonGeometry;
import sim.app.geo.urbanSim.*;

public class RegionalRouting {
	
	ArrayList<Integer> visitedRegions = new ArrayList<Integer>();
	ArrayList<NodeGraph> sequence = new ArrayList<NodeGraph>();
	ArrayList<Pair<NodeGraph, NodeGraph>> badExits = new ArrayList<Pair<NodeGraph, NodeGraph>>();
	
	NodeGraph originNode;
	NodeGraph destinationNode;
	NodeGraph currentLocation;
	NodeGraph previousLocation;
	int currentRegion;
	int specificRegion;
	int targetRegion;
	boolean finalRegion = false;
	boolean barriersRouting = false;
	HashMap<Integer, EdgeGraph> edgesMap;
	HashMap<Pair<NodeGraph, NodeGraph>, GatewayData> gatewaysMap;
	

	public ArrayList<GeomPlanarGraphDirectedEdge> pathFinderRegions(NodeGraph originNode, 
			NodeGraph destinationNode, String localHeuristic, boolean barriersRouting)
	{	
		this.edgesMap = PedestrianSimulation.edgesMap;
		this.gatewaysMap = PedestrianSimulation.gatewaysMap;
		this.barriersRouting = barriersRouting;
		this.originNode = originNode;
	    this.destinationNode = destinationNode;
	    
		currentLocation = originNode;
		currentRegion = originNode.district;
		targetRegion = destinationNode.district;
		visitedRegions.add(currentRegion);
		previousLocation = null;
		sequence.add(originNode);
		// rough plan
		while (finalRegion == false)
		{
			specificRegion = 999999;
			Pair<NodeGraph, NodeGraph> gateways =  nextGateways(currentLocation, currentRegion, specificRegion);

  			if (gateways == null)
			{
				if (previousLocation != null)
				{
					Pair<NodeGraph, NodeGraph> badPair = new Pair <NodeGraph, NodeGraph> (sequence.get(sequence.size()-2), 
							sequence.get(sequence.size()-1));
					badExits.add(badPair); // add last exit
					currentLocation = previousLocation;
					currentRegion = previousLocation.district;
					sequence = new ArrayList<NodeGraph>(sequence.subList(0, sequence.size()-2));
					visitedRegions.remove(visitedRegions.size()-1);
					previousLocation = null;
					continue;
				}
					
		        if (localHeuristic == "roadDistance")
		        {
		        	return routePlanning.roadDistance(originNode, destinationNode, null, false, barriersRouting, "dijkstra").edges;
		        }
		        else if (localHeuristic == "angularChange")
		        {
		        	System.out.println("didn't formulate regional path");
		        	ArrayList<GeomPlanarGraphDirectedEdge> completePath =  new ArrayList<GeomPlanarGraphDirectedEdge>();
		        	completePath = routePlanning.angularChange(originNode, destinationNode, null, null, false, false, "dijkstra").edges;
		        	return completePath;
		        }
			}	
  			
  			previousLocation = currentLocation;
  			sequence.add(gateways.getValue0());
  			sequence.add(gateways.getValue1());
  			currentLocation = gateways.getValue1();
			currentRegion = currentLocation.district;
			visitedRegions.add(currentRegion);
			
			if (currentRegion == targetRegion)
			{
				finalRegion = true;
				sequence.add(destinationNode);
				break;
			}		
		}
		
		visitedRegions.clear();
		badExits.clear();
		
		ArrayList<GeomPlanarGraphDirectedEdge> completePath =  new ArrayList<GeomPlanarGraphDirectedEdge>();
		//sub-goals and barriers - navigation
		ArrayList<NodeGraph> newSequence = new ArrayList<NodeGraph>();

		if (barriersRouting)
		{
			ArrayList<Integer> adjacentBarriers = new ArrayList<Integer>();
			ArrayList<NodeGraph> toIgnore = new ArrayList<NodeGraph>();
			
			for (NodeGraph gateway: sequence)
			{
				if (toIgnore.contains(gateway)) continue;
				newSequence.add(gateway);
				int indexOf = sequence.indexOf(gateway);
				if ((indexOf > 0) && (indexOf%2 != 0)) continue; //continue when exit gateways and destination
				
				//check if there are good barriers in line of movement
				Set<Integer> intersectingBarriers = BarriersRouting.intersectingBarriers(gateway, destinationNode, "all");
				if (intersectingBarriers.size() == 0) continue;
				ArrayList<EdgeGraph> incomingEdges = gateway.getEdges();
				for (EdgeGraph edge : incomingEdges) if (edge.barriers != null) adjacentBarriers.addAll(edge.barriers);
				
				DistrictData district = PedestrianSimulation.districtsMap.get(gateway.district);
				intersectingBarriers.retainAll(new HashSet<Integer>(district.primalGraph.getBarriersGraph()));
				if (intersectingBarriers.size() == 0) continue;
				
				Set<Integer> visitedBarriers = new HashSet<Integer>(adjacentBarriers);
				intersectingBarriers.removeAll(visitedBarriers);
				if (intersectingBarriers.size() == 0) continue;
				
				NodeGraph subGoal = null;
				EdgeGraph edgeGoal = null;
				HashMap<Integer, Double> possibleBarriers = new HashMap<Integer, Double> ();
				//look for orienting barriers
				
				for (int barrierID : intersectingBarriers)
				{
					MasonGeometry barrierGeometry = PedestrianSimulation.barriersMap.get(barrierID);
					Geometry viewField = Utilities.viewField(gateway, destinationNode);
					Coordinate intersection = viewField.intersection(barrierGeometry.getGeometry()).getCoordinate();
					double distanceIntersection = Utilities.euclideanDistance(gateway.getCoordinate(), intersection);
					if (distanceIntersection > Utilities.euclideanDistance(gateway.getCoordinate(),	destinationNode.getCoordinate())) continue;
					possibleBarriers.put(barrierID, distanceIntersection);
				}
				if ((possibleBarriers.size() == 0) || (possibleBarriers == null)) continue;

				//ordered by distance (further away first)
				LinkedHashMap<Integer, Double> validSorted = (LinkedHashMap<Integer, Double>) Utilities.sortByValue(possibleBarriers, "descending"); 
				ArrayList<EdgeGraph> districtEdges = district.primalGraph.getParentEdges(district.primalGraph.getEdges());
				
				ArrayList<Integer> withinBarriers = new ArrayList<Integer>();
				ArrayList<EdgeGraph> possibleEdgeGoals = new ArrayList<EdgeGraph>();
				
				for (int barrierID : validSorted.keySet())
				{
					MasonGeometry barrierGeometry = PedestrianSimulation.barriersMap.get(barrierID);
					String type = barrierGeometry.getStringAttribute("type");
					HashMap<EdgeGraph, Double> thisBarrierEdgeGoals = new HashMap<EdgeGraph, Double>();
					ArrayList<EdgeGraph> alongEdges = PedestrianSimulation.barriersEdgesMap.get(barrierID);
					alongEdges.retainAll(districtEdges);
					for (EdgeGraph edge: alongEdges)
					{
						double distanceEdge = Utilities.euclideanDistance(gateway.getCoordinate(), edge.getCoordsCentroid());
						if (distanceEdge > Utilities.euclideanDistance(gateway.getCoordinate(), destinationNode.getCoordinate())) continue;
						thisBarrierEdgeGoals.put(edge, distanceEdge);
					}
					if (thisBarrierEdgeGoals.size() == 0) continue; //doensn't have decent edges around
					
					LinkedHashMap<EdgeGraph, Double> thisBarrierSubGoalSorted = (LinkedHashMap<EdgeGraph, Double>) 
							Utilities.sortByValue(thisBarrierEdgeGoals, "ascending");
					EdgeGraph possibleEdgeGoal = thisBarrierSubGoalSorted.keySet().iterator().next();
					int waterCounter = 0;
					int parkCounter = 0;
					
					if (type.equals("water"))
					{
						withinBarriers.add(waterCounter, barrierID);
						possibleEdgeGoals.add(waterCounter, possibleEdgeGoal);
						waterCounter += 1;
						parkCounter = waterCounter +1;
					}
					else if (type.equals("park"))
					{
						withinBarriers.add(parkCounter, barrierID);
						possibleEdgeGoals.add(parkCounter, possibleEdgeGoal);
						parkCounter += 1;
					}
					else
					{
						withinBarriers.add(barrierID);
						possibleEdgeGoals.add(possibleEdgeGoal);
					}				
				}
				
				if ((possibleEdgeGoals.size() == 0) || (possibleEdgeGoals.get(0) == null )) continue;
					
				edgeGoal = possibleEdgeGoals.get(0);
				int barrier = withinBarriers.get(0);

				NodeGraph u = edgeGoal.u;
				NodeGraph v = edgeGoal.v;
				if ((Utilities.nodesDistance(gateway, u)) < (Utilities.nodesDistance(gateway, v))) subGoal = u;
				else subGoal = v;
				adjacentBarriers.add(barrier);
				if (sequence.contains(subGoal)) continue;				
				newSequence.add(subGoal);
				if (indexOf == sequence.size()-2) continue;
				int desiredRegion = sequence.get(indexOf+2).district;
				
				// if subGoal is a newGateway
				if ((subGoal.gateway) && (subGoal.adjacentRegions.contains(desiredRegion)))
				{
					// no need to go through another gateway
					// ignore next exit
					toIgnore.add(sequence.get(indexOf+1));
					double deviation = Double.MAX_VALUE;
					NodeGraph bestEntry = null;
					for (NodeGraph entry : subGoal.adjacentEntries)
					{
						if (entry.district != desiredRegion) continue;
						double entryAngle = Angle.angle(subGoal, entry);
						if (entryAngle < deviation)
						{
							deviation = entryAngle;
							bestEntry = entry;
						}
					}
					// replace entry
					sequence.set(indexOf+2, bestEntry);
				}
				
				// pick a better gateway
				else 
				{
					Pair<NodeGraph, NodeGraph> newGateways = nextGateways(subGoal, subGoal.district, desiredRegion);
					if (newGateways == null) continue;
					sequence.set(indexOf+1, newGateways.getValue0());
					sequence.set(indexOf+2, newGateways.getValue1());
				}
			}
		}
		else newSequence = sequence;
		
		if (localHeuristic == "angularChange") completePath = routePlanning.angularChangeSequence(newSequence, true, barriersRouting, "dijkstra");
		else System.out.println("wrongHeuristic");
		return completePath;
	}
	
	
	public Pair<NodeGraph, NodeGraph> nextGateways(NodeGraph currentLocation, int currentRegion, int specificRegion)
	{
		//retrieve current region's exits
		ArrayList<GatewayData> possibleGates = PedestrianSimulation.districtsMap.get(currentRegion).gateways;
		HashMap<Pair<NodeGraph, NodeGraph>, Double> validGates = new HashMap<Pair<NodeGraph, NodeGraph>, Double> ();
		HashMap<Pair<NodeGraph, NodeGraph>, Double> otherGates = new HashMap<Pair<NodeGraph, NodeGraph>, Double> ();
	
		double destinationAngle = Angle.angle(currentLocation, destinationNode);
		double distanceTarget = Utilities.nodesDistance(currentLocation, destinationNode);
		if (barriersRouting & BarriersRouting.intersectingBarriers(currentLocation, destinationNode, "separating").size()>0) 
			distanceTarget = distanceTarget*Utilities.fromNormalDistribution(1, 0.10, "right");
		
		for (GatewayData gd : possibleGates)
		{	
			if (gd.node == currentLocation) continue;
			if (badExits.contains(gd.gatewayID)) continue;
			if ((specificRegion != 999999) && (specificRegion != gd.regionTo)) continue;
			if (visitedRegions.contains(gd.regionTo)) continue;

			double exitAngle = Angle.angle(currentLocation, gd.node);
			double distanceFromGate = Utilities.nodesDistance(currentLocation, gd.node);
			if (barriersRouting & BarriersRouting.intersectingBarriers(currentLocation, gd.node, "separating").size()>0)
				distanceFromGate = distanceFromGate*Utilities.fromNormalDistribution(1, 0.20, "right");
			
			if ((distanceFromGate > distanceTarget) ||
				(Angle.isInDirection(destinationAngle, exitAngle, 140) == false) || 
				(Angle.isInDirection(destinationAngle, gd.entryAngle, 140) == false))
			{
				double cost = Angle.differenceAngles(exitAngle, destinationAngle); 
				if (cost > 95) continue;
				cost += Angle.differenceAngles(exitAngle, gd.entryAngle);
				otherGates.put(gd.gatewayID, cost);
				continue;
			}
			double cost = Angle.differenceAngles(exitAngle, destinationAngle);
			cost += Angle.differenceAngles(exitAngle, gd.entryAngle);   
			validGates.put(gd.gatewayID, cost);
		}
		if ((validGates.size() == 0) && (specificRegion != 999999)) return null;
		if (validGates.size() == 0) validGates = otherGates;
		if (validGates.size() == 0) return null;
		
		LinkedHashMap<Pair<NodeGraph, NodeGraph>, Double> validSorted = (LinkedHashMap<Pair<NodeGraph, NodeGraph>, Double>) 
				Utilities.sortByValue(validGates, "ascending"); 
		
		// sequence formulation
		for (Entry<Pair<NodeGraph, NodeGraph>, Double> gatewaysPair: validSorted.entrySet())
		{
			return gatewaysPair.getKey();
		}
		return null;

	}
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
}

