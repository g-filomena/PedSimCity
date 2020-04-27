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
	    
	    sequence.add(originNode);
		currentLocation = originNode;
		currentRegion = originNode.district;
		targetRegion = destinationNode.district;
		visitedRegions.add(currentRegion);
		previousLocation = null;
		
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
		        	return routePlanning.roadDistance(originNode, destinationNode, null, false, barriersRouting, 999999, "dijkstra").edges;
		        }
		        else if (localHeuristic == "angularChange")
		        {
		        	ArrayList<GeomPlanarGraphDirectedEdge> completePath =  new ArrayList<GeomPlanarGraphDirectedEdge>();
		        	completePath = routePlanning.angularChange(originNode, destinationNode, null, null, false, barriersRouting, 999999, "dijkstra").edges;
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
		HashMap<NodeGraph, Integer> barrierSubGoals = new HashMap<NodeGraph, Integer>();

		if (barriersRouting)
		{
			ArrayList<Integer> pBarriers = new ArrayList<Integer>();
			ArrayList<NodeGraph> toIgnore = new ArrayList<NodeGraph>();
			for (NodeGraph gateway: sequence)
			{
				if (toIgnore.contains(gateway)) continue;
				newSequence.add(gateway);
				int indexOf = sequence.indexOf(gateway);
				if (indexOf == sequence.size()-2) continue; // entry before destination
				if ((indexOf > 0) && (indexOf%2 != 0)) continue; //continue when exit gateways and destination
				
				//check if there are good barriers in line of movement
				ArrayList<MasonGeometry> intersectingBarriers = BarriersRouting.intersectingBarriers(gateway, destinationNode, "positive");
				MasonGeometry farthest = null;
				NodeGraph subGoal = null;
				EdgeGraph edgeGoal = null;
				ArrayList<EdgeGraph> incomingEdges = gateway.getEdgesNode();
				for (EdgeGraph edge : incomingEdges) if (edge.positiveBarriers != null) pBarriers.addAll(edge.positiveBarriers);
				Set<Integer> adjacentBarriers = new HashSet<Integer>(pBarriers);

				if (intersectingBarriers.size() >= 1)
				{
					//look for orienting barriers
					NodeGraph nextEntry = sequence.get(indexOf+2);
					double distance = 0.0;
					
					for (Object i:intersectingBarriers)
					{
						MasonGeometry geoBarrier = (MasonGeometry) i;
						if (adjacentBarriers.contains(geoBarrier.getIntegerAttribute("barrierID"))) continue;
						
						Geometry viewField = utilities.viewField(gateway, destinationNode);
						Coordinate intersection = viewField.intersection(geoBarrier.getGeometry()).getCoordinate();
						double distanceIntersection = utilities.euclideanDistance(gateway.getCoordinate(), intersection);
						if (distanceIntersection > utilities.euclideanDistance(gateway.getCoordinate(),	nextEntry.getCoordinate())) continue;
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
					DistrictData dd = PedestrianSimulation.districtsMap.get(gateway.district);
					ArrayList<EdgeGraph> districtEdges = dd.primalGraph.getEdges();
					double distance = Double.MAX_VALUE;
					for (Object o: districtEdges)
					{
						EdgeGraph edge =  dd.primalGraph.getParentEdge((EdgeGraph) o);
						List<Integer> positiveBarriers = edge.positiveBarriers;
						if ((positiveBarriers == null) || (!positiveBarriers.contains(barrierID))) continue;
						double distanceEdge = utilities.euclideanDistance(gateway.getCoordinate(), edge.getCoordsCentroid());
						if (distanceEdge < distance)
						{
							distance = distanceEdge;
							edgeGoal = edge;
						}
					}
				}
				if (edgeGoal == null) continue;
				else
				{
					NodeGraph u = edgeGoal.u;
					NodeGraph v = edgeGoal.v;
					if ((utilities.nodesDistance(gateway, u)) < (utilities.nodesDistance(gateway, v))) subGoal = u;
					else subGoal = v;
					if (sequence.contains(subGoal)) continue;
				}
				newSequence.add(subGoal);
				pBarriers.add(farthest.getIntegerAttribute("barrierID"));
				barrierSubGoals.put(subGoal, farthest.getIntegerAttribute("barrierID"));
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
					if (newGateways == null) 
						
						{
						System.out.println("newGateways wierd "+subGoal.getID()+ " "+destinationNode.getID());
						continue; //no change then
						}
					sequence.set(indexOf+1, newGateways.getValue0());
					sequence.set(indexOf+2, newGateways.getValue1());
				}
			}
		}
		else newSequence = sequence;
		
		NodeGraph tmpOrigin = null;
		NodeGraph tmpDestination = null;
		
		for (NodeGraph subGoal : newSequence)
		{
			int indexOf = newSequence.indexOf(subGoal);	
			ArrayList<GeomPlanarGraphDirectedEdge> resultPartial =  new ArrayList<GeomPlanarGraphDirectedEdge>();
			boolean towardsBarrier = false;
			
			if (indexOf == 0) //start
			{
				tmpOrigin = subGoal;
				continue;
			}
			else tmpDestination = subGoal; // actual tmp destination
			
			EdgeGraph edge = tmpOrigin.getEdgeBetween(tmpDestination);
			if (edge != null)
			{
				if (!completePath.contains(((GeomPlanarGraphDirectedEdge) edge.getDirEdge(0))))
					completePath.add((GeomPlanarGraphDirectedEdge) edge.getDirEdge(0));
				tmpOrigin = tmpDestination;
				continue;
			}	
			int barrierID = 999999;
			if (barrierSubGoals.get(tmpOrigin) != null) barrierID = barrierSubGoals.get(tmpOrigin);
			if (barrierSubGoals.get(tmpDestination)!= null) towardsBarrier = true;
			
			if (localHeuristic == "roadDistance") 
				{resultPartial = routePlanning.roadDistance(tmpOrigin, tmpDestination, null, true, barriersRouting, barrierID, "dijkstra").edges;}
			else 
			{
				ArrayList<NodeGraph> centroidsToAvoid = utilities.centroidsFromPath(completePath);
				boolean next = false;
				if (towardsBarrier)
				{
					NodeGraph nodeToAvoid = newSequence.get(newSequence.indexOf(subGoal)+1);
					

					for (EdgeGraph incomingEdge : nodeToAvoid.getEdgesNode()) 
					{
						if (incomingEdge.getOtherNode(tmpDestination) == nodeToAvoid)
						{
							// tmpOrigin = tmpOrigin;
							next = true;
							break;
						}
						centroidsToAvoid.add(incomingEdge.getDual());
					}
				}
				if (next) continue; //keep the same tmpOrigin, ignore this barrier-subgoal as it's at the same edge with the next gateway anyway
				
				NodeGraph pJ = null;
				resultPartial = routePlanning.angularChange(tmpOrigin, tmpDestination, centroidsToAvoid, pJ, true, barriersRouting,
						barrierID, "dijkstra").edges;
			}

			tmpOrigin = tmpDestination;
			completePath.addAll(resultPartial);
		}
		return completePath;	
	}
	
	
	public Pair<NodeGraph, NodeGraph> nextGateways(NodeGraph currentLocation, int currentRegion, int specificRegion)
	{
		ArrayList<GatewayData> possibleGates = PedestrianSimulation.districtsMap.get(currentRegion).gateways;
		HashMap<Pair<NodeGraph, NodeGraph>, Double> validGates = new HashMap<Pair<NodeGraph, NodeGraph>, Double> ();
		HashMap<Pair<NodeGraph, NodeGraph>, Double> otherGates = new HashMap<Pair<NodeGraph, NodeGraph>, Double> ();
	
		double destinationAngle = Angle.angle(currentLocation, destinationNode);
		double distanceTarget = utilities.nodesDistance(currentLocation, destinationNode);
		if (barriersRouting & BarriersRouting.intersectingBarriers(currentLocation, destinationNode, "separating").size()>0) 
			distanceTarget = distanceTarget*1.10;

		for (GatewayData gd : possibleGates)
		{	
			if (gd.node == currentLocation) continue;
			if (badExits.contains(gd.gatewayID)) continue;
			if ((specificRegion != 999999) && (specificRegion != gd.regionTo)) continue;
			if (visitedRegions.contains(gd.regionTo)) continue;

			double gateAngle = Angle.angle(currentLocation, gd.node);
			double distanceFromGate = utilities.nodesDistance(gd.node, destinationNode);
			
			if (barriersRouting & BarriersRouting.intersectingBarriers(gd.node, destinationNode, "separating").size()>0)
				distanceFromGate = distanceFromGate*utilities.fromNormalDistribution(1, 0.20, "right");
			
			if ((distanceFromGate > distanceTarget) ||
				(Angle.isInDirection(destinationAngle, gateAngle, 140) == false) || 
				(Angle.isInDirection(destinationAngle, gd.entryAngle, 140) == false))
			{
				double cost = Angle.differenceAngles(gateAngle, destinationAngle); 
				if (cost > 95) continue;
				cost += Angle.differenceAngles(gateAngle, gd.entryAngle);
				otherGates.put(gd.gatewayID, cost);
				continue;
			}
			double cost = Angle.differenceAngles(gateAngle, destinationAngle);
			cost += Angle.differenceAngles(gateAngle, gd.entryAngle);   
			validGates.put(gd.gatewayID, cost);
		}
		if ((validGates.size() == 0) && (specificRegion != 999999)) return null;
		if (validGates.size() == 0) validGates = otherGates;
		if (validGates.size() == 0) return null;
		
		LinkedHashMap<Pair<NodeGraph, NodeGraph>, Double> validSorted = (LinkedHashMap<Pair<NodeGraph, NodeGraph>, Double>) 
				utilities.sortByValue(validGates); 
		
		// sequence formulation
		for (Entry<Pair<NodeGraph, NodeGraph>, Double> gatewaysPair: validSorted.entrySet())
		{
			return gatewaysPair.getKey();
		}
		return null;

	}
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
}

