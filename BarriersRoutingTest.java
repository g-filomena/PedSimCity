package sim.app.geo.pedestrianSimulation;

import java.util.*;
import com.vividsolutions.jts.geom.*;

import sim.util.Bag;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.MasonGeometry;
import sim.app.geo.urbanSim.*;

public class BarriersRoutingTest {

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
//		HashMap<NodeGraph, Integer> barrierSubGoals = new HashMap<NodeGraph, Integer>();
		sequence.add(originNode);
		currentLocation = originNode;
		ArrayList<Integer> adjacentBarriers = new ArrayList<Integer>();
		
		while (true)
		{
			//check if there are good barriers in line of movement
			Set<Integer> intersectingBarriers = intersectingBarriers(currentLocation, destinationNode, "all");
			if (intersectingBarriers.size() == 0) break;

			ArrayList<EdgeGraph> incomingEdges = currentLocation.getEdges();
			for (EdgeGraph edge : incomingEdges) if (edge.barriers != null) adjacentBarriers.addAll(edge.barriers);
			Set<Integer> visitedBarriers = new HashSet<Integer>(adjacentBarriers);
			intersectingBarriers.removeAll(visitedBarriers);
			if (intersectingBarriers.size() == 0) break;

			NodeGraph subGoal = null;
			EdgeGraph edgeGoal = null;
			
			HashMap<Integer, Double> possibleBarriers = new HashMap<Integer, Double> ();
			boolean positiveBarriers = false;

			//look for orienting barriers
			for (int barrierID : intersectingBarriers)
			{
				MasonGeometry barrierGeometry = PedestrianSimulation.barriersMap.get(barrierID);
				Geometry viewField = Utilities.viewField(currentLocation, destinationNode);
				Coordinate intersection = viewField.intersection(barrierGeometry.getGeometry()).getCoordinate();
				double distanceIntersection = Utilities.euclideanDistance(currentLocation.getCoordinate(), intersection);
				if (distanceIntersection > Utilities.euclideanDistance(currentLocation.getCoordinate(),	destinationNode.getCoordinate())) continue;
				possibleBarriers.put(barrierID, distanceIntersection);
				if ((barrierGeometry.getStringAttribute("type").equals("water")) || (barrierGeometry.getStringAttribute("type").equals("park"))) 
					positiveBarriers = true;
			}
			
			if ((possibleBarriers.size() == 0) || (possibleBarriers == null)) break;
			
			//ordered by distance (further away first)
			LinkedHashMap<Integer, Double> validSorted = (LinkedHashMap<Integer, Double>) Utilities.sortByValue(possibleBarriers, "descending"); 
			EdgeGraph replacement = null;
			int barrier = 999999;
			int replacementBarrier = 999999;
			
			for (int barrierID : validSorted.keySet())
			{

				MasonGeometry barrierGeometry = PedestrianSimulation.barriersMap.get(barrierID);
				String type = barrierGeometry.getStringAttribute("type");
				
				for (EdgeGraph edge : edgesMap.values())
				{
					List<Integer> edgeBarriers = edge.barriers;
					if ((edgeBarriers == null) || (!edgeBarriers.contains(barrierID))) continue;
					double distanceEdge = Utilities.euclideanDistance(currentLocation.getCoordinate(), edge.getCoordsCentroid());
					if (distanceEdge > Utilities.euclideanDistance(currentLocation.getCoordinate(), destinationNode.getCoordinate())) continue;
				
					if ((positiveBarriers) && ((type.equals("water")) || (type.equals("park")))) 
					{
						barrier = barrierID;
						edgeGoal = edge;
						break;
					}
					//but this one is not
					else if ((positiveBarriers) && (replacement == null)) 
					{
						replacement = edge;
						barrier = replacementBarrier;
					}
					else if (!positiveBarriers)
					{
						barrier = barrierID;
						edgeGoal = edge;
						break;
					}
					else continue;
				}	
			}
			
			if (edgeGoal == null) 
			{
				edgeGoal = replacement;
				barrier = replacementBarrier;
			}
			if (edgeGoal == null) break;
			else
			{
				NodeGraph u = edgeGoal.u;
				NodeGraph v = edgeGoal.v;
				if ((Utilities.nodesDistance(currentLocation, u)) < (Utilities.nodesDistance(currentLocation, v))) subGoal = u;
				else subGoal = v;
			}
			sequence.add(subGoal);
			currentLocation = subGoal;
			adjacentBarriers.add(barrier);
		}
		sequence.add(destinationNode);
		if (localHeuristic == "angularChange") completePath = routePlanning.angularChangeSequence(sequence, false, true, "dijkstra");
		else System.out.println("wrongHeuristic");
		return completePath;
	}
	
	public static Set<Integer> intersectingBarriers(NodeGraph originNode, NodeGraph destinationNode, String type)
	{
		Geometry viewField = Utilities.viewField(originNode, destinationNode);
		Bag intersecting = PedestrianSimulation.barriers.getIntersecting(viewField);
    	Set<Integer> intersectingBarriers = new HashSet<Integer>();
    	ArrayList<MasonGeometry> intersectingGeometries = new ArrayList<MasonGeometry>();

		for (Object iB : intersecting)
		{
			MasonGeometry geoBarrier = (MasonGeometry) iB;
			String barrierType = geoBarrier.getStringAttribute("type");

			if (type.equals("all")) intersectingGeometries.add(geoBarrier);
			else if ((type.equals("positive")) & (barrierType.equals("park")) || (barrierType.equals("water")))
				intersectingGeometries.add(geoBarrier);
			else if ((type.equals("negative")) && (barrierType.equals("railway")) || (barrierType.equals("road")))
				intersectingGeometries.add(geoBarrier);
			else if ((type.equals("separating")) && (!barrierType.equals("parks"))) intersectingGeometries.add(geoBarrier);
			else if (type == barrierType) intersectingGeometries.add(geoBarrier);
		} 
		for (MasonGeometry i: intersectingGeometries) intersectingBarriers.add(i.getIntegerAttribute("barrierID"));
		return intersectingBarriers;
	}
}

