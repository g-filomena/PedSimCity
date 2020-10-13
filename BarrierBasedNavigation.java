package sim.app.geo.pedSimCity;

import java.util.*;
import com.vividsolutions.jts.geom.*;

import sim.util.Bag;
import sim.util.geo.MasonGeometry;
import sim.app.geo.urbanSim.*;

public class BarrierBasedNavigation {

	NodeGraph currentLocation;
	HashMap<Integer, EdgeGraph> edgesMap;
	

	public ArrayList<NodeGraph> sequenceBarriers(NodeGraph originNode, NodeGraph destinationNode)
	{	
		this.edgesMap = PedSimCity.edgesMap;
		this.currentLocation = originNode;

		//sub-goals and barriers - navigation
		ArrayList<NodeGraph> sequence = new ArrayList<NodeGraph>();
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

			//look for orienting barriers
			for (int barrierID : intersectingBarriers)
			{
				MasonGeometry barrierGeometry = PedSimCity.barriersMap.get(barrierID);
				Geometry viewField = Utilities.viewField(currentLocation, destinationNode);
				Coordinate intersection = viewField.intersection(barrierGeometry.getGeometry()).getCoordinate();
				double distanceIntersection = Utilities.euclideanDistance(currentLocation.getCoordinate(), intersection);
				if (distanceIntersection > Utilities.euclideanDistance(currentLocation.getCoordinate(),	destinationNode.getCoordinate())) continue;
				possibleBarriers.put(barrierID, distanceIntersection);
			}
			
			if ((possibleBarriers.size() == 0) || (possibleBarriers == null)) break;
			
			//ordered by distance (further away first)
			LinkedHashMap<Integer, Double> validSorted = (LinkedHashMap<Integer, Double>) Utilities.sortByValue(possibleBarriers, "descending"); 
			ArrayList<Integer> withinBarriers = new ArrayList<Integer>();
			ArrayList<EdgeGraph> possibleEdgeGoals = new ArrayList<EdgeGraph>();
			
			for (int barrierID : validSorted.keySet())
			{
				MasonGeometry barrierGeometry = PedSimCity.barriersMap.get(barrierID);
				String type = barrierGeometry.getStringAttribute("type");
				ArrayList<EdgeGraph> alongEdges = PedSimCity.barriersEdgesMap.get(barrierID);
				HashMap<EdgeGraph, Double> thisBarrierEdgeGoals = new HashMap<EdgeGraph, Double>();
				
				for (EdgeGraph edge : alongEdges)
				{
					double distanceEdge = Utilities.euclideanDistance(currentLocation.getCoordinate(), edge.getCoordsCentroid());
					if (distanceEdge > Utilities.euclideanDistance(currentLocation.getCoordinate(), destinationNode.getCoordinate())) continue;
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
			
			if (possibleEdgeGoals.size() == 0 || possibleEdgeGoals.get(0) == null ) break;
			edgeGoal = possibleEdgeGoals.get(0);
			int barrier = withinBarriers.get(0);

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
		return sequence;
	}
	
	public static Set<Integer> intersectingBarriers(NodeGraph originNode, NodeGraph destinationNode, String type)
	{
		Geometry viewField = Utilities.viewField(originNode, destinationNode);
		Bag intersecting = PedSimCity.barriers.getIntersecting(viewField);
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

