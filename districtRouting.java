package sim.app.geo.pedestrianSimulation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

import com.vividsolutions.jts.planargraph.DirectedEdgeStar;
import com.vividsolutions.jts.planargraph.Node;
import com.vividsolutions.jts.planargraph.Subgraph;
import sim.util.geo.GeomPlanarGraph;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.GeomPlanarGraphEdge;

public class districtRouting {
	ArrayList<Integer> visited = new ArrayList<Integer>();
	ArrayList<Node> sequence = new ArrayList<Node>();
	ArrayList<Integer> sequenceGateways = new ArrayList<Integer>();
	ArrayList<Integer> sequenceNodes = new ArrayList<Integer>();
	ArrayList<Integer> badExits = new ArrayList<Integer>();
	Node originNode;
	Node destinationNode;
	Node currentLocation;
	Node previousLocation;
	int currentRegion;
	int targetRegion;
	boolean found = false;
	
	HashMap<Integer,EdgeData> edgesMap;
	HashMap<Integer,NodeData> nodesMap;
	HashMap<Integer,CentroidData> centroidsMap;
	HashMap<Integer,GatewayData> gatewaysMap;
	HashMap<Integer,ArrayList<GatewayData>> exitDistrictsMap;
	HashMap<Integer,GeomPlanarGraph> districtDualMap;
	

	public ArrayList<GeomPlanarGraphDirectedEdge> pathfinderDistrict(Node originNode, Node destinationNode, pedestrianSimulation state)
	{	
		
		this.edgesMap = state.edgesMap;
		this.nodesMap = state.nodesMap;
		this.gatewaysMap = state.gatewaysMap;
		this.exitDistrictsMap = state.exitDistrictsMap;
		this.districtDualMap = state.districtDualMap;
		this.centroidsMap = state.centroidsMap;
		
		this.originNode = originNode;
	    this.destinationNode = destinationNode;
		currentLocation = originNode;
		currentRegion =  nodesMap.get(currentLocation.getData()).district;
		targetRegion = nodesMap.get(destinationNode.getData()).district;
		visited.add(currentRegion);
		sequence.add(currentLocation);
		sequenceGateways.add((Integer) currentLocation.getData());
		sequenceNodes.add((Integer) currentLocation.getData());
		badExits.clear();
		previousLocation = null;
		
		while (found == false)
		{
			ArrayList<GatewayData> possibleGates = exitDistrictsMap.get(currentRegion);
			HashMap<Integer, Double> validGates = new HashMap<Integer, Double> ();
			HashMap<Integer, Double> otherGates = new HashMap<Integer, Double> ();
			
			double destinationAngle = utilities.angle(currentLocation.getCoordinate(), destinationNode.getCoordinate());
			double distanceTarget = utilities.nodesDistance(currentLocation, destinationNode);

			for (GatewayData gd : possibleGates)
				{	
					if (gd.nodeID == currentLocation.getData()) continue;
					if (badExits.contains(gd.gatewayID)) continue;
					if (visited.contains(gd.regionTo)) continue;
					double gateAngle = utilities.angle(currentLocation.getCoordinate(), gd.n.getCoordinate());
					if ((utilities.nodesDistance(gd.n, destinationNode) > distanceTarget) ||
						(utilities.inDirection(destinationAngle, gateAngle, 140) == false) || 
						(utilities.inDirection(destinationAngle, gd.entryAngle, 140) == false))
					{
						double cost = utilities.angleDiff(gateAngle, destinationAngle); 
						if (cost > 95) continue;
						cost += utilities.angleDiff(gateAngle, gd.entryAngle);
						otherGates.put(gd.gatewayID, cost);
						continue;
					}
					double cost = utilities.angleDiff(gateAngle, destinationAngle);
					cost += utilities.angleDiff(gateAngle, gd.entryAngle);   
					validGates.put(gd.gatewayID, cost);
				}
			
			if (validGates.size() == 0) validGates = otherGates;
			if (validGates.size() == 0) 
				{

					if (previousLocation != null)
						{
//						System.out.println("going to previous "+ originNode.getData() + "  "+ destinationNode.getData());
						badExits.add(sequenceGateways.get(sequenceGateways.size()-2));
						currentLocation = previousLocation;
						currentRegion = nodesMap.get(previousLocation.getData()).district;
						sequence.remove(sequence.size() - 1);
						sequence.remove(sequence.size() - 1);
						sequenceGateways.remove(sequenceGateways.size()-1);
						sequenceGateways.remove(sequenceGateways.size()-1);
						sequenceNodes.remove(sequenceNodes.size()-1);
						sequenceNodes.remove(sequenceNodes.size()-1);
						visited.remove(visited.size()-1);
						previousLocation = null;
						continue;
						}
					
//					System.out.println("CHECK HERE " + sequenceNodes);
					for (int i : badExits) System.out.println(gatewaysMap.get(i).nodeID);
//					System.out.println(" --------- lastLocation "+ currentLocation.getData()); 
//					System.out.println("route  "+ originNode.getData() + "  "+ destinationNode.getData());
					AStarAngular pathfinderAngular = new AStarAngular();
					Node originNodeDual = utilities.getDualNode(originNode, state.dualNetwork);
					Node destinationNodeDual = utilities.getDualNode(destinationNode, state.dualNetwork);
					return pathfinderAngular.astarPath(originNodeDual, destinationNodeDual, state, null, true).edges;
				}
			Map validSorted = utilities.sortByValue(validGates); 
			Iterator it = validSorted.entrySet().iterator();
	//		Map.Entry<String,String> entry = map.entrySet().iterator().next();
			while (it.hasNext())
			{
				Map.Entry<Integer, Double> pair = (Map.Entry<Integer, Double>)it.next();
				Integer exitID = pair.getKey();
				Node exitNode = gatewaysMap.get(exitID).n;
				
				Integer entryID = gatewaysMap.get(exitID).entryID;
			  	NodeData nd = nodesMap.get(entryID);
			  	sequenceGateways.add(exitID);
			  	sequenceGateways.add(entryID);
			  	sequenceNodes.add(gatewaysMap.get(exitID).nodeID);
			  	sequenceNodes.add(entryID);
	  			visited.add(nd.district);
	  			previousLocation = currentLocation;
	  			currentLocation = nd.node;
	  			currentRegion = nd.district;
	  			sequence.add(exitNode);
	  			sequence.add(nd.node);
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
//		return sequence;
		ArrayList<GeomPlanarGraphDirectedEdge> completePath =  new ArrayList<GeomPlanarGraphDirectedEdge>();
		Node originT = null;
		Node destinationT = null;
//		System.out.println(sequenceNodes + " final sequ");
		
		for (int i = 0; i < sequence.size(); i++)
		{
			
			if (i%2 != 0) continue;
			ArrayList<GeomPlanarGraphDirectedEdge> resultPartial =  new ArrayList<GeomPlanarGraphDirectedEdge>();
			GeomPlanarGraph districtNetwork;
			Node originPrimal = null;

			if (i == 0) 
			{
				originPrimal = sequence.get(0);
				originT = utilities.getDualNode(originPrimal, state.dualNetwork);
			}
			
			AStarAngular pathfinderAngular = new AStarAngular();
			if (i < sequence.size()-2)
			{
				int exitID = (int) sequenceGateways.get(i+1);
				int connectingEdgeID = gatewaysMap.get(exitID).edgeID;
				GeomPlanarGraphEdge connectingEdge = edgesMap.get(connectingEdgeID).planarEdge;
				destinationT = state.dualNetwork.findNode(connectingEdge.getLine().getCentroid().getCoordinate());
			}
			else destinationT = utilities.getDualNode(sequence.get(sequence.size()-1), state.dualNetwork);
			resultPartial = pathfinderAngular.astarPath(originT, destinationT, state, null, true).edges;
			originT = destinationT;
			completePath.addAll(resultPartial);
		}
	
		return completePath;	
	}
}

