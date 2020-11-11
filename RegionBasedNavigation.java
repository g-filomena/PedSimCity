/**
 * It computes a sequence of gateways that indicate in turn the series of traversed regions across the city, between an origin and a destination.
 * This represent the so-called coarse plan that is then refined later on.
 *
 * */

package sim.app.geo.pedSimCity;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.Map.Entry;
import java.util.Set;

import org.javatuples.Pair;

import sim.app.geo.urbanSim.Angles;
import sim.app.geo.urbanSim.EdgeGraph;
import sim.app.geo.urbanSim.NodeGraph;
import sim.app.geo.urbanSim.Utilities;

public class RegionBasedNavigation {

	ArrayList<Integer> visitedRegions = new ArrayList<Integer>();
	ArrayList<NodeGraph> sequence = new ArrayList<NodeGraph>();
	ArrayList<Pair<NodeGraph, NodeGraph>> badExits = new ArrayList<Pair<NodeGraph, NodeGraph>>();

	NodeGraph originNode, destinationNode, currentLocation, previousLocation;
	int currentRegion, specificRegion, targetRegion;
	boolean finalRegion = false;

	HashMap<Integer, EdgeGraph> edgesMap;
	HashMap<Pair<NodeGraph, NodeGraph>, Gateway> gatewaysMap;

	/**
	 * It returns a sequence of nodes, wherein, besides the origin and the destination nodes, the other nodes represent entry and exit gateways
	 * of the traversed regions. The traversed regions are identified as well within this function.
	 *
	 * If the agent also uses barriers, barrier sub-goals are also identified when applicable.
	 *
	 *
	 * @param originNode the origin node;
	 * @param destinationNode the destination node;
	 * @param ap agent properties;
	 */

	public ArrayList<NodeGraph> sequenceRegions(NodeGraph originNode, NodeGraph destinationNode, AgentProperties ap) {

		this.edgesMap = PedSimCity.edgesMap;
		this.gatewaysMap = PedSimCity.gatewaysMap;
		this.originNode = originNode;
		this.destinationNode = destinationNode;

		currentLocation = originNode;
		currentRegion = originNode.region;
		targetRegion = destinationNode.region;
		// sequence of regions
		visitedRegions.add(currentRegion);
		previousLocation = null;
		sequence.add(originNode);

		// rough plan
		if (currentRegion == targetRegion) {
			finalRegion = true;
			sequence.add(destinationNode);
		}

		while (!finalRegion) {
			//exit- entry next region
			Pair<NodeGraph, NodeGraph> gateways =  nextGateways(currentLocation, currentRegion, 999999);

			// no gateway has been identified, go back, if possible
			if (gateways == null) {
				if (previousLocation != null) {

					Pair<NodeGraph, NodeGraph> badPair = new Pair <NodeGraph, NodeGraph> (sequence.get(sequence.size()-2),
							sequence.get(sequence.size()-1));
					// add last exit to the ones to avoid in feature
					badExits.add(badPair);
					currentLocation = previousLocation;
					currentRegion = previousLocation.region;
					sequence = new ArrayList<NodeGraph>(sequence.subList(0, sequence.size()-2));
					visitedRegions.remove(visitedRegions.size()-1);
					previousLocation = null;
					continue;
				}
				// otherwise the agent will compute a path from the origin and the destination nodes withouth going through regions
				// it should not happen.
				else {
					sequence.add(destinationNode);
					return sequence;
				}
			}

			previousLocation = currentLocation;
			sequence.add(gateways.getValue0()); //exit
			sequence.add(gateways.getValue1()); //entry next region
			currentLocation = gateways.getValue1();
			currentRegion = currentLocation.region;
			visitedRegions.add(currentRegion);

			if (currentRegion == targetRegion) {
				finalRegion = true;
				sequence.add(destinationNode);
				break;
			}
		}

		//clear the sets
		visitedRegions.clear();
		badExits.clear();

		// sub-goals and barriers - navigation
		ArrayList<NodeGraph> newSequence = new ArrayList<NodeGraph>();
		// if also barrier navigation, insert barrier-sub goals into the sequence
		if (ap.barrierBasedNavigation) newSequence = regionalBarriers(sequence, ap);
		else newSequence = sequence;
		return newSequence;
	}

	/**
	 * Given a sequence of nodes of the type [O, exit, entry, exit, entry .., D] where exit and entry are gateways, it identifies barriers
	 * within the regions traversed (i.e within an entry (can be O too) and the exit) that may help further the agent in navigating.
	 * When a barrier is identified, the function also verifies if there's a better exit from that region to the following on.
	 * Thus, the sequence of gateways may change, although the sequence of regions don't.
	 *
	 * @param sequence the sequence of nodes;
	 * @param ap agent properties;
	 */

	public ArrayList<NodeGraph> regionalBarriers(ArrayList<NodeGraph> sequence, AgentProperties ap)
	{
		// it stores the barriers that the agent has already been exposed to
		ArrayList<Integer> adjacentBarriers = new ArrayList<Integer>();
		// exits that are going to be ignored
		ArrayList<NodeGraph> toIgnore = new ArrayList<NodeGraph>();
		ArrayList<NodeGraph> newSequence = new ArrayList<NodeGraph>();

		for (NodeGraph gateway: sequence)
		{
			if (toIgnore.contains(gateway)) continue;
			newSequence.add(gateway);
			int indexOf = sequence.indexOf(gateway);
			// continue when exit gateways and destination
			if ((indexOf > 0) && (indexOf%2 != 0)) continue;

			// check if there are good barriers in line of movement towards the destination
			Set<Integer> intersectingBarriers = BarrierBasedNavigation.intersectingBarriers(gateway, destinationNode, ap.typeBarriers);
			//no barriers
			if (intersectingBarriers.size() == 0) continue;

			// identify barriers around this gateway
			ArrayList<EdgeGraph> incomingEdges = gateway.getEdges();
			for (EdgeGraph edge : incomingEdges) if (edge.barriers != null) adjacentBarriers.addAll(edge.barriers);

			// identify all the barriers in the region
			Region region = PedSimCity.regionsMap.get(gateway.region);
			intersectingBarriers.retainAll(new HashSet<Integer>(region.primalGraph.getSubGraphBarriers()));
			if (intersectingBarriers.size() == 0) continue;

			// disregard barriers that have been already walked along
			Set<Integer> visitedBarriers = new HashSet<Integer>(adjacentBarriers);
			intersectingBarriers.removeAll(visitedBarriers);
			if (intersectingBarriers.size() == 0) continue;

			NodeGraph subGoal = null;
			// given the intersecting barriers, identify the best one and the relative edge close to it
			Pair<EdgeGraph, Integer> barrierGoal = BarrierBasedNavigation.barrierGoal(intersectingBarriers, gateway, destinationNode, region);
			if (barrierGoal == null) continue;
			EdgeGraph edgeGoal = barrierGoal.getValue0();
			int barrier = barrierGoal.getValue1();

			// pick the closest barrier sub-goal
			NodeGraph u = edgeGoal.u;
			NodeGraph v = edgeGoal.v;
			if ((Utilities.nodesDistance(gateway, u)) < (Utilities.nodesDistance(gateway, v))) subGoal = u;
			else subGoal = v;

			adjacentBarriers.add(barrier);
			// continue if this subgoal it's in the sequence, i.e if it's an exit gateway
			if (sequence.contains(subGoal)) continue;
			newSequence.add(subGoal);

			//it this the entry gateway to the last region
			if (indexOf == sequence.size()-2) continue;
			int desiredRegion = sequence.get(indexOf+2).region;

			// if subGoal is a newGateway itself and it leads to the next region
			if ((subGoal.gateway) && (subGoal.adjacentRegions.contains(desiredRegion))) {

				// no need to go through another gateway
				// --> ignore the next exit
				toIgnore.add(sequence.get(indexOf+1));
				double deviation = Double.MAX_VALUE;

				//get a new entry
				NodeGraph bestEntry = null;
				for (NodeGraph entry : subGoal.adjacentEntries) {
					if (entry.region != desiredRegion) continue;
					double entryAngle = Angles.angle(subGoal, entry);

					if (entryAngle < deviation) {
						deviation = entryAngle;
						bestEntry = entry;
					}
				}
				// replace the old entry
				sequence.set(indexOf+2, bestEntry);
			}

			// if the subGoal is not a gateway, check whether there's a better gateway towards the next region (desiredRegion)
			else {
				Pair<NodeGraph, NodeGraph> newGateways = nextGateways(subGoal, subGoal.region, desiredRegion);
				if (newGateways == null) continue;
				sequence.set(indexOf+1, newGateways.getValue0());
				sequence.set(indexOf+2, newGateways.getValue1());
			}
		}
		return newSequence;
	}


	/**
	 * Given a current location and a current region, it identifies the next pair of gateways, towards the best region.
	 *
	 * @param currentLocation the current location of the agent.
	 * @param currentRegion the current region;
	 * @param specificRegion a desired region, when for example, a new gateway could be identified after a barrier sub-goal.
	 */

	private Pair<NodeGraph, NodeGraph> nextGateways(NodeGraph currentLocation, int currentRegion, int specificRegion) {

		// retrieve current region's exits
		ArrayList<Gateway> possibleGates = PedSimCity.regionsMap.get(currentRegion).gateways;
		HashMap<Pair<NodeGraph, NodeGraph>, Double> validGates = new HashMap<Pair<NodeGraph, NodeGraph>, Double> ();
		HashMap<Pair<NodeGraph, NodeGraph>, Double> otherGates = new HashMap<Pair<NodeGraph, NodeGraph>, Double> ();

		// check compliance with criteria
		double destinationAngle = Angles.angle(currentLocation, destinationNode);
		double distanceTarget = Utilities.nodesDistance(currentLocation, destinationNode);

		for (Gateway gd : possibleGates) {
			if (gd.node == currentLocation) continue;
			// the gateway entraps the agent
			if (badExits.contains(gd.gatewayID)) continue;
			if ((specificRegion != 999999) && (specificRegion != gd.regionTo)) continue;
			if (visitedRegions.contains(gd.regionTo)) continue;

			double exitAngle = Angles.angle(currentLocation, gd.node);
			double distanceFromGate = Utilities.nodesDistance(currentLocation, gd.node);

			// criteria are not met
			if (distanceFromGate > distanceTarget ||
					!Angles.isInDirection(destinationAngle, exitAngle, 140) ||
					!Angles.isInDirection(destinationAngle, gd.entryAngle, 140) == false) {

				double cost = Angles.differenceAngles(exitAngle, destinationAngle);
				if (cost > 90) continue; //too much
				cost += Angles.differenceAngles(exitAngle, gd.entryAngle);
				otherGates.put(gd.gatewayID, cost); // use as alternatives
				continue;
			}
			double cost = Angles.differenceAngles(exitAngle, destinationAngle);
			cost += Angles.differenceAngles(exitAngle, gd.entryAngle);
			validGates.put(gd.gatewayID, cost);
		}
		if ((validGates.size() == 0) && (specificRegion != 999999)) return null;
		if (validGates.size() == 0) validGates = otherGates;
		if (validGates.size() == 0) return null;

		// sort the valid gates, rewarding the ones with the lowest deviation towards the destination
		LinkedHashMap<Pair<NodeGraph, NodeGraph>, Double> validSorted = (LinkedHashMap<Pair<NodeGraph, NodeGraph>, Double>)
				Utilities.sortByValue(validGates, "ascending");

		// return the first gateway pair
		for (Entry<Pair<NodeGraph, NodeGraph>, Double> gatewaysPair: validSorted.entrySet()) return gatewaysPair.getKey();
		return null;
	}
}

