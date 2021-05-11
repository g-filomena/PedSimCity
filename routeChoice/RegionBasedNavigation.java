/**
 * It computes a sequence of gateways that indicate in turn the series of traversed regions across the city, between an origin and a destination.
 * This represent the so-called coarse plan that is then refined later on.
 *
 * */
package pedsimcity.routeChoice;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map.Entry;
import java.util.Set;

import org.javatuples.Pair;

import pedsimcity.agents.AgentProperties;
import pedsimcity.elements.Gateway;
import pedsimcity.elements.Region;
import pedsimcity.graph.EdgeGraph;
import pedsimcity.graph.NodeGraph;
import pedsimcity.main.PedSimCity;
import pedsimcity.utilities.Angles;
import pedsimcity.utilities.Utilities;

public class RegionBasedNavigation {

	ArrayList<Integer> visitedRegions = new ArrayList<>();
	ArrayList<NodeGraph> sequence = new ArrayList<>();
	ArrayList<Pair<NodeGraph, NodeGraph>> badExits = new ArrayList<>();

	NodeGraph originNode, destinationNode, currentLocation, previousLocation;
	int currentRegion, specificRegion, targetRegion;
	boolean finalRegion = false;

	HashMap<Integer, EdgeGraph> edgesMap;
	HashMap<Pair<NodeGraph, NodeGraph>, Gateway> gatewaysMap;

	/**
	 * It returns a sequence of nodes, wherein, besides the origin and the
	 * destination nodes, the other nodes represent entry and exit gateways of the
	 * traversed regions. The traversed regions are identified as well within this
	 * function.
	 *
	 * If the agent also uses barriers, barrier sub-goals are also identified when
	 * applicable.
	 *
	 *
	 * @param originNode      the origin node;
	 * @param destinationNode the destination node;
	 * @param ap              agent properties;
	 */
	public ArrayList<NodeGraph> sequenceRegions(NodeGraph originNode, NodeGraph destinationNode, AgentProperties ap) {

		this.edgesMap = PedSimCity.edgesMap;
		this.gatewaysMap = PedSimCity.gatewaysMap;
		this.originNode = originNode;
		this.destinationNode = destinationNode;

		this.currentLocation = originNode;
		this.currentRegion = originNode.region;
		this.targetRegion = destinationNode.region;
		// sequence of regions
		this.visitedRegions.add(this.currentRegion);
		this.previousLocation = null;
		this.sequence.add(originNode);

		// rough plan
		if (this.currentRegion == this.targetRegion) {
			this.finalRegion = true;
			this.sequence.add(destinationNode);
		}

		while (!this.finalRegion) {
			// exit- entry next region
			final Pair<NodeGraph, NodeGraph> gateways = this.nextGateways(this.currentLocation, this.currentRegion,
					999999);
			// no gateway has been identified, go back, if possible
			if (gateways == null)
				if (this.previousLocation != null) {

					final Pair<NodeGraph, NodeGraph> badPair = new Pair<>(this.sequence.get(this.sequence.size() - 2),
							this.sequence.get(this.sequence.size() - 1));
					// add last exit to the ones to avoid in feature
					this.badExits.add(badPair);
					this.currentLocation = this.previousLocation;
					this.currentRegion = this.previousLocation.region;
					this.sequence = new ArrayList<>(this.sequence.subList(0, this.sequence.size() - 2));
					this.visitedRegions.remove(this.visitedRegions.size() - 1);
					this.previousLocation = null;
					continue;
				}
				// otherwise the agent will compute a path from the origin and the destination
				// nodes without going through regions
				// it should not happen.
				else {
					this.sequence.add(destinationNode);
					return this.sequence;
				}

			this.previousLocation = this.currentLocation;
			// only new entry, when the last entry can send the agent directly to the next
			// region
			this.sequence.add(gateways.getValue0());
			this.sequence.add(gateways.getValue1());

			this.currentLocation = this.sequence.get(this.sequence.size() - 1);
			this.currentRegion = this.currentLocation.region;
			this.visitedRegions.add(this.currentRegion);

			if (this.currentRegion == this.targetRegion) {
				this.finalRegion = true;
				this.sequence.add(destinationNode);
				break;
			}
		}

		// clear the sets
		this.visitedRegions.clear();
		this.badExits.clear();

		// sub-goals and barriers - navigation
		ArrayList<NodeGraph> newSequence = new ArrayList<>();
		// if also barrier navigation, insert barrier-sub goals into the sequence
		if (ap.barrierBasedNavigation)
			newSequence = this.regionalBarriers(this.sequence, ap);
		else
			newSequence = this.sequence;

		// remove duplicates and maintains order
		final Set<NodeGraph> ns = new LinkedHashSet<>(newSequence);
		newSequence = new ArrayList<>(ns);
		return newSequence;
	}

	/**
	 * Given a sequence of nodes of the type [O, exit, entry, exit, entry .., D]
	 * where exit and entry are gateways, it identifies barriers within the regions
	 * traversed (i.e within an entry (can be O too) and the exit) that may help
	 * further the agent in navigating. When a barrier is identified, the function
	 * also verifies if there's a better exit from that region to the following on.
	 * Thus, the sequence of gateways may change, although the sequence of regions
	 * don't.
	 *
	 * @param sequence the sequence of nodes;
	 * @param ap       agent properties;
	 */

	public ArrayList<NodeGraph> regionalBarriers(ArrayList<NodeGraph> sequence, AgentProperties ap) {
		// it stores the barriers that the agent has already been exposed to
		final ArrayList<Integer> adjacentBarriers = new ArrayList<>();
		// exits that are going to be ignored
		final ArrayList<NodeGraph> toIgnore = new ArrayList<>();
		final ArrayList<NodeGraph> newSequence = new ArrayList<>();

		for (final NodeGraph gateway : sequence) {
			if (toIgnore.contains(gateway))
				continue;
			newSequence.add(gateway);
			final int indexOf = sequence.indexOf(gateway);
			// continue when exit gateways and destination
			if (indexOf > 0 && indexOf % 2 != 0)
				continue;

			// check if there are good barriers in line of movement towards the destination
			final Set<Integer> intersectingBarriers = BarrierBasedNavigation.intersectingBarriers(gateway,
					this.destinationNode, ap.typeBarriers);
			// no barriers
			if (intersectingBarriers.size() == 0)
				continue;

			// identify barriers around this gateway
			final ArrayList<EdgeGraph> incomingEdges = gateway.getEdges();
			for (final EdgeGraph edge : incomingEdges)
				if (edge.barriers != null)
					adjacentBarriers.addAll(edge.barriers);

			// identify all the barriers in the region
			final Region region = PedSimCity.regionsMap.get(gateway.region);
			intersectingBarriers.retainAll(new HashSet<>(region.primalGraph.getSubGraphBarriers()));
			if (intersectingBarriers.size() == 0)
				continue;

			// disregard barriers that have been already walked along
			final Set<Integer> visitedBarriers = new HashSet<>(adjacentBarriers);
			intersectingBarriers.removeAll(visitedBarriers);
			if (intersectingBarriers.size() == 0)
				continue;

			NodeGraph subGoal = null;
			// given the intersecting barriers, identify the best one and the relative edge
			// close to it
			final BarrierBasedNavigation barrierBasedNavigation = new BarrierBasedNavigation();
			final Pair<EdgeGraph, Integer> barrierGoal = barrierBasedNavigation.barrierGoal(intersectingBarriers,
					gateway, this.destinationNode, region);
			if (barrierGoal == null)
				continue;
			final EdgeGraph edgeGoal = barrierGoal.getValue0();
			final int barrier = barrierGoal.getValue1();

			// pick the closest barrier sub-goal
			final NodeGraph u = edgeGoal.u;
			final NodeGraph v = edgeGoal.v;
			if (NodeGraph.nodesDistance(gateway, u) < NodeGraph.nodesDistance(gateway, v))
				subGoal = u;
			else
				subGoal = v;

			adjacentBarriers.add(barrier);
			// continue if this subgoal it's in the sequence, i.e if it's an exit gateway
			if (sequence.contains(subGoal))
				continue;
			newSequence.add(subGoal);

			// it this the entry gateway to the last region
			if (indexOf == sequence.size() - 2)
				continue;
			final int desiredRegion = sequence.get(indexOf + 2).region;

			// if subGoal is a newGateway itself and it leads to the next region
			if (subGoal.gateway && subGoal.adjacentRegions.contains(desiredRegion)) {

				// no need to go through another gateway
				// --> ignore the next exit
				toIgnore.add(sequence.get(indexOf + 1));
				double deviation = Double.MAX_VALUE;

				// get a new entry
				NodeGraph bestEntry = null;
				for (final NodeGraph entry : subGoal.adjacentEntries) {
					if (entry.region != desiredRegion)
						continue;
					final double entryAngle = Angles.angle(subGoal, entry);

					if (entryAngle < deviation) {
						deviation = entryAngle;
						bestEntry = entry;
					}
				}
				// replace the old entry
				sequence.set(indexOf + 2, bestEntry);
			}

			// if the subGoal is not a gateway, check whether there's a better gateway
			// towards the next region (desiredRegion)
			else {
				final Pair<NodeGraph, NodeGraph> newGateways = this.nextGateways(subGoal, subGoal.region,
						desiredRegion);
				if (newGateways == null)
					continue;
				sequence.set(indexOf + 1, newGateways.getValue0());
				sequence.set(indexOf + 2, newGateways.getValue1());
			}
		}

		return newSequence;
	}

	/**
	 * Given a current location and a current region, it identifies the next pair of
	 * gateways, towards the best region.
	 *
	 * @param currentLocation the current location of the agent.
	 * @param currentRegion   the current region;
	 * @param specificRegion  a desired region, when for example, a new gateway
	 *                        could be identified after a barrier sub-goal.
	 */

	private Pair<NodeGraph, NodeGraph> nextGateways(NodeGraph currentLocation, int currentRegion, int specificRegion) {

		// retrieve current region's exits
		final ArrayList<Gateway> possibleGates = PedSimCity.regionsMap.get(currentRegion).gateways;
		HashMap<Pair<NodeGraph, NodeGraph>, Double> validGates = new HashMap<>();
		final HashMap<Pair<NodeGraph, NodeGraph>, Double> otherGates = new HashMap<>();

		// check compliance with criteria
		final double locationDestinationAngle = Angles.angle(currentLocation, this.destinationNode);
		final double distanceTarget = NodeGraph.nodesDistance(currentLocation, this.destinationNode);
		for (final Gateway gd : possibleGates) {
			if (this.badExits.contains(gd.gatewayID))
				continue;
			if (specificRegion != 999999 && specificRegion != gd.regionTo)
				continue;
			if (this.visitedRegions.contains(gd.regionTo))
				continue;

			final double locationExitAngle = Angles.angle(currentLocation, gd.exit);
			final double exitEntryAngle = gd.entryAngle;
			final double exitDestintionAngle = Angles.angle(gd.exit, this.destinationNode);
			final double differenceExitEntry = Angles.differenceAngles(locationExitAngle, exitDestintionAngle);
			final double distanceFromGate = NodeGraph.nodesDistance(currentLocation, gd.exit);

			double cost = 0.0;
			final boolean entryInDirection = Angles.isInDirection(locationDestinationAngle, exitEntryAngle, 140.0);
			final boolean exitInDirection = Angles.isInDirection(locationDestinationAngle, locationExitAngle, 140.0);

			// criteria are not met
			if (gd.exit == currentLocation && !entryInDirection) {
				cost = Angles.differenceAngles(exitEntryAngle, locationDestinationAngle);
				otherGates.put(gd.gatewayID, cost); // use as alternatives
				continue;
			} else if (gd.exit != currentLocation
					&& (distanceFromGate > distanceTarget || !exitInDirection || !entryInDirection)) {
				cost = Angles.differenceAngles(locationExitAngle, locationDestinationAngle);
				if (cost > 90)
					continue;
				cost += differenceExitEntry;
				otherGates.put(gd.gatewayID, cost); // use as alternatives
				continue;
			}
			// in this case just consider the deviation caused by the entry
			if (gd.exit == currentLocation)
				cost = Angles.differenceAngles(gd.entryAngle, locationDestinationAngle);
			else {
				cost = Angles.differenceAngles(locationExitAngle, locationDestinationAngle);
				cost += differenceExitEntry;
			}
			validGates.put(gd.gatewayID, cost);
		}
		if (validGates.size() == 0 && specificRegion != 999999)
			return null;
		if (validGates.size() == 0)
			validGates = otherGates;
		if (validGates.size() == 0)
			return null;

		// sort the valid gates, rewarding the ones with the lowest deviation towards
		// the destination
		final LinkedHashMap<Pair<NodeGraph, NodeGraph>, Double> validSorted = (LinkedHashMap<Pair<NodeGraph, NodeGraph>, Double>) Utilities
				.sortByValue(validGates, false);

		// return the first gateway pair
		for (final Entry<Pair<NodeGraph, NodeGraph>, Double> gatewaysPair : validSorted.entrySet())
			return gatewaysPair.getKey();
		return null;
	}
}
