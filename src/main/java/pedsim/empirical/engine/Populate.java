package pedsim.empirical.engine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.javatuples.Pair;

import pedsim.core.engine.PedSimCity;
import pedsim.core.parameters.Pars;
import pedsim.core.parameters.RouteChoicePars;
import pedsim.empirical.agent.EmpiricalAgent;
import pedsim.empirical.agent.EmpiricalAgentsGroup;
import pedsim.empirical.agent.EmpiricalGroup;
import pedsim.empirical.parameters.EmpiricalPars;
import sim.graph.Graph;
import sim.graph.NodeGraph;
import sim.graph.NodesLookup;

/**
 * Populates the empirical ABM with OD-based agents sampled from empirical
 * groups.
 *
 * This class intentionally depends on core.PedSimCity and empirical.Agent
 * classes, not on cityimage.PedSimCityImage or cityimage.Agent.
 */
public class Populate extends pedsim.core.engine.Populate {

	private static final double WORK_SHARE = 0.30;
	private static final double VISIT_SHARE = 0.46;
	private static final double RANDOM_SHARE = 0.24;
	private static final String TYPE_LIVE = "live";

	private final ArrayList<Pair<NodeGraph, NodeGraph>> odMatrix = new ArrayList<>();

	private PedSimCity state;
	private Graph network;

	public static HashMap<String, Double> destinationsDMA = new HashMap<>();

	public void populateEmpiricalGroups(PedSimCityEmpirical state) {
		this.state = state;
		this.network = PedSimCity.network;
		this.odMatrix.clear();

		int numberTrips = Pars.numAgents * EmpiricalPars.numberTripsPerAgent;

		if (EmpiricalPars.usingDMA) {
			generateODMatrixUsingDMA(numberTrips);
		} else {
			generateRandomODMatrix(numberTrips);
		}

		assignODMatrixToEmpiricalGroups();
	}

	private void generateRandomODMatrix(int numberTrips) {
		for (int i = 0; i < numberTrips; i++) {
			NodeGraph originNode = NodesLookup.randomNode(network);
			NodeGraph destinationNode = randomDestination(originNode);

			odMatrix.add(new Pair<>(originNode, destinationNode));
		}
	}

	private void generateODMatrixUsingDMA(int numberTrips) {
		setDMAMap();

		HashMap<String, Integer> numberDestinationsDMA = new HashMap<>();
		int tripsToAllocate = numberTrips;

		for (String typeDMA : destinationsDMA.keySet()) {
			int number = (int) Math.round(destinationsDMA.get(typeDMA) * numberTrips);
			number = Math.min(number, tripsToAllocate);
			numberDestinationsDMA.put(typeDMA, number);
			tripsToAllocate -= number;
		}

		// Put any rounding residue into random destinations.
		numberDestinationsDMA.merge("random", tripsToAllocate, Integer::sum);

		for (int i = 0; i < numberTrips; i++) {
			NodeGraph originNode = safeRandomNodeDMA(TYPE_LIVE);
			String destinationDMA = nextAvailableDMA(numberDestinationsDMA);
			NodeGraph destinationNode = safeRandomDestinationDMA(originNode, destinationDMA);

			if (originNode == null || destinationNode == null) {
				originNode = NodesLookup.randomNode(network);
				destinationNode = randomDestination(originNode);
			}

			odMatrix.add(new Pair<>(originNode, destinationNode));
		}
	}

	private NodeGraph safeRandomNodeDMA(String dmaType) {
		try {
			return NodesLookup.randomNodeDMA(network, dmaType);
		} catch (Exception e) {
			return null;
		}
	}

	private NodeGraph safeRandomDestinationDMA(NodeGraph originNode, String dmaType) {
		if (originNode == null) {
			return null;
		}

		try {
			NodeGraph destinationNode = null;
			int attempts = 0;

			while ((destinationNode == null || destinationNode.gateway) && attempts < 50) {
				destinationNode = NodesLookup.randomNodeBetweenDistanceIntervalDMA(network, originNode,
						RouteChoicePars.minTripDistance, RouteChoicePars.maxTripDistance, dmaType);
				attempts++;
			}

			return destinationNode;
		} catch (Exception e) {
			return null;
		}
	}

	private NodeGraph randomDestination(NodeGraph originNode) {
		NodeGraph destinationNode = NodesLookup.randomNodeBetweenDistanceInterval(network, originNode,
				RouteChoicePars.minTripDistance, RouteChoicePars.maxTripDistance);

		int attempts = 0;
		while (destinationNode != null && destinationNode.gateway && attempts < 50) {
			destinationNode = NodesLookup.randomNodeBetweenDistanceInterval(network, originNode,
					RouteChoicePars.minTripDistance, RouteChoicePars.maxTripDistance);
			attempts++;
		}

		return destinationNode;
	}

	private String nextAvailableDMA(HashMap<String, Integer> numberDestinationsDMA) {
		for (String typeDMA : numberDestinationsDMA.keySet()) {
			int remaining = numberDestinationsDMA.get(typeDMA);
			if (remaining > 0) {
				numberDestinationsDMA.put(typeDMA, remaining - 1);
				return typeDMA;
			}
		}

		return "random";
	}

	private void setDMAMap() {
		destinationsDMA = new HashMap<>(3);
		destinationsDMA.put("work", WORK_SHARE);
		destinationsDMA.put("visit", VISIT_SHARE);
		destinationsDMA.put("random", RANDOM_SHARE);
	}

	private void assignODMatrixToEmpiricalGroups() {
		ensureGroupsLoaded();

		int agentID = 0;

		// Full-population benchmark configurations.
		for (EmpiricalAgentsGroup group : PedSimCityEmpirical.empiricalGroups) {
			if (group.groupName == EmpiricalGroup.POPULATION || group.groupName == EmpiricalGroup.NULLGROUP) {
				agentID = createAgentsForGroup(agentID, group, Pars.numAgents, odMatrix);
			}
		}

		// Heterogeneous empirical configuration: split the same population across
		// empirical clusters.
		List<EmpiricalAgentsGroup> empiricalClusters = new ArrayList<>();

		for (EmpiricalAgentsGroup group : PedSimCityEmpirical.empiricalGroups) {
			if (group.groupName != EmpiricalGroup.POPULATION && group.groupName != EmpiricalGroup.NULLGROUP) {
				empiricalClusters.add(group);
			}
		}

		if (empiricalClusters.isEmpty()) {
			return;
		}

		int agentsToAllocate = Pars.numAgents;
		int odCursor = 0;

		for (int groupIndex = 0; groupIndex < empiricalClusters.size(); groupIndex++) {
			EmpiricalAgentsGroup group = empiricalClusters.get(groupIndex);

			int numberAgentsForGroup;
			if (groupIndex == empiricalClusters.size() - 1) {
				numberAgentsForGroup = agentsToAllocate;
			} else {
				numberAgentsForGroup = (int) Math.round(Pars.numAgents * group.share);
				numberAgentsForGroup = Math.min(numberAgentsForGroup, agentsToAllocate);
				agentsToAllocate -= numberAgentsForGroup;
			}

			int tripsForGroup = numberAgentsForGroup * EmpiricalPars.numberTripsPerAgent;
			int odEnd = Math.min(odCursor + tripsForGroup, odMatrix.size());

			List<Pair<NodeGraph, NodeGraph>> groupOD = new ArrayList<>(odMatrix.subList(odCursor, odEnd));
			agentID = createAgentsForGroup(agentID, group, numberAgentsForGroup, groupOD);

			odCursor = odEnd;
		}
	}

	private void ensureGroupsLoaded() {
		if (!PedSimCityEmpirical.empiricalGroups.isEmpty()) {
			return;
		}

		EmpiricalAgentsGroup nullGroup = new EmpiricalAgentsGroup();
		nullGroup.setGroup(EmpiricalGroup.NULLGROUP, new String[] { "NULLGROUP" });
		PedSimCityEmpirical.empiricalGroups.add(nullGroup);
	}

	private int createAgentsForGroup(int firstAgentID, EmpiricalAgentsGroup group, int numberAgents,
			List<Pair<NodeGraph, NodeGraph>> groupODMatrix) {

		int agentID = firstAgentID;
		int lowerLimit = 0;

		for (int i = 0; i < numberAgents; i++) {
			int upperLimit = lowerLimit + EmpiricalPars.numberTripsPerAgent;

			if (upperLimit > groupODMatrix.size()) {
				break;
			}

			List<Pair<NodeGraph, NodeGraph>> agentODs = new ArrayList<>(groupODMatrix.subList(lowerLimit, upperLimit));

			EmpiricalAgent agent = new EmpiricalAgent(state, group);
			agent.agentID = agentID;
			agent.setOD(agentODs);

			state.agents.addGeometry(agent.getLocation());
			state.agentsList.add(agent);

			agentID++;
			lowerLimit = upperLimit;
		}

		return agentID;
	}
}