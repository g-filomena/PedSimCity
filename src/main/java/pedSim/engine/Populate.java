package pedSim.engine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;

import org.javatuples.Pair;

import pedSim.agents.Agent;
import pedSim.agents.EmpiricalAgentsGroup;
import pedSim.utilities.StringEnum.Groups;
import pedSim.utilities.StringEnum.RouteChoice;
import sim.graph.Graph;
import sim.graph.NodeGraph;
import sim.graph.NodesLookup;

/**
 * The Populate class is responsible for generating test agents, building the OD
 * matrix, and populating empirical groups for pedestrian simulation.
 */
public class Populate {

	private PedSimCity state;
	private Graph network = new Graph();
	private final ArrayList<Pair<NodeGraph, NodeGraph>> OD = new ArrayList<>();

	public static boolean usingDMA = true;
	public static HashMap<String, Double> destinationsDMA = new HashMap<>();
	List<Integer> testOrigins = new ArrayList<>();
	List<Integer> testDestinations = new ArrayList<>();
	final static double WORK_SHARE = 0.30;
	final static double VISIT_SHARE = 0.46;
	final static double RANDOM_SHARE = 0.24;
	final static String TYPE_LIVE = "live";

	/**
	 * Populates test agents, OD matrix, and empirical groups for pedestrian
	 * simulation.
	 *
	 * @param state The PedSimCity simulation state.
	 */
	public void populateTests(PedSimCity state) {

		this.state = state;
		this.network = PedSimCity.network;

		if (Parameters.testingSpecificOD)
			prepareManualODmatrix();
		generateTestODmatrix();
		generateTestAgents();
	}

	/**
	 * Prepares a manual OD matrix based on specified test origins and destinations.
	 */
	public void prepareManualODmatrix() {

		testOrigins.clear();
		testDestinations.clear();
		for (Integer nodeID : Parameters.originsTmp)
			testOrigins.add(nodeID);
		for (Integer nodeID : Parameters.destinationsTmp)
			testDestinations.add(nodeID);
	}

	/**
	 * Generates the OD matrix for the test-based simulations.
	 */
	private void generateTestODmatrix() {

		// a trip per agent
		for (int i = 0; i < Parameters.numberTripsPerAgent; i++) {
			NodeGraph originNode = null;
			NodeGraph destinationNode = null;

			if (Parameters.testingSpecificOD) {
				originNode = PedSimCity.nodesMap.get(testOrigins.get(i));
				destinationNode = PedSimCity.nodesMap.get(testDestinations.get(i));
			} else if (Parameters.testingLandmarks) {
				originNode = NodesLookup.randomNode(network);
				destinationNode = NodesLookup.randomNodeFromDistancesSet(network, PedSimCity.junctions, originNode,
						PedSimCity.distances);
			} else if (Parameters.testingSubdivisions) {
				originNode = NodesLookup.randomNodeFromGeometriest(network, PedSimCity.startingNodes);
				destinationNode = NodesLookup.randomNodeBetweenDistanceInterval(network, originNode, 1000, 3000);
			} else if (Parameters.testingModels) {
				originNode = NodesLookup.randomNodeFromGeometriest(network, PedSimCity.startingNodes);
				destinationNode = NodesLookup.randomNodeBetweenDistanceInterval(network, originNode,
						Parameters.minDistance, Parameters.maxDistance);
			}

			Pair<NodeGraph, NodeGraph> pair = new Pair<>(originNode, destinationNode);
			this.OD.add(pair);
		}
	}

	/**
	 * Generates test agents for the simulation.
	 */
	private void generateTestAgents() {

		// One Model, One Agent
		Parameters.numAgents = Parameters.routeChoiceModels.length;
		final RouteChoice[] routeChoiceModels = Parameters.routeChoiceModels;
		for (int agentID = 0; agentID < Parameters.numAgents; agentID++) {
			Agent agent = new Agent(this.state);
			agent.initialiseAgentProperties();
			agent.getProperties().setRouteChoice(routeChoiceModels[agentID]);
			addAgent(agent, agentID, OD);
		}
	}

	/**
	 * Adds an agent to the simulation.
	 *
	 * @param agent        The agent to be added.
	 * @param agentID      The identifier of the agent.
	 * @param thisAgentODs The OD matrix for this agent.
	 */
	private void addAgent(Agent agent, int agentID, ArrayList<Pair<NodeGraph, NodeGraph>> thisAgentODs) {

		agent.OD = new LinkedList<>(thisAgentODs);
		agent.agentID = agentID;
		state.agents.addGeometry(agent.getGeometry());
		state.agentsList.add(agent);
	}

	/**
	 * Populates empirical groups for pedestrian simulation.
	 *
	 * @param state The PedSimCity simulation state.
	 */
	public void populateEmpiricalGroups(PedSimCity state) {

		this.state = state;
		this.network = PedSimCity.network;
		final int numODs = Parameters.numAgents * Parameters.numberTripsPerAgent;

		if (Parameters.usingDMA)
			usingDMA(numODs);
		else
			for (int i = 0; i < numODs; i++) {
				NodeGraph originNode = NodesLookup.randomNode(network);
				NodeGraph destinationNode = NodesLookup.randomNodeBetweenDistanceInterval(network, originNode,
						Parameters.minDistance, Parameters.maxDistance);
				while (destinationNode.gateway)
					destinationNode = NodesLookup.randomNodeBetweenDistanceInterval(network, originNode,
							Parameters.minDistance, Parameters.maxDistance);
				Pair<NodeGraph, NodeGraph> pair = new Pair<>(originNode, destinationNode);
				OD.add(pair);
				originNode = destinationNode = null;
			}
		assignODmatrixToEmpiricalGroups();
	}

	/**
	 * Generates an OD matrix on the basis of the Urban DMA categorisation.
	 *
	 * @param numODs The number of OD pairs to generate.
	 */
	private void usingDMA(int numODs) {

		setDMAmap();
		final HashMap<String, Integer> nrDestinationsDMA = new HashMap<>();
		int agentsToAllocate = numODs;
		String DMA = "";
		NodeGraph originNode = null;
		NodeGraph destinationNode = null;

		for (String typeDMA : destinationsDMA.keySet()) {
			int nr = (int) (destinationsDMA.get(typeDMA) * numODs);
			if (nr > agentsToAllocate)
				nr = agentsToAllocate;
			nrDestinationsDMA.put(typeDMA, nr);
		}

		for (int i = 0; i < numODs; i++) {
			originNode = NodesLookup.randomNodeDMA(network, TYPE_LIVE);
			for (String typeDMA : nrDestinationsDMA.keySet()) {
				int nr = nrDestinationsDMA.get(typeDMA);
				if (nr < 1)
					continue;
				else {
					DMA = typeDMA;
					nrDestinationsDMA.put(typeDMA, nr - 1);
					break;
				}
			}

			while (destinationNode == null | destinationNode.gateway)
				destinationNode = NodesLookup.randomNodeBetweenDistanceIntervalDMA(network, originNode,
						Parameters.minDistance, Parameters.maxDistance, DMA);

			Pair<NodeGraph, NodeGraph> pair = new Pair<>(originNode, destinationNode);
			OD.add(pair);
			originNode = destinationNode = null;
		}
	}

	/**
	 * Sets up the DMA map with destination types and shares.
	 */
	private void setDMAmap() {

		// Specify the initial capacity based on the number of elements to be added
		destinationsDMA = new HashMap<>(3);
		destinationsDMA.put("work", WORK_SHARE);
		destinationsDMA.put("visit", VISIT_SHARE);
		destinationsDMA.put("random", RANDOM_SHARE);
	}

	/**
	 * Assigns the OD matrix to empirical groups for pedestrian simulation.
	 */
	private void assignODmatrixToEmpiricalGroups() {

		int agentID = 0;
		final ArrayList<EmpiricalAgentsGroup> actualEmpiricalGroups = new ArrayList<>();
		ArrayList<Pair<NodeGraph, NodeGraph>> configurationOD = new ArrayList<>(OD);
		int numAgentsEmpiricalGroup;
		int agentsToAllocate = Parameters.numAgents;

		for (EmpiricalAgentsGroup empiricalGroup : PedSimCity.empiricalGroups) {
			if (!empiricalGroup.groupName.equals(Groups.POPULATION)
					&& !empiricalGroup.groupName.equals(Groups.NULLGROUP))
				actualEmpiricalGroups.add(empiricalGroup);

			if (empiricalGroup.groupName.equals(Groups.POPULATION)
					|| empiricalGroup.groupName.equals(Groups.NULLGROUP)) {
				numAgentsEmpiricalGroup = Parameters.numAgents;
				configurationOD = new ArrayList<>(OD);
			}
			// last group
			else if (actualEmpiricalGroups.size() == PedSimCity.empiricalGroups.size() - 2)
				numAgentsEmpiricalGroup = agentsToAllocate;
			// any other group
			else {
				numAgentsEmpiricalGroup = (int) (Parameters.numAgents * empiricalGroup.share);
				agentsToAllocate -= numAgentsEmpiricalGroup;
			}

			int groupTripsToComplete = numAgentsEmpiricalGroup * Parameters.numberTripsPerAgent;
			ArrayList<Pair<NodeGraph, NodeGraph>> groupOD = new ArrayList<>(
					configurationOD.subList(0, groupTripsToComplete));

			int lowLimit = 0;
			int upLimit = Parameters.numberTripsPerAgent;

			for (int i = 0; i < numAgentsEmpiricalGroup; i++) {
				ArrayList<Pair<NodeGraph, NodeGraph>> thisAgentODs = new ArrayList<>();

				if (Parameters.numberTripsPerAgent == 1)
					thisAgentODs.add(groupOD.get(i));
				else
					thisAgentODs = new ArrayList<>(groupOD.subList(lowLimit, upLimit));

				Agent agent = new Agent(this.state);
				agent.initialiseAgentProperties(empiricalGroup);
				addAgent(agent, agentID, thisAgentODs);

				agentID++;
				lowLimit = upLimit;
				upLimit = lowLimit + Parameters.numberTripsPerAgent;
			}
			configurationOD = new ArrayList<>(configurationOD.subList(groupTripsToComplete, configurationOD.size()));
		}
	}
}
