package pedSim.engine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.javatuples.Pair;
import org.locationtech.jts.geom.Envelope;

import pedSim.agents.Agent;
import pedSim.agents.EmpiricalAgentsGroup;
import pedSim.cognitiveMap.Barrier;
import pedSim.cognitiveMap.Gateway;
import pedSim.cognitiveMap.Region;
import pedSim.utilities.RouteData;
import pedSim.utilities.StringEnum.RouteChoice;
import sim.engine.SimState;
import sim.engine.Stoppable;
import sim.field.geo.VectorLayer;
import sim.graph.Building;
import sim.graph.EdgeGraph;
import sim.graph.Graph;
import sim.graph.NodeGraph;
import sim.util.geo.MasonGeometry;

/**
 * The PedSimCity class represents the main simulation environment.
 */
public class PedSimCity extends SimState {
	private static final long serialVersionUID = 1L;

	// Urban elements: graphs, buildings, etc.
	public static VectorLayer roads = new VectorLayer();
	public static VectorLayer buildings = new VectorLayer();
	public static VectorLayer barriers = new VectorLayer();
	public static VectorLayer junctions = new VectorLayer();
	public static VectorLayer sightLines = new VectorLayer();

	public static Graph network = new Graph();
	public static Graph dualNetwork = new Graph();

	// dual graph
	public static VectorLayer intersectionsDual = new VectorLayer();
	public static VectorLayer centroids = new VectorLayer();

	// supporting HashMaps, bags and Lists
	public static HashMap<Integer, Building> buildingsMap = new HashMap<>();
	public static HashMap<Integer, Region> regionsMap = new HashMap<>();
	public static HashMap<Integer, Barrier> barriersMap = new HashMap<>();
	public static HashMap<Pair<NodeGraph, NodeGraph>, Gateway> gatewaysMap = new HashMap<>();
	public static HashMap<Integer, NodeGraph> nodesMap = new HashMap<>();
	public static HashMap<Integer, EdgeGraph> edgesMap = new HashMap<>();

	public static HashMap<Integer, NodeGraph> centroidsMap = new HashMap<>();
	public static ArrayList<RouteData> routesData = new ArrayList<>();
	public static HashMap<EdgeGraph, ArrayList<Agent>> edgesVolume = new HashMap<>();

	// OD related variables
	public static List<Float> distances = new ArrayList<>();
	public static ArrayList<MasonGeometry> startingNodes = new ArrayList<>();
	// used only when loading OD sets

	public int currentJob;
	public static ArrayList<EmpiricalAgentsGroup> empiricalGroups = new ArrayList<>();

	// agents
	public static VectorLayer agents = new VectorLayer();
	public static ArrayList<Agent> agentsList = new ArrayList<>();

	public static Envelope MBR = null;

	/**
	 * Constructs a new instance of the PedSimCity simulation environment.
	 *
	 * @param seed The random seed for the simulation.
	 * @param job  The current job number for multi-run simulations.
	 */
	public PedSimCity(long seed, int job) {
		super(seed);
		this.currentJob = job;
	}

	/**
	 * Initialises the simulation by defining the simulation mode, initialising edge
	 * volumes, and preparing the simulation environment. It then proceeds to
	 * populate the environment with agents and starts the agent movement.
	 */
	@Override
	public void start() {
		Parameters.defineMode();
		initializeEdgeVolumes();
		super.start();
		prepareEnvironment();
		populateEnvironment();
		startMovingAgents();
	}

	/**
	 * Initialises the edge volumes for the simulation. This method assigns initial
	 * volume values to edges based on the selected route choice models or empirical
	 * agent groups. If the simulation is not empirical, it initialises volumes
	 * based on the route choice models. If the simulation is empirical-based, it
	 * initialises volumes based on empirical agent groups.
	 */
	private void initializeEdgeVolumes() {
		if (!Parameters.empirical) {
			for (RouteChoice routeChoice : Parameters.routeChoiceModels) {
				for (Object o : network.getEdges()) {
					EdgeGraph edge = (EdgeGraph) o;
					edge.volumes.put(routeChoice.toString(), 0);
				}
			}
		} else {
			for (EmpiricalAgentsGroup empiricalGroup : empiricalGroups) {
				for (Object o : network.getEdges()) {
					EdgeGraph edge = (EdgeGraph) o;
					edge.volumes.put(empiricalGroup.groupName, 0);
				}
			}
		}
	}

	/**
	 * Prepares the environment for the simulation. This method sets up the minimum
	 * bounding rectangle (MBR) to encompass both the road and building layers and
	 * updates the MBR of the road layer accordingly.
	 */
	private void prepareEnvironment() {
		MBR = roads.getMBR();
		if (!buildings.getGeometries().isEmpty());
			MBR.expandToInclude(buildings.getMBR());
		if (!barriers.getGeometries().isEmpty());
			MBR.expandToInclude(barriers.getMBR());
		roads.setMBR(MBR);
	}

	/**
	 * Populates the simulation environment with agents and other entities based on
	 * the selected simulation parameters. This method uses the Populate class to
	 * generate the agent population.
	 */
	private void populateEnvironment() {
		Populate populate = new Populate();
		if (Parameters.testing)
			populate.populateTests(this);
		if (Parameters.empirical)
			populate.populateEmpiricalGroups(this);
		else {
			// Populate.populate();
		}
	}

	/**
	 * Starts moving agents in the simulation. This method schedules agents for
	 * repeated movement updates and sets up the spatial index for agents.
	 */
	private void startMovingAgents() {
		for (Agent agent : agentsList) {
			Stoppable stop = schedule.scheduleRepeating(agent);
			agent.setStoppable(stop);
			schedule.scheduleRepeating(agents.scheduleSpatialIndexUpdater(), Integer.MAX_VALUE, 1.0);
		}
		agents.setMBR(MBR);
	}

	/**
	 * Completes the simulation by saving results and performing cleanup operations.
	 */
	@Override
	public void finish() {
		try {
			Export.saveResults(this.currentJob);
		} catch (final Exception e) {
			e.printStackTrace();
		}
		super.finish();
	}

	/**
	 * The main function that allows the simulation to be run in stand-alone,
	 * non-GUI mode.
	 *
	 * @param args Command-line arguments.
	 * @throws Exception If an error occurs during simulation execution.
	 */
	public static void main(String[] args) throws Exception {

		Import importer = new Import();
		importer.importFiles();
		Environment.prepare();

		for (int job = 0; job < Parameters.jobs; job++) {
			System.out.println("Run nr.. " + job);
			for (EdgeGraph edge : network.getEdges())
				edge.resetVolumes();

			final SimState state = new PedSimCity(System.currentTimeMillis(), job);
			state.start();
			while (state.schedule.step(state)) {
			}
		}
		System.exit(0);
	}
}