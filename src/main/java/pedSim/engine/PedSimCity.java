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

	// OD related variables
	public static List<Float> distances = new ArrayList<>();
	public static ArrayList<MasonGeometry> startingNodes = new ArrayList<>();
	// used only when loading OD sets

	public int currentJob;

	public FlowHandler flowHandler;
	public static ArrayList<EmpiricalAgentsGroup> empiricalGroups = new ArrayList<>();
	public static Envelope MBR = null;

	public VectorLayer agents;
	public ArrayList<Agent> agentsList;

	/**
	 * Constructs a new instance of the PedSimCity simulation environment.
	 *
	 * @param seed The random seed for the simulation.
	 * @param job  The current job number for multi-run simulations.
	 */
	public PedSimCity(long seed, int job) {
		super(seed);
		this.currentJob = job;
		this.flowHandler = new FlowHandler(job);
		this.agentsList = new ArrayList<>();
		this.agents = new VectorLayer(); // create a new vector layer for each job
	}

	public List<Agent> getAgentsList() {
		return agentsList;
	}

	/**
	 * Initialises the simulation by defining the simulation mode, initialising edge
	 * volumes, and preparing the simulation environment. It then proceeds to
	 * populate the environment with agents and starts the agent movement.
	 */
	@Override
	public void start() {
		super.start();
		prepareEnvironment();
		populateEnvironment();
		startMovingAgents();
	}

	/**
	 * Prepares the environment for the simulation. This method sets up the minimum
	 * bounding rectangle (MBR) to encompass both the road and building layers and
	 * updates the MBR of the road layer accordingly.
	 */
	private void prepareEnvironment() {
		MBR = roads.getMBR();
		if (!buildings.getGeometries().isEmpty())
			MBR.expandToInclude(buildings.getMBR());
		if (!barriers.getGeometries().isEmpty())
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
		}
		// Populate.populate();
	}

	/**
	 * Starts moving agents in the simulation. This method schedules agents for
	 * repeated movement updates and sets up the spatial index for agents.
	 */
	private void startMovingAgents() {
		for (Agent agent : this.agentsList) {
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
			Export export = new Export();
			export.saveResults(flowHandler);
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

		Parameters.defineMode();
		Import importer = new Import();
		importer.importFiles();
		Environment.prepare();

		for (int job = 0; job < Parameters.jobs; job++) {
			System.out.println("Run nr.. " + job);
			final SimState state = new PedSimCity(System.currentTimeMillis(), job);
			state.start();
			while (state.schedule.step(state)) {
			}
		}
		System.exit(0);
	}
}