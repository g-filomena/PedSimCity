package sim.app.geo.PedSimCity;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.javatuples.Pair;

import com.vividsolutions.jts.geom.Envelope;

import sim.app.geo.UrbanSim.Building;
import sim.app.geo.UrbanSim.EdgeGraph;
import sim.app.geo.UrbanSim.Graph;
import sim.app.geo.UrbanSim.NodeGraph;
import sim.app.geo.UrbanSim.NodesLookup;
import sim.app.geo.UrbanSim.VectorLayer;
import sim.engine.SimState;
import sim.engine.Stoppable;
import sim.util.Bag;
import sim.util.geo.MasonGeometry;

/**
 * The  simulation core.
 *
 *
 */

public class PedSimCity extends SimState {
	private static final long serialVersionUID = 1L;

	// Urban elements: graphs, buildings, etc.
	public static VectorLayer roads = new VectorLayer();
	public static VectorLayer buildings = new VectorLayer();
	public static VectorLayer junctions = new VectorLayer();
	public static VectorLayer localLandmarks = new VectorLayer();
	public static VectorLayer globalLandmarks = new VectorLayer();
	public static VectorLayer sightLines = new VectorLayer();
	public static Graph network = new Graph();

	//dual graph
	public static VectorLayer intersectionsDual = new VectorLayer();
	public static VectorLayer centroids = new VectorLayer();
	public static Graph dualNetwork = new Graph();

	// supporting HashMaps, bags and Lists
	public static HashMap<Integer, EdgeGraph> edgesMap = new HashMap<Integer, EdgeGraph>();
	public static HashMap<Integer, NodeGraph> nodesMap = new HashMap<Integer, NodeGraph>();
	public static HashMap<Integer, NodeGraph> centroidsMap =  new HashMap<Integer, NodeGraph>();
	public static HashMap<Integer, Building> buildingsMap = new HashMap<Integer, Building>();
	public static ArrayList<RouteData> routesData = new ArrayList<RouteData>();

	// OD related variables
	ArrayList<Pair<NodeGraph, NodeGraph>> OD = new ArrayList<Pair<NodeGraph, NodeGraph>>();
	static List<Float> distances = new ArrayList<Float>();
	// used only when loading OD sets

	public int numTripsScenario, numAgents, currentJob;
	double height, width, ratio;

	// agents
	public static VectorLayer agents = new VectorLayer();
	ArrayList<Pedestrian> agentsList = new ArrayList<Pedestrian>();
	public static String routeChoiceModels[];

	/** Constructor */

	public PedSimCity(long seed, int job)	{
		super(seed);
		this.currentJob = job;
	}

	/** Initialization **/

	@Override
	public void start()	{

		if (UserParameters.testingSpecificRoutes) routeChoiceModels = UserParameters.routeChoices;
		else if (UserParameters.testingLandmarks) routeChoiceModels = UserParameters.routeChoicesLandmarks;
		if (UserParameters.testingLandmarks || UserParameters.testingSpecificRoutes) numAgents = routeChoiceModels.length;
		else numAgents = UserParameters.numAgents;
		super.start();

		// prepare environment
		Envelope MBR = null;
		MBR = roads.getMBR();
		MBR.expandToInclude(buildings.getMBR());
		roads.setMBR(MBR);

		// populate
		populate();
		agents.setMBR(MBR);
	}

	public void populate() {
		// prepare to start the simulation - OD Matrix
		int numTripsScenario = 1;
		if (UserParameters.testingSpecificRoutes) {
			UserParameters.setTestingMatrix();
			numTripsScenario = UserParameters.OR.size();
		}
		//		else if (UserParameters.testingLandmarks) numTripsScenario = distances.size();
		else if (UserParameters.testingLandmarks) numTripsScenario = 10;
		ArrayList<ArrayList<NodeGraph>> listSequences = new ArrayList<ArrayList<NodeGraph>> ();

		for (int i = 0; i < numTripsScenario; i++) {
			NodeGraph originNode = null;
			NodeGraph destinationNode = null;

			if (UserParameters.readingFromPrevious || UserParameters.testingSpecificRoutes) {
				originNode = nodesMap.get(UserParameters.OR.get(i));
				destinationNode = nodesMap.get(UserParameters.DE.get(i));
			}
			else if (UserParameters.testingLandmarks) {
				while (originNode == null) originNode = NodesLookup.randomNode(network);
				destinationNode = NodesLookup.randomNodeFromDistancesSet(network, originNode, distances);
				AgentProperties apFictionary = new AgentProperties();
				apFictionary.landmarkBasedNavigation = true;
				apFictionary.usingLocalLandmarks = true;
				apFictionary.typeLandmarks = "local";
				ArrayList<NodeGraph> sequence = LandmarkNavigation.onRouteMarks(originNode, destinationNode, apFictionary);
				listSequences.add(sequence);
			}
			Pair<NodeGraph, NodeGraph> pair = new Pair<NodeGraph, NodeGraph> (originNode, destinationNode);
			OD.add(pair);
		}


		for (int i = 0; i < numAgents; i++)	{
			AgentProperties ap = new AgentProperties();
			ap.setProperties(routeChoiceModels[i]);
			ap.setOD(OD, listSequences);
			ap.agentID = i;

			Pedestrian a = new Pedestrian(this, ap);
			MasonGeometry newGeometry = a.getGeometry();
			newGeometry.isMovable = true;
			agents.addGeometry(newGeometry);
			agentsList.add(a);
			a.getGeometry().setUserData(a);

			Stoppable stop = schedule.scheduleRepeating(a);
			a.setStoppable(stop);
			schedule.scheduleRepeating(agents.scheduleSpatialIndexUpdater(), Integer.MAX_VALUE, 1.0);
		}
	}

	@Override
	public void finish()
	{
		try {ImportingExporting.saveResults(this.currentJob);}
		catch (IOException e) {e.printStackTrace();}
		super.finish();
	}

	// Main function allows simulation to be run in stand-alone, non-GUI mode/
	public static void main(String[] args) throws IOException
	{
		int jobs = UserParameters.jobs;
		ImportingExporting.importFiles();
		prepareLayers();

		for (int job = 0; job < jobs; job++) {
			System.out.println("Run nr.. "+job);
			for (Object o : network.getEdges())	{
				EdgeGraph edge = (EdgeGraph) o;
				edge.resetDensities();
			}
			if (UserParameters.readingFromPrevious) ImportingExporting.readingOD(UserParameters.outputFolderRoutes+"routes_"+job+".csv");
			SimState state = new PedSimCity(System.currentTimeMillis(), job);
			state.start();
			while (state.schedule.step(state)) {}
		}
		System.exit(0);
	}


	static public void prepareLayers() {

		// Element 1 - Nodes: assign scores and attributes to nodes

		for (MasonGeometry  nodeGeometry : junctions.geometriesList) {
			// street junctions and betweenness centrality
			NodeGraph node = network.findNode(nodeGeometry.geometry.getCoordinate());
			node.setID(nodeGeometry.getIntegerAttribute("nodeID"));
			node.masonGeometry = nodeGeometry;
			node.primalEdge = null;
			//centrality
			try {node.centrality = nodeGeometry.getDoubleAttribute("Bc_multi");}
			catch (java.lang.NullPointerException e){node.centrality = nodeGeometry.getDoubleAttribute("Bc_Rd");}
			// set adjacent edges and nodes
			node.setNeighbouringComponents();
			nodesMap.put(node.getID(), node);
		}
		// generate the centrality map of the graph
		network.generateCentralityMap();

		// Element 2 - Landmarks -
		if (UserParameters.testingLandmarks) {

			Bag bagLocal = LandmarkNavigation.getLandmarks(buildings, UserParameters.localLandmarkThreshold, "local");
			for (Object l : bagLocal) {
				MasonGeometry landmark = (MasonGeometry) l;
				localLandmarks.addGeometry(landmark);
			}

			Bag bagGlobal = LandmarkNavigation.getLandmarks(buildings, UserParameters.globalLandmarkThreshold, "global");
			for (Object g : bagGlobal) {
				MasonGeometry landmark = (MasonGeometry) g;
				globalLandmarks.addGeometry(landmark);
			}

			for (MasonGeometry buildingGeometry : buildings.geometriesList) {
				Building building = new Building();
				building.buildingID = buildingGeometry.getIntegerAttribute("buildingID");
				building.globalLandmarkness = buildingGeometry.getDoubleAttribute("gScore_sc");
				building.localLandmarkness = buildingGeometry.getDoubleAttribute("lScore_sc");
				building.landUse = buildingGeometry.getStringAttribute("land_use");
				building.geometry = buildingGeometry;
				buildingsMap.put(building.buildingID, building);
			}

			// Integrate landmarks into the street network
			List<Integer> globalLandmarksID = globalLandmarks.getIntColumn("buildingID");
			VectorLayer sightLinesLight = sightLines.selectFeatures("buildingID", globalLandmarksID, true);
			// free up memory
			sightLines = null;

			System.out.println("incorporating local landmarks");
			network.setLocalLandmarkness(localLandmarks, buildingsMap, UserParameters.distanceNodeLandmark);
			System.out.println("incorporating global landmarks");
			network.setGlobalLandmarkness(globalLandmarks, buildingsMap, UserParameters.distanceAnchors, sightLinesLight,
					UserParameters.nrAnchors);
		}

		// Element 3 - Street segments: assign attributes
		for (Object o : network.getEdges())	{
			EdgeGraph edge = (EdgeGraph) o;
			int edgeID = edge.getIntegerAttribute("edgeID");
			edge.setID(edgeID);
			edgesMap.put(edgeID, edge);
		}

		// Element 3A - Centroids (Dual Graph): assign edgeID to centroids in the dual graph
		for (MasonGeometry centroidGeometry : centroids.geometriesList) {
			int edgeID = centroidGeometry.getIntegerAttribute("edgeID");
			NodeGraph cen = dualNetwork.findNode(centroidGeometry.geometry.getCoordinate());
			cen.masonGeometry = centroidGeometry;
			cen.setID(edgeID);
			cen.primalEdge = edgesMap.get(edgeID);
			edgesMap.get(edgeID).dualNode = cen;
			centroidsMap.put(edgeID, cen);
			cen.setNeighbouringComponents();
		}

		for (Object o : dualNetwork.getEdges()) {
			EdgeGraph edge = (EdgeGraph) o;
			edge.deflectionDegrees =  edge.getDoubleAttribute("deg");
		}
		System.out.println("Creation environment completed");
	}
}