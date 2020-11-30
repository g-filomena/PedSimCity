package sim.app.geo.pedSimCity;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;

import org.javatuples.Pair;

import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.planargraph.DirectedEdgeStar;

import sim.app.geo.urbanSim.Angles;
import sim.app.geo.urbanSim.Building;
import sim.app.geo.urbanSim.EdgeGraph;
import sim.app.geo.urbanSim.Graph;
import sim.app.geo.urbanSim.NodeGraph;
import sim.app.geo.urbanSim.NodesLookup;
import sim.app.geo.urbanSim.SubGraph;
import sim.app.geo.urbanSim.VectorLayer;
import sim.engine.SimState;
import sim.engine.Stoppable;
import sim.util.Bag;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
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
	public static VectorLayer barriers = new VectorLayer();
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
	public static HashMap<Integer, Region> regionsMap = new HashMap<Integer, Region>();
	public static HashMap<Integer, Building> buildingsMap = new HashMap<Integer, Building>();
	public static HashMap<Integer, Barrier> barriersMap = new HashMap<Integer, Barrier>();
	public static HashMap<Pair<NodeGraph, NodeGraph>, Gateway> gatewaysMap = new HashMap<Pair<NodeGraph, NodeGraph>, Gateway>();

	public static ArrayList<MasonGeometry> startingNodes = new ArrayList<MasonGeometry> ();
	public static ArrayList<RouteData> routesData = new ArrayList<RouteData>();

	// OD related variables
	ArrayList<Pair<NodeGraph, NodeGraph>> OD = new ArrayList<Pair<NodeGraph, NodeGraph>>();
	static List<Float> distances = new ArrayList<Float>();
	// used only when loading OD sets


	public int numTripsScenario, numAgents, currentJob;
	ArrayList<Integer> groupBounds = new ArrayList<Integer>();
	ArrayList<Group> groups = new ArrayList<Group>();

	double height, width, ratio;

	// agents
	public static VectorLayer agents = new VectorLayer();
	ArrayList<Pedestrian> agentsList = new ArrayList<Pedestrian>();
	public static String criteria[];

	/** Constructor */

	public PedSimCity(long seed, int job)	{
		super(seed);
		this.currentJob = job;
	}

	/** Initialization **/

	@Override
	public void start()	{

		if (UserParameters.testingSpecificRoutes) criteria = UserParameters.routeChoices;
		else if (UserParameters.testingRegions) criteria = UserParameters.routeChoicesRegions;
		else if (UserParameters.testingLandmarks) criteria = UserParameters.routeChoicesLandmarks;
		if (UserParameters.testingRegions || UserParameters.testingLandmarks || UserParameters.testingSpecificRoutes) numAgents = criteria.length;
		else numAgents = UserParameters.numAgents;
		super.start();

		// prepare environment
		Envelope MBR = null;
		MBR = roads.getMBR();
		MBR.expandToInclude(buildings.getMBR());
		MBR.expandToInclude(barriers.getMBR());
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
		else if (UserParameters.testingLandmarks) numTripsScenario = distances.size();
		else if (UserParameters.testingRegions) numTripsScenario = 2000;

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
			}
			else if (UserParameters.testingRegions) {
				while ((destinationNode == null) || (originNode == destinationNode)) {
					while (originNode == null) originNode = NodesLookup.randomNode(network, startingNodes);
					destinationNode = NodesLookup.randomNodeBetweenLimits(network, originNode, 1000, 3000);
				}
			}
			if (UserParameters.testingLandmarks) {
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
		if (UserParameters.fiveElements) prepareGroups();

		for (int i = 0; i < numAgents; i++)	{
			AgentProperties ap = new AgentProperties();

			if (UserParameters.fiveElements) {

				for (int g : groupBounds){
					if (i <= g) {
						groups.get(groupBounds.indexOf(g)).setAgentProperties(ap);
						break;
					}
				}

				ap.setLocations();
			}
			else {
				ap.setProperties(criteria[i]);
				ap.setOD(OD, listSequences);
				ap.agentID = i;
			}

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
		try {ImportingExporting.saveCSV(this.currentJob);}
		catch (IOException e) {e.printStackTrace();}
		super.finish();
	}

	// Main function allows simulation to be run in stand-alone, non-GUI mode/
	public static void main(String[] args) throws IOException
	{
		int jobs = UserParameters.jobs;
		if (UserParameters.testingLandmarks) jobs = 50;
		if (UserParameters.testingRegions) jobs = 5;
		ImportingExporting.importFiles();
		prepareLayers();

		for (int i = 0; i < jobs; i++) {
			System.out.println("Run nr.. "+i);
			if (UserParameters.readingFromPrevious) ImportingExporting.readingOD(UserParameters.outputFolder+"routes_angularChange_"+i+".csv");
			SimState state = new PedSimCity(System.currentTimeMillis(), i);
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
			try {node.centrality = nodeGeometry.getDoubleAttribute("Bc_multi");}
			catch (java.lang.NullPointerException e){node.centrality = nodeGeometry.getDoubleAttribute("Bc_Rd");}
			// set adjacent edges and nodes
			node.setNeighbouringComponents();
			nodesMap.put(node.getID(), node);
		}
		// generate the centrality map of the graph
		network.generateCentralityMap();

		// Element 2 - Landmarks -
		if (UserParameters.testingLandmarks || UserParameters.fiveElements ) {

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
				building.landUse = buildingGeometry.getStringAttribute("land_use");
				building.geometry = buildingGeometry;

				Bag nearestNodes = junctions.getObjectsWithinDistance(building.geometry, 500);
				MasonGeometry closest = null;
				double lowestDistance = 501.0;

				for (Object nN : nearestNodes) {
					MasonGeometry node = (MasonGeometry) nN;
					double distance = node.geometry.distance(buildingGeometry.geometry);

					if (distance < lowestDistance) {
						closest = node;
						lowestDistance = node.geometry.distance(buildingGeometry.geometry);
					}
				}
				if (closest == null) building.node = null;
				else building.node = network.findNode(closest.getGeometry().getCoordinate());
				buildingsMap.put(building.buildingID, building);
			}

			// Integrating landmarks into the street network
			List<Integer> globalLandmarksID = globalLandmarks.getIntColumn("buildingID");
			VectorLayer sightLinesLight = sightLines.selectFeatures("buildingID", globalLandmarksID, true);
			// free up memory
			sightLines = null;

			System.out.println("incorporating local landmarks");
			network.setLocalLandmarkness(localLandmarks, buildingsMap, UserParameters.distanceNodeLandmark);
			System.out.println("incorporating global landmarks");
			network.setGlobalLandmarkness(globalLandmarks, buildingsMap, UserParameters.distanceAnchors, sightLinesLight);
		}


		// Identify gateways
		if (UserParameters.testingRegions || UserParameters.fiveElements ) {

			for (NodeGraph node : nodesMap.values()) {
				node.region = node.masonGeometry.getIntegerAttribute("district");

				if (regionsMap.get(node.region) == null) {
					Region region = new Region();
					regionsMap.put(node.region, region);
				}
			}

			for (NodeGraph node : nodesMap.values()) {

				Integer gateway = node.masonGeometry.getIntegerAttribute("gateway"); // 1 or 0
				Integer region = node.region;
				if (gateway == 0) {
					// nodes that are not gateways
					startingNodes.add(node.masonGeometry);
					continue;
				}

				for (EdgeGraph bridge : node.getEdges()) {

					NodeGraph oppositeNode = (NodeGraph) bridge.getOppositeNode(node);
					int possibleRegion = oppositeNode.region;
					if (possibleRegion == region) continue;

					Gateway gd = new Gateway();
					gd.node = node;
					gd.edgeID = bridge.getID();
					gd.gatewayID = new Pair<NodeGraph, NodeGraph>(node, oppositeNode);
					gd.regionTo = possibleRegion;
					gd.entry = oppositeNode;
					gd.distance = bridge.getLength();
					gd.entryAngle = Angles.angle(node, oppositeNode);

					regionsMap.get(region).gateways.add(gd);
					gatewaysMap.put(new Pair<NodeGraph, NodeGraph>(node, oppositeNode), gd);
					node.gateway = true;
				}
				node.adjacentRegions = node.getAdjacentRegion();
			}
		}

		// Element 3 - Street segments: assign attributes
		for (Object o : network.getEdges())	{

			EdgeGraph edge = (EdgeGraph) o;
			int edgeID = edge.getIntegerAttribute("edgeID");
			edge.setID(edgeID);
			edge.resetDensities();

			if (UserParameters.testingRegions | UserParameters.fiveElements)  {
				edge.setBarriers();
				// add edges to the regions' information

				if (edge.u.region == edge.v.region)	{
					int region = edge.u.region;
					edge.region = region;
					regionsMap.get(region).edges.add(edge);
				}
				else edge.region = 999999;
			}
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

		// Element 4 - Regions: create regions' subgraphs and store other information about regions (landmarks, barriers)
		if (UserParameters.testingRegions || UserParameters.fiveElements) {

			for (Entry<Integer, Region> entry : regionsMap.entrySet()) {

				ArrayList<EdgeGraph> edgesRegion = entry.getValue().edges;
				ArrayList<EdgeGraph> dualEdgesRegion = new ArrayList<EdgeGraph>();
				SubGraph primalGraph = new SubGraph(network, edgesRegion);

				for (Object e: edgesRegion)	{
					EdgeGraph edge = (EdgeGraph) e;
					NodeGraph cen = edge.dualNode;
					cen.region = entry.getKey();
					DirectedEdgeStar dirEdges = cen.getOutEdges();

					for (Object dE : dirEdges.getEdges()) {
						GeomPlanarGraphDirectedEdge dEdge = (GeomPlanarGraphDirectedEdge) dE;
						dualEdgesRegion.add((EdgeGraph) dEdge.getEdge());
					}
				}

				SubGraph dualGraph = new SubGraph(dualNetwork, dualEdgesRegion);
				primalGraph.setSubGraphBarriers();
				regionsMap.get(entry.getKey()).primalGraph = primalGraph;
				regionsMap.get(entry.getKey()).dualGraph = dualGraph;

				// set the landmarks of this region
				if (UserParameters.fiveElements) {
					primalGraph.setSubGraphLandmarks();
					VectorLayer regionNetwork = new VectorLayer();
					for (EdgeGraph edge : edgesRegion) regionNetwork.addGeometry(edge.masonGeometry);
					regionsMap.get(entry.getKey()).regionNetwork = regionNetwork;
					Bag buildings = LandmarkNavigation.getBuildings(null, null, entry.getKey());
					regionsMap.get(entry.getKey()).buildings = buildings;
					regionsMap.get(entry.getKey()).assignLandmarks();
					regionsMap.get(entry.getKey()).computeComplexity("local");
				}
			}

			// Element 5 - Barriers: create barriers map
			for (MasonGeometry barrierGeometry : barriers.geometriesList ) {
				int barrierID = barrierGeometry.getIntegerAttribute("barrierID");
				Barrier barrier = new Barrier();
				barrier.masonGeometry = barrierGeometry;
				barrier.type =  barrierGeometry.getStringAttribute("type");
				ArrayList<EdgeGraph> edgesAlong = new ArrayList<EdgeGraph>();
				for (EdgeGraph edge : network.edgesGraph)
					if ((edge.barriers != null) && (edge.barriers.contains(barrierID))) edgesAlong.add(edge);
				barrier.edgesAlong = edgesAlong;
				barrier.type =  barrierGeometry.getStringAttribute("type");
				barriersMap.put(barrierID, barrier);
			}
		}
		System.out.println("Creation environment completed");
	}

	public void prepareGroups() {


		List<Double> composition = Arrays.asList(UserParameters.composition);
		int accumulated = 0;
		for (double p : composition) {

			int indexOf = composition.indexOf(p);
			Group group = new Group();
			group.configureGroup(indexOf);
			groups.add(group);
			accumulated += (int) (numAgents*p);
			if (indexOf == 0) groupBounds.add((int) (numAgents*p));
			else groupBounds.add(accumulated);
		}
		if (groupBounds.get(groupBounds.size() - 1) == numAgents) groupBounds.set(groupBounds.size() - 1, numAgents);
	}

}