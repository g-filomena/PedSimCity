package pedsimcity.main;

//import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;

import org.javatuples.Pair;

import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.planargraph.DirectedEdgeStar;

import pedsimcity.agents.AgentGroupProperties;
import pedsimcity.agents.AgentProperties;
import pedsimcity.agents.Group;
import pedsimcity.agents.Pedestrian;
import pedsimcity.elements.Barrier;
import pedsimcity.elements.Building;
import pedsimcity.elements.Gateway;
import pedsimcity.elements.Region;
import pedsimcity.graph.EdgeGraph;
import pedsimcity.graph.Graph;
import pedsimcity.graph.NodeGraph;
import pedsimcity.graph.SubGraph;
import pedsimcity.routeChoice.LandmarkNavigation;
import pedsimcity.utilities.Angles;
import pedsimcity.utilities.NodesLookup;
import pedsimcity.utilities.RouteData;
import pedsimcity.utilities.VectorLayer;
import sim.engine.SimState;
import sim.engine.Stoppable;
import sim.util.Bag;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.MasonGeometry;

/**
 * The simulation core.
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
	public static VectorLayer barriers = new VectorLayer();
	public static Graph network = new Graph();

	// dual graph
	public static VectorLayer intersectionsDual = new VectorLayer();
	public static VectorLayer centroids = new VectorLayer();
	public static Graph dualNetwork = new Graph();

	// supporting HashMaps, bags and Lists
	public static HashMap<Integer, EdgeGraph> edgesMap = new HashMap<>();
	public static HashMap<Integer, NodeGraph> nodesMap = new HashMap<>();
	public static HashMap<Integer, NodeGraph> centroidsMap = new HashMap<>();
	public static ArrayList<RouteData> routesData = new ArrayList<>();

	public static HashMap<Integer, Building> buildingsMap = new HashMap<>();
	public static HashMap<Integer, Region> regionsMap = new HashMap<>();
	public static HashMap<Integer, Barrier> barriersMap = new HashMap<>();
	public static HashMap<Pair<NodeGraph, NodeGraph>, Gateway> gatewaysMap = new HashMap<>();
	public static HashMap<EdgeGraph, ArrayList<Pedestrian>> edgesVolume = new HashMap<>();

	// OD related variables
	ArrayList<Pair<NodeGraph, NodeGraph>> OD = new ArrayList<>();
	static List<Float> distances = new ArrayList<>();
	public static ArrayList<MasonGeometry> startingNodes = new ArrayList<>();
	// used only when loading OD sets

	public int numTripsScenario, numAgents, currentJob;
	double height, width, ratio;
	ArrayList<Integer> groupBounds = new ArrayList<>();
	static ArrayList<Group> groups = new ArrayList<>();

	// agents
	public static VectorLayer agents = new VectorLayer();
	public ArrayList<Pedestrian> agentsList = new ArrayList<>();
	public static String routeChoiceModels[];

	/** Constructor */

	public PedSimCity(long seed, int job) {
		super(seed);
		this.currentJob = job;
	}

	/** Initialization **/

	@Override
	public void start() {
		if (UserParameters.testingSpecificRoutes && !UserParameters.empiricalABM)
			routeChoiceModels = UserParameters.routeChoices;
		else if (UserParameters.testingRegions)
			routeChoiceModels = UserParameters.routeChoicesRegions;
		else if (UserParameters.testingLandmarks)
			routeChoiceModels = UserParameters.routeChoicesLandmarks;
		else
			routeChoiceModels = UserParameters.routeChoices;

		if (UserParameters.testingRegions || UserParameters.testingLandmarks
				|| UserParameters.testingSpecificRoutes && !UserParameters.empiricalABM)
			this.numAgents = routeChoiceModels.length;
		else
			this.numAgents = UserParameters.numAgents;

		// check consistency settings
		if (!UserParameters.empiricalABM)
			for (final String routeChoice : routeChoiceModels)
				for (final Object o : network.getEdges()) {
					final EdgeGraph edge = (EdgeGraph) o;
					edge.volumes.put(routeChoice, 0);
				}
		else
			for (final Group group : groups)
				for (final Object o : network.getEdges()) {
					final EdgeGraph edge = (EdgeGraph) o;
					edge.volumes.put(group.groupName, 0);
				}

		super.start();

		// prepare environment
		Envelope MBR = null;
		MBR = roads.getMBR();
		MBR.expandToInclude(buildings.getMBR());
		roads.setMBR(MBR);

		// populate
		if (UserParameters.empiricalABM)
			this.populateGroups();
		else
			this.populate();

		// moving
		for (final Pedestrian pedestrian : this.agentsList) {
			final Stoppable stop = this.schedule.scheduleRepeating(pedestrian);
			pedestrian.setStoppable(stop);
			this.schedule.scheduleRepeating(agents.scheduleSpatialIndexUpdater(), Integer.MAX_VALUE, 1.0);
		}
		agents.setMBR(MBR);
	}

	public void populate() {
		// prepare to start the simulation - OD Matrix
		int numTripsScenario = 1;
		if (UserParameters.testingSpecificRoutes) {
			UserParameters.setTestingMatrix();
			numTripsScenario = UserParameters.OR.size();
		} else if (UserParameters.testingRegions)
			numTripsScenario = 2000;
		else if (UserParameters.testingLandmarks)
			numTripsScenario = distances.size();
		else if (UserParameters.testingModels)
			numTripsScenario = UserParameters.numTrips;

		for (int i = 0; i < numTripsScenario; i++) {
			NodeGraph originNode = null;
			NodeGraph destinationNode = null;

			if (UserParameters.testingSpecificRoutes && !UserParameters.empiricalABM) {
				originNode = nodesMap.get(UserParameters.OR.get(i));
				destinationNode = nodesMap.get(UserParameters.DE.get(i));
			} else if (UserParameters.testingLandmarks) {
				while (originNode == null)
					originNode = NodesLookup.randomNode(network);
				destinationNode = NodesLookup.randomNodeFromDistancesSet(network, originNode, distances);
			} else if (UserParameters.testingRegions) {
				while (originNode == null)
					originNode = NodesLookup.randomNode(network, startingNodes);
				while (destinationNode == null)
					destinationNode = NodesLookup.randomNodeBetweenLimits(network, originNode, 1000, 3000);
			} else if (UserParameters.testingModels) {
				while (originNode == null)
					originNode = NodesLookup.randomNode(network);
				while (destinationNode == null)
					destinationNode = NodesLookup.randomNodeBetweenLimits(network, originNode,
							UserParameters.minDistance, UserParameters.maxDistance);
			}

			final Pair<NodeGraph, NodeGraph> pair = new Pair<>(originNode, destinationNode);
			this.OD.add(pair);
		}

		for (int i = 0; i < this.numAgents; i++) {
			final AgentProperties ap = new AgentProperties();
			ap.setRouteChoice(routeChoiceModels[i]);
			ap.OD = this.OD;

			final Pedestrian a = new Pedestrian(this, ap);
			final MasonGeometry newGeometry = a.getGeometry();
			newGeometry.isMovable = true;
			a.agentID = i;

			agents.addGeometry(newGeometry);
			this.agentsList.add(a);
		}
	}

	public void populateGroups() {

		this.numAgents = UserParameters.numAgents;
		final int numODs = this.numAgents * UserParameters.numTrips;
		NodeGraph originNode = null;
		NodeGraph destinationNode = null;

		if (!UserParameters.testingSpecificRoutes && !UserParameters.usingDMA)
			for (int i = 0; i < numODs; i++) {

				while (originNode == null)
					originNode = NodesLookup.randomNode(network);
				while (destinationNode == null)
//				destinationNode = NodesLookup.randomNodeBetweenLimits(network, originNode,
//						UserParameters.minDistance, UserParameters.maxDistance);
					destinationNode = NodesLookup.randomSalientNodeBetweenLimits(network, originNode,
							UserParameters.minDistance, UserParameters.maxDistance, 0.75);

				final Pair<NodeGraph, NodeGraph> pair = new Pair<>(originNode, destinationNode);
				this.OD.add(pair);
				originNode = destinationNode = null;
			}

		else if (UserParameters.usingDMA) {
			UserParameters.setDMAmap();
			final HashMap<String, Integer> nrDestinationsDMA = new HashMap<>();
			final int left = numODs;
			String DMA = "";
			for (final String s : UserParameters.destinationsDMA.keySet()) {
				int nr = (int) (UserParameters.destinationsDMA.get(s) * numODs);
				if (nr > left)
					nr = left;
				nrDestinationsDMA.put(s, nr);
			}

			for (int i = 0; i < numODs; i++) {
				while (originNode == null)
					originNode = NodesLookup.randomNodeDMA(network, null, 0.0, 0.0, "live");

				for (final String s : nrDestinationsDMA.keySet()) {
					final int nr = nrDestinationsDMA.get(s);
					if (nr < 1)
						continue;
					else {
						DMA = s;
						nrDestinationsDMA.put(s, nr - 1);
						break;
					}
				}

				while (destinationNode == null)
					destinationNode = NodesLookup.randomNodeDMA(network, originNode, UserParameters.minDistance,
							UserParameters.maxDistance, DMA);

				final Pair<NodeGraph, NodeGraph> pair = new Pair<>(originNode, destinationNode);
				this.OD.add(pair);
				originNode = destinationNode = null;
			}
		}

		else if (UserParameters.testingSpecificRoutes) {
			UserParameters.setTestingMatrix();
			for (int nA = 0; nA < this.numAgents; nA++)
				for (final int oR : UserParameters.OR) {
					originNode = nodesMap.get(oR);
					final int ix = UserParameters.OR.indexOf(oR);
					destinationNode = nodesMap.get(UserParameters.DE.get(ix));
					final Pair<NodeGraph, NodeGraph> pair = new Pair<>(originNode, destinationNode);
					this.OD.add(pair);
				}
		}

		int counter = 0;
		final ArrayList<Group> actualGroups = new ArrayList<>();
		ArrayList<Pair<NodeGraph, NodeGraph>> configurationOD = new ArrayList<>(this.OD);
		int groupAgents;
		int left = this.numAgents;

		for (final Group group : groups) {
			if (!group.groupName.equals("population") && !group.groupName.equals("nullGroup"))
				actualGroups.add(group);

			if (group.groupName.equals("population") || group.groupName.equals("nullGroup")) {
				groupAgents = this.numAgents;
				configurationOD = new ArrayList<>(this.OD);
			} else if (actualGroups.size() == groups.size() - 2)
				groupAgents = left;
			else {
				groupAgents = (int) (this.numAgents * group.portion);
				left -= groupAgents;
			}
			final int numTripsGroup = groupAgents * UserParameters.numTrips;
			final ArrayList<Pair<NodeGraph, NodeGraph>> groupOD = new ArrayList<>(
					configurationOD.subList(0, numTripsGroup));

			int lowLimit = 0;
			int upLimit = UserParameters.numTrips;

			for (int i = 0; i < groupAgents; i++) {
				ArrayList<Pair<NodeGraph, NodeGraph>> agentODs = new ArrayList<>();
				if (UserParameters.numTrips == 1)
					agentODs.add(groupOD.get(i));
				else
					agentODs = new ArrayList<>(groupOD.subList(lowLimit, upLimit));

				final AgentGroupProperties agp = new AgentGroupProperties(group);
				agp.OD = agentODs;
				final Pedestrian a = new Pedestrian(this, agp);
				final MasonGeometry newGeometry = a.getGeometry();
				newGeometry.isMovable = true;
				a.agentID = counter;

				agents.addGeometry(newGeometry);
				this.agentsList.add(a);
				counter += 1;
				lowLimit = upLimit;
				upLimit = lowLimit + UserParameters.numTrips;
			}
			configurationOD = new ArrayList<>(configurationOD.subList(numTripsGroup, configurationOD.size()));
			System.out.println(
					group.groupName + " nr. agents: " + groupAgents + "  -- nr of OD pairs: " + groupOD.size());
		}
	}

	@Override
	public void finish() {
		try {
			ImportingExporting.saveResults(this.currentJob);
		} catch (final Exception e) {
			e.printStackTrace();
		}
		super.finish();
	}

	// Main function allows simulation to be run in stand-alone, non-GUI mode/
	public static void main(String[] args) throws Exception {
		final int jobs = UserParameters.jobs;
		ImportingExporting.importFiles();
		prepareLayers();

		for (int job = 0; job < jobs; job++) {
			System.out.println("Run nr.. " + job);
			for (final Object o : network.getEdges()) {
				final EdgeGraph edge = (EdgeGraph) o;
				edge.resetDensities();
			}
			final SimState state = new PedSimCity(System.currentTimeMillis(), job);
			state.start();
			while (state.schedule.step(state)) {
			}
		}
		System.exit(0);
	}

	static public void prepareLayers() {

		// Element 1 - Nodes: assign scores and attributes to nodes

		for (final MasonGeometry nodeGeometry : junctions.geometriesList) {
			// street junctions and betweenness centrality
			final NodeGraph node = network.findNode(nodeGeometry.geometry.getCoordinate());
			node.setID(nodeGeometry.getIntegerAttribute("nodeID"));
			node.masonGeometry = nodeGeometry;
			node.primalEdge = null;
			// centrality
			try {
				node.centrality = nodeGeometry.getDoubleAttribute("Bc_multi");
			} catch (final java.lang.NullPointerException e) {
				node.centrality = nodeGeometry.getDoubleAttribute("Bc_Rd");
				node.reachCentrality = nodeGeometry.getIntegerAttribute("Rc400").floatValue();
			}
			// set adjacent edges and nodes
			node.setNeighbouringComponents();
			nodesMap.put(node.getID(), node);
		}
		// generate the centrality map of the graph
		network.generateCentralityMap();
		// Element 2 - Landmarks -
		if (buildings != null) {

			final Bag bagLocal = LandmarkNavigation.getLandmarks(buildings, UserParameters.localLandmarkThreshold,
					"local");
			for (final Object l : bagLocal) {
				final MasonGeometry landmark = (MasonGeometry) l;
				localLandmarks.addGeometry(landmark);
			}

			final Bag bagGlobal = LandmarkNavigation.getLandmarks(buildings, UserParameters.globalLandmarkThreshold,
					"global");
			for (final Object g : bagGlobal) {
				final MasonGeometry landmark = (MasonGeometry) g;
				globalLandmarks.addGeometry(landmark);
			}
			localLandmarks.generateGeometriesList();
			globalLandmarks.generateGeometriesList();

			for (final MasonGeometry buildingGeometry : buildings.geometriesList) {
				final Building building = new Building();
				building.buildingID = buildingGeometry.getIntegerAttribute("buildingID");
				building.globalLandmarkness = buildingGeometry.getDoubleAttribute("gScore_sc");
				building.localLandmarkness = buildingGeometry.getDoubleAttribute("lScore_sc");
				building.landUse = buildingGeometry.getStringAttribute("land_use");
				building.DMA = buildingGeometry.getStringAttribute("DMA");
				building.geometry = buildingGeometry;

				if (UserParameters.empiricalABM) {
					final Bag nearestNodes = junctions.getObjectsWithinDistance(building.geometry, 500);
					MasonGeometry closest = null;
					double lowestDistance = 501.0;

					for (final Object nN : nearestNodes) {
						final MasonGeometry node = (MasonGeometry) nN;
						final double distance = node.geometry.distance(buildingGeometry.geometry);

						if (distance < lowestDistance) {
							closest = node;
							lowestDistance = node.geometry.distance(buildingGeometry.geometry);
						}
					}
					if (closest == null)
						building.node = null;
					else
						building.node = network.findNode(closest.getGeometry().getCoordinate());

				}
				buildingsMap.put(building.buildingID, building);
			}

			// Integrate landmarks into the street network
			final List<Integer> globalLandmarksID = globalLandmarks.getIntColumn("buildingID");
			final VectorLayer sightLinesLight = sightLines.selectFeatures("buildingID", globalLandmarksID, true);
			// free up memory
			sightLines = null;

			System.out.println("incorporating local landmarks");
			network.setLocalLandmarkness(localLandmarks, buildingsMap, UserParameters.distanceNodeLandmark);
			System.out.println("incorporating global landmarks");
			network.setGlobalLandmarkness(globalLandmarks, buildingsMap, UserParameters.distanceAnchors,
					sightLinesLight, UserParameters.nrAnchors);

			network.getNodes().forEach((node) -> {

				final Bag nearestBuildings = buildings.getObjectsWithinDistance(node.masonGeometry.geometry, 100);
				for (final Object b : nearestBuildings) {
					final MasonGeometry building = (MasonGeometry) b;
					final int buildingID = building.getIntegerAttribute("buildingID");
					final String DMA = buildingsMap.get(buildingID).DMA;
					node.DMA = DMA;
				}
			});
		}

		// Identify gateways
		if (barriers != null) {

			for (final NodeGraph node : nodesMap.values()) {
				node.region = node.masonGeometry.getIntegerAttribute("district");

				if (regionsMap.get(node.region) == null) {
					final Region region = new Region();
					regionsMap.put(node.region, region);
				}
			}

			for (final NodeGraph node : nodesMap.values()) {

				final Integer gateway = node.masonGeometry.getIntegerAttribute("gateway"); // 1 or 0
				final Integer region = node.region;
				if (gateway == 0) {
					// nodes that are not gateways
					startingNodes.add(node.masonGeometry);
					continue;
				}

				for (final EdgeGraph bridge : node.getEdges()) {

					final NodeGraph oppositeNode = (NodeGraph) bridge.getOppositeNode(node);
					final int possibleRegion = oppositeNode.region;
					if (possibleRegion == region)
						continue;

					final Gateway gd = new Gateway();
					gd.exit = node;
					gd.edgeID = bridge.getID();
					gd.gatewayID = new Pair<>(node, oppositeNode);
					gd.regionTo = possibleRegion;
					gd.entry = oppositeNode;
					gd.distance = bridge.getLength();
					gd.entryAngle = Angles.angle(node, oppositeNode);
					regionsMap.get(region).gateways.add(gd);
					gatewaysMap.put(new Pair<>(node, oppositeNode), gd);
					node.gateway = true;
				}
				node.adjacentRegions = node.getAdjacentRegion();
			}
		}

		// Element 3 - Street segments: assign attributes
		for (final Object o : network.getEdges()) {
			final EdgeGraph edge = (EdgeGraph) o;
			final int edgeID = edge.getIntegerAttribute("edgeID");
			edge.setID(edgeID);

			if (barriers != null) {
				edge.setBarriers();
				// add edges to the regions' information

				if (edge.u.region == edge.v.region) {
					final int region = edge.u.region;
					edge.region = region;
					regionsMap.get(region).edges.add(edge);
				} else
					edge.region = 999999;
			}
			edgesMap.put(edgeID, edge);
		}

		// Element 3A - Centroids (Dual Graph): assign edgeID to centroids in the dual
		// graph
		for (final MasonGeometry centroidGeometry : centroids.geometriesList) {
			final int edgeID = centroidGeometry.getIntegerAttribute("edgeID");
			final NodeGraph cen = dualNetwork.findNode(centroidGeometry.geometry.getCoordinate());
			cen.masonGeometry = centroidGeometry;
			cen.setID(edgeID);
			cen.primalEdge = edgesMap.get(edgeID);
			edgesMap.get(edgeID).dualNode = cen;
			centroidsMap.put(edgeID, cen);
			cen.setNeighbouringComponents();
		}

		for (final Object o : dualNetwork.getEdges()) {
			final EdgeGraph edge = (EdgeGraph) o;
			edge.deflectionDegrees = edge.getDoubleAttribute("deg");
		}

		// Element 4 - Regions: create regions' subgraphs and store other information
		// about regions (landmarks, barriers)
		if (barriers != null) {
			for (final Entry<Integer, Region> entry : regionsMap.entrySet()) {
				final ArrayList<EdgeGraph> edgesRegion = entry.getValue().edges;
				final ArrayList<EdgeGraph> dualEdgesRegion = new ArrayList<>();
				final SubGraph primalGraph = new SubGraph(network, edgesRegion);

				for (final Object e : edgesRegion) {
					final EdgeGraph edge = (EdgeGraph) e;
					final NodeGraph cen = edge.dualNode;
					cen.region = entry.getKey();
					final DirectedEdgeStar dirEdges = cen.getOutEdges();

					for (final Object dE : dirEdges.getEdges()) {
						final GeomPlanarGraphDirectedEdge dEdge = (GeomPlanarGraphDirectedEdge) dE;
						dualEdgesRegion.add((EdgeGraph) dEdge.getEdge());
					}
				}

				final SubGraph dualGraph = new SubGraph(dualNetwork, dualEdgesRegion);
				primalGraph.setSubGraphBarriers();
				primalGraph.generateSubGraphCentralityMap();
				regionsMap.get(entry.getKey()).primalGraph = primalGraph;
				regionsMap.get(entry.getKey()).dualGraph = dualGraph;

				// set the landmarks of this region
				if (buildings != null) {
					primalGraph.setSubGraphLandmarks();
					final VectorLayer regionNetwork = new VectorLayer();
					for (final EdgeGraph edge : edgesRegion)
						regionNetwork.addGeometry(edge.masonGeometry);
					regionsMap.get(entry.getKey()).regionNetwork = regionNetwork;
					final Bag buildings = LandmarkNavigation.getBuildings(null, null, entry.getKey());
					regionsMap.get(entry.getKey()).buildings = buildings;
					regionsMap.get(entry.getKey()).assignLandmarks();
					regionsMap.get(entry.getKey()).computeComplexity("local");
				}
			}

			// Element 5 - Barriers: create barriers map
			for (final MasonGeometry barrierGeometry : barriers.geometriesList) {
				final int barrierID = barrierGeometry.getIntegerAttribute("barrierID");
				final Barrier barrier = new Barrier();
				barrier.masonGeometry = barrierGeometry;
				barrier.type = barrierGeometry.getStringAttribute("type");
				final ArrayList<EdgeGraph> edgesAlong = new ArrayList<>();

				for (final EdgeGraph edge : network.getEdges())
					if (edge.barriers != null && edge.barriers.contains(barrierID))
						edgesAlong.add(edge);

				barrier.edgesAlong = edgesAlong;
				barrier.type = barrierGeometry.getStringAttribute("type");
				barriersMap.put(barrierID, barrier);
			}
		}
		System.out.println("Creation environment completed");
	}

}