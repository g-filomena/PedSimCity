package sim.app.geo.pedSimCity;

import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.net.URL;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Random;

import org.apache.commons.lang3.ArrayUtils;
import org.javatuples.Pair;
import org.supercsv.io.CsvMapWriter;
import org.supercsv.io.ICsvMapWriter;
import org.supercsv.prefs.CsvPreference;

import com.opencsv.CSVReader;
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
import sim.io.geo.ShapeFileImporter;
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
	public static HashMap<Integer, MasonGeometry> barriersMap = new HashMap<Integer, MasonGeometry>();
	public static HashMap<Integer, Building> buildingsMap = new HashMap<Integer, Building>();
	public static HashMap<Integer, ArrayList<EdgeGraph>> barriersEdgesMap = new HashMap<Integer, ArrayList<EdgeGraph>>();
	public static ArrayList<RouteData> routesData = new ArrayList<RouteData>();
	List<Integer> regions = new ArrayList<Integer>();
	public static HashMap<Integer, Region> regionsMap = new HashMap<Integer, Region>();
	public static HashMap<Pair<NodeGraph, NodeGraph>, Gateway> gatewaysMap = new HashMap<Pair<NodeGraph, NodeGraph>, Gateway>();
	public static Bag startingNodes = new Bag();
	public static Bag centroidsGeometries;
	public static Bag nodesGeometries;
	public static Bag buildingsGeometries;
	public static String [][] visibilityMatrix;


	// OD related variables
	ArrayList<Pair<NodeGraph, NodeGraph>> OD = new ArrayList<Pair<NodeGraph, NodeGraph>>();
	static List<Float> distances = new ArrayList<Float>();
	// used only when loading OD sets
	static boolean readingFromPrevious =  false;
	static List<Integer> OR = new ArrayList<Integer>();
	static List<Integer> DE = new ArrayList<Integer>();

	public int numTripsScenario, numAgents, currentJob;
	ArrayList<Integer> groupBounds = new ArrayList<Integer>();
	ArrayList<Group> groups = new ArrayList<Group>();

	double height, width, ratio;

	// agents
	public static VectorLayer agents = new VectorLayer();
	ArrayList<Pedestrian> agentsList = new ArrayList<Pedestrian>();
	public static String criteria[];
	String criteriaLandmarks[] = {"roadDistance", "angularChange", "roadDistanceLandmarks", "angularChangeLandmarks",
			"localLandmarks", "globalLandmarks"};
	String criteriaRegions[] = {"roadDistance", "angularChange", "roadDistanceRegions", "angularChangeRegions",
			"roadDistanceBarriers", "angularChangeBarriers", "roadDistanceRegionsBarriers", "angularChangeRegionsBarriers"};

	//directories
	public static String outputFolderLandmarks = "C:/Users/g_filo01/sciebo/Scripts/ABM analysis/Input/ResearchParameters.testingLandmarks/"+ResearchParameters.cityName;
	public static String outputFolderRegions = "C:/Users/g_filo01/sciebo/Scripts/ABM analysis/Input/ResearchParameters.testingRegions/"+ResearchParameters.cityName;

	/** Constructor */

	public PedSimCity(long seed, int job)	{
		super(seed);
		this.currentJob = job;
	}

	/** Initialization **/

	@Override
	public void start()	{

		if (ResearchParameters.testingRegions) criteria = criteriaRegions;
		else if (ResearchParameters.testingLandmarks) criteria = criteriaLandmarks;

		if (ResearchParameters.testingRegions || ResearchParameters.testingLandmarks) numAgents = criteria.length;
		else numAgents = ResearchParameters.numAgents;
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
		if (ResearchParameters.testingLandmarks) numTripsScenario = distances.size();
		else if (ResearchParameters.testingRegions) numTripsScenario = 2000;

		ArrayList<ArrayList<NodeGraph>> listSequences = new ArrayList<ArrayList<NodeGraph>> ();

		if (ResearchParameters.testingLandmarks) {

			for (int i = 0; i < numTripsScenario; i++) {
				NodeGraph originNode = null;
				NodeGraph destinationNode = null;

				if (!readingFromPrevious) {
					while (originNode == null) originNode = NodesLookup.randomNode(nodesGeometries, network);
					destinationNode = NodesLookup.nodeWithinFromDistances(originNode, junctions, distances, network);
				}
				else {
					originNode = nodesMap.get(OR.get(i));
					destinationNode = nodesMap.get(DE.get(i));
				}

				Pair<NodeGraph, NodeGraph> pair = new Pair<NodeGraph, NodeGraph> (originNode, destinationNode);
				ArrayList<NodeGraph> sequence = LandmarkNavigation.findSequenceSubGoals(originNode, destinationNode, false, "local");
				listSequences.add(sequence);
				OD.add(pair);
			}
		}

		else if (ResearchParameters.testingRegions)
		{
			for (int i = 0; i < numTripsScenario; i++) {
				NodeGraph originNode = null;
				NodeGraph destinationNode = null;

				if (!readingFromPrevious) {
					while ((destinationNode == null) || (originNode == destinationNode)) {
						while (originNode == null) originNode = NodesLookup.randomNode(startingNodes, network);
						destinationNode = NodesLookup.nodeWithinDistance(originNode, 1000, 3000, network);
					}
				}
				else {
					originNode = nodesMap.get(OR.get(i));
					destinationNode = nodesMap.get(DE.get(i));
				}
				Pair<NodeGraph, NodeGraph> pair = new Pair<NodeGraph, NodeGraph> (originNode, destinationNode);
				OD.add(pair);
			}
		}
		System.out.println("OD matrix ready");

		if (ResearchParameters.fiveElements) prepareGroups();

		for (int i = 0; i < numAgents; i++)	{
			AgentProperties ap = new AgentProperties();

			if (ResearchParameters.fiveElements) {

				Random random = new Random();
				double p = random.nextFloat();
				if (p <= 0.45) ap.worker = true;
				else if (p <= 0.70) ap.student = true;
				else if (p <= 0.85 ) ap.flaneur = true;
				else if (p <= 0.85 ) ap.homeBased = true;

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
		//		try {saveCSV();}
		//		catch (IOException e) {e.printStackTrace();}
		super.finish();
	}

	// Main function allows simulation to be run in stand-alone, non-GUI mode/
	public static void main(String[] args) throws IOException
	{
		int jobs = ResearchParameters.jobs;
		if (ResearchParameters.testingLandmarks) jobs = 50;
		if (ResearchParameters.testingRegions) jobs = 5;
		importFiles();
		prepareLayers();
		for (int i = 0; i < jobs; i++) {

			if (readingFromPrevious) readingOD(outputFolderLandmarks+"_angularChange_"+i+".csv");
			System.out.println("Run nr.. "+i);
			SimState state = new PedSimCity(System.currentTimeMillis(), i);
			state.start();
			while (state.schedule.step(state)) {}
		}


		System.exit(0);
	}

	public static void importFiles() throws IOException {

		String inputDataDirectory = null;
		if (ResearchParameters.testingLandmarks) {
			inputDataDirectory = "landmarksData"+"/"+ResearchParameters.cityName+"/";
			/// read distances
			System.out.println("reading distances");
			CSVReader readerDistances = new CSVReader(new FileReader(PedSimCity.class.getResource(inputDataDirectory).toString().substring(6)
					+"/"+ResearchParameters.cityName+"_tracks_distances.csv"));
			String[] nextLineDistances;

			int ds = 0;
			while ((nextLineDistances = readerDistances.readNext()) != null) {
				ds += 1;
				if (ds == 1) continue;
				distances.add(Float.parseFloat(nextLineDistances[2]));
			}
			readerDistances.close();
		}
		else if (ResearchParameters.testingRegions) inputDataDirectory = "districtsData/"+ResearchParameters.cityName+"/";
		else if (ResearchParameters.fiveElements) inputDataDirectory = "data/"+ResearchParameters.cityName+"/";

		if (ResearchParameters.testingLandmarks  || ResearchParameters.fiveElements) {

			// read buildings
			System.out.println("reading buildings (landmarks) layer");
			URL landmarksFile = PedSimCity.class.getResource(inputDataDirectory+"/"+ResearchParameters.cityName+"_landmarks.shp");
			ShapeFileImporter.read(landmarksFile, buildings);
			buildings.setID("buildingID");
		}

		if (ResearchParameters.testingRegions || ResearchParameters.fiveElements) {

			System.out.println("reading barriers layer...");
			URL barriersFile = PedSimCity.class.getResource(inputDataDirectory+"/"+ResearchParameters.cityName+"_barriers.shp");
			ShapeFileImporter.read(barriersFile, barriers);
		}

		// read the street network shapefiles and create the primal and the dual graph
		System.out.println("reading the graphs");

		URL roadsFile = PedSimCity.class.getResource(inputDataDirectory+"/"+ResearchParameters.cityName+"_edges.shp");
		URL junctionsFile = PedSimCity.class.getResource(inputDataDirectory+"/"+ResearchParameters.cityName+"_nodes.shp");
		URL roadsDualFile = PedSimCity.class.getResource(inputDataDirectory+"/"+ResearchParameters.cityName+"_edgesDual.shp");
		URL centroidsFile = PedSimCity.class.getResource(inputDataDirectory+"/"+ResearchParameters.cityName+"_nodesDual.shp");

		ShapeFileImporter.read(roadsFile, roads);
		ShapeFileImporter.read(junctionsFile, junctions);
		ShapeFileImporter.read(roadsDualFile, intersectionsDual);
		ShapeFileImporter.read(centroidsFile, centroids);

		network.fromGeomField(roads);
		dualNetwork.fromGeomField(intersectionsDual);
		nodesGeometries = junctions.getGeometries();
		centroidsGeometries = centroids.getGeometries();
		if (ResearchParameters.testingLandmarks | ResearchParameters.fiveElements) buildingsGeometries = buildings.getGeometries();

		// visibility
		if (ResearchParameters.visibility)	{

			System.out.println("reading visibility");
			URL sightLinesFile = PedSimCity.class.getResource(inputDataDirectory+"/"+ResearchParameters.cityName+"_sight_lines.shp");
			ShapeFileImporter.read(sightLinesFile, sightLines);

			visibilityMatrix = new String[buildings.getGeometries().size()+1][nodesGeometries.size()+1];
			CSVReader reader = new CSVReader(new FileReader(PedSimCity.class.getResource(inputDataDirectory).toString().substring(6)
					+ResearchParameters.cityName+"_visibility_matrix_simplified.csv"));
			String [] nextLine;

			int v = 0;
			while ((nextLine = reader.readNext()) != null) {
				System.out.println("reading");
				visibilityMatrix[v] = nextLine;
				v = v+1;
			}
			reader.close();
		}
		System.out.println("files imported successfully");
	}


	static public void prepareLayers() {

		// Element 1 - Nodes: assign scores and attributes to nodes

		for (Object nG : nodesGeometries) {

			// street junctions and betweenness centrality
			MasonGeometry nodeGeometry = (MasonGeometry) nG;
			NodeGraph node = network.findNode(nodeGeometry.geometry.getCoordinate());
			node.setID(nodeGeometry.getIntegerAttribute("nodeID"));
			node.masonGeometry = nodeGeometry;
			node.primalEdge = null;
			if (ResearchParameters.testingRegions) node.centrality = nodeGeometry.getDoubleAttribute("Bc_multi");
			else  node.centrality = nodeGeometry.getDoubleAttribute("Bc_Rd");

			// set adjacent edges and nodes
			node.setNeighbouringComponents();
			nodesMap.put(node.getID(), node);
		}
		// generate the centrality map of the graph
		network.generateCentralityMap();

		// Element 2 - Landmarks -
		if (ResearchParameters.testingLandmarks || ResearchParameters.fiveElements ) {

			Bag bagLocal = LandmarkNavigation.getLandmarks(buildingsGeometries, ResearchParameters.localLandmarkThreshold, "local");
			for (Object l : bagLocal) {
				MasonGeometry landmark = (MasonGeometry) l;
				localLandmarks.addGeometry(landmark);
			}

			Bag bagGlobal = LandmarkNavigation.getLandmarks(buildingsGeometries, ResearchParameters.globalLandmarkThreshold, "global");
			for (Object g : bagGlobal) {
				MasonGeometry landmark = (MasonGeometry) g;
				globalLandmarks.addGeometry(landmark);
			}

			for (Object b : buildings.getGeometries()) {

				MasonGeometry buildingGeometry = (MasonGeometry) b;
				Building building = new Building();
				building.buildingID = buildingGeometry.getIntegerAttribute("buildingID");
				building.landUse = buildingGeometry.getStringAttribute("landUse");
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
				building.node = network.findNode(closest.getGeometry().getCoordinate());
				buildingsMap.put(building.buildingID, building);
			}

			// Integrating landmarks into the street network
			List<Integer> globalLandmarksID = globalLandmarks.getIntColumn("buildingID");
			sightLines = sightLines.select("buildingID", globalLandmarksID, "equal");
			network.setLocalLandmarkness(localLandmarks, buildingsMap, ResearchParameters.distanceNodeLandmark, visibilityMatrix);
			network.setGlobalLandmarkness(globalLandmarks, buildingsMap, ResearchParameters.distanceAnchors, sightLines);
			sightLines = null;
			visibilityMatrix = null;
		}


		// Identify gateways
		if (ResearchParameters.testingRegions || ResearchParameters.fiveElements ) {

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

					Region dd = regionsMap.get(region);
					dd.gateways.add(gd);
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

			if (ResearchParameters.testingRegions | ResearchParameters.fiveElements)  {
				edge.setBarriers();
				// add edges to the regions' information
				if (edge.u.region == edge.v.region)	{
					int region = edge.u.region;
					edge.region = region;
					Region dd = regionsMap.get(region);
					dd.edges.add(edge);
				}
				else edge.region = 999999;
			}
			edgesMap.put(edgeID, edge);
		}

		// Element 3A - Centroids (Dual Graph): assign edgeID to centroids in the dual graph
		for (Object cG : centroidsGeometries) {
			MasonGeometry centroidGeometry = (MasonGeometry) cG;
			int edgeID = centroidGeometry.getIntegerAttribute("edgeID");

			NodeGraph cen = dualNetwork.findNode(centroidGeometry.geometry.getCoordinate());
			cen.masonGeometry = centroidGeometry;
			cen.setID(edgeID);
			cen.primalEdge = edgesMap.get(edgeID);
			edgesMap.get(edgeID).dualNode = cen;
			centroidsMap.put(edgeID, cen);
		}

		// Element 4 - Regions: create regions' subgraphs and store other information about regions (landmarks, barriers)
		if (ResearchParameters.testingRegions || ResearchParameters.fiveElements) {

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
				primalGraph.setBarriersGraph();
				regionsMap.get(entry.getKey()).primalGraph = primalGraph;
				regionsMap.get(entry.getKey()).dualGraph = dualGraph;

				// set the landmarks of this region
				if (ResearchParameters.fiveElements) {
					primalGraph.setLandmarksGraph();
					VectorLayer regionNetwork = new VectorLayer();
					for (EdgeGraph edge : edgesRegion) regionNetwork.addGeometry(edge.masonGeometry);

					Bag buildings = LandmarkNavigation.getBuildings(null, null, entry.getKey());
					regionsMap.get(entry.getKey()).regionNetwork = regionNetwork;
					regionsMap.get(entry.getKey()).buildings = buildings;
					regionsMap.get(entry.getKey()).assignLandmarks();
					regionsMap.get(entry.getKey()).computeComplexity("local");
				}
			}

			// Element 5 - Barriers: create barriers map
			Bag barriersGeometries = barriers.getGeometries();

			for (Object bG : barriersGeometries ) {
				MasonGeometry barrierGeo = (MasonGeometry) bG;
				int barrierID = barrierGeo.getIntegerAttribute("barrierID");
				barriersMap.put(barrierID, barrierGeo);
				ArrayList<EdgeGraph> alongEdges = new ArrayList<EdgeGraph>();
				for (EdgeGraph edge : network.edgesGraph)
					if ((edge.barriers != null) && (edge.barriers.contains(barrierID))) alongEdges.add(edge);
				barriersEdgesMap.put(barrierID, alongEdges);
			}
		}
		System.out.println("layers ready");
	}


	// it reads OD from an existing file
	public static void readingOD(String inputFile) throws IOException {

		CSVReader readerOD;
		readerOD = new CSVReader(new FileReader(inputFile));
		String[] nextLine;
		OR.clear();
		DE.clear();

		int dv = 0;
		while ((nextLine = readerOD.readNext()) != null) {
			dv += 1;
			if (dv == 1) continue;
			OR.add(Integer.parseInt(nextLine[1]));
			DE.add(Integer.parseInt(nextLine[2]));
		}
		readerOD.close();
	}

	public void prepareGroups() {


		List<Double> composition = Arrays.asList(ResearchParameters.composition);
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

	// save the simulation output
	public void saveCSV() throws IOException {

		System.out.println("saving Densities");
		Bag edgesGeometries = roads.getGeometries();
		String csvSegments = null;

		if (ResearchParameters.testingRegions) {
			csvSegments = outputFolderRegions+"_PedSim_regions_"+(currentJob)+".csv";
			FileWriter writerDensitiesData = new FileWriter(csvSegments);
			CSVUtils.writeLine(writerDensitiesData, Arrays.asList("edgeID", "AC", "RB", "BB", "BRB"));


			int rGeoSize = edgesGeometries.size();
			for (int i = 0; i < rGeoSize; i++) {
				MasonGeometry segment = (MasonGeometry) edgesGeometries.objs[i];
				EdgeGraph ed = edgesMap.get(segment.getIntegerAttribute("edgeID"));
				CSVUtils.writeLine(writerDensitiesData, Arrays.asList(Integer.toString(
						ed.getID()), Integer.toString(ed.roadDistance),
						Integer.toString(ed.angularChange), Integer.toString(ed.angularChangeRegions),
						Integer.toString(ed.angularChangeBarriers),	Integer.toString(ed.angularChangeRegionsBarriers)));
			}
			writerDensitiesData.flush();
			writerDensitiesData.close();
		}

		else if (ResearchParameters.testingLandmarks) {

			csvSegments = outputFolderLandmarks+"_PedSim_landmarks_"+(currentJob)+".csv";
			FileWriter writerDensitiesData = new FileWriter(csvSegments);
			CSVUtils.writeLine(writerDensitiesData, Arrays.asList("edgeID", "RD", "AC", "RL", "AL", "LL", "GL"));

			int rGeoSize = edgesGeometries.size();
			for (int i = 0; i < rGeoSize; i++) {
				MasonGeometry segment = (MasonGeometry) edgesGeometries.objs[i];
				EdgeGraph ed = edgesMap.get(segment.getIntegerAttribute("edgeID"));
				CSVUtils.writeLine(writerDensitiesData, Arrays.asList(Integer.toString(ed.getID()),
						Integer.toString(ed.roadDistance), Integer.toString(ed.angularChange),
						Integer.toString(ed.roadDistanceLandmarks), Integer.toString(ed.angularChangeLandmarks),
						Integer.toString(ed.localLandmarks), Integer.toString(ed.globalLandmarks)));
			}
			writerDensitiesData.flush();
			writerDensitiesData.close();
		}

		System.out.println("saving Routes");
		for (String cr : criteria) {

			String csvRoutes;
			// prexifOutput
			if (ResearchParameters.testingLandmarks) csvRoutes = outputFolderLandmarks+"_PedSim_landmarks_routes_"+cr+"_"+(currentJob)+".csv";
			else csvRoutes = outputFolderRegions+"_PedSim_regions_routes_"+cr+"_"+(currentJob)+".csv";

			List<String> header = new ArrayList<String>();
			header.addAll(Arrays.asList(new String[] {"routeID", "origin", "destination"}));

			for (int i = 0; i < edgesGeometries.size(); i++) {
				MasonGeometry segment = (MasonGeometry) edgesGeometries.objs[i];
				header.add(Integer.toString(segment.getIntegerAttribute("edgeID")));
			}

			String[] headerArray = new String[header.size()];
			header.toArray(headerArray);
			List<Map<String, Object>> rows = new ArrayList<Map<String, Object>>();

			for (RouteData rD : routesData) {
				String routeCriteria =  rD.criteria;
				if (cr != routeCriteria) continue;
				Map<String, Object> row = new HashMap<String, Object>();
				List<Integer> sequenceEdges = rD.sequenceEdges;

				int originNode =  rD.origin;
				int destinationNode = rD.destination;
				row.put(headerArray[0], Integer.toString(originNode)+"_"+Integer.toString(destinationNode));
				row.put(headerArray[1], originNode);
				row.put(headerArray[2], destinationNode);

				for (int e = 0; e < sequenceEdges.size(); e++) {
					Integer position = ArrayUtils.indexOf(headerArray, Integer.toString(sequenceEdges.get(e)));
					row.put(headerArray[position], 1);
				}
				rows.add(row);
			}

			ICsvMapWriter mapWriter = null;
			try {
				mapWriter = new CsvMapWriter(new FileWriter(csvRoutes), CsvPreference.STANDARD_PREFERENCE);
				// write the header
				mapWriter.writeHeader(headerArray);
				// write the customer maps
				for (int r = 0; r < rows.size(); r++) mapWriter.write(rows.get(r), headerArray);
			}
			finally {if( mapWriter != null ) mapWriter.close();}
		}
		routesData.clear();
	}


}