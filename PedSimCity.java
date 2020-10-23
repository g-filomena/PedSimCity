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

import org.apache.commons.lang3.ArrayUtils;
import org.javatuples.Pair;
import org.supercsv.io.CsvMapWriter;
import org.supercsv.io.ICsvMapWriter;
import org.supercsv.prefs.CsvPreference;

import com.opencsv.CSVReader;
import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.planargraph.DirectedEdgeStar;

import sim.app.geo.urbanSim.Angles;
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
	public static HashMap<Integer, ArrayList<EdgeGraph>> barriersEdgesMap = new HashMap<Integer, ArrayList<EdgeGraph>>();
	public static ArrayList<RouteData> routesData = new ArrayList<RouteData>();
	List<Integer> regions = new ArrayList<Integer>();
	public static HashMap<Integer, RegionData> regionsMap = new HashMap<Integer, RegionData>();
	public static HashMap<Pair<NodeGraph, NodeGraph>, GatewayData> gatewaysMap = new HashMap<Pair<NodeGraph, NodeGraph>, GatewayData>();
	public static Bag startingNodes = new Bag();
	public static Bag centroidsGeometries;
	public static Bag nodesGeometries;
	public static String [][] visibilityMatrix;


	// OD related variables
	ArrayList<Pair<NodeGraph, NodeGraph>> OD = new ArrayList<Pair<NodeGraph, NodeGraph>>();
	static List<Float> distances = new ArrayList<Float>();
	// used only when loading OD sets
	static boolean readingFromPrevious =  false;
	static List<Integer> OR = new ArrayList<Integer>();
	static List<Integer> DE = new ArrayList<Integer>();

	public int numTripsScenario, numAgents, currentJob;
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
		else criteria = criteriaLandmarks;
		numAgents = criteria.length;
		super.start();

		// prepare environment
		Envelope MBR = null;
		MBR = roads.getMBR();
		MBR.expandToInclude(buildings.getMBR());
		MBR.expandToInclude(barriers.getMBR());
		roads.setMBR(MBR);

		// Element 1 - Nodes: assign scores and attributes to nodes, including landmarks (Element 2)
		for (Object nG : nodesGeometries) {

			// street junctions and betweenness centrality
			MasonGeometry nodeGeometry = (MasonGeometry) nG;
			Integer nodeID = nodeGeometry.getIntegerAttribute("nodeID");
			NodeGraph node = network.findNode(nodeGeometry.geometry.getCoordinate());
			node.setID(nodeID);
			node.masonGeometry = nodeGeometry;
			node.primalEdge = null;
			if (ResearchParameters.testingRegions) node.centrality = nodeGeometry.getDoubleAttribute("Bc_multi");
			else  node.centrality = nodeGeometry.getDoubleAttribute("Bc_Rd");

			// Assign to each Node landmarks at the Junction
			if (ResearchParameters.testingLandmarks) {
				// Element 2 - Landmarks
				node.setLandmarkness(nodeGeometry);

				// landmarks visible from the junction
				if (ResearchParameters.visibility)	{
					int column = 0;
					for (int j = 1; j < visibilityMatrix[0].length; j++) {

						int examined = Integer.valueOf(visibilityMatrix[0][j]);
						if  (examined == nodeID) {
							column = j;
							break;
						}
					}
					for (int z = 1; z < visibilityMatrix.length; z++) {
						int visibility = Integer.valueOf(visibilityMatrix[z][column]);
						if (visibility == 1) node.visible2d.add(Integer.valueOf(visibilityMatrix[z][0]));
					}
				}
			}
			// set adjacent edges and nodes
			node.setNeighbouringComponents();
			nodesMap.put(nodeID, node);
		}
		// generate the centrality map of the graph
		network.generateCentralityMap();

		// Identify gateways
		if (ResearchParameters.testingRegions) {
			for (NodeGraph node : nodesMap.values()) {
				node.region = node.masonGeometry.getIntegerAttribute("district");

				if (regionsMap.get(node.region) == null) {
					RegionData region = new RegionData();
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

					GatewayData gd = new GatewayData();
					gd.node = node;
					gd.edgeID = bridge.getID();
					gd.gatewayID = new Pair<NodeGraph, NodeGraph>(node, oppositeNode);
					gd.regionTo = possibleRegion;
					gd.entry = oppositeNode;
					gd.distance = bridge.getLength();
					gd.entryAngle = Angles.angle(node, oppositeNode);

					RegionData dd = regionsMap.get(region);
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

			if (ResearchParameters.testingRegions) {
				edge.setBarriers();
				// add edges to the regions' information
				if (edge.u.region == edge.v.region)	{
					int region = edge.u.region;
					edge.region = region;
					RegionData dd = regionsMap.get(region);
					dd.edgesMap.add(edge);
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
		for (Entry<Integer, RegionData> entry : regionsMap.entrySet()) {
			ArrayList<EdgeGraph> edgesRegion = entry.getValue().edgesMap;
			ArrayList<EdgeGraph> dualEdgesRegion = new ArrayList<EdgeGraph>();
			SubGraph primalGraph = new SubGraph(network, edgesRegion);

			for (EdgeGraph edge: edgesRegion)	{
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

				ArrayList<MasonGeometry> buildings = LandmarkNavigation.getBuildings(null, null, entry.getKey());
				regionsMap.get(entry.getKey()).regionNetwork = regionNetwork;
				regionsMap.get(entry.getKey()).buildings = buildings;
				regionsMap.get(entry.getKey()).assignLandmarks();
				regionsMap.get(entry.getKey()).computeComplexity("local");
			}
		}

		// Element 5 - Barriers: create barriers map
		if (ResearchParameters.testingRegions) {

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

		// populate
		populate();
		agents.setMBR(MBR);
	}

	public void populate() {

		// prepare to start the simulation - OD Matrix
		int numTripsScenario;
		if (ResearchParameters.testingLandmarks) numTripsScenario = distances.size();
		else if (ResearchParameters.testingRegions) numTripsScenario = 2000;
		else numTripsScenario = 1;

		ArrayList<ArrayList<NodeGraph>> listSequences = new ArrayList<ArrayList<NodeGraph>> ();

		if (ResearchParameters.testingLandmarks)	{

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

		for (int i = 0; i < numAgents; i++)	{
			AgentProperties ap = new AgentProperties();
			ap.setProperties(criteria[i]);
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
		//		try {saveCSV();}
		//		catch (IOException e) {e.printStackTrace();}
		super.finish();
	}

	// Main function allows simulation to be run in stand-alone, non-GUI mode/
	public static void main(String[] args)
	{
		int jobs = 40;

		try	{

			Bag junctionsAttributes = new Bag();
			junctionsAttributes.add("nodeID");
			String inputDataDirectory = null;

			if (ResearchParameters.testingLandmarks | ResearchParameters.fiveElements) {

				inputDataDirectory = "landmarksData"+"/"+ResearchParameters.cityName+"/";
				// list of local landmarks at a node
				junctionsAttributes.add("loc_land");
				// list of their scores (local landmarkness)
				junctionsAttributes.add("loc_scor");
				// list of global landmarks visible from a node
				junctionsAttributes.add("dist_land");
				// list of their scores (global landmarkness)
				junctionsAttributes.add("dist_scor");
				// list of their scores (global landmarkness)
				junctionsAttributes.add("anchors");
				// list of distances of the anchors from the node
				junctionsAttributes.add("dist_anch");
				junctionsAttributes.add("distances");
				junctionsAttributes.add("Bc_Rd");

				// read buildings
				System.out.println("reading buildings (landmarks) layer");
				Bag buildingsAttributes = new Bag();
				buildingsAttributes.add("buildingID");
				// local landmarkness building
				buildingsAttributes.add("lScore_sc");
				// global landmarkness building
				buildingsAttributes.add("gScore_sc");
				URL landmarksFile = PedSimCity.class.getResource(inputDataDirectory+"/"+ResearchParameters.cityName+"_landmarks.shp");
				ShapeFileImporter.read(landmarksFile, buildings);

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

			if (ResearchParameters.testingRegions | ResearchParameters.fiveElements) {
				inputDataDirectory = "districtsData/"+ResearchParameters.cityName+"/";
				junctionsAttributes.add("nodeID");
				junctionsAttributes.add("district");
				junctionsAttributes.add("gateway");
				junctionsAttributes.add("Bc_multi");

				// read barriers
				Bag barriersAttributes = new Bag();
				System.out.println("reading barriers layer...");
				barriersAttributes.add("barrierID");
				barriersAttributes.add("type");
				URL barriersFile = PedSimCity.class.getResource(inputDataDirectory+"/"+ResearchParameters.cityName+"_barriers.shp");
				ShapeFileImporter.read(barriersFile, barriers);
			}

			// read the street network shapefiles and create the primal and the dual graph
			System.out.println("reading the graphs");
			Bag centroidsAttributes = new Bag();
			centroidsAttributes.add("edgeID");

			URL roadsFile = PedSimCity.class.getResource(inputDataDirectory+"/"+ResearchParameters.cityName+"_edges.shp");
			URL junctionsFile = PedSimCity.class.getResource(inputDataDirectory+"/"+ResearchParameters.cityName+"_nodes.shp");
			ShapeFileImporter.read(roadsFile, roads);
			ShapeFileImporter.read(junctionsFile, junctions, junctionsAttributes);

			URL roadsDualFile = PedSimCity.class.getResource(inputDataDirectory+"/"+ResearchParameters.cityName+"_edgesDual.shp");
			URL centroidsFile = PedSimCity.class.getResource(inputDataDirectory+"/"+ResearchParameters.cityName+"_nodesDual.shp");
			ShapeFileImporter.read(roadsDualFile, intersectionsDual);
			ShapeFileImporter.read(centroidsFile, centroids, centroidsAttributes);

			network.fromGeomField(roads);
			dualNetwork.fromGeomField(intersectionsDual);
			nodesGeometries = junctions.getGeometries();
			centroidsGeometries = centroids.getGeometries();

			// visibility
			if (ResearchParameters.visibility)	{
				System.out.println("reading visibility");
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
			for (int i = 0; i < jobs; i++) {

				if (readingFromPrevious) readingOD(outputFolderLandmarks+"_angularChange_"+i+".csv");
				System.out.println("Run nr.. "+i);
				SimState state = new PedSimCity(System.currentTimeMillis(), i);
				state.start();
				while (state.schedule.step(state)) {}
			}
		}
		catch (IOException e) {e.printStackTrace();}
		System.exit(0);
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