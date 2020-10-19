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

import sim.app.geo.urbanSim.Angle;
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

	public static VectorLayer roads = new VectorLayer();
	public static VectorLayer buildings = new VectorLayer();
	public static VectorLayer junctions = new VectorLayer();
	public static VectorLayer barriers = new VectorLayer();
	public static Graph network = new Graph();
	public static Bag centroidsGeometries;
	public static Bag nodesGeometries;
	public static String [][] visibilityMatrix;

	public static HashMap<Integer, EdgeGraph> edgesMap = new HashMap<Integer, EdgeGraph>();
	public static HashMap<Integer, NodeGraph> nodesMap = new HashMap<Integer, NodeGraph>();
	public static HashMap<Integer, NodeGraph> centroidsMap =  new HashMap<Integer, NodeGraph>();
	public static HashMap<Integer, MasonGeometry> barriersMap = new HashMap<Integer, MasonGeometry>();
	public static HashMap<Integer, ArrayList<EdgeGraph>> barriersEdgesMap = new HashMap<Integer, ArrayList<EdgeGraph>>();

	public static ArrayList<RouteData> routesData = new ArrayList<RouteData>();
	public static Bag startingNodes = new Bag();
	public static HashMap<Integer, RegionData> regionsMap = new HashMap<Integer, RegionData>();
	public static HashMap<Pair<NodeGraph, NodeGraph>, GatewayData> gatewaysMap = new HashMap<Pair<NodeGraph, NodeGraph>, GatewayData>();

	//dual graph
	public static VectorLayer intersectionsDual = new VectorLayer();
	public static VectorLayer centroids = new VectorLayer();
	public static Graph dualNetwork = new Graph();

	//	HashMap<EdgeGraph, ArrayList<Pedestrian>> edgeTraffic = new HashMap<EdgeGraph, ArrayList<Pedestrian>>();
	public static VectorLayer agents = new VectorLayer();
	ArrayList<Pedestrian> agentsList = new ArrayList<Pedestrian>();
	List<Integer> regions = new ArrayList<Integer>();
	ArrayList<Pair<NodeGraph, NodeGraph>> OD = new ArrayList<Pair<NodeGraph, NodeGraph>>();
	static List<Float> distances = new ArrayList<Float>();

	//used only when loading OD sets
	static boolean readingFromPrevious =  false;
	static List<Integer> OR = new ArrayList<Integer>();
	static List<Integer> DE = new ArrayList<Integer>();

	public int numTripsScenario, numAgents, currentJob;
	double height, width, ratio;

	static boolean testingRegions = false;
	static boolean testingLandmarks = true;
	static boolean fiveElements = false;
	static boolean visibility = false;

	public static String criteria[];
	String criteriaLandmarks[] = {"roadDistance", "angularChange", "roadDistanceLandmarks", "angularChangeLandmarks",
			"localLandmarks", "globalLandmarks"};
	String criteriaRegions[] = {"roadDistance", "angularChange", "roadDistanceRegions", "angularChangeRegions",
			"roadDistanceBarriers", "angularChangeBarriers", "roadDistanceRegionsBarriers", "angularChangeRegionsBarriers"};

	//directories
	public static String cityName;
	public static String outputFolderLandmarks = "C:/Users/g_filo01/sciebo/Scripts/ABM analysis/Input/testingLandmarks/"+cityName;
	public static String outputFolderRegions = "C:/Users/g_filo01/sciebo/Scripts/ABM analysis/Input/testingRegions/"+cityName;

	/** Constructor */

	public PedSimCity(long seed, int job, String cityName)	{
		super(seed);
		this.currentJob = job;
		PedSimCity.cityName = cityName;
	}


	/** Initialization **/

	@Override
	public void start()	{
		if (testingRegions) criteria = criteriaRegions;
		else criteria = criteriaLandmarks;
		numAgents = criteria.length;
		super.start();
		Envelope MBR = null;
		MBR = roads.getMBR();
		MBR.expandToInclude(buildings.getMBR());
		MBR.expandToInclude(barriers.getMBR());
		roads.setMBR(MBR);

		// Nodes: - assigning scores to nodes and probabilities
		for (Object nG : nodesGeometries) {
			//street junctions and betweenness centrality
			MasonGeometry nodeGeometry = (MasonGeometry) nG;
			Integer nodeID = nodeGeometry.getIntegerAttribute("nodeID");
			NodeGraph node = network.findNode(nodeGeometry.geometry.getCoordinate());
			node.setID(nodeID);
			node.masonGeometry = nodeGeometry;
			node.primalEdge = null;
			if (testingRegions) node.centrality = nodeGeometry.getDoubleAttribute("Bc_multi");
			else  node.centrality = nodeGeometry.getDoubleAttribute("Bc_Rd");

			// Assigning to each Node landmarks at the Junction
			if (testingLandmarks)	{
				node.setLandmarkness(nodeGeometry);

				// a) landmarks visible from the junction
				if (visibility)	{
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
			node.setNeighboouringComponents();
			nodesMap.put(nodeID, node);
		}
		network.generateCentralityMap();

		// Extracting gateways
		if (testingRegions) {
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
					//nodes which are not gateways
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
					gd.entryAngle = Angle.angle(node, oppositeNode);

					RegionData dd = regionsMap.get(region);
					dd.gateways.add(gd);
					gatewaysMap.put(new Pair<NodeGraph, NodeGraph>(node, oppositeNode), gd);
					node.gateway = true;
				}
				node.adjacentRegions = node.getAdjacentRegion();
			}
		}

		// Street Segments: Assigning attributes
		for (Object o : network.getEdges())	{

			EdgeGraph edge = (EdgeGraph) o;
			int edgeID = edge.getIntegerAttribute("edgeID");
			edge.setID(edgeID);
			edge.resetDensities();

			if (testingRegions) {
				edge.setBarriers();

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

		// Centroids (Dual Graph): assigning edgeID to centroids in the dual graph

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

		// Creating regions' subgraphs
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

			try	{

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
			catch(java.lang.NullPointerException e){;}

			if (fiveElements) {
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

		// Populate
		populate();
		agents.setMBR(MBR);
	}




	public void populate() {

		int numTripsScenario;
		if (testingLandmarks) numTripsScenario = distances.size();
		else if (testingRegions) numTripsScenario = 2000;
		else numTripsScenario = 1000;
		ArrayList<ArrayList<NodeGraph>> listSequences = new ArrayList<ArrayList<NodeGraph>> ();
		AgentProperties ap = new AgentProperties();

		if (testingLandmarks)	{

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

		else if (testingRegions)
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
			ap.setProperties(criteria[i]);
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
		try {saveCSV();}
		catch (IOException e) {e.printStackTrace();}
		super.finish();
	}

	// Main function allows simulation to be run in stand-alone, non-GUI mode/

	public static void main(String[] args)
	{
		int jobs = 40;

		//		String cityName = "London";
		String cityName = args[0];
		//    	String directory = ("C:/Users/g_filo01/sciebo/Scripts/Image of the City/Outputs/"+cityName+"/");

		try	{
			///// READING PRIMAL GRAPH
			System.out.println("reading primal graph...");
			Bag junctionsAttributes = new Bag();
			junctionsAttributes.add("nodeID");
			String inputDataDirectory = null;

			if (testingLandmarks | fiveElements) {
				inputDataDirectory = "landmarksData"+"/"+cityName+"/";
				junctionsAttributes.add("loc_land"); //list of local landmarks at a node
				junctionsAttributes.add("loc_scor"); //list of their scores (local landmarkness)
				junctionsAttributes.add("dist_land"); //list of global landmarks visible from a node
				junctionsAttributes.add("dist_scor"); //list of their scores (global landmarkness)
				junctionsAttributes.add("anchors");   // list of buildings anchoring a node, as a destination
				junctionsAttributes.add("dist_anch"); // list of distances of the anchors from the node
				junctionsAttributes.add("distances");
				junctionsAttributes.add("Bc_Rd");

				// reading buildings
				System.out.println("reading buildings (landmarks) layer");
				Bag buildingsAttributes = new Bag();
				buildingsAttributes.add("buildingID");
				buildingsAttributes.add("lScore_sc");  //local landmarkness building
				buildingsAttributes.add("gScore_sc");  //global landmarkness building
				URL landmarksFile = PedSimCity.class.getResource(inputDataDirectory+"/"+cityName+"_landmarks.shp");
				ShapeFileImporter.read(landmarksFile, buildings);

				/// reading distances
				System.out.println("reading distances");
				CSVReader readerDistances = new CSVReader(new FileReader(inputDataDirectory+"/"+cityName+"_tracks_distances.csv"));
				String[] nextLineDistances;

				int ds = 0;
				while ((nextLineDistances = readerDistances.readNext()) != null) {
					ds += 1;
					if (ds == 1) continue;
					distances.add(Float.parseFloat(nextLineDistances[2]));
				}
				readerDistances.close();

				// visibility
				if (visibility)	{
					System.out.println("reading visibility");
					visibilityMatrix = new String[buildings.getGeometries().size()+1][nodesGeometries.size()+1];
					CSVReader reader = new CSVReader(new FileReader(inputDataDirectory+cityName+"_visibility_matrix_simplified.csv"));
					String [] nextLine;

					int v = 0;
					while ((nextLine = reader.readNext()) != null) {
						visibilityMatrix[v] = nextLine;
						v = v+1;
					}
					reader.close();
				}
			}

			if (testingRegions | fiveElements) {
				inputDataDirectory = "districtsData/"+cityName+"/";
				junctionsAttributes.add("nodeID");
				junctionsAttributes.add("district");
				junctionsAttributes.add("gateway");
				junctionsAttributes.add("Bc_multi");

				// reading barriers
				Bag barriersAttributes = new Bag();
				System.out.println("reading barriers layer...");
				barriersAttributes.add("barrierID");
				barriersAttributes.add("type");
				URL barriersFile = PedSimCity.class.getResource(inputDataDirectory+"/"+cityName+"_barriers.shp");
				ShapeFileImporter.read(barriersFile, barriers);
			}


			// read the street network shapefiles and create the primal and the dual graph
			System.out.println("reading the graphs");
			Bag centroidsAttributes = new Bag();
			centroidsAttributes.add("edgeID");

			URL roadsFile = PedSimCity.class.getResource(inputDataDirectory+"/"+cityName+"_edges.shp");
			URL junctionsFile = PedSimCity.class.getResource(inputDataDirectory+"/"+cityName+"_nodes.shp");
			ShapeFileImporter.read(roadsFile, roads);
			ShapeFileImporter.read(junctionsFile, junctions, junctionsAttributes);

			URL roadsDualFile = PedSimCity.class.getResource(inputDataDirectory+"/"+cityName+"_edgesDual.shp");
			URL centroidsFile = PedSimCity.class.getResource(inputDataDirectory+"/"+cityName+"_nodesDual.shp");
			ShapeFileImporter.read(roadsDualFile, intersectionsDual);
			ShapeFileImporter.read(centroidsFile, centroids, centroidsAttributes);

			network.fromGeomField(roads);
			dualNetwork.fromGeomField(intersectionsDual);
			nodesGeometries = junctions.getGeometries();
			centroidsGeometries = centroids.getGeometries();

			System.out.println("files imported successfully");

			for (int i = 0; i < jobs; i++) {

				if (readingFromPrevious) readingOD(outputFolderLandmarks+"_angularChange_"+i+".csv");
				System.out.println("Run nr.. "+i);
				SimState state = new PedSimCity(System.currentTimeMillis(), i, cityName);
				state.start();
				while (state.schedule.step(state)) {}
			}
		}
		catch (IOException e) {e.printStackTrace();}
		System.exit(0);
	}

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


	public void saveCSV() throws IOException {

		System.out.println("saving Densities");
		Bag edgesGeometries = roads.getGeometries();
		String csvSegments = null;

		if (testingRegions) {
			csvSegments = outputFolderRegions+"_PedSim_regions_"+(currentJob)+".csv";
			FileWriter writerDensitiesData = new FileWriter(csvSegments);
			CSVUtils.writeLine(writerDensitiesData, Arrays.asList("edgeID", "AC", "RB", "BB", "RBB"));


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

		else if (testingLandmarks) {

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
			if (testingLandmarks) csvRoutes = outputFolderLandmarks+"_PedSim_landmarks_routes_"+cr+"_"+(currentJob)+".csv";
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