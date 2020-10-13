package sim.app.geo.pedSimCity;

import com.opencsv.CSVReader;
import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.planargraph.DirectedEdgeStar;

import java.io.*;
import java.net.URL;
import java.util.*;
import java.util.Map.Entry;
import java.util.concurrent.ThreadLocalRandom;
import org.apache.commons.lang3.ArrayUtils;
import org.javatuples.Pair;


import sim.engine.SimState;
import sim.engine.Stoppable;
import sim.io.geo.ShapeFileImporter;
import sim.util.Bag;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.MasonGeometry;

import org.supercsv.io.CsvMapWriter;
import org.supercsv.io.ICsvMapWriter;
import org.supercsv.prefs.CsvPreference;

import sim.app.geo.urbanSim.*;

/**
 * The  simulation core.
 * 
 * The simulation can require a LOT of memory, so make sure the virtual machine has enough.
 * Do this by adding the following to the command line, or by setting up your run 
 * configuration in Eclipse to include the VM argument:
 * 
 * 		-Xmx2048M
 * 
 * With smaller simulations this chunk of memory is obviously not necessary. You can 
 * take it down to -Xmx800M or some such. If you get an OutOfMemory error, push it up.
 */


public class PedSimCity extends SimState
{
    private static final long serialVersionUID = 1L;
    
    //primal graph
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
    public static HashMap<Integer, NodeGraph> startingNodesMap = new HashMap<Integer, NodeGraph>();
    public static HashMap<MasonGeometry, Integer> nodesGeometryMap = new HashMap<MasonGeometry, Integer>();
    public static HashMap<Integer, BuildingData> buildingsMap = new HashMap<Integer, BuildingData>();
    public static HashMap<Integer, NodeGraph> centroidsMap =  new HashMap<Integer, NodeGraph>();
    public static HashMap<Integer, MasonGeometry> barriersMap = new HashMap<Integer, MasonGeometry>();
    public static HashMap<Integer, ArrayList<EdgeGraph>> barriersEdgesMap = new HashMap<Integer, ArrayList<EdgeGraph>>();
    public static HashMap<String, CriteriaData> criteriaMap = new HashMap<String, CriteriaData>();
    public static ArrayList<RouteData> routesData = new ArrayList<RouteData>();
    public static Bag startingNodes = new Bag();
    public static HashMap<Integer, RegionData> regionsMap = new HashMap<Integer, RegionData>();
    public static HashMap<Pair<NodeGraph, NodeGraph>, GatewayData> gatewaysMap = new HashMap<Pair<NodeGraph, NodeGraph>, GatewayData>();
       
    
    public static HashMap<MasonGeometry, Double> buildingsLS = new HashMap<MasonGeometry, Double>();
    public static HashMap<MasonGeometry, Double> buildingsGS = new HashMap<MasonGeometry, Double>();
    //dual graph
    public static VectorLayer intersectionsDual = new VectorLayer();
    public static VectorLayer centroids = new VectorLayer();
    public static Graph dualNetwork = new Graph();
    
    HashMap<EdgeGraph, ArrayList<Pedestrian>> edgeTraffic = new HashMap<EdgeGraph, ArrayList<Pedestrian>>();
    public static VectorLayer agents = new VectorLayer();
    
    ArrayList<Pedestrian> agentList = new ArrayList<Pedestrian>();
    List<Integer> districts = new ArrayList<Integer>();
    ArrayList<Pair<NodeGraph, NodeGraph>> OD = new ArrayList<Pair<NodeGraph, NodeGraph>>();

    static List<Float> distances = new ArrayList<Float>();
    
    static List<Integer> OR = new ArrayList<Integer>();
	static List<Integer> DE = new ArrayList<Integer>();
    

    public int numTripsScenario;
    public int numAgents;
	static boolean visibility = false;
	boolean dynamicRouting = false;
	static boolean regionBasedRouting = false;
	static boolean landmarkNavigation = true;
	static boolean fiveElements = false;
	
    // Some researcher-defined parameter
	static double regionBasedNavigationThreshold = 600; //Region-based navigation Threshold - meters
	static double visibilityThreshold = 300; //2d Advance Visibility threshold; distanLandmarks usage threshold
	static double wayfindingEasinessThreshold = 0.95; //2d Visibility threshold; distanLandmarks usage threshold
	static double globalLandmarknessWeight = 0.80; //weight Global Landmarkness in combination with edge cost
	static double globalLandmarkThreshold = 0.20; //
	static double localLandmarkThreshold = 0.30; //
	static double salientNodesPercentile = 0.75; // Threshold Percentile to identify salient nodes
	static double agentNoobThreshold = 0.25;
	
	public static String criteria[];
//    String criteriaLandmarks[] = {"roadDistance", "angularChange", "roadDistanceLandmarks", "angularChangeLandmarks", 
//    		"localLandmarks", "globalLandmarks"};
    String criteriaLandmarks[] = {"roadDistanceLandmarks", "angularChangeLandmarks", "localLandmarks"};
    String criteriaDistricts[] = {"roadDistance", "angularChange", "roadDistanceRegions", "angularChangeRegions", 
    		"roadDistanceBarriers", "angularChangeBarriers", "roadDistanceRegionsBarriers", "angularChangeRegionsBarriers"};

	double height, width, ratio;
	int currentJob;
	String cityName;
	
    /** Constructor */
	
    public PedSimCity(long seed, int job, String cityName)
    {
        super(seed);
    	this.currentJob = job;
    	this.cityName = cityName;
    }
    
    
    /** Initialization **/

    public void finish()
    {
    	try 
    	{
			saveCSV();
		} 
    	catch (IOException e) 
    	{
			e.printStackTrace();
		}
        super.finish();
    }
    
    public void saveCSV() throws IOException
    {
        System.out.println("saving Densities");
		Bag edgesGeometries = roads.getGeometries();
	   	
		String csvSegments = null;
	   	String csvSegmentsLandmarks = "C:/Users/g_filo01/sciebo/Scripts/ABM analysis/Input/landmarkNavigation/"+cityName+""
	   			+ "_PedSim_landmarks_RAGL_"+(currentJob+10)+".csv";
	   	String csvSegmentsDistricts = "C:/Users/g_filo01/sciebo/Scripts/ABM analysis/Input/regionBasedRouting/"+cityName+""
	   			+ "_PedSim_districts_"+(currentJob)+".csv";
		
		if (regionBasedRouting) 
		{
			csvSegments = csvSegmentsDistricts;
		   	FileWriter writerDensitiesData = new FileWriter(csvSegments);
		   	CSVUtils.writeLine(writerDensitiesData, Arrays.asList("edgeID", "road_distance", "angular_change", 
		   			"road_distance_regions", "angular_change_regions", "road_distance_barriers", 
		   			"angular_change_barriers", "road_distance_regions_barriers", "angular_change_regions_barriers"));
		   	
		   	
		   	int rGeoSize = edgesGeometries.size();
		   	for (int i = 0; i < rGeoSize; i++) 
		   	{
		    	MasonGeometry segment = (MasonGeometry) edgesGeometries.objs[i]; 
		    	EdgeGraph ed = edgesMap.get(segment.getIntegerAttribute("edgeID"));
		        CSVUtils.writeLine(writerDensitiesData, Arrays.asList(Integer.toString(
		        		ed.getID()), Integer.toString(ed.roadDistance),	
		        		Integer.toString(ed.angularChange), Integer.toString(ed.roadDistanceRegions), 
		        		Integer.toString(ed.angularChangeRegions), Integer.toString(ed.roadDistanceBarriers), 
		        		Integer.toString(ed.angularChangeBarriers),	
		        		Integer.toString(ed.roadDistanceRegionsBarriers),
		        		Integer.toString(ed.angularChangeRegionsBarriers)));
		    }
		   	writerDensitiesData.flush();
		   	writerDensitiesData.close();

		}
				
		else if (landmarkNavigation) 
		{
			csvSegments = csvSegmentsLandmarks;

			FileWriter writerDensitiesData = new FileWriter(csvSegments);
//			CSVUtils.writeLine(writerDensitiesData, Arrays.asList("edgeID", "road_distance", "angular_change", 
//	   			"topological", "road_distance_landmarks", "angular_change_landmarks",
//	   			"local_landmarks", "global_landmarks"));
			CSVUtils.writeLine(writerDensitiesData, Arrays.asList("edgeID",  
		   			"road_distance_landmarks", "angular_change_landmarks",
		   			"local_landmarks"));
	   	
		   	int rGeoSize = edgesGeometries.size();
		   	for (int i = 0; i < rGeoSize; i++) 
		   	{
		    	MasonGeometry segment = (MasonGeometry) edgesGeometries.objs[i]; 
		        EdgeGraph ed = edgesMap.get(segment.getIntegerAttribute("edgeID"));
//		        CSVUtils.writeLine(writerDensitiesData, Arrays.asList(Integer.toString(ed.getID()), 
//		        		Integer.toString(ed.roadDistanceLandmarks), Integer.toString(ed.angularChangeLandmarks), 
//		        		Integer.toString(ed.localLandmarks)));
		        CSVUtils.writeLine(writerDensitiesData, Arrays.asList(Integer.toString(ed.getID()), 
		        		Integer.toString(ed.roadDistanceLandmarks), Integer.toString(ed.angularChangeLandmarks), 
		        		Integer.toString(ed.localLandmarks)));
		   	}
		   	writerDensitiesData.flush();
		   	writerDensitiesData.close();
		}
		
	   	System.out.println("saving Routes");
	   	for (String cr : criteria)
	   	{
	   		String csvRoutes;
//	   		prexifOutput
	   		if (landmarkNavigation) csvRoutes = "C:/Users/g_filo01/sciebo/Scripts/ABM analysis/Input/landmarkNavigation/"+cityName
	   						+"_PedSim_landmarks_routes_RAGL_"+cr+"_"+(currentJob+10)+".csv";
	   		else csvRoutes = "C:/Users/g_filo01/sciebo/Scripts/ABM analysis/Input/regionBasedRouting/"+cityName
	   						+"_PedSim_district_routes_"+cr+"_"+(currentJob)+".csv";
	        
	   		List<String> header = new ArrayList<String>();
	        header.addAll(Arrays.asList(new String[] {"routeID", "origin", "destination"}));
	        for (int i = 0; i < edgesGeometries.size(); i++) 
	        {
	        	MasonGeometry segment = (MasonGeometry) edgesGeometries.objs[i]; 
	        	header.add(Integer.toString(segment.getIntegerAttribute("edgeID")));
	        }
	        
	        String[] headerArray = new String[header.size()];
	        header.toArray(headerArray);
	        List<Map<String, Object>> rows = new ArrayList<Map<String, Object>>();
	
	        for (RouteData rD : routesData) 
		   	{
		   		String routeCriteria =  rD.criteria;
		   		if (cr != routeCriteria) continue;
		        Map<String, Object> row = new HashMap<String, Object>();
		   		List<Integer> sequenceEdges = rD.sequenceEdges;

		   		int originNode =  rD.origin;
		   		int destinationNode = rD.destination;
		   		row.put(headerArray[0], Integer.toString(originNode)+"_"+Integer.toString(destinationNode));
		   		row.put(headerArray[1], originNode);
		   		row.put(headerArray[2], destinationNode);
		   		
		   		for (int e = 0; e < sequenceEdges.size(); e++) 
		   		{
		   			Integer position = ArrayUtils.indexOf(headerArray, Integer.toString(sequenceEdges.get(e)));
		   			row.put(headerArray[position], 1);
		   		}
		   		rows.add(row);
		   	}

	   		ICsvMapWriter mapWriter = null;
	   		try 
	   		{
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


    public void start()
    {
        if (regionBasedRouting) criteria = criteriaDistricts;
        else criteria = criteriaLandmarks;
        numAgents = criteria.length;
    	super.start();
    	Envelope MBR = null;
        MBR = roads.getMBR();
        MBR.expandToInclude(buildings.getMBR());    
        MBR.expandToInclude(barriers.getMBR());  
        roads.setMBR(MBR);
    	
        /** Nodes: - assigning scores to nodes and probabilities */
        for (Object nG : nodesGeometries) //street junctions and betweenness centrality
        {
        	MasonGeometry nodeGeometry = (MasonGeometry) nG; 
        	Integer nodeID = nodeGeometry.getIntegerAttribute("nodeID");
        	NodeGraph node = network.findNode(nodeGeometry.geometry.getCoordinate());
        	node.setID(nodeID);
        	node.masonGeometry = nodeGeometry;
        	node.primalEdge = null;
        	if (regionBasedRouting) node.centrality = nodeGeometry.getDoubleAttribute("Bc_multi");
        	else  node.centrality = nodeGeometry.getDoubleAttribute("Bc_E");
        	
            /** Assigning to each Node landmarks at the Junction */
        	if (landmarkNavigation)
        	{
    		    node.setLandmarkness(nodeGeometry);
	        		        	
	        	/** a) landmarks visible from the junction */
	        	if (visibility) 
	        	{
	            	int column = 0;
	            	for (int j = 1; j < visibilityMatrix[0].length; j++)
	            	{          		
	            		int examined = Integer.valueOf(visibilityMatrix[0][j]);
	            		if  (examined == nodeID)
	            	    {
	             	    	column = j;
	            	    	break;
	            	    }
	            	}
	            	for (int z = 1; z < visibilityMatrix.length; z++)
	            	{	
	            		int visibility = Integer.valueOf(visibilityMatrix[z][column]);
	            		if (visibility == 1) node.visible2d.add(Integer.valueOf(visibilityMatrix[z][0]));
	            	}
	        	}
        	}
        	node.setNeighboouringComponents();
        	nodesMap.put(nodeID, node);
        }
    	network.generateCentralityMap();
    	
        /** Extracting gateways */
	    if (regionBasedRouting)
	    {
	    	for (NodeGraph node : nodesMap.values())
        	{
            	node.region = node.masonGeometry.getIntegerAttribute("district");
            	if (regionsMap.get(node.region) == null)
            	{
            		RegionData region = new RegionData();
            		regionsMap.put(node.region, region);  
            	}
            }
	      
	    	for (NodeGraph node : nodesMap.values())
	        {  
	    		
		    	Integer gateway = node.masonGeometry.getIntegerAttribute("gateway"); // 1 or 0
            	Integer region = node.region;
		    	
            	if (gateway == 0) //nodes which are not gateways
		    	{
		    		startingNodes.add(node.masonGeometry);
		    		continue;
		    	}
            	
		        for (EdgeGraph bridge : node.getEdges())
		        {
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
	            
        /** BETWEENNESS CENTRALITY VALUES AND PROBABILITY OF CALLING NODE */
//        double sumBC =  nodesBc.values().stream().mapToDouble(d->d).sum();
//        for (MasonGeometry key : nodesBc.keySet())
//        {
//        	double bc = nodesBc.get(key);
//        	double probBC = bc/sumBC;         	
//        	nodesBc.put(key, probBC);
//        }
//        orderedNodesBc = utilities.sortByValue(nodesBc);
//        orderedNodesBc600 = utilities.sortByValue(nodesBc600);
        
        /** Landmarks: loading local and global scores of each building */
        for (Object o : buildings.getGeometries())
        {
        	MasonGeometry geometryBuilding = (MasonGeometry) o; 
        	Integer buildingID = geometryBuilding.getIntegerAttribute("buildingID");

        	BuildingData bd = new BuildingData();
        	bd.localScore = geometryBuilding.getDoubleAttribute("lScore_sc");
        	bd.globalScore = geometryBuilding.getDoubleAttribute("gScore_sc");
        	buildingsMap.put(buildingID, bd);
        	buildingsLS.put(geometryBuilding, bd.localScore);
        	buildingsGS.put(geometryBuilding, bd.globalScore);
        }
                   
        /** Street Segments: Assigning attributes */
        for (Object o : network.getEdges())
        {
        	EdgeGraph edge = (EdgeGraph) o;
        	int edgeID = edge.getIntegerAttribute("edgeID");
        	edge.setID(edgeID);
        	edge.resetDensities();
        	
            if (regionBasedRouting)
            {
            	edge.setBarriers();	
        		if (edge.u.region == edge.v.region)
        		{
        	    	int region = edge.u.region;
        	    	edge.region = region;
        	    	RegionData dd = regionsMap.get(region);
        	    	dd.edgesMap.add(edge);
        		}
        		else edge.region = 999999;
            }
            edgesMap.put(edgeID, edge);
        }  
        
        /** Centroids (Dual Graph): assigning edgeID to centroids in the dual graph */
        
        for (Object cG : centroidsGeometries) 
        {
        	MasonGeometry centroidGeometry = (MasonGeometry) cG; 
        	int edgeID = centroidGeometry.getIntegerAttribute("edgeID"); 

      	    NodeGraph cen = dualNetwork.findNode(centroidGeometry.geometry.getCoordinate());
      	    cen.masonGeometry = centroidGeometry;
        	cen.setID(edgeID);
        	cen.primalEdge = edgesMap.get(edgeID);
        	edgesMap.get(edgeID).dualNode = cen;
        	centroidsMap.put(edgeID, cen);
        }
        
        /** CRITERIA INFORMATION */
        for (int i = 0; i < criteria.length; i++)
        {
        	CriteriaData cd = new CriteriaData();
        	cd.criteria = criteria[i];   
        	cd.trips = 0;
        	cd.totalDistance = 0;
            criteriaMap.put(criteria[i], cd);
        }
                
        /** CREATING SUBGRAPHS */
        for (Entry<Integer, RegionData> entry : regionsMap.entrySet())
        {
        	ArrayList<EdgeGraph> edgesDistrict = entry.getValue().edgesMap;
        	ArrayList<EdgeGraph> dualEdgesDistrict = new ArrayList<EdgeGraph>();
        	SubGraph primalGraph = new SubGraph(network, edgesDistrict);
        	
        	for (EdgeGraph edge: edgesDistrict) 
        	{
	    		NodeGraph cen = edge.dualNode;
	    		cen.region = entry.getKey();
	    		DirectedEdgeStar dirEdges = cen.getOutEdges();
	            for (Object dE : dirEdges.getEdges())
	    		{
	            	GeomPlanarGraphDirectedEdge dEdge = (GeomPlanarGraphDirectedEdge) dE;
	            	dualEdgesDistrict.add((EdgeGraph) dEdge.getEdge());
	    		 }
        	}
        	SubGraph dualGraph = new SubGraph(dualNetwork, dualEdgesDistrict);
        	primalGraph.setBarriersGraph();
        	regionsMap.get(entry.getKey()).primalGraph = primalGraph;
        	regionsMap.get(entry.getKey()).dualGraph = dualGraph;
        	
        	try 
        	{
        		Bag barriersGeometries = barriers.getGeometries();
        		for (Object bG : barriersGeometries ) 
        		{
        			MasonGeometry barrierGeo = (MasonGeometry) bG;
        			int barrierID = barrierGeo.getIntegerAttribute("barrierID");
        			barriersMap.put(barrierID, barrierGeo);
        			ArrayList<EdgeGraph> alongEdges = new ArrayList<EdgeGraph>();
        			
        			for (EdgeGraph edge : network.edgesGraph)
        			{
        				if ((edge.barriers != null) && (edge.barriers.contains(barrierID))) alongEdges.add(edge);
        			}
        			barriersEdgesMap.put(barrierID, alongEdges);
        		}
           	}
        	catch(java.lang.NullPointerException e){;}
        	
        	if (fiveElements)
        	{
        		VectorLayer regionNetwork = new VectorLayer();
        		for (EdgeGraph edge : edgesDistrict) regionNetwork.addGeometry(edge.masonGeometry);
        		double buildingsComplexity = 1.0;
            	ArrayList<MasonGeometry> buildings = LandmarkNavigation.getBuildings(null, null, entry.getKey());
            	regionsMap.get(entry.getKey()).regionNetwork = regionNetwork;
            	regionsMap.get(entry.getKey()).buildings = buildings;
            	regionsMap.get(entry.getKey()).assignLandmarks();
        	}
        }
            
        /// AGENTS
        populate();
        agents.setMBR(MBR);      
        }
   


    
    

    public void populate()
    {
    	
    	int numTripsScenario;
    	if (landmarkNavigation) numTripsScenario = distances.size();
    	else numTripsScenario = 1;
        ArrayList<ArrayList<NodeGraph>> listSequences = new ArrayList<ArrayList<NodeGraph>> ();


    	if (landmarkNavigation)
    	{
	    	for (int i = 0; i < numTripsScenario; i++)
	    	{
//	    		NodeGraph originNode = null;
//	    		while (originNode == null) originNode = NodesLookup.randomNode(nodesGeometries, network);
////	    		NodeGraph destinationNode = NodesLookup.nodeWithinFromDistances(originNode, junctions, distances, network);
////	    		
//////	    		int OR [] = {15425, 12632, 10024};
//////	    		int DE [] = {28557, 32173, 3999};
////	    		int OR [] = {11555, 12632, 10024};
////	    		int DE [] = {25583, 32173, 3999};
////	    		for (int v = 0; v <= 1; v++)
//	    		
	    		NodeGraph originNode = nodesMap.get(OR.get(i));
	    		NodeGraph destinationNode = nodesMap.get(DE.get(i));
//	    		NodeGraph originNode = nodesMap.get(21392 );
//	    		NodeGraph destinationNode = nodesMap.get(30719);
	    		if (i == 0) System.out.println(" ORIGIN "+ OR.get(i)+"  "+DE.get(i));
		    	Pair<NodeGraph, NodeGraph> pair = new Pair<NodeGraph, NodeGraph> (originNode, destinationNode);
    		
    			ArrayList<NodeGraph> sequence = LandmarkNavigation.findSequenceSubGoals(originNode, destinationNode, false, "local");
    			listSequences.add(sequence);
		    	OD.add(pair);  
	    		}
	    	}

    	else if (regionBasedRouting)
    	{
//    		for (int i = 0; i < numTripsScenario; i++)
//	    	{	
//    			NodeGraph originNode = null;
//	    		NodeGraph destinationNode = null;
//	    		while ((destinationNode == null) || (originNode == destinationNode))
//	    		{
//	    			originNode = null;
//	    			while (originNode == null) originNode = NodesLookup.randomNode(startingNodes, network);
//	    			destinationNode = NodesLookup.nodeWithinDistance(originNode, 1000, 3000, network);
//	    		}
//	    		Pair<NodeGraph, NodeGraph> pair = new Pair<NodeGraph, NodeGraph> (originNode, destinationNode);
//	    	    OD.add(pair);  
//	    	}
    	}
    	System.out.println("OD matrix ready");

    	for (int i = 0; i < numAgents; i++)
        {   
        	String routeCriteria = null;
        	routeCriteria = criteria[i];
        	Pedestrian a = new Pedestrian(this, routeCriteria, OD, listSequences); //, goalEdge )

	        MasonGeometry newGeometry = a.getGeometry();
	        newGeometry.isMovable = true;
	        agents.addGeometry(newGeometry);
	        agentList.add(a);
	        a.getGeometry().setUserData(a);
	        Stoppable stop = schedule.scheduleRepeating(a);
	        a.setStoppable(stop);
	        schedule.scheduleRepeating(agents.scheduleSpatialIndexUpdater(), Integer.MAX_VALUE, 1.0);
      }
    }
        
    // Main function allows simulation to be run in stand-alone, non-GUI mode/
    
    public static void main(String[] args)
    {
    	int jobs = 40;
    	String cityName = "London";
//    	String directory = ("C:/Users/g_filo01/sciebo/Scripts/Image of the City/Outputs/"+cityName+"/");
        
    	try 
        {
        	///// READING PRIMAL GRAPH
        	System.out.println("reading primal graph...");
            Bag junctionsAttributes = new Bag();
            junctionsAttributes.add("nodeID"); 
            String data = null;

            if (landmarkNavigation)
            {
            	data = "landmarksData"+"/"+cityName+"/";
	            junctionsAttributes.add("loc_land"); //list of local landmarks at a node
	            junctionsAttributes.add("loc_scor"); //list of their scores (local landmarkness)
	            junctionsAttributes.add("dist_land"); //list of global landmarks visible from a node
	            junctionsAttributes.add("dist_scor"); //list of their scores (global landmarkness)
	            junctionsAttributes.add("anchors");   // list of buildings anchoring a node, as a destination
	            junctionsAttributes.add("dist_anch"); // list of distances of the anchors from the node 
	            junctionsAttributes.add("distances");  
	            junctionsAttributes.add("Bc_E"); 
	            
	            Bag buildingsAttributes = new Bag();
	            ///// READING BUILDINGS
	            System.out.println("reading buildings layer...");
	            buildingsAttributes.add("buildingID");  
	            buildingsAttributes.add("lScore_sc");  //local landmarkness building
	            buildingsAttributes.add("gScore_sc");  //global landmarkness building
	            URL landmarksFile = PedSimCity.class.getResource(data+"/"+cityName+"_landmarks.shp");
	            ShapeFileImporter.read(landmarksFile, buildings);
	            
	            /// READING DISTANCES
				System.out.println("reading distances...");
				CSVReader readerDistances = new CSVReader(new FileReader
				  		("C:/Users/g_filo01/sciebo/Scripts/GPS Trajectories/Outputs/"+cityName+"/"+cityName+"_tracks_distances.csv"));
				String[] nextLineDistances;
				  
				int ds = 0;
				while ((nextLineDistances = readerDistances.readNext()) != null) 
				{
				  ds += 1;
				  if (ds == 1) continue;
				  distances.add(Float.parseFloat(nextLineDistances[2]));
				}
				readerDistances.close();
	                        
	            if (visibility)
	            {
		            System.out.println("reading visibility...");
		            visibilityMatrix = new String[buildings.getGeometries().size()+1][nodesGeometries.size()+1];
		            CSVReader reader = new CSVReader(new FileReader(data+cityName+"_visibility_matrix_simplified.csv"));
		            String [] nextLine;
		            
		            int v = 0;
		            while ((nextLine = reader.readNext()) != null) 
		            {
		            	visibilityMatrix[v] = nextLine;
		            	v = v+1;
		            }
		            reader.close();
	            }

            }
            
            if (regionBasedRouting)
    		{
            	data = "districtsData/"+cityName+"/";
		        junctionsAttributes.add("nodeID"); 
		        junctionsAttributes.add("district"); 
		        junctionsAttributes.add("gateway"); 
	            junctionsAttributes.add("Bc_multi"); 
	            

	            ///// READING BARRIERS
	            Bag barriersAttributes = new Bag();
	            System.out.println("reading barriers layer...");
	            barriersAttributes.add("barrierID");  
	            barriersAttributes.add("type");  
	            URL barriersFile = PedSimCity.class.getResource(data+"/"+cityName+"_barriers.shp");
	            ShapeFileImporter.read(barriersFile, barriers);
    		}
            

            ///// READING DUAL GRAPH ------------
            System.out.println("reading dual graph...");
            Bag centroidsAttributes = new Bag();
            centroidsAttributes.add("edgeID");
            String simplified = "_simplified";

            URL roadsDualFile = PedSimCity.class.getResource(data+"/"+cityName+"_edgesDual"+simplified+".shp");
            URL centroidsFile = PedSimCity.class.getResource(data+"/"+cityName+"_nodesDual"+simplified+".shp");            
            ShapeFileImporter.read(roadsDualFile, intersectionsDual);
            ShapeFileImporter.read(centroidsFile, centroids, centroidsAttributes);
            
	        URL roadsFile = PedSimCity.class.getResource(data+"/"+cityName+"_edges"+simplified+".shp");
	        URL junctionsFile = PedSimCity.class.getResource(data+"/"+cityName+"_nodes"+simplified+".shp");
            ShapeFileImporter.read(roadsFile, roads);
            ShapeFileImporter.read(junctionsFile, junctions, junctionsAttributes);
            network.fromGeomField(roads);
            dualNetwork.fromGeomField(intersectionsDual);          			
            nodesGeometries = junctions.getGeometries();
            centroidsGeometries = centroids.getGeometries();
            
            System.out.println("files imported successfully");
	        
	    	for (int i = 0; i < jobs; i++)
	    	{
	    		int run = i+10;
				CSVReader readerOD;
				readerOD = new CSVReader(new FileReader("C:/Users/g_filo01/sciebo/Scripts/ABM analysis/Input/landmarkNavigation/"
						+ "London_PedSim_landmarks_routes_angularChange_"+run+".csv"));

				String[] nextLine;
				OR.clear();
				DE.clear();
				  
				int dv = 0;
				while ((nextLine = readerOD.readNext()) != null) 
				{
				  dv += 1;
				  if (dv == 1) continue;
				  OR.add(Integer.parseInt(nextLine[1]));
				  DE.add(Integer.parseInt(nextLine[2]));
				}	
				readerOD.close();    		
	    		
		    	System.out.println("Run nr.. "+i);
		    	SimState state = new PedSimCity(System.currentTimeMillis(), i, cityName); 
		    	state.start(); 
		    	while (state.schedule.step(state)) {}
	    	}
		} 
        catch (IOException e)
        {
			e.printStackTrace();
		}
    	System.exit(0);
    }
    

}