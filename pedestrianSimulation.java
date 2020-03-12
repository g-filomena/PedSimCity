package sim.app.geo.pedestrianSimulation;

import com.opencsv.CSVReader;
import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.planargraph.DirectedEdgeStar;
import com.vividsolutions.jts.planargraph.Node;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

import java.net.URL;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ThreadLocalRandom;

import org.apache.commons.lang3.ArrayUtils;
import org.javatuples.Pair;

import sim.engine.SimState;
import sim.engine.Stoppable;
import sim.field.geo.GeomVectorField;
import sim.io.geo.ShapeFileImporter;
import sim.util.Bag;
import sim.util.geo.GeomPlanarGraph;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.GeomPlanarGraphEdge;
import sim.util.geo.MasonGeometry;

import org.supercsv.io.CsvMapWriter;
import org.supercsv.io.ICsvMapWriter;
import org.supercsv.prefs.CsvPreference;



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


public class PedestrianSimulation extends SimState
{
    private static final long serialVersionUID = 1L;
    
    //primal graph
    public static GeomVectorField roads = new GeomVectorField();
    public static GeomVectorField buildings = new GeomVectorField();
    public static GeomVectorField junctions = new GeomVectorField();
    public static GeomVectorField barriers = new GeomVectorField();
    public static GeomPlanarGraph network = new GeomPlanarGraph();
    public static Bag centroidsGeometries;
    public static Bag nodesGeometries;
    public static String [][] visibilityMatrix;
    
    public HashMap<Integer, EdgeData> edgesMap = new HashMap<Integer, EdgeData>();
    public HashMap<Integer, NodeData> nodesMap = new HashMap<Integer, NodeData>();
    public static HashMap<Integer, Node> startingNodesMap = new HashMap<Integer, Node>();
    public HashMap<MasonGeometry, Integer> nodesGeometryMap = new HashMap<MasonGeometry, Integer>();
    public HashMap<Integer, BuildingData> buildingsMap = new HashMap<Integer, BuildingData>();
    public HashMap<Integer, CentroidData> centroidsMap =  new HashMap<Integer, CentroidData>();
    public HashMap<String, CriteriaData> criteriaMap = new HashMap<String, CriteriaData>();
    public ArrayList<RouteData> routesData = new ArrayList<RouteData>();
    
    public HashMap<Integer, GeomPlanarGraph> districtMap = new HashMap<Integer, GeomPlanarGraph>();
    public HashMap<Integer, GeomPlanarGraph> districtDualMap = new HashMap<Integer, GeomPlanarGraph>();
    public HashMap<Integer, Integer> edgesDistrictMap = new HashMap<Integer, Integer>();
    public HashMap<Integer,ArrayList<GatewayData>> exitDistrictsMap = new HashMap<Integer,ArrayList<GatewayData>>();
    public HashMap<Integer,GatewayData> gatewaysMap = new HashMap<Integer,GatewayData>();
    public HashMap<Pair<Node, Node>, Double> euclideanDistancesMap = new HashMap<Pair<Node, Node>, Double>();
    public HashMap<Pair<Node, Node>, Double> angularDistancesMap = new HashMap<Pair<Node, Node>, Double>();
       
    //dual graph
    public static GeomVectorField intersectionsDual = new GeomVectorField();
    public static GeomVectorField centroids = new GeomVectorField();
    public static GeomPlanarGraph dualNetwork = new GeomPlanarGraph();
    
    HashMap<GeomPlanarGraphEdge, ArrayList<Pedestrian>> edgeTraffic = new HashMap<GeomPlanarGraphEdge, ArrayList<Pedestrian>>();
    HashMap<MasonGeometry, Double> nodesBc =  new HashMap<MasonGeometry, Double>();
    HashMap<MasonGeometry, Double> nodesBc600 =  new HashMap<MasonGeometry, Double>();
    public HashMap<MasonGeometry, Double> buildingsLS = new HashMap<MasonGeometry, Double>();
    public HashMap<MasonGeometry, Double> buildingsGS = new HashMap<MasonGeometry, Double>();
    Map orderedNodesBc;
    Map orderedNodesBc600;
    
//    HashMap<MasonGeometry, Double> edgesLength =  new HashMap<MasonGeometry, Double>();
//    HashMap<Double, HashMap<MasonGeometry, Double>> nodesCDF =  new HashMap<Double, HashMap<MasonGeometry, Double>>();

    public GeomVectorField agents = new GeomVectorField();
    public GeomVectorField newRoads = new GeomVectorField();
    
    ArrayList<Pedestrian> agentList = new ArrayList<Pedestrian>();
    List<Integer> districts = new ArrayList<Integer>();
    ArrayList<Pair<Node, Node>> OD = new ArrayList<Pair<Node, Node>>();

    static List<Float> distances = new ArrayList<Float>();
    
    // landmark routing parameter
    public double t = 300.0;
//    public int numTripsScenario = 271;

    public int numAgents;
    
	static boolean visibility = false;
	boolean dynamicRouting = false;
	static boolean regionalRouting = true;
	static boolean landmarksRouting = false;
	
	String criteria[];
    String criteriaLandmarks[] = {"roadDistance", "angularChange", "roadDistanceLandmarks", "angularChangeLandmarks", 
    		"localLandmarks", "globalLandmarks"};
    String criteriaDistricts[] = {"roadDistance", "angularChange", "roadDistanceRegions", "angularChangeRegions", 
    		"roadDistanceRegionsBarriers", "angularChangeRegionsBarriers"};

	double height, width, ratio;
	int current_job;
	String cityName;
	
    /** Constructor */
	
    public PedestrianSimulation(long seed, int job, String cityName)
    {
        super(seed);
    	this.current_job = job;
    	this.cityName = cityName;
//    	String [][] visibilityMatrix; 
    }
    
    
    /** Initialization 
     **/
    public void finish()
    {
    	try {
			saveCSV();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
        super.finish();
    }
    
    public void saveCSV() throws IOException
    {
        System.out.println("saving Densities");
		Bag rGeometries = roads.getGeometries();
	   	
		String csvSegments = null;
	   	String csvSegmentsLandmarks = "C:/Users/g_filo01/sciebo/GIS Data/Simulation/landmarksRouting/"+cityName+"_PedSim_landmarks_"+current_job+".csv";
	   	String csvSegmentsDistricts = "C:/Users/g_filo01/sciebo/GIS Data/Simulation/regionalRouting/"+cityName+"_PedSim_districts_"+current_job+".csv";

		
		if (regionalRouting) 
		{
			csvSegments = csvSegmentsDistricts;
		   	FileWriter writerDensitiesData = new FileWriter(csvSegments);
		   	CSVUtils.writeLine(writerDensitiesData, Arrays.asList("edgeID", "road_distance", "angular_change", 
		   			"road_distance_regions", "angular_change_regions", "road_distance_regions_barriers", 
		   			"angular_change_regions_barriers"));
		   	
		   	int rGeoSize = rGeometries.size();
		   	for (int i = 0; i < rGeoSize; i++) 
		   	{
		    	MasonGeometry segment = (MasonGeometry) rGeometries.objs[i]; 
		        EdgeData ed = edgesMap.get(segment.getIntegerAttribute("edgeID"));
		        CSVUtils.writeLine(writerDensitiesData, Arrays.asList(Integer.toString(segment.getIntegerAttribute("edgeID")), 
		        		Integer.toString(ed.roadDistance),	Integer.toString(ed.angularChange), 
		        		Integer.toString(ed.roadDistanceRegions), Integer.toString(ed.angularChangeRegions), 
		        		Integer.toString(ed.roadDistanceRegionsBarriers), Integer.toString(ed.angularChangeRegionsBarriers)));
		    }
		   	writerDensitiesData.flush();
		   	writerDensitiesData.close();

		}
				
		else if (landmarksRouting) 
		{
			csvSegments = csvSegmentsLandmarks;

			FileWriter writerDensitiesData = new FileWriter(csvSegments);
			CSVUtils.writeLine(writerDensitiesData, Arrays.asList("edgeID", "road_distance", "angular_change", 
	   			"topological", "road_distance", "angular_change_landmarks",
	   			"local_landmarks", "global_landmarks"));
	   	
		   	int rGeoSize = rGeometries.size();
		   	for (int i = 0; i < rGeoSize; i++) 
		   	{
		    	MasonGeometry segment = (MasonGeometry) rGeometries.objs[i]; 
		        EdgeData ed = edgesMap.get(segment.getIntegerAttribute("edgeID"));
		        CSVUtils.writeLine(writerDensitiesData, Arrays.asList(Integer.toString(segment.getIntegerAttribute("edgeID")), 
		        		Integer.toString(ed.roadDistance),	Integer.toString(ed.angularChange), Integer.toString(ed.topological), 
		        		Integer.toString(ed.roadDistanceLandmarks), Integer.toString(ed.angularChangeLandmarks), 
		        		Integer.toString(ed.localLandmarks), Integer.toString(ed.globalLandmarks)));
		   	}
		   	writerDensitiesData.flush();
		   	writerDensitiesData.close();
		}
		
	   	System.out.println("saving Routes");
	   	for (String cr : criteria)
	   	{
	   		String csvRoutes;
	   		if (landmarksRouting) csvRoutes = "C:/Users/g_filo01/sciebo/GIS Data/Simulation/landmarksRouting/PedSim_landmarks_routes_"+cr+"_"+current_job+".csv";
	   		else csvRoutes = "C:/Users/g_filo01/sciebo/GIS Data/Simulation/regionalRouting/PedSim_district_routes_"+cr+"_"+current_job+".csv";
	        
	   		List<String> header = new ArrayList<String>();
	        header.addAll(Arrays.asList(new String[] {"routeID", "origin", "destination"}));
	        for (int i = 0; i < rGeometries.size(); i++) 
	        {
	        	MasonGeometry segment = (MasonGeometry) rGeometries.objs[i]; 
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
    }


    public void start()
    {
        if (regionalRouting) criteria = criteriaDistricts;
        else criteria = criteriaLandmarks;
        numAgents = criteria.length;
    	super.start();
    	Envelope MBR = null;
        MBR = roads.getMBR();
        MBR.expandToInclude(buildings.getMBR());    
        MBR.expandToInclude(barriers.getMBR());  
        roads.setMBR(MBR);
    	int counterGateways = 0;   
    	
        ///// NODES - assigning scores to nodes and probabilities ///////////////////////////////////////////
        for (Object nG : nodesGeometries) //street junctions and betweenness centrality
        {
        	MasonGeometry nodeGeometry = (MasonGeometry) nG; 
        	Integer nodeID = nodeGeometry.getIntegerAttribute("nodeID");
        	Node node = network.findNode(nodeGeometry.geometry.getCoordinate());
        	node.setData(nodeID);
        	NodeData nd = new NodeData();
        	nd.node = node;
        	nodesGeometryMap.put(nodeGeometry, nodeID);
        	
            ///// ASSIGNING TO EACH NODE LANDMARKS AT THE JUNCTION
        	if (landmarksRouting)
        	{
    		   	double Bc = nodeGeometry.getDoubleAttribute("Bc_E");
    		    nodesBc.put(nodeGeometry, Bc);
	        	String localString = nodeGeometry.getStringAttribute("loc_land");
	        	String lScoresString = nodeGeometry.getStringAttribute("loc_scor");
	        	String distantString = nodeGeometry.getStringAttribute("dist_land");
	        	String dScoresString = nodeGeometry.getStringAttribute("dist_scor");
	        	String anchorsString = nodeGeometry.getStringAttribute("anchors");
	        	String distancesString = nodeGeometry.getStringAttribute("distances");
	        	
	           
	        	List<Integer> localLandmarks = new ArrayList<Integer>();
	        	List<Double> localScores = new ArrayList<Double>();
	        	List<Integer> distantLandmarks = new ArrayList<Integer>();
	        	List<Double> distantScores = new ArrayList<Double>();
	        	List<Integer> anchors = new ArrayList<Integer>();
	        	List<Double> landmarkDistances = new ArrayList<Double>();
	        	

	        	if (!localString.equals("[]"))
	        	{
	        		String l = localString.replaceAll("[^-?0-9]+", " ");
	        		String s = lScoresString.replaceAll("[^0-9.]+", " ");
	            	for(String t : (Arrays.asList(l.trim().split(" ")))) localLandmarks.add(Integer.valueOf(t));
	            	for(String t : (Arrays.asList(s.trim().split(" ")))) localScores.add(Double.valueOf(t));
	            	nd.localLandmarks = localLandmarks;
	            	nd.localScores = localScores;
	        	}
	        	else
	        	{
	            	nd.localLandmarks = null;
	            	nd.localScores = null;
	        	}
	        	
	        	
	        	if (!distantString.equals("[]"))
	        	{
	        		String l = distantString.replaceAll("[^-?0-9]+", " ");
	        		String s = dScoresString.replaceAll("[^0-9.]+", " ");
	        		for(String t : (Arrays.asList(l.trim().split(" ")))) distantLandmarks.add(Integer.valueOf(t));
	        		for(String t : (Arrays.asList(s.trim().split(" ")))) distantScores.add(Double.valueOf(t));
	        		nd.distantLandmarks = distantLandmarks;
	        		nd.distantScores = distantScores;
	        	}
	        	else
	        	{
	            	nd.distantLandmarks = null;
	            	nd.distantScores = null;
	        	}
	            	
	        	if (!anchorsString.equals("[]"))
	        	{
					String l = anchorsString.replaceAll("[^-?0-9]+", " ");
					String d = distancesString.replaceAll("[^0-9.]+", " ");
					for(String t : (Arrays.asList(l.trim().split(" ")))) anchors.add(Integer.valueOf(t));
					for(String t : (Arrays.asList(d.trim().split(" ")))) landmarkDistances.add(Double.valueOf(t));
					if (anchors.size() > landmarkDistances.size()) System.out.println("------------ " + nodeID);
					nd.anchors = anchors;
					nd.distances = landmarkDistances;
	        	}
	        	else nd.anchors = null;  
	        	
	        	///// ASSIGNING TO EACH NODE LANDMARKS VISIBLE FROM THE JUNCTION
	        	if (visibility) 
	        	{
	        		List<Integer> visible2d = new ArrayList<Integer>();
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
	            		if (visibility == 1) visible2d.add(Integer.valueOf(visibilityMatrix[z][0]));
	            	}
	            	nd.visible2d = visible2d;
	        	}
	        	
        	}
        	
        	nodesMap.put(nodeID, nd);
            ///// DISTRICT INFORMATION

        }

        ///// GATEWAYS
	    if (regionalRouting)
	    {
	    	for (Object nG : nodesGeometries)
        	{
	    		MasonGeometry nodeGeometry = (MasonGeometry) nG; 
	        	Integer nodeID = nodeGeometry.getIntegerAttribute("nodeID");
            	Integer district = nodeGeometry.getIntegerAttribute("district");
	    		NodeData nd = nodesMap.get(nodeID);
            	nd.district = district;
            	nodesMap.put(nodeID, nd);
            	
            	if (!districts.contains(district))
            	{
	        		districts.add(district);
//	        		GeomPlanarGraph districtNetwork = new GeomPlanarGraph();
//	        		GeomPlanarGraph districtDualNetwork = new GeomPlanarGraph();
//	            	districtMap.put(district, districtNetwork);
//	            	districtDualMap.put(district, districtDualNetwork);            	
	            	ArrayList<GatewayData> gateways = new ArrayList<GatewayData>();
	            	exitDistrictsMap.put(district, gateways);
            	}
            }
	      
	        for (Object nG : nodesGeometries) //street junctions and betweenness centrality
	        {  
	        	MasonGeometry nodeGeometry = (MasonGeometry) nG; 
	        	Integer nodeID = nodeGeometry.getIntegerAttribute("nodeID");
		    	Integer gateway = nodeGeometry.getIntegerAttribute("gateway");
		    	Node node = network.findNode(nodeGeometry.geometry.getCoordinate());
            	Integer district = nodeGeometry.getIntegerAttribute("district");
		    	
            	if (gateway == 0) //nodes which are not gateways
		    	{
            		if (nodeID == 5525) System.out.println("wrong "+gateway+"  "+ district);
		    		startingNodesMap.put(nodeID, node);
		    		continue;
		    	}

		    	DirectedEdgeStar dirEdges = node.getOutEdges();
		        for (Object dE : dirEdges.getEdges())
		        {
		           GeomPlanarGraphDirectedEdge l = (GeomPlanarGraphDirectedEdge) dE;
		           GeomPlanarGraphEdge d = (GeomPlanarGraphEdge) l.getEdge();
		           Node oppositeNode = d.getOppositeNode(node);
		           
		           Integer opossiteNodeID = (Integer) oppositeNode.getData();
		           int possibleRegion = nodesMap.get(opossiteNodeID).district;
		           if (possibleRegion == district) continue;
		
		           GatewayData gd = new GatewayData();
			       gd.nodeID = nodeID;
			       gd.node = node;
			       gd.edgeID = d.getIntegerAttribute("edgeID");
			       gd.gatewayID = counterGateways;
			       gd.district = district;
			       gd.regionTo = possibleRegion;
			       gd.entryID = opossiteNodeID;
			       gd.distance = d.getLine().getLength();
			       gd.entryAngle = utilities.angle(nodeGeometry.geometry.getCoordinate(), oppositeNode.getCoordinate());
			       ArrayList<GatewayData> gateways = exitDistrictsMap.get(district);
			       gateways.add(gd);
			       
			       exitDistrictsMap.put(district, gateways);
			       gatewaysMap.put(counterGateways, gd);
			       
			       NodeData nd =nodesMap.get(nodeID);
			       nd.gateway = true;
			       nodesMap.put(nodeID, nd);
			       counterGateways += 1;
		        }
	        }
	    }
        
        ///// BETWEENNESS CENTRALITY VALUES AND PROBABILITY OF CALLING NODE
//        double sumBC =  nodesBc.values().stream().mapToDouble(d->d).sum();
//        for (MasonGeometry key : nodesBc.keySet())
//        {
//        	double bc = nodesBc.get(key);
//        	double probBC = bc/sumBC;         	
//        	nodesBc.put(key, probBC);
//        }
//        orderedNodesBc = utilities.sortByValue(nodesBc);
//        orderedNodesBc600 = utilities.sortByValue(nodesBc600);
        
        //// BUILDINGS ///////////////////////////////////////////
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
                   
        ///// CENTROIDS: assigning edgeID to centroids in the dual graph

        for (Object cG : centroidsGeometries) 
        {
        	MasonGeometry centroidGeometry = (MasonGeometry) cG; 
        	int edgeID = centroidGeometry.getIntegerAttribute("edgeID"); 
      	    Node cen = dualNetwork.findNode(centroidGeometry.geometry.getCoordinate());
        	cen.setData(edgeID);
        	
        	CentroidData cd = new CentroidData();
        	cd.c = cen;
        	centroidsMap.put(edgeID, cd);
        }
        

        ///// ASSIGNING DATA TO STREET SEGMENTS ///////////////////////////////////////////
        ArrayList <GeomPlanarGraphDirectedEdge> processed = new ArrayList <GeomPlanarGraphDirectedEdge>();
        for (Object o : network.getEdges())
        {
        	GeomPlanarGraphEdge edge = (GeomPlanarGraphEdge) o;
        	EdgeData ed = new EdgeData();
        	int edgeID =  edge.getIntegerAttribute("edgeID");
        	edge.setData(edgeID);
        	
        	ed.planarEdge = edge;   
        	ed.fromNode = edge.getIntegerAttribute("u");  
        	ed.toNode = edge.getIntegerAttribute("v");  
            edgesMap.put(edgeID, ed);

            if (regionalRouting)
            {
            	String pBarriersString = edge.getStringAttribute("p_barr");
            	String nBarriersString = edge.getStringAttribute("n_barr");

            	List<Integer> positiveBarriers = new ArrayList<Integer>();
            	List<Integer> negativeBarriers = new ArrayList<Integer>();
	
            	///// ASSIGNING TO EACH NODE LANDMARKS AT THE JUNCTION
            	if (!pBarriersString.equals("[]"))
            	{
            		String p = pBarriersString.replaceAll("[^-?0-9]+", " ");
                	for(String t : (Arrays.asList(p.trim().split(" ")))) positiveBarriers.add(Integer.valueOf(t));
                	ed.positiveBarriers = positiveBarriers;
            	}
            	else ed.positiveBarriers = null;
            	
            	if (!nBarriersString.equals("[]"))
            	{
            		String n = nBarriersString.replaceAll("[^-?0-9]+", " ");
            		for(String t : (Arrays.asList(n.trim().split(" ")))) negativeBarriers.add(Integer.valueOf(t));
                	ed.negativeBarriers = negativeBarriers;
            	}
            	else ed.negativeBarriers = null;
            	
            	int district =  edge.getIntegerAttribute("district");
            	edgesDistrictMap.put(edgeID, district);
//	            GeomPlanarGraph districtNetwork = districtMap.get(district);
//	            districtNetwork.addFromEdge(edge);
//	            districtMap.put(district, districtNetwork);
//	            GeomPlanarGraph districtDualNetwork = districtDualMap.get(district);
	            Node cen = centroidsMap.get(edgeID).c;
	            DirectedEdgeStar dirEdges = cen.getOutEdges();
	            
	            for (Object dE : dirEdges.getEdges())
	    		{
	            	GeomPlanarGraphDirectedEdge dEdge = (GeomPlanarGraphDirectedEdge) dE;
	            	if (processed.contains(dEdge)) continue;
	    		 	GeomPlanarGraphEdge edgeDual = (GeomPlanarGraphEdge) dEdge.getEdge();
//	    		 	districtDualNetwork.addFromEdge(edgeDual);
	    		 	processed.add(dEdge);
	    		 }
	            
//	            districtDualMap.put(district, districtDualNetwork);
            }
        }  
       
        ///// CRITERIA INFORMATION
        for (int i = 0; i < criteria.length; i++)
        {

        	CriteriaData cd = new CriteriaData();
        	cd.criteria = criteria[i];   
        	cd.trips = 0;
        	cd.totalDistance = 0;
            criteriaMap.put(criteria[i], cd);
        }
            
        /// AGENTS

        populate();
        agents.setMBR(MBR);      
            
        }
   
    public double fromNormalDistribution(double mean, double sd)
    {
    	double error = random.nextGaussian()*sd+mean;
    	return error;
    }
    
    

    public void populate()
    {
    	
//    	int numTripsScenario = distances.size();
    	int numTripsScenario = 2000;
//    	Node originNode = nodesMap.get(10081).node;
//    	Node destinationNode = nodesMap.get(14137).node;
//    	Pair<Node, Node> pair = new Pair<Node, Node> (originNode, destinationNode);
//	    OD.add(pair);
//    	
    	if (landmarksRouting)
    	{
	    	for (int i = 0; i < numTripsScenario; i++)
	    	{
	    		
	    		Node originNode = null;
	    		while (originNode == null) originNode = nodesLookup.searchRandomNode(nodesGeometries, this);
	    		Node destinationNode = nodesLookup.searchNodeWithin(originNode, junctions, distances, this);
	    		Pair<Node, Node> pair = new Pair<Node, Node> (originNode, destinationNode);
	    	    OD.add(pair);  
	    	}
    	}
    	else if (regionalRouting)
    	{
    	for (int i = 0; i < numTripsScenario; i++)
	    	{	
	    		Node originNode = null;
	    		while (originNode == null) originNode = nodesLookup.searchRandomNode(nodesGeometries, this);
	    		double radius = ThreadLocalRandom.current().nextInt(400, 1200);
	    		int startingDistrict = nodesMap.get(originNode.getData()).district;		
	    		Node destinationNode = null;
	    		{
	    			destinationNode = nodesLookup.searchNodeOutsideDistrict(originNode, radius, startingDistrict, this);
	    		}
	    		while (destinationNode == null || originNode == destinationNode)
	    		{
	    			originNode = null;
	    			while (originNode == null) originNode = nodesLookup.searchRandomNode(nodesGeometries, this);
	    			startingDistrict = nodesMap.get(originNode.getData()).district;	
	    			radius = random.nextInt(1000);
	    			destinationNode = nodesLookup.searchNodeOutsideDistrict(originNode, radius, startingDistrict, this);
	    		}
	    		Pair<Node, Node> pair = new Pair<Node, Node> (originNode, destinationNode);
	    	    OD.add(pair);  
	    	}
    	}
    	System.out.println("OD matrix ready");
    	for (int i = 0; i < numAgents; i++)
        {   
        	String routeCriteria = null;
        	routeCriteria = criteria[i];
        	Pedestrian a = new Pedestrian(this, routeCriteria, OD); //, goalEdge )
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
    	int jobs = 5;
    	String cityName = "Torino";
    	String directory = ("C:/Users/g_filo01/sciebo/Scripts/Image of the City/Outputs/"+cityName+"/");
        
    	try 
        {
        	///// READING PRIMAL GRAPH
        	System.out.println("reading primal graph...");
            Bag junctionsAttributes = new Bag();
            junctionsAttributes.add("nodeID"); 
            String data = null;

            if (landmarksRouting)
            {
            	data = "landamrksData";
	            junctionsAttributes.add("loc_land"); 
	            junctionsAttributes.add("loc_scor"); 
	            junctionsAttributes.add("dist_land"); 
	            junctionsAttributes.add("dist_scor"); 
	            junctionsAttributes.add("anchors");   
	            junctionsAttributes.add("dist_anch");      
	            junctionsAttributes.add("distances");  
	            junctionsAttributes.add("Bc_E"); 
	            
	            Bag buildingsAttributes = new Bag();
	            ///// READING BUILDINGS
	            System.out.println("reading buildings layer...");
	            buildingsAttributes.add("buildingID");  
	            buildingsAttributes.add("lScore_sc");  
	            buildingsAttributes.add("gScore_sc");  
	            URL landmarksFile = PedestrianSimulation.class.getResource("data/"+cityName+"_landmarks.shp");
	            ShapeFileImporter.read(landmarksFile, buildings);
	            
	            /// READING DISTANCES
				System.out.println("reading distances...");
				CSVReader readerDistances = new CSVReader(new FileReader
				  		("C:/Users/g_filo01/sciebo/Scripts/GPS Trajectories/Outputs/"+cityName+"/GPStracks/"+
				  					cityName+"_traj_distances.csv"));
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
		            CSVReader reader = new CSVReader(new FileReader(directory+cityName+"_visibility_matrix_simplified.csv"));
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
            
            if (regionalRouting)
    		{
            	data = "districtsData/"+cityName+"/";
		        junctionsAttributes.add("nodeID"); 
		        junctionsAttributes.add("district"); 
		        junctionsAttributes.add("gateway"); 

	            ///// READING BARRIERS
	            Bag barriersAttributes = new Bag();
	            System.out.println("reading barriers layer...");
	            barriersAttributes.add("barrierID");  
	            barriersAttributes.add("type");  
	            URL barriersFile = PedestrianSimulation.class.getResource(data+"/"+cityName+"_barriers.shp");
	            ShapeFileImporter.read(barriersFile, barriers);
    		}
            
            
            
            ///// READING DUAL GRAPH ------------
            System.out.println("reading dual graph...");
            Bag centroidsAttributes = new Bag();
            centroidsAttributes.add("edgeID");
            String simplified = "_simplified";
            URL roadsDualFile = PedestrianSimulation.class.getResource(data+"/"+cityName+"_edgesDual"+simplified+".shp");
            URL centroidsFile = PedestrianSimulation.class.getResource(data+"/"+cityName+"_nodesDual"+simplified+".shp");            
            ShapeFileImporter.read(roadsDualFile, intersectionsDual);
            ShapeFileImporter.read(centroidsFile, centroids, centroidsAttributes);
            
	        URL roadsFile = PedestrianSimulation.class.getResource(data+"/"+cityName+"_edges"+simplified+".shp");
	        URL junctionsFile = PedestrianSimulation.class.getResource(data+"/"+cityName+"_nodes"+simplified+".shp");
            ShapeFileImporter.read(roadsFile, roads);
            ShapeFileImporter.read(junctionsFile, junctions, junctionsAttributes);
            
            network.createFromGeomField(roads);
            dualNetwork.createFromGeomField(intersectionsDual);          			
            nodesGeometries = junctions.getGeometries();
            centroidsGeometries = centroids.getGeometries();
            
            System.out.println("files imported successfully");
	        
	    	for (int i = 0; i < jobs; i++)
	    	{
		    	System.out.println("Run nr.. "+i);
		    	SimState state = new PedestrianSimulation(System.currentTimeMillis(), i, cityName); 
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