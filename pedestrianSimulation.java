package sim.app.geo.pedestrianSimulation;

import com.opencsv.CSVReader;
import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.planargraph.DirectedEdgeStar;

import java.io.*;
import java.net.URL;
import java.util.*;
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


public class PedestrianSimulation extends SimState
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
    public static HashMap<String, CriteriaData> criteriaMap = new HashMap<String, CriteriaData>();
    public static ArrayList<RouteData> routesData = new ArrayList<RouteData>();
    public static Bag startingNodes = new Bag();
    public static HashMap<Integer, DistrictData> districtsMap = new HashMap<Integer, DistrictData>();
    public static HashMap<Pair<NodeGraph, NodeGraph>, GatewayData> gatewaysMap = new HashMap<Pair<NodeGraph, NodeGraph>, GatewayData>();
       
    //dual graph
    public static VectorLayer intersectionsDual = new VectorLayer();
    public static VectorLayer centroids = new VectorLayer();
    public static Graph dualNetwork = new Graph();
    
    HashMap<EdgeGraph, ArrayList<Pedestrian>> edgeTraffic = new HashMap<EdgeGraph, ArrayList<Pedestrian>>();
    public static HashMap<MasonGeometry, Double> buildingsLS = new HashMap<MasonGeometry, Double>();
    public static HashMap<MasonGeometry, Double> buildingsGS = new HashMap<MasonGeometry, Double>();
    public static VectorLayer agents = new VectorLayer();
    
    ArrayList<Pedestrian> agentList = new ArrayList<Pedestrian>();
    List<Integer> districts = new ArrayList<Integer>();
    ArrayList<Pair<NodeGraph, NodeGraph>> OD = new ArrayList<Pair<NodeGraph, NodeGraph>>();

    static List<Float> distances = new ArrayList<Float>();
    
    // landmark routing parameter
    public static double t = 300.0;
//    public int numTripsScenario = 271;

    public int numAgents;
    
	static boolean visibility = false;
	boolean dynamicRouting = false;
	static boolean regionalRouting = true;
	static boolean landmarksRouting = false;
	
	String criteria[];
    String criteriaLandmarks[] = {"roadDistance", "angularChange", "roadDistanceLandmarks", "angularChangeLandmarks", 
    		"localLandmarks", "globalLandmarks"};
//    String criteriaDistricts[] = {"roadDistance", "angularChange", "roadDistanceRegions", "angularChangeRegions", 
//    		"roadDistanceBarriers", "angularChangeBarriers", "roadDistanceRegionsBarriers", "angularChangeRegionsBarriers"};
    String criteriaDistricts[] = {"angularChange", "angularChangeRegions", 
    		"angularChangeBarriers","angularChangeRegionsBarriers"};
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
		Bag edgesGeometries = roads.getGeometries();
	   	
		String csvSegments = null;
	   	String csvSegmentsLandmarks = "C:/Users/g_filo01/sciebo/GIS Data/Simulation/landmarksRouting/"+cityName+"_PedSim_landmarks_"+current_job+".csv";
	   	String csvSegmentsDistricts = "C:/Users/g_filo01/sciebo/GIS Data/Simulation/regionalRouting/"+cityName+"_PedSim_districts_"+current_job+".csv";
		
		if (regionalRouting) 
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
				
		else if (landmarksRouting) 
		{
			csvSegments = csvSegmentsLandmarks;

			FileWriter writerDensitiesData = new FileWriter(csvSegments);
			CSVUtils.writeLine(writerDensitiesData, Arrays.asList("edgeID", "road_distance", "angular_change", 
	   			"topological", "road_distance", "angular_change_landmarks",
	   			"local_landmarks", "global_landmarks"));
	   	
		   	int rGeoSize = edgesGeometries.size();
		   	for (int i = 0; i < rGeoSize; i++) 
		   	{
		    	MasonGeometry segment = (MasonGeometry) edgesGeometries.objs[i]; 
		        EdgeGraph ed = edgesMap.get(segment.getIntegerAttribute("edgeID"));
		        CSVUtils.writeLine(writerDensitiesData, Arrays.asList(Integer.toString(ed.getID()), 
		        		Integer.toString(ed.roadDistance),	Integer.toString(ed.angularChange), 
		        		Integer.toString(ed.topological), 
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
    	
        ///// NODES - assigning scores to nodes and probabilities ///////////////////////////////////////////
        for (Object nG : nodesGeometries) //street junctions and betweenness centrality
        {
        	MasonGeometry nodeGeometry = (MasonGeometry) nG; 
        	Integer nodeID = nodeGeometry.getIntegerAttribute("nodeID");
        	NodeGraph node = network.findNode(nodeGeometry.geometry.getCoordinate());
        	node.setID(nodeID);
        	node.masonGeometry = nodeGeometry;
        	node.primalEdge = null;
        	node.centrality = nodeGeometry.getDoubleAttribute("Bc_multi");

            ///// ASSIGNING TO EACH NODE LANDMARKS AT THE JUNCTION
        	if (landmarksRouting)
        	{
    		    node.setLandmarkness(nodeGeometry);
	        		        	
	        	///// ASSIGNING TO EACH NODE LANDMARKS VISIBLE FROM THE JUNCTION
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

        	nodesMap.put(nodeID, node);
        }
    	network.generateCentralityMap();
        ///// GATEWAYS
	    if (regionalRouting)
	    {
	    	for (NodeGraph node : nodesMap.values())
        	{
            	node.district = node.masonGeometry.getIntegerAttribute("district");
            	if (districtsMap.get(node.district) == null)
            	{
            		DistrictData district = new DistrictData();
            		districtsMap.put(node.district, district);  
            	}
            }
	      
	    	for (NodeGraph node : nodesMap.values())
	        {  
	    		
		    	Integer gateway = node.masonGeometry.getIntegerAttribute("gateway"); // 1 or 0
            	Integer district = node.district;
		    	
            	if (gateway == 0) //nodes which are not gateways
		    	{
		    		startingNodes.add(node.masonGeometry);
		    		continue;
		    	}
            	
		        for (EdgeGraph bridge : node.getEdgesNode())
		        {
		           NodeGraph oppositeNode = (NodeGraph) bridge.getOppositeNode(node);
		           int possibleRegion = oppositeNode.district;
		           if (possibleRegion == district) continue;
		
		           GatewayData gd = new GatewayData();
			       gd.node = node;
			       gd.edgeID = bridge.getID();
			       gd.gatewayID = new Pair<NodeGraph, NodeGraph>(node, oppositeNode);
			       gd.regionTo = possibleRegion;
			       gd.entry = oppositeNode;
			       gd.distance = bridge.getLength();
			       gd.entryAngle = Angle.angle(node, oppositeNode);
			       
			       DistrictData dd = districtsMap.get(district);
			       dd.gateways.add(gd);
			       gatewaysMap.put(new Pair<NodeGraph, NodeGraph>(node, oppositeNode), gd);
			       node.gateway = true;
		        }
		        node.adjacentRegions = node.getAdjacentRegion();

	        }
	    }
	    
	    
        ///// CENTROIDS: assigning edgeID to centroids in the dual graph


        
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
                   
        ///// ASSIGNING DATA TO STREET SEGMENTS ///////////////////////////////////////////
        for (Object o : network.getEdges())
        {
        	EdgeGraph edge = (EdgeGraph) o;
        	int edgeID = edge.getIntegerAttribute("edgeID");
        	edge.setID(edgeID);
        	
            if (regionalRouting)
            {
            	edge.setBarriers();	
        		if (edge.u.district == edge.v.district)
        		{
        	    	int district = edge.u.district;
        	    	edge.district = district;
        	    	DistrictData dd = districtsMap.get(district);
        	    	dd.edgesMap.add(edge);
        		}
        		else edge.district = 999999;
            }
            edgesMap.put(edgeID, edge);
        }  
        
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
        
        ///// CRITERIA INFORMATION
        for (int i = 0; i < criteria.length; i++)
        {
        	CriteriaData cd = new CriteriaData();
        	cd.criteria = criteria[i];   
        	cd.trips = 0;
        	cd.totalDistance = 0;
            criteriaMap.put(criteria[i], cd);
        }
                
        ///// CREATING SUBGRAPHS 
        for ( int d : districtsMap.keySet())
        {
        	ArrayList<EdgeGraph> edgesDistrict = districtsMap.get(d).edgesMap;
        	ArrayList<EdgeGraph> dualEdgesDistrict = new ArrayList<EdgeGraph>();
        	SubGraph primalGraph = new SubGraph(network, edgesDistrict);
        	
        	for (EdgeGraph edge: edgesDistrict) 
        	{
	    		NodeGraph cen = edge.dualNode;
	    		cen.district = d;
	    		DirectedEdgeStar dirEdges = cen.getOutEdges();
	            for (Object dE : dirEdges.getEdges())
	    		{
	            	GeomPlanarGraphDirectedEdge dEdge = (GeomPlanarGraphDirectedEdge) dE;
	            	dualEdgesDistrict.add((EdgeGraph) dEdge.getEdge());
	    		 }
        	}
        	SubGraph dualGraph = new SubGraph(dualNetwork, dualEdgesDistrict);
        	districtsMap.get(d).primalGraph = primalGraph;
        	districtsMap.get(d).dualGraph = dualGraph;

        }
            
        /// AGENTS
        populate();
        agents.setMBR(MBR);      
        }
   


    
    

    public void populate()
    {
    	
//    	int numTripsScenario = distances.size();
    	int numTripsScenario = 500;
//    	NodeGraph originNode = nodesMap.get( 19977  );
//    	NodeGraph destinationNode = nodesMap.get(42331);
//    	Pair<NodeGraph, NodeGraph> pair = new Pair<NodeGraph, NodeGraph> (originNode, destinationNode);
//	    OD.add(pair);
    	
    	if (landmarksRouting)
    	{
	    	for (int i = 0; i < numTripsScenario; i++)
	    	{
	    		
	    		NodeGraph originNode = null;
	    		while (originNode == null) originNode = nodesLookup.randomNode(nodesGeometries, network);
	    		NodeGraph destinationNode = nodesLookup.nodeWithin(originNode, junctions, distances, network);
	    		Pair<NodeGraph, NodeGraph> pair = new Pair<NodeGraph, NodeGraph> (originNode, destinationNode);
	    	    OD.add(pair);  
	    	}
    	}
    	else if (regionalRouting)
    	{
    		for (int i = 0; i < numTripsScenario; i++)
	    	{	
    			NodeGraph originNode = null;
	    		NodeGraph destinationNode = null;
	    		while (destinationNode == null || originNode == destinationNode)
	    		{
	    			originNode = null;
	    			while (originNode == null) originNode = nodesLookup.randomNode(startingNodes, network);
	    			double radius = ThreadLocalRandom.current().nextInt(1000, 4000);
	    			destinationNode = nodesLookup.outsideDistrictByCentrality(originNode, radius);
	    		}
	    		Pair<NodeGraph, NodeGraph> pair = new Pair<NodeGraph, NodeGraph> (originNode, destinationNode);
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
    	int jobs = 1;
    	String cityName = "Paris";
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
	            junctionsAttributes.add("Bc_multi"); 
	            
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
	            junctionsAttributes.add("Bc_multi"); 
	            

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
            
            network.fromGeomField(roads);
            dualNetwork.fromGeomField(intersectionsDual);          			
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