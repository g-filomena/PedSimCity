/**
 ** Gridlock.java
 **
 ** Copyright 2011 by Sarah Wise, Mark Coletti, Andrew Crooks, and
 ** George Mason University.
 **
 ** Licensed under the Academic Free License version 3.0
 **
 ** See the file "LICENSE" for more information
 **
 * $Id: Gridlock.java 849 2013-01-08 22:56:52Z mcoletti $
 * 
 **/
package sim.app.geo.pedestrianSimulation;

import com.opencsv.CSVReader;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.GeometryFactory;
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


public class pedestrianSimulation extends SimState
{
    private static final long serialVersionUID = 1L;
    
    //primal graph
    public static GeomVectorField roads = new GeomVectorField();
    public static GeomVectorField buildings = new GeomVectorField();
    public static GeomVectorField junctions = new GeomVectorField();
    public static GeomPlanarGraph network = new GeomPlanarGraph();
    public static Bag geometriesCentroids;
    public static Bag geometriesNodes;
    public static String [][] visibilityMatrix;
    
    public HashMap<Integer, edgeData> edgesMap = new HashMap<Integer, edgeData>();
    public HashMap<Integer, nodeData> nodesMap = new HashMap<Integer, nodeData>();
    public HashMap<Integer, Node> startingNodesMap = new HashMap<Integer, Node>();
    public HashMap<MasonGeometry, Integer> nodesGeometryMap = new HashMap<MasonGeometry, Integer>();
    public HashMap<Integer, buildingData> buildingsMap = new HashMap<Integer, buildingData>();
    public HashMap<Integer, centroidData> centroidsMap =  new HashMap<Integer, centroidData>();
    public HashMap<String, criteriaData> criteriaMap = new HashMap<String, criteriaData>();
    
    public HashMap<Integer, GeomPlanarGraph> districtMap = new HashMap<Integer, GeomPlanarGraph>();
    public HashMap<Integer, GeomPlanarGraph> districtDualMap = new HashMap<Integer, GeomPlanarGraph>();
    public HashMap<Integer, Integer> edgeDistrictMap = new HashMap<Integer, Integer>();
    public HashMap<Integer,ArrayList<gatewayData>> exitDistrictsMap = new HashMap<Integer,ArrayList<gatewayData>>();
    public HashMap<Integer,gatewayData> gatewaysMap = new HashMap<Integer,gatewayData>();
    
    public HashMap<Pair, Double> euclideanDistancesMap = new HashMap<Pair, Double>();
    public HashMap<Pair, Double> angularDistancesMap = new HashMap<Pair, Double>();
       
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
    ArrayList<Pair> OD = new ArrayList<Pair>();

    static List<Float> distances = new ArrayList<Float>();
    
    // landmark routing parameter
    public double t = 300.0;
    boolean singleAgent = false;
//    public int numTripsScenario = 271;

    public int numAgents;
    
	boolean visibility = true;
	boolean dynamicRouting = false;
	static boolean districtRouting = false;
	
	String criteria[];
    String criteriaLandmarks[] = {"euclidean", "angular", "topological",  "euclideanLand", "angularLand"};
    String criteriaDistricts[] = {"euclidean", "angular", "topological",  "district"};

    
	double height, width, ratio;
	int current_job;
	
    /** Constructor */
	
    public pedestrianSimulation(long seed, int job)
    {
        super(seed);
    	this.current_job = job;
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
        System.out.println("saving");
		Bag rGeometries = roads.getGeometries();
	   	
		String csvSegments;
		String csvSegmentsLandmarksSingle = "C:/Users/g_filo01/sciebo/GIS Data/Simulation/landmarks/PedSim_landmarks_SR.csv";
	   	String csvSegmentsLandmarks = "C:/Users/g_filo01/sciebo/GIS Data/Simulation/landmarks/PedSim_landmarks_"+current_job+".csv";
	   	
	   	String csvSegmentsDistricts = "C:/Users/g_filo01/sciebo/GIS Data/Simulation/districtRouting/PedSim_districts_SR.csv";
	   	String csvSegmentsDistrictsSingle = "C:/Users/g_filo01/sciebo/GIS Data/Simulation/districtRouting/PedSim_districts_"+current_job+".csv";
		
		if (districtRouting == true)
		{
			if (singleAgent == true) csvSegments = csvSegmentsDistrictsSingle;
			else csvSegments = csvSegmentsDistricts;
		}
		else
		{
			if (singleAgent == true) csvSegments = csvSegmentsLandmarksSingle;
			else csvSegments = csvSegmentsLandmarks;

		}
	   	FileWriter writerSegmentsData = new FileWriter(csvSegments);
	   	CSVUtils.writeLine(writerSegmentsData, Arrays.asList("streetID", "euclidean", "angular", "topological", "euclideanLandmark", "angularLandmark",
	   			"euclideanDistrict", "angularDistrict"));
	   	
	   	for (int i = 0; i < rGeometries.size(); i++) 
	   	{
	    	MasonGeometry segment = (MasonGeometry) rGeometries.objs[i]; 
	        edgeData ed = edgesMap.get(segment.getIntegerAttribute("streetID"));
	        CSVUtils.writeLine(writerSegmentsData, Arrays.asList(Integer.toString(segment.getIntegerAttribute("streetID")), Integer.toString(ed.euclidean),
	        		Integer.toString(ed.angular), Integer.toString(ed.topological), Integer.toString(ed.euclideanLand), Integer.toString(ed.angularLand), 
	        	    Integer.toString(ed.districtRoutingEuclidean),Integer.toString(ed.districtRoutingAngular)));
	    }

	    writerSegmentsData.flush();
	    writerSegmentsData.close();
	    
    }


    public void start()
    {
    	
    	
        if (districtRouting == true) criteria = criteriaDistricts;
        else criteria = criteriaLandmarks;
        numAgents = criteria.length;
    	super.start();
    	Envelope MBR = null;
        MBR = roads.getMBR();
        MBR.expandToInclude(buildings.getMBR());          
        roads.setMBR(MBR);
            
        ///// NODES - assigning scores to nodes and probabilities
        for (int i = 0; i < geometriesNodes.size(); i++) //street junctions and betweenness centrality
        {
        	MasonGeometry geometryNode = (MasonGeometry) geometriesNodes.objs[i]; 
        	Integer nodeID = geometryNode.getIntegerAttribute("nodeID");

        	String localString = geometryNode.getStringAttribute("loc_land");
        	String lScoresString = geometryNode.getStringAttribute("loc_scor");
        	String distantString = geometryNode.getStringAttribute("dist_land");
        	String dScoresString = geometryNode.getStringAttribute("dist_scor");
        	String anchorsString = geometryNode.getStringAttribute("anchors");
        	String distancesString = geometryNode.getStringAttribute("distances");
        	
            double Bc = geometryNode.getDoubleAttribute("Bc_E_sc");
        	double Bc600 = geometryNode.getDoubleAttribute("Bc_E_600"); 
        	
        	List<Integer> localLandmarks = new ArrayList<Integer>();
        	List<Double> localScores = new ArrayList<Double>();
        	List<Integer> distantLandmarks = new ArrayList<Integer>();
        	List<Double> distantScores = new ArrayList<Double>();
        	List<Integer> anchors = new ArrayList<Integer>();
        	List<Double> landmarkDistances = new ArrayList<Double>();
        	
            nodesBc.put(geometryNode, Bc);
            nodesBc600.put(geometryNode, Bc600);           	
        	Node node = network.findNode(geometryNode.geometry.getCoordinate());
        	if (node == null) System.out.println("null node "+ nodeID);
        	if (nodeID == 8434) System.out.println("attenzione!!");
        	node.setData(nodeID);
        	nodeData nd = new nodeData();
        	nd.node = node;

        	///// ASSIGNING TO EACH NODE LANDMARKS AT THE JUNCTION
        	if (localString.equals("None") == false)
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
        	
        	
        	if (distantString.equals("None") == false)
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
            	
            	if (anchorsString.equals("None") == false)
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
        	
            ///// DISTRICT INFORMATION
        	if (districtRouting == true)
        	{
            	Integer district = geometryNode.getIntegerAttribute("district");
            	nd.district = district;
            	if (districts.contains(district) == false)
            	{
            		districts.add(district);
            		GeomPlanarGraph districtNetwork = new GeomPlanarGraph();
            		GeomPlanarGraph districtDualNetwork = new GeomPlanarGraph();

                	districtMap.put(district, districtNetwork);
                	districtDualMap.put(district, districtDualNetwork);            	
                	ArrayList<gatewayData> gateways = new ArrayList<gatewayData>();
                	exitDistrictsMap.put(district, gateways);
            	}
        	}
        	
        ///// ASSIGNING TO EACH NODE LANDMARKS VISIBLE FROM THE JUNCTION
        	if (visibility == true)
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
        	nodesGeometryMap.put(geometryNode, nodeID);
        	nodesMap.put(nodeID, nd);
    	}
        
        ///// GATEWAYS
        if (districtRouting == true)
        {
            int counter = 0;
            for (Object o : junctions.getGeometries())
            {
            	MasonGeometry geometryNode = (MasonGeometry) o; 
            	Integer nodeID = geometryNode.getIntegerAttribute("nodeID");
            	String regionsToSt = geometryNode.getStringAttribute("regionsTo");
            	Integer CM = geometryNode.getIntegerAttribute("CM");
            	Node n = network.findNode(geometryNode.geometry.getCoordinate());
            	Integer district = geometryNode.getIntegerAttribute("district");
            	
            	if (regionsToSt.equals("None"))
            	{
            		startingNodesMap.put(nodeID, n);
            		continue;
            	}
            	DirectedEdgeStar dirEdges = n.getOutEdges();
            	
            	for (Object dE : dirEdges.getEdges())
                {
                   GeomPlanarGraphDirectedEdge l = (GeomPlanarGraphDirectedEdge) dE;
                   GeomPlanarGraphEdge d = (GeomPlanarGraphEdge) l.getEdge();
                   Node oN = d.getOppositeNode(n);
                   
                   Integer oNodeID = (Integer) oN.getData();
                   int pr = nodesMap.get(oNodeID).district;
                   if (pr == district) continue;

                   gatewayData gd = new gatewayData();
        	       gd.nodeID = nodeID;
        	       gd.n = n;
        	       gd.edgeID = d.getIntegerAttribute("streetID");
        	       gd.gatewayID = counter;
        	       gd.district = district;
        	       gd.regionTo = pr;
        	       gd.entryID = oNodeID;
        	       gd.distance = d.getLine().getLength();
        	       
        	       gd.entryAngle = utilities.angle(geometryNode.geometry.getCoordinate(), oN.getCoordinate());
        	       if (CM == 1) gd.cognitiveMap = true;
        	       ArrayList<gatewayData> gateways = exitDistrictsMap.get(district);
        	       gateways.add(gd);
        	       exitDistrictsMap.put(district, gateways);
        	       nodesMap.get(nodeID).gateway = true;
        	       gatewaysMap.put(counter, gd);
        	       counter += 1;
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
        
        //
        for (Object o : buildings.getGeometries())
        {
        	MasonGeometry geometryBuilding = (MasonGeometry) o; 
        	Integer buildingID = geometryBuilding.getIntegerAttribute("buildingID");

        	buildingData bd = new buildingData();
        	bd.localScore = geometryBuilding.getDoubleAttribute("lScore_sc");
        	bd.globalScore = geometryBuilding.getDoubleAttribute("gScore_sc");
        	buildingsMap.put(buildingID, bd);
        	buildingsLS.put(geometryBuilding, bd.localScore);
        	buildingsGS.put(geometryBuilding, bd.globalScore);
        }
                   
        ///// CENTROIDS: assigning streetID to centroids in the dual graph

        for (Object o : centroids.getGeometries()) 
        {
        	MasonGeometry geometryNode = (MasonGeometry) o; 
        	int streetID = geometryNode.getIntegerAttribute("streetID"); 
      	    Node cen = dualNetwork.findNode(geometryNode.geometry.getCoordinate());
        	cen.setData(streetID);
        	
        	centroidData cd = new centroidData();
        	cd.c = cen;
        	centroidsMap.put(streetID, cd);
        }
        

        ///// ASSIGNING DATA TO STREET SEGMENTS
        ArrayList <GeomPlanarGraphDirectedEdge> processed = new ArrayList <GeomPlanarGraphDirectedEdge>();
        for (Object o : network.getEdges())
        {
        	GeomPlanarGraphEdge edge = (GeomPlanarGraphEdge) o;
        	edgeData ed = new edgeData();
        	int streetID =  edge.getIntegerAttribute("streetID");
        	
        	ed.planarEdge = edge;
        	ed.distanceScaled = edge.getDoubleAttribute("length_sc");  
        	
        	ed.bC = edge.getDoubleAttribute("Ab_sc");  
        	ed.fromNode = edge.getIntegerAttribute("u");  
        	ed.toNode = edge.getIntegerAttribute("v");  
            edgesMap.put(streetID, ed);

            
            if (districtRouting == true)
            {
            	int district =  edge.getIntegerAttribute("district");
                edgeDistrictMap.put(streetID, district);
	            GeomPlanarGraph districtNetwork = districtMap.get(district);
	////        Object data = e.getData();
	//          Map<String, AttributeValue> data = new HashMap<String, AttributeValue>();
	//          data.put("length", (AttributeValue) e.getAttribute("length"));
	//          dataDual.put("length", (AttributeValue) edge.getAttribute("rad"))
	//          System.out.print("here       "+e.getDirEdge(0));
	            districtNetwork.addFromEdge(edge);
	            districtMap.put(district, districtNetwork);
	                
	            GeomPlanarGraph districtDualNetwork = districtDualMap.get(district);
	            Node cen = centroidsMap.get(streetID).c;
	            DirectedEdgeStar dirEdges = cen.getOutEdges();
	            
	            for (Object dE : dirEdges.getEdges())
	    		{
	            	GeomPlanarGraphDirectedEdge dEdge = (GeomPlanarGraphDirectedEdge) dE;
	            	if (processed.contains(dEdge)) continue;
	    		 	GeomPlanarGraphEdge edgeDual = (GeomPlanarGraphEdge) dEdge.getEdge();
	    		 	districtDualNetwork.addFromEdge(edgeDual);
	    		 	processed.add(dEdge);
	    		 }
	            
	            districtDualMap.put(district, districtDualNetwork);
            }
        }  
       
        ///// CRITERIA INFORMATION
        for (int i = 0; i < criteria.length; i++)
        {

        	criteriaData cd = new criteriaData();

        	cd.criteria = criteria[i];   
        	cd.trips = 0;
        	cd.totalDistance = 0;
            criteriaMap.put(criteria[i], cd);
        }
            
        ///// AGENTS
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
    	int numTripsScenario = distances.size();
        if (singleAgent == true) numTripsScenario = 2;
    	ArrayList<Node> control = new ArrayList<Node>();
		
//    	Node originNode = nodesMap.get(719).node;
//    	Node destinationNode = nodesMap.get(3983).node;
//    	Pair<Node, Node> pair = new Pair<Node, Node> (originNode, destinationNode);
//	    OD.add(pair);
    	
    	for (int i = 0; i < numTripsScenario; i++)
    	{
    		Node originNode = null;
    		while ((originNode == null) | (control.contains(originNode))) originNode = nodesLookup.searchRandomNode(geometriesNodes, this);
    		GeometryFactory fact = new GeometryFactory();
    		control.add(originNode);
    		MasonGeometry nodeLocation = new MasonGeometry(fact.createPoint(new Coordinate(originNode.getCoordinate())));
    		Node destinationNode = nodesLookup.searchNodeWithin(nodeLocation.geometry, junctions, 400, 4000, this);
    		Pair<Node, Node> pair = new Pair<Node, Node> (originNode, destinationNode);
    	    OD.add(pair);  
    	}
    	
    	System.out.println("populating"  + OD.size());
    	for (int i = 1; i <= numAgents; i++)
        {   
        	String routeCriteria;
        	if (i == 1) routeCriteria = "euclidean";
        	else if (i == 2) routeCriteria = "angular";
        	else if (i == 3) routeCriteria = "topological";
        	else if (i == 4) routeCriteria = "euclideanLand";
        	else routeCriteria = "angularLand";

        	       	
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
    	int jobs = 50;
    	String directory = "C:/Users/g_filo01/sciebo/Scripts/Image of the City/Outputs/London/intermediate/";
        try 
        {
        	///// READING PRIMAL GRAPH
        	System.out.println("reading primal graph...");

            Bag junctionsAttributes = new Bag();
            Bag buildingsAttributes = new Bag();
   
            junctionsAttributes.add("nodeID"); 
            junctionsAttributes.add("loc_land"); 
            junctionsAttributes.add("loc_scor"); 
            junctionsAttributes.add("dist_land"); 
            junctionsAttributes.add("dist_scor"); 
            junctionsAttributes.add("anchors");   
            junctionsAttributes.add("dist_anch");      
            junctionsAttributes.add("distances");  
            junctionsAttributes.add("Bc_E_sc"); 
            junctionsAttributes.add("Bc_E_600"); 
            
            if (districtRouting == true)
    		{
   		        junctionsAttributes.add("Bc_multi");
		        junctionsAttributes.add("nodeID"); 
		        junctionsAttributes.add("district"); 
		        junctionsAttributes.add("regionsTo"); 
		        junctionsAttributes.add("CM"); 
    		}
            
	        URL roadsFile = pedestrianSimulation.class.getResource("data/London_edges_simplified.shp");
	        URL junctionsFile = pedestrianSimulation.class.getResource("data/London_nodes_simplified.shp");
            ShapeFileImporter.read(roadsFile, roads);
            ShapeFileImporter.read(junctionsFile, junctions, junctionsAttributes);
            
            ///// READING DUAL GRAPH ------------
            System.out.println("reading dual graph...");
            Bag centroidsAttributes = new Bag();
            centroidsAttributes.add("streetID");
            URL roadsDualFile = pedestrianSimulation.class.getResource("data/London_edgesDual_simplified.shp");
            URL centroidsFile = pedestrianSimulation.class.getResource("data/London_nodesDual_simplified.shp");            
            ShapeFileImporter.read(roadsDualFile, intersectionsDual);
            ShapeFileImporter.read(centroidsFile, centroids, centroidsAttributes);

            
            ///// READING BUILDINGS
            System.out.println("reading buildings layer...");
            buildingsAttributes.add("buildingID");  
            buildingsAttributes.add("lScore_sc");  
            buildingsAttributes.add("gScore_sc");  
            URL landmarksFile = pedestrianSimulation.class.getResource("data/London_landmarks.shp");
            ShapeFileImporter.read(landmarksFile, buildings);

            /// READING DISTANCES
			System.out.println("reading distances...");
			CSVReader readerDistances = new CSVReader(new FileReader
			  		("C:/Users/g_filo01/sciebo/Tools/Mason/sim/app/geo/pedestrianSimulation/data/London_traj_distances.csv"));
			String[] nextLineDistances;
			  
			int ds = 0;
			while ((nextLineDistances = readerDistances.readNext()) != null) 
			{
			  ds += 1;
			  if (ds == 1) continue;
			  distances.add(Float.parseFloat(nextLineDistances[2]));
			}
			readerDistances.close();
            
            network.createFromGeomField(roads);
            dualNetwork.createFromGeomField(intersectionsDual);          			
            geometriesNodes = junctions.getGeometries();
            
            System.out.println("reading visibility...");
            visibilityMatrix = new String[buildings.getGeometries().size()+1][geometriesNodes.size()+1];
            CSVReader reader = new CSVReader(new FileReader(directory+"London_visibility_matrix_simplified.csv"));
            String [] nextLine;
            
            int v = 0;
            while ((nextLine = reader.readNext()) != null) 
            {
            	visibilityMatrix[v] = nextLine;
            	v = v+1;
            }
            reader.close();
            System.out.println("files imported successfully");
	        
	    	for (int i = 0; i < jobs; i++)
	    	{
		    	System.out.println("Run nr.. "+i);
		    	SimState state = new pedestrianSimulation(System.currentTimeMillis(), i); 
		    	state.start(); 
		    	while (state.schedule.step(state)) {}
	    	}
	
	        reader.close();
		} 
        catch (IOException e)
        {
			e.printStackTrace();
		}
    	System.exit(0);
    }

}