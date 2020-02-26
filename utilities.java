package sim.app.geo.pedestrianSimulation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import java.util.stream.Collectors;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.geom.prep.PreparedGeometryFactory;
import com.vividsolutions.jts.planargraph.DirectedEdgeStar;
import com.vividsolutions.jts.planargraph.Node;
import com.vividsolutions.jts.planargraph.Subgraph;

import sim.field.geo.GeomVectorField;
import sim.util.Bag;
import sim.util.geo.GeomPlanarGraph;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.GeomPlanarGraphEdge;
import sim.util.geo.MasonGeometry;


public class utilities {	
	
	public static double angle (Coordinate origin, Coordinate destination) 
	{
		
	    double [] vectorA = {(origin.x-origin.x), (origin.y-(origin.y + 2000))};
	    double [] vectorB = {(origin.x-destination.x), (origin.y-destination.y)};
	    double dot_prod = dot(vectorA, vectorB);
	    double magA = Math.pow(dot(vectorA, vectorA), 0.5);
	    double magB = Math.pow(dot(vectorB, vectorB), 0.5);
	   
	    double angle_rad = Math.acos(dot_prod/magB/magA);
	    double angle_deg = Math.toDegrees(angle_rad)%360;
	    if (destination.x < origin.x) angle_deg = 180+(180-angle_deg);
		return angle_deg;
	
	}
	
	public static double differenceAngles(Double angleA, Double angleB) 
	{
		double difference = 0;
		//check if same quadrant
		if ((angleA <=180 & angleB <=180) || (angleA > 180 & angleB > 180)) difference = Math.abs(angleA-angleB);
		
		else if (angleA > 180 & angleB <= 180)
		{
			double tmpA = Math.abs(angleB-angleA);
		    double tmpB = Math.abs(angleA-(angleB+360));
			difference = Math.min(tmpA, tmpB);
		}
//		 (angleB > 180 & angleA <= 180)
		else
		{
			double tmpA = Math.abs(angleB-angleA);
		    double tmpB = Math.abs(angleB-(angleA+360));
			difference = Math.min(tmpA, tmpB);
		}
		return difference;
	
	}

	public static double dot (double [] vectorA, double [] vectorB) 
	{
	  return vectorA[0]*vectorB[0]+vectorA[1]*vectorB[1];
	}

	// sort map
	public static <K, V extends Comparable<? super V>> Map<K, V> sortByValue(Map<K, V> map) 
	{
	    return map.entrySet()
	              .stream()
	              .sorted(Map.Entry.comparingByValue(/*Collections.reverseOrder()*/))
	              .collect(Collectors.toMap(
	                Map.Entry::getKey, 
	                Map.Entry::getValue, 
	                (e1, e2) -> e1, 
	                LinkedHashMap::new
	              ));
	}

	public static double euclideanDistance(Coordinate originCoord, Coordinate destinationCoord)
	{
	    return Math.sqrt(Math.pow(originCoord.x - destinationCoord.x, 2)
	            + Math.pow(originCoord.y - destinationCoord.y, 2));
	}
    		
	public static double nodesDistance(Node origin, Node destination)
    {
        Coordinate originCoord = origin.getCoordinate();
        Coordinate destinationCoord = destination.getCoordinate();
        return 	euclideanDistance(originCoord, destinationCoord);

    }
    
    public static double finalLengthRoute(ArrayList<GeomPlanarGraphDirectedEdge> route)
    {
    	double distance = 0;
    
    	for (int i = 0; i < route.size(); i++)
    	{
    		GeomPlanarGraphDirectedEdge edge = route.get(i);
    		GeomPlanarGraphEdge d = (GeomPlanarGraphEdge) edge.getEdge();
    		distance += d.getDoubleAttribute("length");
    	}
		return distance;
    }
    
    public static boolean isInDirection(double angleOD, double angleON, double cone)
    {	
    	    	
        double limitL = angleOD-(cone/2+1);
        double limitR = angleOD+(cone/2+1);
        if (limitL < 0) limitL = 360.0 + limitL;
        if (limitR > 360) limitR = limitR - 360.0;
	    
        if (((limitR > 180) & (angleON < limitL) & (angleON > 0) & (angleON < 180)) ||
        	((limitL > 180) & (limitR <= 180) & (angleON > limitR) & (angleON > limitL)) ||
        	((limitL > 180) & (limitR <= 180) & (angleON < limitR) & (angleON < limitL) & (angleON < 180)) ||
        	((limitL > 180) & (limitR > 180) & (angleON > 180) & (angleON > limitL) & (angleON < limitR)) ||
        	((limitL <= 180) & (limitR <= 180) & (angleON <= 180) & (angleON > limitL) & (angleON < limitR)) ||
        	((limitL <= 180) & (limitR > 180) & (angleON > limitL) & (angleON < limitR))) return true;
        else return false;
    }
    
    public static Node getDualNode(Node primalNode, GeomPlanarGraph dualNetwork)
    {	
    	Node dualNode = null;
		DirectedEdgeStar startingEdges =  primalNode.getOutEdges();
		
	 	for (Object o : startingEdges.getEdges())
	 	{
	 		GeomPlanarGraphDirectedEdge dEdge = (GeomPlanarGraphDirectedEdge) o;
	 		GeomPlanarGraphEdge edge = (GeomPlanarGraphEdge) dEdge.getEdge();
	 		dualNode = dualNetwork.findNode(edge.getLine().getCentroid().getCoordinate());
	 		if (dualNode == null) continue;
	 		else break;
	 	}
	 	return dualNode;
    }	
    
    public static ArrayList<Node> getAdjacentNodes(Node node)
    {	

		DirectedEdgeStar startingEdges =  node.getOutEdges();
		ArrayList<Node> adjacentNodes = new ArrayList<Node>();
	 	for (Object o : startingEdges.getEdges())
	 	{
	 		GeomPlanarGraphDirectedEdge dEdge = (GeomPlanarGraphDirectedEdge) o;
	 		GeomPlanarGraphEdge edge = (GeomPlanarGraphEdge) dEdge.getEdge();
	 		Node otherNode = edge.getOppositeNode(node);
	 		adjacentNodes.add(otherNode);
	 	}
	 	return adjacentNodes;
    }	
       
	public static LineString LineStringBetweenNodes(Node nodeA, Node nodeB)
	{
		Coordinate[] coords = {nodeA.getCoordinate(), nodeB.getCoordinate()};
		LineString line = new GeometryFactory().createLineString(coords);
		return line;
	}
    
    public static MasonGeometry MasonGeometryFromGeometry(Geometry g)
    {
    	MasonGeometry mg = (MasonGeometry) PreparedGeometryFactory.prepare(g);
    	return mg;	
	}
    
	public static Geometry smallestEnclosingCircle(Node nodeA, Node nodeB)
	{			
		LineString line = LineStringBetweenNodes(nodeA, nodeB);
		Point centroid = line.getCentroid();
		Geometry buffer = centroid.buffer(line.getLength()/2);
		return buffer;
	}
    
	public static HashMap<MasonGeometry, Double> filterMap(HashMap<MasonGeometry, Double> map, Bag filter)
	{	
	
	  	HashMap<MasonGeometry, Double> mapFiltered = new HashMap<MasonGeometry, Double> (map);	  	
	  	ArrayList<MasonGeometry> result = new ArrayList<MasonGeometry>();
	  	for(MasonGeometry key : mapFiltered.keySet()) {if(filter.contains(key)) result.add(key);}
	  	mapFiltered.keySet().retainAll(result);
		return mapFiltered;
	}
	
	public static int commonPrimalJunction(Node cen, Node otherCen, PedestrianSimulation state)
	{

	    int edgeID = (int) cen.getData();
	    int otherEdgeID = (int) otherCen.getData();
	    int u = state.edgesMap.get(edgeID).fromNode;
	    int v = state.edgesMap.get(edgeID).toNode;
	    int uC = state.edgesMap.get(otherEdgeID).fromNode;
	    int vC = state.edgesMap.get(otherEdgeID).toNode;
	    
	    if ((u == uC) | (u == vC)) return u;
	    else return v;
	}
	
	public static class Path {
		ArrayList<GeomPlanarGraphDirectedEdge> edges;
		HashMap<Node, DualNodeWrapper> dualMapWrappers;
		HashMap<Node, NodeWrapper> mapWrappers;
	}
	
	
    public static GeomVectorField filterGeomVectorField(GeomVectorField gvf, String attributeName, Integer attributeValue, 
    		String method)
    {
    	Bag objects = new Bag();
    	Bag geometries = gvf.getGeometries();

	    for (int i = 0; i < geometries.size(); i++)
	    {
	    	MasonGeometry mg = (MasonGeometry) geometries.get(i);
	        Integer attribute = mg.getIntegerAttribute(attributeName);
	        if ((method == "different") && (!attribute.equals(attributeValue))) objects.add(mg);
	        else if ((method == "equal") && (attribute == attributeValue)) objects.add(mg);
	        else continue;
	     }
	    
    	GeomVectorField newGeomVectorField = new GeomVectorField();	 
 	    for (Object o : objects)
 	    {
 	    	MasonGeometry mg = (MasonGeometry) o;
 	    	newGeomVectorField.addGeometry(mg);
 	    }
		return newGeomVectorField;
    }
	
    public static GeomVectorField filterGeomVectorField(GeomVectorField gvf, String attributeName, String attributeValue, 
    		String method)
    {
    	Bag objects = new Bag();
    	Bag geometries = gvf.getGeometries();

	    for (int i = 0; i < geometries.size(); i++)
	    {
	    	MasonGeometry mg = (MasonGeometry) geometries.get(i);
	        String attribute = mg.getStringAttribute(attributeName);
	        if ((method == "different") && (!attribute.equals(attributeValue))) objects.add(mg);
	        else if ((method == "equal") && (attribute.equals(attributeValue))) objects.add(mg);
	        else continue;
	     }
	    
    	GeomVectorField newGeomVectorField = new GeomVectorField();	 
 	    for (Object o : objects)
 	    {
 	    	MasonGeometry mg = (MasonGeometry) o;
 	    	newGeomVectorField.addGeometry(mg);
 	    }
		return newGeomVectorField;
    }
	
	public static Bag bagsIntersection(Bag setA, Bag setB) 
	
	{
	    Bag tmp = new Bag();
	    for (Object x : setA) if (setB.contains(x)) tmp.add(x);
	    return tmp;
	}
	
} 
    
    
        

    