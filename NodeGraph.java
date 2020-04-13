package sim.app.geo.pedestrianSimulation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.planargraph.Node;
import sim.util.geo.AttributeValue;
import sim.util.geo.MasonGeometry;

public class NodeGraph extends Node
{
	

	public NodeGraph(Coordinate pt) {super(pt);}
	
	int nodeID;
	int district;
	double bC;
	boolean gateway;
	boolean centroid;
	public MasonGeometry masonGeometry;
	public EdgeGraph primalEdge;
	
	List<Integer> visible2d = new ArrayList<Integer>();
	List<Integer> localLandmarks = new ArrayList<Integer>();
	List<Double> localScores = new ArrayList<Double>();
	List<Integer> distantLandmarks  = new ArrayList<Integer>();
	List<Double> distantScores = new ArrayList<Double>();
	List<Integer> anchors = new ArrayList<Integer>();
	List<Double> distances = new ArrayList<Double>();
	SubGraph SubGraph;
	
//	List<Object> attributes = new ArrayList<Object>();
//	attributes.add(district);
	
    public void setID(int nodeID)
    {
        this.nodeID = nodeID;
    }
    
    public Integer getID()
    {
        return this.nodeID;
    }
	
    
	
    private Map<String, AttributeValue> attributes;
    public Object getAttribute(final String name)
    {
        return this.attributes.get(name);
    }

    public Integer getIntegerAttribute(final String name)
    {
        return this.attributes.get(name).getInteger();
    }
    
    

    public Double getDoubleAttribute(final String name)
    {
        return this.attributes.get(name).getDouble();
    }

    public String getStringAttribute(final String name)
    {
        return this.attributes.get(name).getString();
    }
    
    
    public void setLandmarkness(MasonGeometry geometry)
    {
	    String localString = geometry.getStringAttribute("loc_land");
		String lScoresString = geometry.getStringAttribute("loc_scor");
		String distantString = geometry.getStringAttribute("dist_land");
		String dScoresString = geometry.getStringAttribute("dist_scor");
		String anchorsString = geometry.getStringAttribute("anchors");
		String distancesString = geometry.getStringAttribute("distances");

		if (!localString.equals("[]"))
		{
			String l = localString.replaceAll("[^-?0-9]+", " ");
			String s = lScoresString.replaceAll("[^0-9.]+", " ");
	    	for(String t : (Arrays.asList(l.trim().split(" ")))) this.localLandmarks.add(Integer.valueOf(t));
	    	for(String t : (Arrays.asList(s.trim().split(" ")))) this.localScores.add(Double.valueOf(t));
		}
		else
		{
			this.localLandmarks = null;
			this.localScores = null;
		}
		
		if (!distantString.equals("[]"))
		{
			String l = distantString.replaceAll("[^-?0-9]+", " ");
			String s = dScoresString.replaceAll("[^0-9.]+", " ");
			for(String t : (Arrays.asList(l.trim().split(" ")))) this.distantLandmarks.add(Integer.valueOf(t));
			for(String t : (Arrays.asList(s.trim().split(" ")))) this.distantScores.add(Double.valueOf(t));
		}
		else
		{
			this.distantLandmarks = null;
			this.distantScores = null;
		}
	    	
		if (!anchorsString.equals("[]"))
		{
			String l = anchorsString.replaceAll("[^-?0-9]+", " ");
			String d = distancesString.replaceAll("[^0-9.]+", " ");
			for(String t : (Arrays.asList(l.trim().split(" ")))) this.anchors.add(Integer.valueOf(t));
			for(String t : (Arrays.asList(d.trim().split(" ")))) this.distances.add(Double.valueOf(t));
		}
		else this.anchors = null;  
    }
}

