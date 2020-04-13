package sim.app.geo.pedestrianSimulation;

import java.util.*;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.CoordinateArrays;
import com.vividsolutions.jts.geom.LineString;

import sim.util.geo.GeomPlanarGraphDirectedEdge;



public class SubGraph extends Graph
{
	public SubGraphNodesMap subGraphNodesMap = new SubGraphNodesMap();
	public SubGraphEdgesMap subGraphEdgesMap = new SubGraphEdgesMap();
	Graph parentGraph;
	
	public SubGraph(Graph parentGraph, ArrayList <EdgeGraph> edges) 
	{
		this.parentGraph = parentGraph;
		for (EdgeGraph edge: edges)
		{
			NodeGraph u = edge.u;
			NodeGraph v = edge.v;
			addFromOtherGraph(parentGraph, edge, u,v);
		}	
	}
	
    public SubGraph() 
    {
    }

	public void addFromOtherGraph(Graph parentGraph, EdgeGraph edge, NodeGraph u, NodeGraph v)
    {

        Coordinate uCoord = u.getCoordinate();
        Coordinate vCoord = v.getCoordinate();
        
        NodeGraph newU = this.getNode(uCoord);
        NodeGraph newV = this.getNode(vCoord);
        
        LineString line = (LineString) edge.getLine();
        Coordinate[] coords = CoordinateArrays.removeRepeatedPoints(line.getCoordinates());        
        EdgeGraph newEdge = new EdgeGraph(line);
        
        GeomPlanarGraphDirectedEdge de0 = new GeomPlanarGraphDirectedEdge(newU, newV, coords[1], true);
        GeomPlanarGraphDirectedEdge de1 = new GeomPlanarGraphDirectedEdge(newV, newU, coords[coords.length - 2], false);
        newEdge.setDirectedEdges(de0, de1);
        add(newEdge);
        add(newEdge);
        newEdge.setAttributes(edge.attributes);
        newEdge.setNodes(newU, newV);
        subGraphNodesMap.add(newU, u);
        subGraphNodesMap.add(newV, v);
        newU.primalEdge = u.primalEdge;
        newV.primalEdge = v.primalEdge;
    }
    
    public class SubGraphNodesMap
    {
    	public HashMap<NodeGraph, NodeGraph> map = new HashMap<NodeGraph, NodeGraph>();
	    public void add(NodeGraph node, NodeGraph parentNode)
	    {
	    	map.put(node, parentNode);
	    }

	    public NodeGraph findParent(NodeGraph nodeSubGraph) {return map.get(nodeSubGraph);}
	    public NodeGraph findChild(NodeGraph nodeGraph) {return utilities.getKeyFromValue(map, nodeGraph);}
	}
    
    public class SubGraphEdgesMap
    {
    	private HashMap<EdgeGraph, EdgeGraph> SubGraphEdgesMap = new HashMap<EdgeGraph, EdgeGraph>();
	    public void add(EdgeGraph edge, EdgeGraph parentEdge)
	    {
	    	SubGraphEdgesMap.put(edge, parentEdge);
	    }

	    public EdgeGraph find(EdgeGraph edgeSubGraph) {return SubGraphEdgesMap.get(edgeSubGraph);}
	}
    
    public NodeGraph getParentNode(NodeGraph nodeSubGraph)
    {
    	return subGraphNodesMap.findParent(nodeSubGraph);
    }
    
    public ArrayList<NodeGraph> getChildNodes(ArrayList<NodeGraph> parentNodes)
    {
    	
    	ArrayList<NodeGraph> childNodes = new  ArrayList<NodeGraph>();
    	for (NodeGraph parent: parentNodes) 
    		{
    			NodeGraph child = this.subGraphNodesMap.findChild(parent);
    			if (child != null) childNodes.add(child);
    		}
    	return childNodes;
    }
    
    public EdgeGraph getParentEdge(EdgeGraph edgeSubGraph)
    {
    	return subGraphEdgesMap.find(edgeSubGraph);
    }

    
}