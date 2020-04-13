package sim.app.geo.pedestrianSimulation;

import java.util.ArrayList;
import java.util.Collection;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.CoordinateArrays;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.planargraph.DirectedEdge;
import com.vividsolutions.jts.planargraph.Node;
import com.vividsolutions.jts.planargraph.NodeMap;
import com.vividsolutions.jts.planargraph.PlanarGraph;

import sim.field.geo.GeomVectorField;
import sim.field.network.Network;
import sim.util.Bag;
import sim.util.geo.GeomPlanarGraph;
import sim.util.geo.GeomPlanarGraphDirectedEdge;
import sim.util.geo.MasonGeometry;



/** A JTS PlanarGraph
 *
 * Planar graph useful for exploiting network topology.
 *
 * @see sim.app.geo.networkworld and sim.app.geo.campusworld
 *
 */
public class Graph extends GeomPlanarGraph
{
	
	public Graph()
    {
        super();
    }

	

    /** populate network with lines from a GeomVectorField
     *
     * @param field containing line segments
     *
     * Assumes that 'field' contains co-planar linear objects
     *
     */
    public void fromGeomField(GeomVectorField field)
    {
        Bag geometries = field.getGeometries();

        for (int i = 0; i < geometries.numObjs; i++)
        {
            if (((MasonGeometry) geometries.get(i)).geometry instanceof LineString)
            {
                this.addLineString((MasonGeometry)geometries.get(i));
            }
        }

    }  
    
    private void addLineString(MasonGeometry wrappedLine)
    {
        LineString line = (LineString) wrappedLine.geometry;
        if (line.isEmpty()) return;

        Coordinate[] coords = CoordinateArrays.removeRepeatedPoints(line.getCoordinates());
        if (coords.length < 2) return;

        Coordinate uCoord = coords[0];
        Coordinate vCoord = coords[coords.length - 1];
        NodeGraph u = getNode(uCoord);
        NodeGraph v = getNode(vCoord);

        EdgeGraph edge = new EdgeGraph(line);
        GeomPlanarGraphDirectedEdge de0 = new GeomPlanarGraphDirectedEdge(u, v, coords[1], true);
        GeomPlanarGraphDirectedEdge de1 = new GeomPlanarGraphDirectedEdge(v, u, coords[coords.length - 2], false);

        edge.setDirectedEdges(de0, de1);
        edge.setAttributes(wrappedLine.getAttributes());
        edge.setNodes(u, v);
        add(edge);
    }
    

    
    /** get the node corresponding to the coordinate
     *
     * @param startPt
     * @return graph node associated with point
     *
     * Will create a new Node if one does not exist.
     *
     * @note Some code copied from JTS PolygonizeGraph.getNode() and hacked to fit
     */
    public NodeGraph getNode(Coordinate pt)
    {
        NodeGraph node = (NodeGraph) findNode(pt);
        if (node == null)
        {
            node = new NodeGraph(pt);
            // ensure node is only added once to graph
            add(node);
        }
        return node;
    }

    /** Create a MASON Network from this planar graph
     *
     * XXX Unfortunately we need this since JTS planar graphs do not support
     * shortest distance and other common graph traversals.
     */
    public Network getNetwork()
    {
        Network network = new Network(false); // false == not directed

        for ( Object object : getEdges() )
        {
            DirectedEdge edge = (DirectedEdge) object;

            network.addEdge(edge.getFromNode(), edge.getToNode(), edge);
        }

        return network;
    }
    
    public NodeGraph findNode(Coordinate pt)
    {
      return (NodeGraph) nodeMap.find(pt);
    }
    
    public static EdgeGraph getEdgeBetween(NodeGraph u, NodeGraph v)
    {
    	EdgeGraph edge = null;
		Collection connectingEdge = NodeGraph.getEdgesBetween(u, v);
		for (Object o : connectingEdge) edge = (EdgeGraph) o;
		return edge;
    }
    

    



}












