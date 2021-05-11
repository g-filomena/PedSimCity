package pedsimcity.utilities;

import java.util.List;

import com.vividsolutions.jts.geom.LineString;

/**
 * To store information about the walked routes.
 *
 */

public class RouteData {

	public Integer origin;
	public Integer destination;
	public String onlyMinimisation;
	public String localHeuristic;

	public int regionBased;
	public int routeMarks;
	public int barrierSubGoals;
	public int distantLandmarks;

	public double naturalBarriers;
	public double severingBarriers;

	public List<Integer> sequenceEdges;
	public String routeID;
	public String group;
	public String routeChoice;
	public LineString lineGeometry;

}
