package pedSim.utilities;

import java.util.List;

import org.locationtech.jts.geom.LineString;

/**
 * To store information about the walked routes.
 *
 */

public class RouteData {

	public Integer origin;
	public Integer destination;
	public boolean minimisingDistance;
	public boolean minimisingAngular;
	public boolean localHeuristicDistance;
	public boolean localHeuristicAngular;

	public boolean regionBased;
	public boolean onRouteMarks;
	public boolean barrierSubGoals;
	public boolean distantLandmarks;

	public double naturalBarriers;
	public double severingBarriers;

	public List<Integer> edgeIDsSequence;
	public String routeID;
	public String group;
	public String routeChoice;
	public LineString lineGeometry;
}
