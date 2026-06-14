package pedsim.cityimage.utilities;

import java.util.List;

/**
 * To store information about the walked routes.
 *
 */

public class RouteData extends pedsim.core.utilities.RouteData {

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

  public String group;
}
