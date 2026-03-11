package pedsim.core.utilities;

import java.util.ArrayList;
import java.util.List;
import org.locationtech.jts.geom.LineString;

/**
 * To store information about the walked routes.
 *
 */

public class RouteData {

  public Integer origin;
  public Integer destination;
  public boolean learner;

  public List<Integer> edgeIDsSequence = new ArrayList<>();
  public String routeID;
  public LineString lineGeometry;
  public String scenario;
}
