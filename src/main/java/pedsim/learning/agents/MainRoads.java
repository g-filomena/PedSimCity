package pedsim.learning.agents;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import pedsim.core.parameters.Pars;
import pedsim.core.utilities.StringEnum.RoadType;
import sim.graph.EdgeGraph;

/** Whether a street is a primary or a secondary road. */
public final class MainRoads {

  private static Set<String> classes;

  private MainRoads() {}

  public static synchronized boolean contains(EdgeGraph edge) {
    if (classes == null) {
      classes = new HashSet<>();
      classes.addAll(Arrays.asList(Pars.roadTypes.get(RoadType.PRIMARY)));
      classes.addAll(Arrays.asList(Pars.roadTypes.get(RoadType.SECONDARY)));
    }
    var type = edge.attributes.get("roadType");
    return type != null && classes.contains(type.getString());
  }
}
