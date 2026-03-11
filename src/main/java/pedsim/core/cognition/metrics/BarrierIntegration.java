package pedsim.core.cognition.metrics;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Predicate;
import org.locationtech.jts.geom.Geometry;
import pedsim.core.agents.Agent;
import pedsim.core.cognition.cityimage.Barrier;
import pedsim.core.cognition.cityimage.Region;
import pedsim.core.engine.PedSimCity;
import pedsim.core.utilities.StringEnum.AgentBarrierType;
import pedsim.core.utilities.StringEnum.BarrierType;
import sim.graph.EdgeGraph;
import sim.graph.NodeGraph;
import sim.util.geo.Angles;
import sim.util.geo.AttributeValue;
import sim.util.geo.MasonGeometry;

public class BarrierIntegration {

  /**
   * Returns a set of barriers in the direction of the destination node from a given location.
   *
   * @param currentLocation The current node.
   * @param destinationNode The destination node.
   * @param agent The agent whose barrier type should be considered.
   * @return A mapping of the viewField and the barrierIDs intersecting it.
   */
  // TODO CONTROL PERFORMANCE
  public HashMap<Geometry, Set<Integer>> intersectingBarriers(NodeGraph currentLocation,
      NodeGraph destinationNode, Agent agent) {

    HashMap<Geometry, Set<Integer>> viewFieldIntersectingBarriers = new HashMap<>();
    Set<Integer> intersectingBarrierIDs = new HashSet<>();
    AgentBarrierType agentBarrierCategory = agent.getProperties().barrierType;
    Geometry viewField = Angles.viewField(currentLocation, destinationNode, 70.0);

    // TODO
    List<MasonGeometry> intersectingGeometries =
        PedSimCity.barriers.intersectingFeatures(viewField);
    Set<Integer> agentKnownBarriers = agent.getCognitiveMap().getAgentKnownBarriers();
    List<MasonGeometry> agentKnownBarriersGeometries =
        PedSimCity.barriers.getGeometriesFromIDs(agentKnownBarriers);
    intersectingGeometries.retainAll(agentKnownBarriersGeometries);

    if (agentKnownBarriersGeometries.isEmpty() || intersectingGeometries.isEmpty()) {
      return viewFieldIntersectingBarriers;
    }

    Predicate<Integer> isBarrierRelevant = barrierID -> {
      Barrier barrier = PedSimCity.barriersMap.get(barrierID);
      if (barrier == null) {
        return false; // Skip missing entries
      }

      BarrierType barrierType = barrier.type;

      if (agentBarrierCategory.equals(AgentBarrierType.ALL)) {
        return true;
      }

      if (agentBarrierCategory.equals(AgentBarrierType.POSITIVE)) {
        return barrierType.equals(BarrierType.PARK) || barrierType.equals(BarrierType.WATER);
      }

      if (agentBarrierCategory.equals(AgentBarrierType.NEGATIVE)) {
        return barrierType.equals(BarrierType.RAILWAY) || barrierType.equals(BarrierType.ROAD)
            || barrierType.equals(BarrierType.SECONDARY_ROAD);
      }

      if (agentBarrierCategory.equals(AgentBarrierType.SEPARATING)) {
        return !barrierType.equals(BarrierType.PARK);
      }

      return false;
    };

    intersectingGeometries.stream().map(g -> g.getIntegerAttribute("barrierID")) // Extract
                                                                                 // barrierID
        .filter(isBarrierRelevant) // Check via map and predicate
        .forEach(intersectingBarrierIDs::add);

    viewFieldIntersectingBarriers.put(viewField, intersectingBarrierIDs);
    return viewFieldIntersectingBarriers;
  }

  /**
   * Sets the barrier information for an EdgeGraph based on attribute values. This method parses
   * attribute strings representing different types of barriers, such as positive barriers, negative
   * barriers, rivers, and parks, and populates the corresponding lists in the EdgeGraph. The method
   * retrieves attribute values for positive barriers ("p_barr"), negative barriers ("n_barr"),
   * rivers ("a_rivers"), and parks ("w_parks") from the EdgeGraph's attributes. It parses these
   * strings to extract barrier IDs and adds them to the appropriate lists: positiveBarriers,
   * negativeBarriers, waterBodies, and parks. Additionally, it combines positive and negative
   * barriers into the 'barriers' list for convenient access.
   *
   * @param edge The EdgeGraph for which barrier information is being set.
   */
  public static void setEdgeGraphBarriers(EdgeGraph edge) {

    List<Integer> positiveBarriers = new ArrayList<>();
    List<Integer> negativeBarriers = new ArrayList<>();
    List<Integer> barriers = new ArrayList<>(); // all the barriers

    Map<String, List<Integer>> attributesMap = Map.of("p_barr", new ArrayList<>(), "n_barr",
        new ArrayList<>(), "a_rivers", new ArrayList<>(), "w_parks", new ArrayList<>());

    attributesMap.forEach((key, list) -> {
      String value = edge.attributes.get(key).getString();
      if (!value.equals("[]")) {
        Arrays.stream(value.replaceAll("[^-?0-9]+", " ").trim().split(" ")).map(Integer::valueOf)
            .forEach(list::add);
      }
      edge.attributes.put(
          key.replace("p_barr", "positiveBarriers").replace("n_barr", "negativeBarriers")
              .replace("a_rivers", "waterBodies").replace("w_parks", "parks"),
          new AttributeValue(list));
    });
    //
    //
    // if (!parks.isEmpty())
    // CommunityCognitiveMap.edgesWithinParks.add(edge);

    barriers.addAll(positiveBarriers);
    barriers.addAll(negativeBarriers);

    edge.attributes.put("barriers", new AttributeValue(barriers));
  }

  /**
   * It stores information about the barriers within a given SubGraph.
   *
   * @param subGraph The SubGraph for which the barrier information is being set.
   */
  public static void setRegionBarriers(Region region) {

    for (EdgeGraph childEdge : region.primalGraph.getEdges()) {
      region.barriers.addAll(
          region.primalGraph.getParentEdge(childEdge).attributes.get("barriers").getArray());
    }
  }
}
