package pedsim.activity.agents;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import pedsim.activity.engine.PedSimCityActivity;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import sim.graph.Graph;
import sim.graph.NodeGraph;
import sim.graph.NodesLookup;
import sim.util.geo.MasonGeometry;

/**
 * The places an agent can choose among within a straight-line radius: for a purpose, the nodes
 * holding something for it; for none, or for a purpose with nothing in reach, every node.
 *
 * <p>Each set is laid out once as flat arrays and scanned by distance. Every list is in the order
 * of {@link NodesLookup#getNodesBetweenDistanceInterval}, because {@link DestinationChoice} draws
 * from it by position.
 */
public final class DestinationIndex {

  private static final Map<ActivityPurpose, Layout> byPurpose = new ConcurrentHashMap<>();
  private static volatile Layout everyNode;

  private DestinationIndex() {}

  /**
   * The alternatives for a purpose around an origin, the origin excluded.
   *
   * @param purpose the trip's purpose, or null for none
   * @param origin where the agent is standing
   * @param radius straight-line metres, inclusive
   */
  public static List<NodeGraph> placesWithin(
      ActivityPurpose purpose, NodeGraph origin, double radius) {
    Map<NodeGraph, Double> attraction =
        purpose == null ? null : PedSimCityActivity.nodesPurposeWeight.get(purpose);
    if (attraction != null && !attraction.isEmpty()) {
      List<NodeGraph> places = layoutOf(purpose, attraction).within(origin, radius);
      if (!places.isEmpty()) {
        return places;
      }
    }
    return nodesWithin(origin, radius);
  }

  /** Every node within the radius of an origin, the origin excluded. */
  public static List<NodeGraph> nodesWithin(NodeGraph origin, double radius) {
    return everyNode().within(origin, radius);
  }

  /** Forgets every layout; called when the city's static data is cleared. */
  public static void clear() {
    byPurpose.clear();
    everyNode = null;
  }

  private static Layout layoutOf(ActivityPurpose purpose, Map<NodeGraph, Double> attraction) {
    Layout layout = byPurpose.get(purpose);
    Layout all = everyNode();
    if (layout == null || layout.source != attraction || layout.parent != all) {
      List<NodeGraph> places = new ArrayList<>();
      for (NodeGraph node : all.nodes) {
        if (attraction.getOrDefault(node, 0.0) > 0.0) {
          places.add(node);
        }
      }
      layout = new Layout(places, attraction, all);
      byPurpose.put(purpose, layout);
    }
    return layout;
  }

  private static Layout everyNode() {
    Graph network = SharedCognitiveMap.getCommunityPrimalNetwork();
    Layout layout = everyNode;
    if (layout == null || layout.source != network) {
      layout = new Layout(inLookupOrder(network), network, null);
      everyNode = layout;
    }
    return layout;
  }

  /** The network's nodes in the order a distance lookup lists them. */
  static List<NodeGraph> inLookupOrder(Graph network) {
    // The lookup leaves out the node it searches from, so it searches from one outside the graph.
    MasonGeometry point =
        new MasonGeometry(network.getNodes().get(0).getMasonGeometry().getGeometry().copy());
    NodeGraph outsider = new NodeGraph(point.getGeometry().getCoordinate());
    outsider.setMasonGeometry(point);
    double farthest = 0.0;
    for (NodeGraph node : network.getNodes()) {
      farthest = Math.max(farthest, outsider.getCoordinate().distance(node.getCoordinate()));
    }
    return NodesLookup.getNodesBetweenDistanceInterval(network, outsider, 0.0, farthest + 1.0);
  }

  /** A set of nodes with their coordinates side by side. */
  static final class Layout {
    private final NodeGraph[] nodes;
    private final double[] x;
    private final double[] y;
    private final Object source;
    private final Layout parent;

    Layout(List<NodeGraph> places, Object source, Layout parent) {
      this.nodes = places.toArray(new NodeGraph[0]);
      this.x = new double[nodes.length];
      this.y = new double[nodes.length];
      for (int i = 0; i < nodes.length; i++) {
        x[i] = nodes[i].getCoordinate().x;
        y[i] = nodes[i].getCoordinate().y;
      }
      this.source = source;
      this.parent = parent;
    }

    List<NodeGraph> within(NodeGraph origin, double radius) {
      double originX = origin.getCoordinate().x;
      double originY = origin.getCoordinate().y;
      List<NodeGraph> found = new ArrayList<>();
      for (int i = 0; i < nodes.length; i++) {
        double dx = originX - x[i];
        double dy = originY - y[i];
        if (Math.abs(dx) <= radius
            && Math.abs(dy) <= radius
            && Math.sqrt(dx * dx + dy * dy) <= radius
            && nodes[i] != origin) {
          found.add(nodes[i]);
        }
      }
      return found;
    }
  }
}
