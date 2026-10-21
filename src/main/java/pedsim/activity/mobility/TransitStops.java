package pedsim.activity.mobility;

import ec.util.MersenneTwisterFast;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.logging.Logger;
import pedsim.activity.parameters.ActivityPars;
import pedsim.core.engine.PedSimCity;
import pedsim.core.parameters.Pars;
import sim.graph.NodeGraph;

/**
 * The public-transport stops of the city, as the street nodes they are snapped to and the number of
 * departures each serves on a weekday ({@code <City>_transit_stops.csv}, written by {@code
 * scripts/build_transit_layer.py} from the GTFS feed).
 *
 * <p>A stop is where a journey made by public transport meets the street: the walk to it and from
 * it are pedestrian legs. Which stop an agent uses near a node is drawn among the stops within
 * {@link ActivityPars#transitAccessRadiusMetres}, in proportion to their departures, so a metro
 * station or a busy interchange draws more of the city's riders than a stop served twice an hour.
 */
public final class TransitStops {

  private static final Logger logger = Logger.getLogger(TransitStops.class.getName());

  /** Departures per weekday at each node that holds a stop, nodes in ID order. */
  private static final Map<NodeGraph, Integer> departures = new LinkedHashMap<>();

  /** Stops in reach of a node, computed on first use; entries are in node ID order. */
  private static final Map<NodeGraph, List<NodeGraph>> inReach = new ConcurrentHashMap<>();

  private TransitStops() {}

  /** Whether the city has any stops. */
  public static boolean available() {
    return !departures.isEmpty();
  }

  /**
   * Reads the city's stop layer. Several GTFS stops snapped to one node add their departures. A
   * city without the file has no public transport, and every journey not walked stays off the
   * street as before.
   */
  public static void load() {
    departures.clear();
    inReach.clear();
    String resourceName = Pars.cityName + "/" + Pars.cityName + "_transit_stops.csv";
    URL fileUrl = TransitStops.class.getClassLoader().getResource(resourceName);
    if (fileUrl == null) {
      logger.info(
          "No transit stops at " + resourceName + " - journeys not walked stay off-street.");
      return;
    }
    Map<Integer, Integer> byNode = new java.util.TreeMap<>();
    int skipped = 0;
    try (BufferedReader br =
        new BufferedReader(new InputStreamReader(fileUrl.openStream(), StandardCharsets.UTF_8))) {
      String headerLine = br.readLine();
      List<String> header =
          headerLine == null ? List.of() : Arrays.asList(headerLine.split(",", -1));
      int nodeIdx = header.indexOf("snapped_node_id");
      int depIdx = header.indexOf("departures");
      if (nodeIdx < 0 || depIdx < 0) {
        logger.warning(
            "Transit stops CSV lacks snapped_node_id or departures; header=" + header + ".");
        return;
      }
      String line;
      while ((line = br.readLine()) != null) {
        // Stop and route names may hold commas inside quotes; the two numeric fields are read
        // from a split that respects them.
        String[] parts = splitCsv(line);
        try {
          int node = Integer.parseInt(parts[nodeIdx].trim());
          int served = Integer.parseInt(parts[depIdx].trim());
          if (served > 0) {
            byNode.merge(node, served, Integer::sum);
          }
        } catch (NumberFormatException | ArrayIndexOutOfBoundsException e) {
          skipped++;
        }
      }
    } catch (Exception e) {
      logger.warning("Failed to load transit stops: " + e.getMessage());
      return;
    }
    int missing = 0;
    for (Map.Entry<Integer, Integer> entry : byNode.entrySet()) {
      NodeGraph node = PedSimCity.nodesMap.get(entry.getKey());
      if (node == null) {
        missing++;
      } else {
        departures.put(node, entry.getValue());
      }
    }
    logger.info(
        String.format(
            "transit stops: %d nodes with service, %d departures a weekday (%d rows skipped, %d"
                + " nodes not in the network)",
            departures.size(),
            departures.values().stream().mapToLong(Integer::longValue).sum(),
            skipped,
            missing));
  }

  /**
   * A stop for a journey starting or ending at {@code node}, or null when none is in reach.
   *
   * @param node where the walk to or from the stop starts or ends
   * @param random the agent's generator
   * @return the stop's node
   */
  public static NodeGraph near(NodeGraph node, MersenneTwisterFast random) {
    List<NodeGraph> candidates = inReach.computeIfAbsent(node, TransitStops::stopsInReach);
    if (candidates.isEmpty()) {
      return null;
    }
    double total = 0.0;
    for (NodeGraph stop : candidates) {
      total += departures.get(stop);
    }
    double draw = random.nextDouble() * total;
    for (NodeGraph stop : candidates) {
      draw -= departures.get(stop);
      if (draw < 0.0) {
        return stop;
      }
    }
    return candidates.get(candidates.size() - 1);
  }

  /** Whether stops are in reach of both ends of a journey; draws nothing. */
  public static boolean servesBoth(NodeGraph origin, NodeGraph destination) {
    return available()
        && origin != null
        && destination != null
        && !inReach.computeIfAbsent(origin, TransitStops::stopsInReach).isEmpty()
        && !inReach.computeIfAbsent(destination, TransitStops::stopsInReach).isEmpty();
  }

  /** Departures a weekday at a stop's node; zero where there is no stop. */
  public static int departuresAt(NodeGraph stop) {
    return departures.getOrDefault(stop, 0);
  }

  private static List<NodeGraph> stopsInReach(NodeGraph node) {
    double radius = ActivityPars.transitAccessRadiusMetres;
    List<NodeGraph> found = new ArrayList<>();
    for (NodeGraph stop : departures.keySet()) {
      if (stop.getCoordinate().distance(node.getCoordinate()) <= radius) {
        found.add(stop);
      }
    }
    found.sort(Comparator.comparingInt(NodeGraph::getID));
    return found;
  }

  private static String[] splitCsv(String line) {
    List<String> fields = new ArrayList<>();
    StringBuilder field = new StringBuilder();
    boolean quoted = false;
    for (int i = 0; i < line.length(); i++) {
      char c = line.charAt(i);
      if (c == '"') {
        quoted = !quoted;
      } else if (c == ',' && !quoted) {
        fields.add(field.toString());
        field.setLength(0);
      } else {
        field.append(c);
      }
    }
    fields.add(field.toString());
    return fields.toArray(new String[0]);
  }
}
