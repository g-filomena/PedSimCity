package pedsim.activity.mobility;

import ec.util.MersenneTwisterFast;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.logging.Logger;
import pedsim.activity.parameters.ActivityPars;
import pedsim.core.engine.PedSimCity;
import pedsim.core.parameters.Pars;
import sim.graph.NodeGraph;

/**
 * The city's parking structures ({@code <City>_parking.csv}, their capacity and the street node each
 * is snapped to) and the street nodes inside the zone where a car is left in one, such as a
 * limited-traffic zone ({@code <City>_restricted_zone_nodes.csv}), both written by {@code
 * pipeline/parking.py}.
 *
 * <p>A car driven to a destination inside the zone is left in a structure and the rest of the way
 * is walked; anywhere else it is left at the door, and the journey puts no one on the street.
 *
 * <p>The structures' spaces bound how many cars get there. A commuter's car takes a space for the
 * working day, so commuters are given spaces once, at the run's scale ({@link #scaleTo}, {@link
 * #reserve}); a commuter who finds none does not drive into the zone. A shorter visit needs a
 * space left free by them ({@link #nearFree}); how many such visits one space serves in a day is
 * not modelled.
 */
public final class CarParks {

  private static final Logger logger = Logger.getLogger(CarParks.class.getName());

  /** Spaces at each node holding a structure, nodes in ID order. */
  private static final Map<NodeGraph, Integer> capacity = new LinkedHashMap<>();

  private static final Set<Integer> zoneNodes = new HashSet<>();

  /** Spaces not taken by commuters, at the run's scale, nodes in ID order. */
  private static final Map<NodeGraph, Integer> free = new LinkedHashMap<>();

  private CarParks() {}

  /** Whether the city has both a zone and structures to park in. */
  public static boolean available() {
    return !capacity.isEmpty() && !zoneNodes.isEmpty();
  }

  /** Whether a node lies inside the limited-traffic zone. */
  public static boolean inZone(NodeGraph node) {
    return node != null && zoneNodes.contains(node.getID());
  }

  /**
   * Sets the spaces free for the run: each structure's capacity times {@code share}, the share of
   * the real population the run simulates. Call once, before any {@link #reserve}.
   */
  public static void scaleTo(double share) {
    free.clear();
    for (Map.Entry<NodeGraph, Integer> entry : capacity.entrySet()) {
      free.put(entry.getKey(), (int) Math.round(entry.getValue() * share));
    }
  }

  /**
   * Takes a space for a commuter's car near {@code destination}: in a structure within {@link
   * ActivityPars#carParkRadiusMetres} that still has one, in proportion to the spaces each has
   * left. Null when every structure in reach is full.
   */
  public static NodeGraph reserve(NodeGraph destination, MersenneTwisterFast random) {
    NodeGraph park = nearFree(destination, random);
    if (park != null) {
      free.merge(park, -1, Integer::sum);
    }
    return park;
  }

  /**
   * A structure within {@link ActivityPars#carParkRadiusMetres} of {@code destination} with spaces
   * free of commuters, in proportion to those spaces; null when there is none.
   */
  public static NodeGraph nearFree(NodeGraph destination, MersenneTwisterFast random) {
    List<NodeGraph> inReach = new ArrayList<>();
    double total = 0.0;
    for (Map.Entry<NodeGraph, Integer> entry : free.entrySet()) {
      if (entry.getValue() > 0
          && entry.getKey().getCoordinate().distance(destination.getCoordinate())
              <= ActivityPars.carParkRadiusMetres) {
        inReach.add(entry.getKey());
        total += entry.getValue();
      }
    }
    if (inReach.isEmpty()) {
      return null;
    }
    double draw = random.nextDouble() * total;
    for (NodeGraph park : inReach) {
      draw -= free.get(park);
      if (draw < 0.0) {
        return park;
      }
    }
    return inReach.get(inReach.size() - 1);
  }

  /** Spaces still free of commuters across all structures. */
  public static int freeSpaces() {
    return free.values().stream().mapToInt(Integer::intValue).sum();
  }

  /** Reads both files; a city without them has no zone, and every car is left at the door. */
  public static void load() {
    capacity.clear();
    zoneNodes.clear();
    List<String[]> parks = read(Pars.cityName + "/" + Pars.cityName + "_parking.csv");
    if (!parks.isEmpty()) {
      List<String> header = Arrays.asList(parks.get(0));
      int node = header.indexOf("snapped_node_id");
      int spaces = header.indexOf("capacity");
      java.util.TreeMap<Integer, Integer> byNode = new java.util.TreeMap<>();
      for (String[] row : parks.subList(1, parks.size())) {
        try {
          byNode.merge(
              Integer.parseInt(row[node].trim()),
              (int) Math.round(Double.parseDouble(row[spaces].trim())),
              Integer::sum);
        } catch (RuntimeException e) {
          // a structure without a usable node or capacity is left out
        }
      }
      for (Map.Entry<Integer, Integer> entry : byNode.entrySet()) {
        NodeGraph park = PedSimCity.nodesMap.get(entry.getKey());
        if (park != null && entry.getValue() > 0) {
          capacity.put(park, entry.getValue());
        }
      }
    }
    List<String[]> zone = read(Pars.cityName + "/" + Pars.cityName + "_restricted_zone_nodes.csv");
    for (String[] row : zone.subList(Math.min(1, zone.size()), zone.size())) {
      try {
        zoneNodes.add(Integer.parseInt(row[0].trim()));
      } catch (RuntimeException e) {
        // header or blank line
      }
    }
    logger.info(
        String.format(
            "car parks: %d structures, %d spaces; %d nodes in the limited-traffic zone",
            capacity.size(),
            capacity.values().stream().mapToInt(Integer::intValue).sum(),
            zoneNodes.size()));
  }

  private static List<String[]> read(String resourceName) {
    List<String[]> rows = new ArrayList<>();
    URL fileUrl = CarParks.class.getClassLoader().getResource(resourceName);
    if (fileUrl == null) {
      return rows;
    }
    try (BufferedReader br =
        new BufferedReader(new InputStreamReader(fileUrl.openStream(), StandardCharsets.UTF_8))) {
      String line;
      while ((line = br.readLine()) != null) {
        rows.add(line.split(",", -1));
      }
    } catch (Exception e) {
      logger.warning("Failed to read " + resourceName + ": " + e.getMessage());
      rows.clear();
    }
    return rows;
  }
}
