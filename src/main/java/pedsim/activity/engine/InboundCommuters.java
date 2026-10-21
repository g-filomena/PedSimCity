package pedsim.activity.engine;

import ec.util.MersenneTwisterFast;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.logging.Logger;
import org.locationtech.jts.geom.Coordinate;
import pedsim.core.engine.PedSimCity;
import pedsim.core.parameters.Pars;
import sim.graph.NodeGraph;

/**
 * The people who live outside the city and come into it every day to work or study ({@code
 * <City>_inbound_commuters.csv}, written by {@code pipeline/inbound_commuters_istat.py} from the
 * ISTAT 2011 commuting matrix), the railway stations they can arrive at ({@code
 * <City>_rail_stations.csv}) and which of them trains from each origin reach ({@code
 * <City>_rail_reach.csv}), both from the rail timetables by {@code pipeline/rail_reach.py}.
 *
 * <p>Only those whose journey reaches the street are built as agents: by train they arrive at a
 * station, by tram, metro, urban bus, coach or works bus at a stop near where they are going, by
 * car at a car park when they work inside the limited-traffic zone ({@link
 * pedsim.activity.mobility.CarParks}). Driving
 * anywhere else, or by motorbike, bicycle or any other mode, they put no one on the street in this
 * model, as a resident's journey by those modes does not.
 */
public final class InboundCommuters {

  private static final Logger logger = Logger.getLogger(InboundCommuters.class.getName());

  /** How a flow arrives in the city. */
  public enum Arrival {
    RAIL,
    STOP,
    CAR,
    OFF_STREET
  }

  /**
   * One row of the commuting table: where its people live, why and how they travel, the minutes of
   * the day within which they leave home and how long the journey takes, and how many they are.
   */
  public record Flow(
      String originId,
      Coordinate residence,
      boolean study,
      Arrival arrival,
      int leaveFromMinute,
      int leaveToMinute,
      int journeyFromMinutes,
      int journeyToMinutes,
      boolean female,
      double people) {}

  private static final List<Flow> flows = new ArrayList<>();
  private static final List<NodeGraph> stations = new ArrayList<>();

  /** The city's stations trains from each origin call at, by origin. */
  private static final Map<String, List<NodeGraph>> reach = new HashMap<>();

  private InboundCommuters() {}

  /** The flows read, in file order. */
  public static List<Flow> flows() {
    return flows;
  }

  /**
   * The station a train commuter arrives at. When the timetables say which of the city's stations
   * trains from its municipality call at, the one of those nearest the workplace: a commuter gets
   * off where its own line comes closest to its job. Otherwise the station that makes the shortest
   * straight-line journey from home through it to the workplace, or, where home is not known, the
   * one nearest the workplace.
   *
   * @return the station's node, or null when the city has none
   */
  public static NodeGraph stationFor(Flow flow, NodeGraph work) {
    List<NodeGraph> line = reach.get(flow.originId());
    Coordinate residence = flow.residence();
    List<NodeGraph> candidates = line != null && !line.isEmpty() ? line : stations;
    if (candidates == line) {
      residence = null;
    }
    NodeGraph best = null;
    double bestMetres = Double.MAX_VALUE;
    for (NodeGraph station : candidates) {
      double metres = station.getCoordinate().distance(work.getCoordinate());
      if (residence != null) {
        metres += residence.distance(station.getCoordinate());
      }
      if (metres < bestMetres) {
        best = station;
        bestMetres = metres;
      }
    }
    return best;
  }

  /**
   * Minute of the day the commuter reaches the city: a minute inside the flow's window for leaving
   * home, plus a journey time inside its window, both drawn uniformly.
   */
  public static int arrivalMinute(Flow flow, MersenneTwisterFast random) {
    int leave = flow.leaveToMinute() - flow.leaveFromMinute();
    int journey = flow.journeyToMinutes() - flow.journeyFromMinutes();
    return flow.leaveFromMinute()
        + random.nextInt(Math.max(1, leave + 1))
        + flow.journeyFromMinutes()
        + random.nextInt(Math.max(1, journey + 1));
  }

  /** Reads both files; a city without the commuting table has no inbound commuters. */
  public static void load() {
    flows.clear();
    stations.clear();
    reach.clear();
    List<String[]> rows = read(Pars.cityName + "/" + Pars.cityName + "_inbound_commuters.csv");
    if (rows.isEmpty()) {
      logger.info("No inbound commuters table - the city's population is its residents only.");
      return;
    }
    List<String> header = Arrays.asList(rows.get(0));
    int code = header.indexOf("origin_id");
    int x = header.indexOf("origin_x");
    int y = header.indexOf("origin_y");
    int motive = header.indexOf("motive");
    int mode = header.indexOf("mode");
    int leaveFrom = header.indexOf("leave_from_min");
    int leaveTo = header.indexOf("leave_to_min");
    int journeyFrom = header.indexOf("journey_from_min");
    int journeyTo = header.indexOf("journey_to_min");
    int female = header.indexOf("female");
    int people = header.indexOf("people");
    int skipped = 0;
    for (String[] row : rows.subList(1, rows.size())) {
      try {
        Coordinate residence =
            row[x].isBlank()
                ? null
                : new Coordinate(Double.parseDouble(row[x]), Double.parseDouble(row[y]));
        flows.add(
            new Flow(
                row[code].trim(),
                residence,
                row[motive].equals("study"),
                arrivalBy(row[mode]),
                Integer.parseInt(row[leaveFrom].trim()),
                Integer.parseInt(row[leaveTo].trim()),
                Integer.parseInt(row[journeyFrom].trim()),
                Integer.parseInt(row[journeyTo].trim()),
                row[female].equals("1"),
                Double.parseDouble(row[people])));
      } catch (RuntimeException e) {
        skipped++;
      }
    }
    List<String[]> stationRows = read(Pars.cityName + "/" + Pars.cityName + "_rail_stations.csv");
    if (!stationRows.isEmpty()) {
      int node = Arrays.asList(stationRows.get(0)).indexOf("snapped_node_id");
      for (String[] row : stationRows.subList(1, stationRows.size())) {
        NodeGraph station = PedSimCity.nodesMap.get(Integer.parseInt(row[node].trim()));
        if (station != null && !stations.contains(station)) {
          stations.add(station);
        }
      }
    }
    List<String[]> reachRows = read(Pars.cityName + "/" + Pars.cityName + "_rail_reach.csv");
    if (!reachRows.isEmpty()) {
      List<String> reachHeader = Arrays.asList(reachRows.get(0));
      int origin = reachHeader.indexOf("origin_id");
      int node = reachHeader.indexOf("snapped_node_id");
      for (String[] row : reachRows.subList(1, reachRows.size())) {
        NodeGraph station = PedSimCity.nodesMap.get(Integer.parseInt(row[node].trim()));
        if (station == null) {
          continue;
        }
        List<NodeGraph> line = reach.computeIfAbsent(row[origin].trim(), k -> new ArrayList<>());
        if (!line.contains(station)) {
          line.add(station);
        }
        if (!stations.contains(station)) {
          stations.add(station);
        }
      }
    }
    logger.info(
        String.format(
            "inbound commuters: %d flows (%d rows skipped), %d railway stations, trains from %d"
                + " origins",
            flows.size(), skipped, stations.size(), reach.size()));
  }

  /** The table's mode, by where it leaves the commuter on the street. */
  private static Arrival arrivalBy(String mode) {
    return switch (mode.trim()) {
      case "train" -> Arrival.RAIL;
      case "tram", "metro", "bus", "coach", "works_bus" -> Arrival.STOP;
      case "car_driver", "car_passenger" -> Arrival.CAR;
      default -> Arrival.OFF_STREET;
    };
  }

  private static List<String[]> read(String resourceName) {
    List<String[]> rows = new ArrayList<>();
    URL fileUrl = InboundCommuters.class.getClassLoader().getResource(resourceName);
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
