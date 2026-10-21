package pedsim.activity.engine;

import ec.util.MersenneTwisterFast;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.time.LocalDate;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.logging.Logger;
import pedsim.activity.agents.ActivityPurpose;
import pedsim.activity.parameters.ActivityPars;
import pedsim.core.engine.PedSimCity;
import pedsim.core.parameters.Pars;
import sim.graph.NodeGraph;

/**
 * The city's overnight visitors: where they sleep and what they come to see.
 *
 * <p>Two files, both written by the pipeline. {@code <City>_accommodation.csv} ({@code
 * pipeline/accommodation.py}) gives the beds at each street node, and a night's tourists are placed
 * in proportion to them. {@code <City>_sights.csv} ({@code pipeline/sights.py}) gives each sight's
 * yearly visitors, which become the {@link ActivityPurpose#SIGHTSEEING} attraction in place of the
 * tag-based one. How many tourists there are is {@link ActivityPars#touristNightsByMonth}.
 */
public final class Tourists {

  private static final Logger logger = Logger.getLogger(Tourists.class.getName());

  /** Beds at each node that has any, nodes in ID order. */
  private static final Map<NodeGraph, Double> beds = new LinkedHashMap<>();

  private static NodeGraph[] bedNodes = new NodeGraph[0];
  private static double[] cumulativeBeds = new double[0];

  private static double[] nightsByMonth = new double[0];

  private Tourists() {}

  /** Whether the city has both tourist nights and somewhere to spend them. */
  public static boolean available() {
    return nightsByMonth.length == 12 && !beds.isEmpty();
  }

  /** Reads the accommodation and the sights, and parses the monthly nights. */
  public static void load() {
    beds.clear();
    bedNodes = new NodeGraph[0];
    cumulativeBeds = new double[0];
    nightsByMonth = parseNights(ActivityPars.touristNightsByMonth);

    Map<NodeGraph, Double> loaded = readNodeColumn("_accommodation.csv", "beds");
    beds.putAll(loaded);
    bedNodes = beds.keySet().toArray(new NodeGraph[0]);
    cumulativeBeds = new double[bedNodes.length];
    double total = 0.0;
    for (int i = 0; i < bedNodes.length; i++) {
      total += beds.get(bedNodes[i]);
      cumulativeBeds[i] = total;
    }
    if (!beds.isEmpty()) {
      logger.info(String.format("accommodation: %.0f beds on %d nodes", total, beds.size()));
    }

    Map<NodeGraph, Double> sights = readNodeColumn("_sights.csv", "visitors");
    if (!sights.isEmpty()) {
      PedSimCityActivity.nodesPurposeWeight.put(ActivityPurpose.SIGHTSEEING, sights);
      logger.info(
          String.format(
              "sights: SIGHTSEEING attraction from %.0f visitors a year over %d nodes",
              sights.values().stream().mapToDouble(Double::doubleValue).sum(), sights.size()));
    }
    if (nightsByMonth.length != 12 && !ActivityPars.touristNightsByMonth.isBlank()) {
      logger.warning(
          "touristNightsByMonth needs twelve numbers; got \""
              + ActivityPars.touristNightsByMonth
              + "\". No tourists.");
    }
  }

  /** Tourists in the city on the night of {@code day}: the month's nights over its days. */
  public static double nightsOn(LocalDate day) {
    if (nightsByMonth.length != 12) {
      return 0.0;
    }
    return nightsByMonth[day.getMonthValue() - 1] / day.lengthOfMonth();
  }

  /** A node to sleep at, drawn in proportion to its beds; null when there are none. */
  public static NodeGraph drawAccommodation(MersenneTwisterFast random) {
    if (bedNodes.length == 0) {
      return null;
    }
    double draw = random.nextDouble() * cumulativeBeds[cumulativeBeds.length - 1];
    int index = Arrays.binarySearch(cumulativeBeds, draw);
    index = index >= 0 ? index : -index - 1;
    return bedNodes[Math.min(index, bedNodes.length - 1)];
  }

  private static double[] parseNights(String raw) {
    if (raw == null || raw.isBlank()) {
      return new double[0];
    }
    String[] parts = raw.split(",");
    double[] nights = new double[parts.length];
    try {
      for (int i = 0; i < parts.length; i++) {
        nights[i] = Double.parseDouble(parts[i].trim());
      }
    } catch (NumberFormatException e) {
      return new double[0];
    }
    return nights;
  }

  /**
   * Sums a numeric column per {@code snapped_node_id} from {@code <City><suffix>}, nodes in ID
   * order; empty when the file is absent or lacks either column.
   */
  private static Map<NodeGraph, Double> readNodeColumn(String suffix, String column) {
    Map<NodeGraph, Double> byNode = new LinkedHashMap<>();
    String resourceName = Pars.cityName + "/" + Pars.cityName + suffix;
    URL fileUrl = Tourists.class.getClassLoader().getResource(resourceName);
    if (fileUrl == null) {
      logger.info("No " + resourceName + ".");
      return byNode;
    }
    Map<Integer, Double> byId = new TreeMap<>();
    int skipped = 0;
    try (BufferedReader br =
        new BufferedReader(new InputStreamReader(fileUrl.openStream(), StandardCharsets.UTF_8))) {
      String headerLine = br.readLine();
      List<String> header =
          headerLine == null ? List.of() : Arrays.asList(headerLine.split(",", -1));
      int nodeIdx = header.indexOf("snapped_node_id");
      int valueIdx = header.indexOf(column);
      if (nodeIdx < 0 || valueIdx < 0) {
        logger.warning(resourceName + " lacks snapped_node_id or " + column + "; header=" + header);
        return byNode;
      }
      String line;
      while ((line = br.readLine()) != null) {
        List<String> parts = splitCsv(line);
        try {
          int node = Integer.parseInt(parts.get(nodeIdx).trim());
          double value = Double.parseDouble(parts.get(valueIdx).trim());
          if (value > 0.0) {
            byId.merge(node, value, Double::sum);
          }
        } catch (NumberFormatException | IndexOutOfBoundsException e) {
          skipped++;
        }
      }
    } catch (Exception e) {
      logger.warning("Failed to read " + resourceName + ": " + e.getMessage());
      return byNode;
    }
    int missing = 0;
    for (Map.Entry<Integer, Double> entry : byId.entrySet()) {
      NodeGraph node = PedSimCity.nodesMap.get(entry.getKey());
      if (node == null) {
        missing++;
      } else {
        byNode.put(node, entry.getValue());
      }
    }
    if (skipped > 0 || missing > 0) {
      logger.info(
          String.format(
              "%s: %d rows skipped, %d nodes not in the network", resourceName, skipped, missing));
    }
    return byNode;
  }

  /** Splits one CSV line, keeping commas inside double quotes. */
  private static List<String> splitCsv(String line) {
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
    return fields;
  }
}
