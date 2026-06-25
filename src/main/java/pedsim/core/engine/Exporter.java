package pedsim.core.engine;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.time.LocalDate;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.logging.Logger;
import org.apache.commons.lang3.ArrayUtils;
import pedsim.core.parameters.TimePars;
import pedsim.core.utilities.LoggerUtil;
import pedsim.core.utilities.RouteData;
import pedsim.core.utilities.StringEnum;
import sim.field.geo.VectorLayer;
import sim.io.geo.ShapeFileExporter;
import sim.util.geo.CSVUtils;
import sim.util.geo.MasonGeometry;

/**
 * The Export class is responsible for saving the simulation results to specified output
 * directories.
 */
public class Exporter {

  private String userName = System.getProperty("user.name");
  // Constants for file paths and directories
  public String outputDirectory;
  public String outputRoutesDirectory;
  public String outputVolumesDirectory;
  public String outputCognitiveMapDirectory;
  private String outputLandmarkCognitiveMapDirectory;

  protected static final Logger logger = LoggerUtil.getLogger();
  protected static int nrColumns;
  protected static final int FIELD_LIMIT = 254;
  protected int job;
  protected String currentDate;

  protected FlowHandler flowHandler; // Using a wildcard since we don't know the exact type

  // Constructor accepting any FlowHandler
  public Exporter(FlowHandler flowHandler, String appName) {
    this.flowHandler = flowHandler;
    this.job = flowHandler.job;
    // Results live with the project, under outputs/<appName>/ — portable (works on the Linux
    // server too), co-located with the run, and gitignored. (Was a hardcoded C:\Users\... path.)
    outputDirectory = "outputs" + File.separator + appName;
    DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyyMMdd");
    currentDate = LocalDate.now().format(formatter);
  }

  /**
   * Saves pedestrian volumes data to a CSV file.
   *
   * @param day The simulated day from 1 onwards.
   * @throws Exception If there is an error while saving the data.
   */
  public <E extends Enum<E>> void savePedestrianVolumes(int day, String[] scenarios)
      throws Exception {

    outputVolumesDirectory = verifyOutputPath(outputVolumesDirectory, "streetVolumes");
    outputVolumesDirectory += File.separator + currentDate + "_" + job + "_" + day + ".csv";
    final FileWriter writerVolumesData = new FileWriter(outputVolumesDirectory);

    Map<Integer, Map<String, Integer>> volumesMap = new HashMap<>(flowHandler.volumesMap);

    // Volume cells are keyed "<agentType>_<scenario>" (or "<agentType>" with no scenario).
    Enum<?>[] agentValues = flowHandler.getAgentScenarioValues();
    Enum<?>[] simValues = flowHandler.getSimulationScenarioValues();

    // A single DEFAULT agent type means "no agent sub-typing": its per-agent columns collapse away.
    Enum<?>[] agents =
        (agentValues != null) ? agentValues : new Enum<?>[] {StringEnum.Default.DEFAULT};
    boolean perAgent = !(agents.length == 1 && agents[0] == StringEnum.Default.DEFAULT);
    boolean hourly =
        simValues != null && simValues.length > 0 && simValues[0] instanceof StringEnum.Hour;

    // ---- Header ----
    List<String> headers = new ArrayList<>();
    headers.add("edgeID");
    if (hourly) {
      if (perAgent) {
        for (Enum<?> a : agents) {
          for (Enum<?> s : simValues) headers.add(a + "_" + s); // agent x hour cells
        }
      }
      for (Enum<?> s : simValues) headers.add(s.toString()); // hourly, summed over agent types
      headers.add("DAY");
      headers.add("NIGHT"); // day/night aggregation of the hours
      if (perAgent) {
        for (Enum<?> a : agents) {
          headers.add(a + "_DAY");
          headers.add(a + "_NIGHT");
        }
        for (Enum<?> a : agents) headers.add(a.toString()); // per agent type, over all hours
      }
    } else if (perAgent) {
      for (Enum<?> a : agents) headers.add(a.toString()); // per agent type (no time dimension)
    }
    headers.add("total");
    CSVUtils.writeLine(writerVolumesData, headers);

    // ---- Rows ----
    for (Map.Entry<Integer, Map<String, Integer>> entry : volumesMap.entrySet()) {
      Map<String, Integer> ev = entry.getValue();
      List<String> row = new ArrayList<>();
      row.add(Integer.toString(entry.getKey()));

      if (hourly) {
        if (perAgent) {
          for (Enum<?> a : agents) {
            for (Enum<?> s : simValues) row.add(Integer.toString(cellVolume(ev, a, s)));
          }
        }
        for (Enum<?> s : simValues) {
          int t = 0;
          for (Enum<?> a : agents) t += cellVolume(ev, a, s);
          row.add(Integer.toString(t));
        }
        int day = 0;
        int night = 0;
        for (Enum<?> a : agents) {
          for (Enum<?> s : simValues) {
            int v = cellVolume(ev, a, s);
            if (isNightHour(s)) night += v;
            else day += v;
          }
        }
        row.add(Integer.toString(day));
        row.add(Integer.toString(night));
        if (perAgent) {
          for (Enum<?> a : agents) {
            int d = 0;
            int n = 0;
            for (Enum<?> s : simValues) {
              int v = cellVolume(ev, a, s);
              if (isNightHour(s)) n += v;
              else d += v;
            }
            row.add(Integer.toString(d));
            row.add(Integer.toString(n));
          }
          for (Enum<?> a : agents) {
            int t = 0;
            for (Enum<?> s : simValues) t += cellVolume(ev, a, s);
            row.add(Integer.toString(t));
          }
        }
      } else if (perAgent) {
        for (Enum<?> a : agents) row.add(Integer.toString(cellVolume(ev, a, null)));
      }

      // Grand total across all cells.
      int grandTotal = 0;
      for (Enum<?> a : agents) {
        if (simValues != null) {
          for (Enum<?> s : simValues) grandTotal += cellVolume(ev, a, s);
        } else {
          grandTotal += cellVolume(ev, a, null);
        }
      }
      row.add(Integer.toString(grandTotal));
      CSVUtils.writeLine(writerVolumesData, row);
    }
    writerVolumesData.flush();
    writerVolumesData.close();
    logger.info("Day nr " + day + ": Pedestrian volumes successfully exported.");
  }

  /** Volume in one agent x scenario cell; key is "<agent>_<scenario>" (or "<agent>" with no scenario). */
  private static int cellVolume(Map<String, Integer> edgeVolumes, Enum<?> agent, Enum<?> scenario) {
    String key = (scenario != null) ? agent + "_" + scenario : agent.toString();
    return edgeVolumes.getOrDefault(key, 0);
  }

  /** Whether an {@code Hour} scenario falls in the night window (matching the isDark boundary). */
  private static boolean isNightHour(Enum<?> hour) {
    return hour instanceof StringEnum.Hour && TimePars.isNight(((StringEnum.Hour) hour).ordinal());
  }

  /**
   * Saves pedestrian volumes data to a CSV file.
   *
   * @param day The simulated day from 1 onwards.
   * @throws Exception If there is an error while saving the data.
   */
  public void saveRoutes(int day) throws Exception {

    outputRoutesDirectory = verifyOutputPath(outputRoutesDirectory, "routes");
    outputRoutesDirectory += File.separator + currentDate + "_" + job + "_" + day;
    VectorLayer routes = new VectorLayer();
    nrColumns = 0;

    for (RouteData routeData : flowHandler.routesData) {
      MasonGeometry masonGeometry = new MasonGeometry(routeData.lineGeometry);
      masonGeometry.addIntegerAttribute("O", routeData.origin);
      masonGeometry.addIntegerAttribute("D", routeData.destination);
      masonGeometry.addAttribute("scenario", routeData.scenario);
      formRouteAttributes(masonGeometry, routeData);
      routes.addGeometry(masonGeometry);
    }

    // Avoid geometries without needed columns' values filled in.
    if (nrColumns > 0) {
      for (int counter = 1; counter < nrColumns; counter++) {
        List<MasonGeometry> routeGeometries = routes.getGeometries();
        for (MasonGeometry route : routeGeometries) {
          if (!route.hasAttribute("edgeIDs_" + counter)) {
            route.addAttribute("edgeIDs_" + counter, "None");
          }
        }
      }
    }
    if (routes.getGeometries().isEmpty()) {
      logger.warning("No routes were found to save for day " + day);
      return;
    }
    ShapeFileExporter.write(outputRoutesDirectory, routes);
  }

  public void saveCognitiveMapsData(int day, String[] scenarios) throws Exception {
    outputCognitiveMapDirectory = verifyOutputPath(outputCognitiveMapDirectory, "knownEdges");
    outputCognitiveMapDirectory += File.separator + currentDate + "_" + day + "_" + job + ".csv";
    try (FileWriter writer = new FileWriter(outputCognitiveMapDirectory)) {
      writeKnownByCsv(writer, "edgeID", new HashMap<>(flowHandler.knownEdgesMap));
    }
    logger.info("Day nr " + day + ": Cognitive Maps Data successfully exported.");
  }

  public void saveKnownLandmarksData(int day, String[] scenarios) throws Exception {
    outputLandmarkCognitiveMapDirectory =
        verifyOutputPath(outputLandmarkCognitiveMapDirectory, "knownLandmarks");
    outputLandmarkCognitiveMapDirectory +=
        File.separator + currentDate + "_" + day + "_" + job + ".csv";
    try (FileWriter writer = new FileWriter(outputLandmarkCognitiveMapDirectory)) {
      writeKnownByCsv(writer, "buildingID", new HashMap<>(flowHandler.knownLandmarksMap));
    }
    logger.info("Day nr " + day + ": Landmarks Cognitive Maps Data successfully exported.");
  }

  /**
   * Writes a "known-by" CSV (cognitive map / landmark knowledge) split by agent type, with a total.
   * Cognitive maps are an end-of-day knowledge snapshot, so they carry no time (hour/day-night)
   * dimension — only the agent type matters (e.g. learner vs non-learner). A single DEFAULT agent
   * type collapses to just the total.
   */
  private void writeKnownByCsv(
      FileWriter writer, String idHeader, Map<Integer, Map<String, Integer>> dataMap)
      throws Exception {
    Enum<?>[] agentValues = flowHandler.getAgentScenarioValues();
    Enum<?>[] agents =
        (agentValues != null) ? agentValues : new Enum<?>[] {StringEnum.Default.DEFAULT};
    boolean perAgent = !(agents.length == 1 && agents[0] == StringEnum.Default.DEFAULT);

    List<String> headers = new ArrayList<>();
    headers.add(idHeader);
    if (perAgent) {
      for (Enum<?> a : agents) headers.add(a.toString());
    }
    headers.add("total");
    CSVUtils.writeLine(writer, headers);

    for (Map.Entry<Integer, Map<String, Integer>> entry : dataMap.entrySet()) {
      Map<String, Integer> m = entry.getValue();
      List<String> row = new ArrayList<>();
      row.add(Integer.toString(entry.getKey()));
      int total = 0;
      for (Enum<?> a : agents) {
        int v = m.getOrDefault(a.toString(), 0);
        if (perAgent) row.add(Integer.toString(v));
        total += v;
      }
      row.add(Integer.toString(total));
      CSVUtils.writeLine(writer, row);
    }
  }

  /**
   * Verifies and creates the specified output directory.
   *
   * @param directory The directory path to be created.
   * @return
   */
  private String verifyOutputPath(String directory, String specifier) {

    directory = outputDirectory + File.separator + specifier;
    directory = String.format(directory, userName);

    File outputCheck = new File(directory);
    if (!outputCheck.exists()) {
      try {
        // Create the output path directory and its parent directories recursively
        Files.createDirectories(Paths.get(directory));
      } catch (IOException e) {
        e.printStackTrace();
      }
    }
    return directory;
  }

  /**
   * Forms route attributes and handles splitting long edgeIDs strings.
   *
   * @param masonGeometry The MasonGeometry object representing a route.
   * @param routeData The route data associated with the route.
   */
  private static void formRouteAttributes(MasonGeometry masonGeometry, RouteData routeData) {
    String edgeIDs = ArrayUtils.toString(routeData.edgeIDsSequence);

    if (edgeIDs.length() <= FIELD_LIMIT) {
      masonGeometry.addAttribute("edgeIDs_0", edgeIDs);
    } else {
      String remainingEdges = edgeIDs;
      for (int counter = 0; remainingEdges.length() > 0; counter++) {
        if (counter >= nrColumns) {
          nrColumns += 1;
        }

        String currentPart;
        if (remainingEdges.length() > FIELD_LIMIT) {
          currentPart = remainingEdges.substring(0, FIELD_LIMIT);
          remainingEdges = remainingEdges.substring(FIELD_LIMIT);
        } else {
          currentPart = remainingEdges;
          remainingEdges = "";
        }
        masonGeometry.addAttribute("edgeIDs_" + counter, currentPart);
      }
    }
  }
}
