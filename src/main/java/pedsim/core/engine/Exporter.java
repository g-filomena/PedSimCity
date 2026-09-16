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
import pedsim.core.utilities.LoggerUtil;
import pedsim.core.utilities.RouteData;
import pedsim.core.utilities.StringEnum;
import sim.field.geo.VectorLayer;
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

  /** The volumes CSV written by the latest daily export, or null before the first one. */
  public String lastVolumesFile;

  protected static final Logger logger = LoggerUtil.getLogger();
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

    lastVolumesFile =
        verifyOutputPath("streetVolumes")
            + File.separator
            + currentDate
            + "_"
            + job
            + "_"
            + day
            + ".csv";
    final FileWriter writerVolumesData = new FileWriter(lastVolumesFile);

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
      headers.addAll(extraHourlyHeaders(agents, simValues, perAgent));
      if (perAgent) {
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
        row.addAll(extraHourlyValues(ev, agents, simValues, perAgent, day));
        if (perAgent) {
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

  /**
   * Columns a module adds to the hourly volumes file, after the hour totals and before the
   * per-agent totals. None here: the generic file is hours, agent types and totals.
   *
   * @param agents the agent types being exported
   * @param hours the hour columns
   * @param perAgent whether the file carries per-agent columns
   * @return the extra headers, in file order
   */
  protected List<String> extraHourlyHeaders(Enum<?>[] agents, Enum<?>[] hours, boolean perAgent) {
    return List.of();
  }

  /**
   * One edge's values for {@link #extraHourlyHeaders}, in the same order and of the same length.
   *
   * @param edgeVolumes that edge's cells, keyed "&lt;agent&gt;_&lt;hour&gt;"
   * @param agents the agent types being exported
   * @param hours the hour columns
   * @param perAgent whether the file carries per-agent columns
   * @param day the simulated day, from 1
   * @return the extra values, in file order
   */
  protected List<String> extraHourlyValues(
      Map<String, Integer> edgeVolumes,
      Enum<?>[] agents,
      Enum<?>[] hours,
      boolean perAgent,
      int day) {
    return List.of();
  }

  /** Volume in one agent x scenario cell; key is "<agent>_<scenario>" (or "<agent>" with no scenario). */
  protected static int cellVolume(
      Map<String, Integer> edgeVolumes, Enum<?> agent, Enum<?> scenario) {
    String key = (scenario != null) ? agent + "_" + scenario : agent.toString();
    return edgeVolumes.getOrDefault(key, 0);
  }

  /**
   * Saves pedestrian volumes data to a CSV file.
   *
   * @param day The simulated day from 1 onwards.
   * @throws Exception If there is an error while saving the data.
   */
  public void saveRoutes(int day) throws Exception {

    String routesFile =
        verifyOutputPath("routes") + File.separator + currentDate + "_" + job + "_" + day;
    VectorLayer routes = new VectorLayer();

    for (RouteData routeData : flowHandler.routesData) {
      MasonGeometry masonGeometry = new MasonGeometry(routeData.lineGeometry);
      masonGeometry.addIntegerAttribute("O", routeData.origin);
      masonGeometry.addIntegerAttribute("D", routeData.destination);
      masonGeometry.addAttribute("scenario", routeData.scenario);
      formRouteAttributes(masonGeometry, routeData);
      routes.addGeometry(masonGeometry);
    }

    if (routes.isEmpty()) {
      logger.warning("No routes were found to save for day " + day);
      return;
    }
    // Single-file GeoPackage (was a 3-file ESRI shapefile). GeoPackage TEXT columns have no length
    // limit, so the full edgeIDs sequence lives in a single column (the old shapefile format had to
    // split it across edgeIDs_0..n to stay under the 254-char DBF field limit).
    VectorLayer.writeGPKG(routesFile, routes);
  }

  public void saveCognitiveMapsData(int day, String[] scenarios) throws Exception {
    String knownEdgesFile =
        verifyOutputPath("knownEdges")
            + File.separator
            + currentDate
            + "_"
            + day
            + "_"
            + job
            + ".csv";
    try (FileWriter writer = new FileWriter(knownEdgesFile)) {
      writeKnownByCsv(writer, "edgeID", new HashMap<>(flowHandler.knownEdgesMap));
    }
    logger.info("Day nr " + day + ": Cognitive Maps Data successfully exported.");
  }

  public void saveKnownLandmarksData(int day, String[] scenarios) throws Exception {
    String knownLandmarksFile =
        verifyOutputPath("knownLandmarks")
            + File.separator
            + currentDate
            + "_"
            + day
            + "_"
            + job
            + ".csv";
    try (FileWriter writer = new FileWriter(knownLandmarksFile)) {
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
  /**
   * The directory results of one kind go in, created if it does not exist.
   *
   * @param specifier the subfolder name, e.g. "streetVolumes"
   * @return the directory path
   */
  private String verifyOutputPath(String specifier) {
    String directory = String.format(outputDirectory + File.separator + specifier, userName);

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
   * Stores the full traversed edgeID sequence in a single {@code edgeIDs} attribute. GeoPackage TEXT
   * columns are unbounded, so no field-length splitting is needed (the previous ESRI shapefile
   * export had to split this across {@code edgeIDs_0..n} to stay under the 254-char DBF limit).
   *
   * @param masonGeometry The MasonGeometry object representing a route.
   * @param routeData The route data associated with the route.
   */
  private static void formRouteAttributes(MasonGeometry masonGeometry, RouteData routeData) {
    masonGeometry.addAttribute("edgeIDs", ArrayUtils.toString(routeData.edgeIDsSequence));
  }
}
