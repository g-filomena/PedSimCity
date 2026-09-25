package pedsim.core.engine;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.time.LocalDateTime;
import java.util.List;
import java.util.logging.Logger;
import pedsim.core.parameters.TimePars;
import pedsim.core.utilities.LoggerUtil;

/**
 * Simple diagnostic CSV recorder for agent trips.
 *
 * <p>Columns written:
 * <ul>
 *   <li>agent_id        – integer agent identifier</li>
 *   <li>start_time      – human-readable time (Day D HH:mm) when walking began</li>
 *   <li>end_time        – human-readable time when walking finished</li>
 *   <li>duration_min    – walking duration in minutes</li>
 *   <li>distance_m      – path length in metres (Euclidean, coordinates are projected metres EPSG:3003)</li>
 *   <li>nodes_walked    – semicolon-separated list of node IDs visited</li>
 *   <li>edges_walked    – semicolon-separated list of edge IDs traversed</li>
 *   <li>vulnerable      – true/false</li>
 * </ul>
 *
 * <p>Call {@link #save(String, List)} once after the simulation finishes.
 */
public class TripDiagnostic {

  private static final Logger logger = LoggerUtil.getLogger();

  // Coordinates are in a projected CRS (EPSG:3003, units = metres).
  // Raw Euclidean distance is therefore already in metres – no conversion needed.

  // -----------------------------------------------------------------------

  /** Keeps job 0's established filename and gives later replicates separate files. */
  public static String jobFilename(String filename, int job) {
    if (job == 0) return filename;
    int dot = filename.lastIndexOf('.');
    return dot < 0
        ? filename + "_job" + job
        : filename.substring(0, dot) + "_job" + job + filename.substring(dot);
  }

  /** Resolves relative filenames under outputs/, or preserves an explicit absolute path. */
  public static String outputsPath(String filename) {
    if (java.nio.file.Path.of(filename).isAbsolute()) return filename;
    File dir = new File("outputs");
    if (!dir.exists()) dir.mkdirs();
    return "outputs" + File.separator + filename;
  }

  /** Writes diagnostics from the completed trips of one job. */
  public static void save(String filename, List<TripRouteRecorder.TripRecord> trips) {
    String path = outputsPath(filename);
    logger.info("[TripDiagnostic] Writing " + trips.size() + " trips to " + path);

    try (FileWriter fw = new FileWriter(path)) {

      // Header
      fw.write(
          "agent_id,start_time,end_time,duration_min,distance_m,nodes_walked,edges_walked,vulnerable,mean_lux\n");

      for (TripRouteRecorder.TripRecord t : trips) {

        // --- times ------------------------------------------------
        String startTime = stepToTime(t.startStep);
        String endTime = stepToTime(t.endStep);

        // Duration in minutes
        double durationSteps = t.endStep - t.startStep;
        long durationMin = Math.round(durationSteps * TimePars.STEP_DURATION / 60.0);

        // --- node / edge lists ------------------------------------
        String nodes = joinInts(t.nodeIds);
        String edges = joinInts(t.edgeIds);

        double luxVal = Double.isNaN(t.meanLux) ? 0.0 : t.meanLux;
        fw.write(
            String.format(
                java.util.Locale.ROOT,
                "%d,%s,%s,%d,%.1f,%s,%s,%b,%.2f%n",
                t.agentId,
                startTime,
                endTime,
                durationMin,
                t.distanceMetres,
                nodes,
                edges,
                t.vulnerable,
                luxVal));
      }

      logger.info("[TripDiagnostic] Saved successfully → " + filename);

    } catch (IOException e) {
      logger.severe("[TripDiagnostic] Failed to write: " + e.getMessage());
    }
  }

  // -----------------------------------------------------------------------
  // Helpers
  // -----------------------------------------------------------------------

  /**
   * Converts a simulation step number into a human-readable "Day D HH:mm" string.
   */
  public static String stepToTime(double step) {
    try {
      LocalDateTime dt = TimePars.getTime(step);
      // Day number: count from 1
      int day = (int) (step * TimePars.STEP_DURATION / 86_400.0) + 1;
      return String.format(
          java.util.Locale.ROOT, "Day%d %02d:%02d", day, dt.getHour(), dt.getMinute());
    } catch (Exception e) {
      return String.valueOf(step);
    }
  }

  /** Joins a list of integers with semicolons. */
  public static String joinInts(List<Integer> ids) {
    if (ids == null || ids.isEmpty()) return "";
    StringBuilder sb = new StringBuilder();
    for (int i = 0; i < ids.size(); i++) {
      if (i > 0) sb.append(';');
      sb.append(ids.get(i));
    }
    return sb.toString();
  }
}
