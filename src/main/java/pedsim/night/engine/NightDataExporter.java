package pedsim.night.engine;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.Locale;
import java.util.logging.Logger;
import org.locationtech.jts.geom.Coordinate;
import pedsim.core.engine.TripRouteRecorder;
import pedsim.core.utilities.LoggerUtil;
import pedsim.core.utilities.StringEnum;

/**
 * Plain-data result files for a night run, written alongside (not instead of) the standard
 * {@code streetVolumes} / {@code routes} exports.
 *
 * <p>These are for analyses that need per-hour, per-agent-type street loads without reconstructing
 * them from trip paths, and that must stay small enough to transfer off the run machine:
 *
 * <ul>
 *   <li>{@code edge_volume.csv} — {@code edgeID,hour,vuln_count,norm_count}, one row per (edge,
 *       clock hour) that carried at least one pedestrian. The long-format view of the wide
 *       {@code streetVolumes} CSV, with the vulnerability split already applied.
 *   <li>{@code trips.csv} — one row per completed trip:
 *       {@code trip_id,agent_id,start_step,end_step,vulnerable,path_length_m,mean_lux,edge_ids}.
 *       The path is carried as the traversed edgeID sequence, not as coordinates.
 * </ul>
 *
 * <p>Both are non-spatial; the matching geometry is the city's edges layer, which already carries
 * {@code edgeID} and {@code mean_lux} in the projected city CRS.
 */
public final class NightDataExporter {

  private static final Logger logger = LoggerUtil.getLogger();

  private NightDataExporter() {}

  /** Writes both files under {@code outputs/PedSimCityNight/data/}. */
  public static void export(int job, List<TripRouteRecorder.TripRecord> trips, Path volumesFile) {
    Path dir = Paths.get("outputs", "PedSimCityNight", "data");
    try {
      Files.createDirectories(dir);
    } catch (IOException e) {
      logger.severe("[NightDataExporter] Could not create " + dir + ": " + e.getMessage());
      return;
    }
    // Job 0 keeps the plain names; further jobs are suffixed so replications do not overwrite.
    String suffix = (job == 0) ? "" : "_job" + job;
    writeEdgeVolumes(dir.resolve("edge_volume" + suffix + ".csv"), volumesFile);
    writeTrips(dir.resolve("trips" + suffix + ".csv"), trips);
  }

  /**
   * Long-format hourly volumes, read from the wide {@code streetVolumes} CSV the standard exporter
   * has just written rather than from {@code volumesMap}: the in-memory map is reset at every day
   * boundary, so the CSV is the accumulated, exported truth and the two files cannot disagree.
   */
  private static void writeEdgeVolumes(Path target, Path source) {
    if (source == null) {
      logger.warning(
          "[NightDataExporter] No streetVolumes CSV found; skipping " + target.getFileName() + ".");
      return;
    }

    StringEnum.Hour[] hours = StringEnum.Hour.values();
    long rows = 0;
    try (BufferedReader reader = Files.newBufferedReader(source, StandardCharsets.UTF_8);
        BufferedWriter writer = Files.newBufferedWriter(target, StandardCharsets.UTF_8)) {

      String headerLine = reader.readLine();
      if (headerLine == null) {
        logger.warning("[NightDataExporter] " + source + " is empty; skipping.");
        return;
      }
      List<String> header = List.of(headerLine.split(",", -1));
      int edgeIdx = header.indexOf("edgeID");
      int[] vulnIdx = new int[hours.length];
      int[] normIdx = new int[hours.length];
      for (int h = 0; h < hours.length; h++) {
        vulnIdx[h] = header.indexOf(StringEnum.Vulnerable.VULNERABLE + "_" + hours[h]);
        normIdx[h] = header.indexOf(StringEnum.Vulnerable.NON_VULNERABLE + "_" + hours[h]);
      }
      if (edgeIdx < 0 || vulnIdx[0] < 0 || normIdx[0] < 0) {
        logger.warning(
            "[NightDataExporter] "
                + source.getFileName()
                + " has no per-hour vulnerability columns (header="
                + header
                + "); skipping "
                + target.getFileName()
                + ".");
        return;
      }

      writer.write("edgeID,hour,vuln_count,norm_count\n");
      String line;
      while ((line = reader.readLine()) != null) {
        if (line.isBlank()) continue;
        String[] cells = line.split(",", -1);
        String edgeID = cells[edgeIdx];
        for (int h = 0; h < hours.length; h++) {
          int vuln = cell(cells, vulnIdx[h]);
          int norm = cell(cells, normIdx[h]);
          if (vuln == 0 && norm == 0) continue; // rows only for non-zero
          // Hour bucket h01..h24 covers clock hours 0..23, so the bucket index is the clock hour.
          writer.write(edgeID + "," + h + "," + vuln + "," + norm + "\n");
          rows++;
        }
      }
    } catch (IOException e) {
      logger.severe("[NightDataExporter] Failed to write " + target + ": " + e.getMessage());
      return;
    }
    logger.info("[NightDataExporter] " + rows + " rows -> " + target);
  }

  /** One row per completed trip, with the path as an edgeID sequence. */
  private static void writeTrips(Path target, List<TripRouteRecorder.TripRecord> trips) {
    try (BufferedWriter writer = Files.newBufferedWriter(target, StandardCharsets.UTF_8)) {
      writer.write(
          "trip_id,agent_id,start_step,end_step,vulnerable,path_length_m,mean_lux,edge_ids\n");
      int tripId = 0;
      for (TripRouteRecorder.TripRecord trip : trips) {
        StringBuilder edges = new StringBuilder();
        for (int i = 0; i < trip.edgeIds.size(); i++) {
          if (i > 0) {
            edges.append(';');
          }
          edges.append(trip.edgeIds.get(i));
        }
        writer.write(
            String.format(
                Locale.ROOT,
                "%d,%d,%.0f,%.0f,%b,%.1f,%s,%s%n",
                tripId++,
                trip.agentId,
                trip.startStep,
                trip.endStep,
                trip.vulnerable,
                pathLengthMetres(trip.pathCoords),
                Double.isNaN(trip.meanLux) ? "" : String.format(Locale.ROOT, "%.2f", trip.meanLux),
                edges));
      }
    } catch (IOException e) {
      logger.severe("[NightDataExporter] Failed to write " + target + ": " + e.getMessage());
      return;
    }
    logger.info("[NightDataExporter] " + trips.size() + " trips -> " + target);
  }

  private static int cell(String[] cells, int index) {
    if (index < 0 || index >= cells.length) {
      return 0;
    }
    String raw = cells[index].trim();
    if (raw.isEmpty()) {
      return 0;
    }
    try {
      return Integer.parseInt(raw);
    } catch (NumberFormatException e) {
      return 0;
    }
  }

  /**
   * Euclidean path length. Coordinates are in the projected city CRS (metres), so no conversion is
   * applied — the same convention as {@code TripDiagnostic}.
   */
  private static double pathLengthMetres(List<Coordinate> coords) {
    if (coords == null || coords.size() < 2) {
      return 0.0;
    }
    double total = 0.0;
    for (int i = 0; i < coords.size() - 1; i++) {
      Coordinate a = coords.get(i);
      Coordinate b = coords.get(i + 1);
      double dx = b.x - a.x;
      double dy = b.y - a.y;
      total += Math.sqrt(dx * dx + dy * dy);
    }
    return total;
  }
}
