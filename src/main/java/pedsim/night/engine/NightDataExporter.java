package pedsim.night.engine;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.Locale;
import java.util.logging.Logger;
import pedsim.core.engine.TripDiagnostic;
import pedsim.core.engine.TripRouteRecorder;
import pedsim.core.parameters.TimePars;
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
                trip.distanceMetres,
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
   * Side-by-side comparison of each A/B pair: the vulnerable twin's route, duration and distance
   * against its non-vulnerable twin's, one row per trip index.
   *
   * <p>It reads a pairing only this module creates: {@code NightPopulate} spawns twins at
   * consecutive agent IDs, so {@code agentId / 2} is the pair.
   */
  public static void saveABTestComparison(
      String filename, List<TripRouteRecorder.TripRecord> allTrips) {
    String path = TripDiagnostic.outputsPath(filename);
    logger.info("[NightDataExporter] Writing A/B test comparison to " + path);

    try (FileWriter fw = new FileWriter(path)) {
      // Header
      fw.write(
          "pair_id,trip_index,start_node,dest_node,vuln_start_time,vuln_end_time,vuln_duration_min,vuln_distance_m,vuln_route,normal_start_time,normal_end_time,normal_duration_min,normal_distance_m,normal_route,routes_differ\n");

      // Derive pair IDs from recorded agents so configured populations are never truncated.
      java.util.SortedSet<Integer> pairIds = new java.util.TreeSet<>();
      for (TripRouteRecorder.TripRecord trip : allTrips) {
        pairIds.add(trip.agentId / 2);
      }
      for (int pairId : pairIds) {
        final int vId = pairId * 2;
        final int nId = pairId * 2 + 1;

        java.util.List<TripRouteRecorder.TripRecord> vulnTrips = new java.util.ArrayList<>();
        java.util.List<TripRouteRecorder.TripRecord> normalTrips = new java.util.ArrayList<>();

        for (TripRouteRecorder.TripRecord t : allTrips) {
          if (t.agentId == vId) {
            vulnTrips.add(t);
          } else if (t.agentId == nId) {
            normalTrips.add(t);
          }
        }

        vulnTrips.sort(java.util.Comparator.comparingDouble(t -> t.startStep));
        normalTrips.sort(java.util.Comparator.comparingDouble(t -> t.startStep));

        int maxTrips = Math.max(vulnTrips.size(), normalTrips.size());
        for (int k = 0; k < maxTrips; k++) {
          TripRouteRecorder.TripRecord vt = k < vulnTrips.size() ? vulnTrips.get(k) : null;
          TripRouteRecorder.TripRecord nt = k < normalTrips.size() ? normalTrips.get(k) : null;

          int startNode = -1;
          int destNode = -1;
          if (vt != null) {
            startNode = vt.originNodeId;
            destNode = vt.destNodeId;
          } else if (nt != null) {
            startNode = nt.originNodeId;
            destNode = nt.destNodeId;
          }

          String vStart = vt != null ? TripDiagnostic.stepToTime(vt.startStep) : "";
          String vEnd = vt != null ? TripDiagnostic.stepToTime(vt.endStep) : "";
          long vDur =
              vt != null
                  ? Math.round((vt.endStep - vt.startStep) * TimePars.STEP_DURATION / 60.0)
                  : -1;
          double vDist = vt != null ? vt.distanceMetres : -1.0;
          String vRoute = vt != null ? TripDiagnostic.joinInts(vt.nodeIds) : "";

          String nStart = nt != null ? TripDiagnostic.stepToTime(nt.startStep) : "";
          String nEnd = nt != null ? TripDiagnostic.stepToTime(nt.endStep) : "";
          long nDur =
              nt != null
                  ? Math.round((nt.endStep - nt.startStep) * TimePars.STEP_DURATION / 60.0)
                  : -1;
          double nDist = nt != null ? nt.distanceMetres : -1.0;
          String nRoute = nt != null ? TripDiagnostic.joinInts(nt.nodeIds) : "";

          boolean routesDiffer = true;
          if (vt != null && nt != null) {
            routesDiffer = !vt.nodeIds.equals(nt.nodeIds);
          }

          fw.write(
              String.format(
                  java.util.Locale.ROOT,
                  "%d,%d,%d,%d,%s,%s,%s,%.1f,%s,%s,%s,%s,%.1f,%s,%b%n",
                  pairId,
                  k,
                  startNode,
                  destNode,
                  vStart,
                  vEnd,
                  vDur >= 0 ? String.valueOf(vDur) : "",
                  vDist >= 0 ? vDist : 0.0,
                  vRoute,
                  nStart,
                  nEnd,
                  nDur >= 0 ? String.valueOf(nDur) : "",
                  nDist >= 0 ? nDist : 0.0,
                  nRoute,
                  routesDiffer));
        }
      }
      logger.info("[NightDataExporter] A/B test comparison saved successfully → " + filename);
    } catch (IOException e) {
      logger.severe("[NightDataExporter] Failed to write A/B comparison: " + e.getMessage());
    }
  }
}
