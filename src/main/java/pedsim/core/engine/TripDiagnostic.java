package pedsim.core.engine;

import java.io.FileWriter;
import java.io.IOException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.List;
import java.util.logging.Logger;
import org.locationtech.jts.geom.Coordinate;
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
 * <p>Call {@link #save(String)} once after the simulation finishes.
 */
public class TripDiagnostic {

    private static final Logger logger = LoggerUtil.getLogger();
    private static final DateTimeFormatter FMT = DateTimeFormatter.ofPattern("'Day 'D' 'HH:mm");

    // Coordinates are in a projected CRS (EPSG:3003, units = metres).
    // Raw Euclidean distance is therefore already in metres – no conversion needed.

    // -----------------------------------------------------------------------

    /**
     * Writes a clean diagnostic CSV from the trips already stored in
     * {@link TripRouteRecorder}.
     *
     * @param filename absolute or relative path for the output file
     */
    public static void save(String filename) {
        List<TripRouteRecorder.TripRecord> trips = TripRouteRecorder.getRecords();
        logger.info("[TripDiagnostic] Writing " + trips.size() + " trips to " + filename);

        try (FileWriter fw = new FileWriter(filename)) {

            // Header
            fw.write("agent_id,start_time,end_time,duration_min,distance_m,nodes_walked,edges_walked,vulnerable\n");

            for (TripRouteRecorder.TripRecord t : trips) {

                // --- times ------------------------------------------------
                String startTime = stepToTime(t.startStep);
                String endTime   = stepToTime(t.endStep);

                // Duration in minutes
                double durationSteps = t.endStep - t.startStep;
                long durationMin     = Math.round(durationSteps * TimePars.STEP_DURATION / 60.0);

                // --- distance ---------------------------------------------
                double distanceM = computeDistanceMetres(t.pathCoords);

                // --- node / edge lists ------------------------------------
                String nodes = joinInts(t.nodeIds);
                String edges = joinInts(t.edgeIds);

                fw.write(String.format("%d,%s,%s,%d,%.1f,%s,%s,%b%n",
                        t.agentId,
                        startTime,
                        endTime,
                        durationMin,
                        distanceM,
                        nodes,
                        edges,
                        t.vulnerable));
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
    private static String stepToTime(double step) {
        try {
            LocalDateTime dt = TimePars.getTime(step);
            // Day number: count from 1
            int day = (int) (step * TimePars.STEP_DURATION / 86_400.0) + 1;
            return String.format("Day%d %02d:%02d",
                    day,
                    dt.getHour(),
                    dt.getMinute());
        } catch (Exception e) {
            return String.valueOf(step);
        }
    }

    /**
     * Sums Euclidean segment lengths over the path coordinate list.
     * Coordinates are in a projected CRS (EPSG:3003), so units are already metres.
     * Raw Euclidean distance is used directly – no degree-to-metre conversion needed.
     */
    private static double computeDistanceMetres(List<Coordinate> coords) {
        if (coords == null || coords.size() < 2) return 0.0;
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

    /** Joins a list of integers with semicolons. */
    private static String joinInts(List<Integer> ids) {
        if (ids == null || ids.isEmpty()) return "";
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < ids.size(); i++) {
            if (i > 0) sb.append(';');
            sb.append(ids.get(i));
        }
        return sb.toString();
    }

    /**
     * Generates a side-by-side comparison of vulnerable and normal twins routes, durations, distances.
     */
    public static void saveABTestComparison(String filename) {
        List<TripRouteRecorder.TripRecord> allTrips = TripRouteRecorder.getRecords();
        logger.info("[TripDiagnostic] Writing A/B test comparison to " + filename);

        try (FileWriter fw = new FileWriter(filename)) {
            // Header
            fw.write("pair_id,trip_index,start_node,dest_node,vuln_start_time,vuln_end_time,vuln_duration_min,vuln_distance_m,vuln_route,normal_start_time,normal_end_time,normal_duration_min,normal_distance_m,normal_route,routes_differ\n");

            for (int pairId = 0; pairId < 72; pairId++) {
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

                    String vStart = vt != null ? stepToTime(vt.startStep) : "";
                    String vEnd = vt != null ? stepToTime(vt.endStep) : "";
                    long vDur = vt != null ? Math.round((vt.endStep - vt.startStep) * TimePars.STEP_DURATION / 60.0) : -1;
                    double vDist = vt != null ? computeDistanceMetres(vt.pathCoords) : -1.0;
                    String vRoute = vt != null ? joinInts(vt.nodeIds) : "";

                    String nStart = nt != null ? stepToTime(nt.startStep) : "";
                    String nEnd = nt != null ? stepToTime(nt.endStep) : "";
                    long nDur = nt != null ? Math.round((nt.endStep - nt.startStep) * TimePars.STEP_DURATION / 60.0) : -1;
                    double nDist = nt != null ? computeDistanceMetres(nt.pathCoords) : -1.0;
                    String nRoute = nt != null ? joinInts(nt.nodeIds) : "";

                    boolean routesDiffer = true;
                    if (vt != null && nt != null) {
                        routesDiffer = !vt.nodeIds.equals(nt.nodeIds);
                    }

                    fw.write(String.format("%d,%d,%d,%d,%s,%s,%s,%.1f,%s,%s,%s,%s,%.1f,%s,%b%n",
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
            logger.info("[TripDiagnostic] A/B test comparison saved successfully → " + filename);
        } catch (IOException e) {
            logger.severe("[TripDiagnostic] Failed to write A/B comparison: " + e.getMessage());
        }
    }
}
