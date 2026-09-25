package pedsim.activity.engine;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.time.LocalDate;
import java.time.format.DateTimeFormatter;
import java.util.logging.Logger;
import pedsim.core.engine.PedSimCity;
import pedsim.core.parameters.TimePars;
import pedsim.core.utilities.LoggerUtil;

/**
 * One row per simulated day for every module on the activity tier: legs against the day's budget,
 * metres, darkness exposure, commute shares and the trace's error counts.
 *
 * <p>Appended to {@code outputs/<app>/daySummary/<date>_<job>.csv} as each day ends.
 */
public final class DaySummary {

  private static final Logger logger = LoggerUtil.getLogger();

  private static final String[] HEADERS = {
    "date",
    "day",
    "job",
    "agents",
    "legs",
    "planned_m",
    "walked_m",
    "m_per_agent",
    "legs_dark",
    "mandatory_legs",
    "budget_legs",
    "chains_per_person",
    "legs_per_chain",
    "worker_walk_share",
    "student_walk_share",
    "discretionary_walk_share",
    "unusable_lengths",
    "band_widenings",
    "destination_fallbacks",
    "full_network_escalations",
    "incomplete_island_merges"
  };

  private DaySummary() {}

  /**
   * Appends the day's row and logs its darkness exposure: legs begun in the dark, and how many of
   * those a fixed {@code [20:00, 06:00)} window would not count as night.
   *
   * @param state the running state, with the day's trace still intact
   * @param appName the module's output folder name
   * @param day the day that has just finished, counting from 1
   */
  public static void append(PedSimCity state, String appName, int job, int day) {
    if (!(state instanceof PedSimCityActivity activityState)) {
      return;
    }

    LocalDate date = TimePars.SIMULATION_START_DATE.plusDays((long) day - 1);
    int agents = state.agentsList.size();
    double planned = state.trace().plannedRouteMeters();
    long legsDark = activityState.legsInDarkness.sum();
    long legs = state.trace().legsPlanned();

    ActivityTravelDemand demand =
        state.travelDemand() instanceof ActivityTravelDemand activityDemand ? activityDemand : null;

    String[] row = {
      date.toString(),
      Integer.toString(day),
      Integer.toString(job),
      Integer.toString(agents),
      Long.toString(legs),
      String.format("%.0f", planned),
      String.format("%.0f", state.trace().walkedRouteMeters()),
      String.format("%.0f", agents > 0 ? planned / agents : 0.0),
      Long.toString(legsDark),
      demand == null ? "" : String.format("%.0f", demand.mandatoryLegs()),
      demand == null ? "" : String.format("%.0f", demand.budgetLegs()),
      demand == null ? "" : String.format("%.4f", demand.discretionaryChainsPerPerson()),
      demand == null ? "" : String.format("%.2f", demand.legsPerChain()),
      demand == null ? "" : String.format("%.4f", demand.workerWalkShare()),
      demand == null ? "" : String.format("%.4f", demand.studentWalkShare()),
      String.format("%.4f", activityState.realisedWalkShare()),
      Long.toString(state.trace().unusableRouteLengths()),
      Long.toString(state.trace().destinationWidenings()),
      Long.toString(state.trace().destinationFallbacks()),
      Long.toString(state.trace().fullNetworkEscalations()),
      Long.toString(sim.graph.Islands.incompleteMerges())
    };

    write(appName, job, row);

    logger.info(String.format("day %s: %d of %d legs set off in darkness", date, legsDark, legs));
    logWalkShares(activityState, date);

    activityState.resetDarknessCounters();
    activityState.resetModeChoiceCounters();
  }

  /**
   * Logs the walking share the day produced, whole and cut two ways: by how long the trip is, which
   * is what the curve was fitted on, and by how far from the centre the traveller lives, which is
   * what the curve is being asked to explain.
   *
   * <p>A prediction to check. ISFORT puts walking at about a fifth of Italian trips; nothing in the
   * run is arranged to land there, and a gap is a statement about the curve rather than a number to
   * move.
   */
  private static void logWalkShares(PedSimCityActivity state, LocalDate date) {
    double overall = state.realisedWalkShare();
    if (Double.isNaN(overall)) {
      return;
    }
    logger.info(
        String.format(
            "day %s: %.1f%% of discretionary trips walked | by length %s | by ring %s",
            date,
            100.0 * overall,
            bandsOf(
                state.walkShareByBand(),
                state.legsOfferedByBand(),
                null,
                PedSimCityActivity.MODE_DISTANCE_BANDS,
                "m"),
            bandsOf(
                state.walkShareByRing(),
                state.legsOfferedByRing(),
                state.meanLegMetresByRing(),
                PedSimCityActivity.MODE_RING_BANDS,
                "m out")));
  }

  /**
   * Formats per-band shares with the count behind each one, and the mean leg length where it is
   * given: {@code <=500m 82.1% n=310 | >500m 4.2% n=12}.
   */
  private static String bandsOf(
      double[] shares, long[] counts, double[] meanMetres, double[] edges, String unit) {
    StringBuilder text = new StringBuilder();
    for (int i = 0; i < shares.length; i++) {
      if (i > 0) {
        text.append(" | ");
      }
      text.append(
          i < edges.length
              ? String.format("<=%.0f%s ", edges[i], unit)
              : String.format(">%.0f%s ", edges[edges.length - 1], unit));
      text.append(Double.isNaN(shares[i]) ? "-" : String.format("%.1f%%", 100.0 * shares[i]));
      text.append(String.format(" n=%d", counts[i]));
      if (meanMetres != null && !Double.isNaN(meanMetres[i])) {
        text.append(String.format(" mean %.0fm", meanMetres[i]));
      }
    }
    return text.toString();
  }

  /** Appends one row, writing the header first if the file is new. */
  private static void write(String appName, int job, String[] row) {
    File directory = new File("outputs" + File.separator + appName + File.separator + "daySummary");
    if (!directory.exists() && !directory.mkdirs()) {
      logger.warning("Could not create " + directory + "; no day summary written.");
      return;
    }
    String runDate = LocalDate.now().format(DateTimeFormatter.ofPattern("yyyyMMdd"));
    File file = new File(directory, runDate + "_" + job + ".csv");
    boolean isNew = !file.exists();

    try (FileWriter writer = new FileWriter(file, true)) {
      if (isNew) {
        writer.write(String.join(",", HEADERS));
        writer.write(System.lineSeparator());
      }
      writer.write(String.join(",", row));
      writer.write(System.lineSeparator());
    } catch (IOException e) {
      logger.warning("Could not append the day summary: " + e.getMessage());
    }
  }
}
