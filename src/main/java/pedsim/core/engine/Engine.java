package pedsim.core.engine;

import java.awt.Desktop;
import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.logging.Logger;
import java.util.stream.IntStream;
import pedsim.core.parameters.Pars;
import pedsim.core.parameters.TimePars;
import pedsim.core.utilities.LoggerUtil;
import pedsim.core.website.GeoJsonExporter;
import pedsim.core.website.HtmlExporter;
import sim.util.geo.Utilities;

public class Engine {

  private static final Logger logger = LoggerUtil.getLogger();

  protected final StateFactory stateFactory;
  protected final long baseSeed;

  /** Records a compact agent-position snapshot at every simulation step. */
  private TrajectoryRecorder trajectoryRecorder;

  @FunctionalInterface
  public interface StateFactory {
    PedSimCity create(long seed, int job, ScenarioConfig scenarioConfig);
  }

  /**
   * Seeded from {@link Pars#resolvedSeed()}.
   *
   * <p>This is the constructor every headless run reaches: {@code SimulationLauncher} goes through
   * {@code module.createEngine()}, and every module builds its engine here. Seeding it from anything
   * other than {@link Pars#resolvedSeed()} puts the whole run beyond reach of its configured seed,
   * however carefully the generators below derive from it.
   */
  public Engine(StateFactory stateFactory) {
    this(stateFactory, Pars.resolvedSeed());
  }

  public Engine(StateFactory stateFactory, long baseSeed) {
    this.stateFactory = stateFactory;
    this.baseSeed = baseSeed;
  }

  public synchronized void runJobs(ScenarioConfig scenarioConfig, boolean parallel)
      throws Exception {
    if (SimulationStateStore.getInstance().running) {
      logger.warning("Simulation is already running! Ignoring new run request.");
      return;
    }

    try {
      SimulationStateStore.getInstance().reset();
      SimulationStateStore.getInstance().running = true;

      clearStaticData();
      Pars.setSimulationParameters();
      afterSetParameters();

      createImporter().importFiles();

      // Export road network as GeoJSON once so the browser map can draw it
      SimulationStateStore.getInstance()
          .setRoadsGeoJson(GeoJsonExporter.exportRoads(PedSimCity.roads));

      prepareEnvironment();
      logger.info("Environment prepared. About to start simulation (base seed " + baseSeed + ")");

      // Module hook: a diagnostic that needs the prepared city but no simulated days. Calibrating
      // where workplaces go, for instance, depends on the census homes and the WORK tags and on
      // nothing a simulated day produces - running one to find out costs minutes and adds noise.
      if (runDiagnosticsInstead()) {
        return;
      }

      jobTotals.clear();
      boolean runParallel = parallel && supportsParallel();
      if (parallel && !runParallel) {
        logger.info("This module does not support parallel jobs; running them sequentially.");
      }

      if (runParallel) {
        IntStream.range(0, Pars.jobs)
            .parallel()
            .forEach(
                jobNr -> {
                  try {
                    Engine engine = createWorkerEngine(); // one engine per worker
                    logger.info("Executing Job nr.: " + jobNr);
                    engine.executeJob(jobNr, scenarioConfig);
                  } catch (Exception e) {
                    throw new RuntimeException("Error executing parallel job " + jobNr, e);
                  }
                });
      } else {
        for (int jobNr = 0; jobNr < Pars.jobs; jobNr++) {
          logger.info("Executing Job nr.: " + jobNr);
          executeJob(jobNr, scenarioConfig);
        }
      }

      reportReplicates();

    } finally {
      SimulationStateStore.getInstance().running = false;
      SimulationStateStore.getInstance().finished = true;
    }
  }

  protected void clearStaticData() {
    PedSimCity.clearStaticData();
  }

  protected Import createImporter() {
    return new Import();
  }

  /**
   * Prepares the simulation environment. Subclasses override to run their module-specific
   * preparation (data joins on top of the core infrastructure).
   */
  protected void prepareEnvironment() {
    Environment.prepare();
  }

  /**
   * Module hook: run a diagnostic against the prepared environment and skip the simulation.
   *
   * @return true when a diagnostic ran and no jobs should be executed
   */
  protected boolean runDiagnosticsInstead() {
    return false;
  }

  protected Engine createWorkerEngine() {
    return new Engine(stateFactory, baseSeed);
  }

  /**
   * Whether this module's jobs may run in parallel. Modules with shared mutable state that is not
   * thread-safe override this to force sequential runs even when {@code Pars.parallel} is set.
   */
  protected boolean supportsParallel() {
    return true;
  }

  public void executeJob(int job, ScenarioConfig scenarioConfig) throws Exception {
    int currentDay = 0;

    long seed = seedForJob(job);
    PedSimCity state = stateFactory.create(seed, job, scenarioConfig);

    // Clear and initialise the trajectory recorders for this job
    TripRouteRecorder.clear();
    trajectoryRecorder = new TrajectoryRecorder(state);

    state.start();

    onJobStarted(job, state, scenarioConfig);

    logger.info("---------- Beginning day Nr " + (currentDay + 1));
    AgentReleaseManager currentDayReleaseManager = new AgentReleaseManager(state, currentDay + 1);

    java.util.Map<Integer, java.util.Map<String, Integer>> finalVolumesMap =
        new java.util.HashMap<>();

    try {
      double nextAgentRelease = 1.0;

      while (continueSimulation(state)) {
        onJobStep(job, state, scenarioConfig);

        double steps = state.schedule.getSteps();

        onStepUpdate(state, steps);

        if (SimulationStateStore.getInstance().running && Pars.stepDelayMs > 0) {
          try {
            Thread.sleep(Pars.stepDelayMs);
          } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
          }
        }

        if (SimulationStateStore.getInstance().stopRequested) {
          logger.info("Stop requested by dashboard - ending simulation.");
          break;
        }

        String simTime = TimePars.getTime(steps).toLocalTime().toString();
        SimulationStateStore.getInstance()
            .updateStep(
                (int) steps, simTime, state.agentsWalking.size(), state.agentsAtHome.size(), 0);

        if (Pars.exportHtmlDashboard) {
          trajectoryRecorder.record((long) steps);
        }

        if (isNextDay(steps, currentDay)) {
          state.flowHandler.exportFlowsData(currentDay + 1);
          exportCognitiveData(state, currentDay + 1);
          onDayFinished(state, job, currentDay + 1);
          currentDay++;

          currentDayReleaseManager.close();

          if (currentDay % 6 == 0) {
            handleEndWeek(state, job, scenarioConfig);
          }

          logger.info("---------- Beginning day Nr " + (currentDay + 1));
          currentDayReleaseManager = new AgentReleaseManager(state, currentDay + 1);
        }

        if (steps >= nextAgentRelease) {
          currentDayReleaseManager.releaseAgents(steps);
          nextAgentRelease += TimePars.releaseAgentsEverySteps;
        }
      }

      // Capture volumesMap before exportFlowsData clears it for the HTML dashboard
      for (java.util.Map.Entry<Integer, java.util.Map<String, Integer>> entry :
          state.flowHandler.volumesMap.entrySet()) {
        finalVolumesMap.put(entry.getKey(), new java.util.HashMap<>(entry.getValue()));
      }

      state.flowHandler.exportFlowsData(currentDay + 1);
      exportCognitiveData(state, currentDay + 1);
      onDayFinished(state, job, currentDay + 1);

    } finally {
      currentDayReleaseManager.close();
    }

    onJobFinished(job, state, scenarioConfig);
    recordJobTotals(job, seed, state);
    state.finish();

    TripRouteRecorder.saveToFile("test_trips.csv");
    TripDiagnostic.save("trip_diagnostic.csv");

    // Module-specific plain-data exports (CSV / GeoPackage), independent of the HTML dashboard.
    onJobExport(job, state, currentDay + 1, finalVolumesMap);

    // Generate the self-contained HTML dashboard and open it in the browser
    if (Pars.exportHtmlDashboard) {
      generateAndOpenHtmlDashboard(job, state, currentDay, finalVolumesMap);
    } else {
      logger.info("[Engine] HTML dashboard export disabled (exportHtmlDashboard=false).");
    }
  }

  private void generateAndOpenHtmlDashboard(
      int job,
      PedSimCity state,
      int currentDay,
      java.util.Map<Integer, java.util.Map<String, Integer>> finalVolumesMap) {
    try {
      logger.info("[Engine] Compiling HTML dashboard for job " + job + "…");

      String htmlPath =
          HtmlExporter.export(
              currentDay + 1, // day (1-based)
              job,
              TripRouteRecorder.getRecords(),
              finalVolumesMap);

      if (htmlPath != null && Desktop.isDesktopSupported()) {
        Desktop.getDesktop().browse(new File(htmlPath).toURI());
        logger.info("[Engine] Opened dashboard in browser: " + htmlPath);
      }

    } catch (Exception e) {
      logger.warning("[Engine] Could not open HTML dashboard: " + e.getMessage());
    }
  }

  protected long seedForJob(int job) {
    return baseSeed + job;
  }

  /**
   * One row per finished job: job, seed, legs, planned metres, walked metres, agents. A run is
   * deterministic from its seed, so a single job has no error bar; job <i>n</i> uses {@code seed + n}.
   */
  private final List<double[]> jobTotals = Collections.synchronizedList(new ArrayList<>());

  private void recordJobTotals(int job, long seed, PedSimCity state) {
    jobTotals.add(
        new double[] {
          job,
          seed,
          state.ledger().runLegsPlanned(),
          state.ledger().runPlannedRouteMeters(),
          state.ledger().runWalkedRouteMeters(),
          state.agentsList.size()
        });
  }

  /** Logs each replicate, and the spread across them when there is more than one. */
  private void reportReplicates() {
    List<double[]> rows;
    synchronized (jobTotals) {
      rows = new ArrayList<>(jobTotals);
    }
    if (rows.isEmpty()) {
      return;
    }
    rows.sort((a, b) -> Double.compare(a[0], b[0]));

    StringBuilder out = new StringBuilder("Replicates:");
    for (double[] r : rows) {
      out.append(
          String.format(
              "%n  job %.0f (seed %.0f): %.0f legs, %.0f m planned, %.0f m walked, %.0f agents",
              r[0], r[1], r[2], r[3], r[4], r[5]));
    }
    if (rows.size() > 1) {
      appendSpread(out, "legs", rows, 2);
      appendSpread(out, "planned m", rows, 3);
      appendSpread(out, "walked m", rows, 4);
    }
    logger.info(out.toString());
  }

  /** Mean and sample standard deviation of one column, as an absolute figure and a percentage. */
  private static void appendSpread(StringBuilder out, String label, List<double[]> rows, int col) {
    double mean = rows.stream().mapToDouble(r -> r[col]).average().orElse(0.0);
    double sumSquares = 0.0;
    for (double[] r : rows) {
      sumSquares += (r[col] - mean) * (r[col] - mean);
    }
    double sd = Math.sqrt(sumSquares / (rows.size() - 1));
    out.append(
        String.format(
            "%n  %-10s mean %.0f, sd %.0f (%.1f%%) over %d replicates",
            label, mean, sd, mean == 0.0 ? 0.0 : 100.0 * sd / Math.abs(mean), rows.size()));
  }

  protected boolean continueSimulation(PedSimCity state) {
    return state.schedule.step(state)
        && (state.schedule.getSteps() <= TimePars.simulationDurationInSteps);
  }

  protected void onJobStarted(int job, PedSimCity state, ScenarioConfig scenarioConfig) {
    // no-op
  }

  protected void onJobStep(int job, PedSimCity state, ScenarioConfig scenarioConfig) {
    // no-op
  }

  protected void afterSetParameters() {
    // no-op
  }

  protected void onStepUpdate(PedSimCity state, double steps) {
    // no-op
  }

  /**
   * A day has ended and its ledger is still intact — the next day's release manager clears it.
   *
   * @param day the day that has just finished, counting from 1
   */
  protected void onDayFinished(PedSimCity state, int job, int day) {
    // no-op
  }

  protected void onJobFinished(int job, PedSimCity state, ScenarioConfig scenarioConfig) {
    // no-op
  }

  /**
   * Hook to write module-specific plain-data result files once a job has finished. No-op by default.
   *
   * @param job the job number just completed.
   * @param state the simulation state, still populated.
   * @param day the number of simulated days (1-based).
   * @param volumes per-edge volumes snapshotted before the daily export cleared them, keyed edgeID
   *     -> "&lt;agentType&gt;_&lt;hour&gt;" -> count.
   */
  protected void onJobExport(
      int job, PedSimCity state, int day, java.util.Map<Integer, java.util.Map<String, Integer>> volumes) {
    // no-op
  }

  protected void handleEndWeek(PedSimCity state, int job, ScenarioConfig scenarioConfig) {
    // no-op
  }

  /**
   * Hook to persist agent cognitive-map data (known edges / known landmarks) at each day boundary.
   * No-op by default: cognitive-map export is entirely a learning-module concern, so only
   * {@code LearningEngine} overrides it. Volumes/flows export stays in the core loop for all modules.
   */
  protected void exportCognitiveData(PedSimCity state, int day) throws Exception {
    // no-op
  }

  protected boolean isNextDay(double steps, int currentDay) {
    return getDays(steps) > currentDay && (currentDay + 1 < Pars.durationDays);
  }

  public static long getDays(double totalSteps) {
    long totalMinutes = (long) (totalSteps * (TimePars.STEP_DURATION / 60));
    return totalMinutes / (24 * 60);
  }

}
