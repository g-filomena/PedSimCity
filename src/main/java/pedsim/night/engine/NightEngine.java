package pedsim.night.engine;

import pedsim.activity.engine.ActivityEngine;
import pedsim.core.engine.Engine;
import pedsim.core.engine.Import;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.engine.TripDiagnostic;
import pedsim.night.parameters.NightPars;

/**
 * Night-simulation engine: extends {@link ActivityEngine} with the night importer, night
 * environment preparation, night-specific data clearing and the day/night step update.
 */
public class NightEngine extends ActivityEngine {

  public NightEngine(StateFactory stateFactory) {
    super(stateFactory);
  }

  public NightEngine(StateFactory stateFactory, long baseSeed) {
    super(stateFactory, baseSeed);
  }

  private static final java.util.logging.Logger LOG = pedsim.core.utilities.LoggerUtil.getLogger();

  /** The dashboard is this module's; core only decides when to ask for one. */
  @Override
  protected void generateAndOpenHtmlDashboard(
      int job,
      pedsim.core.engine.PedSimCity state,
      int currentDay,
      java.util.Map<Integer, java.util.Map<String, Integer>> finalVolumesMap) {
    try {
      LOG.info("[NightEngine] Compiling HTML dashboard for job " + job + "…");

      String htmlPath =
          pedsim.night.website.HtmlExporter.export(
              currentDay + 1, // day (1-based)
              job,
              state.tripRecorder.getRecords(),
              finalVolumesMap);

      if (htmlPath != null && java.awt.Desktop.isDesktopSupported()) {
        java.awt.Desktop.getDesktop().browse(new java.io.File(htmlPath).toURI());
        LOG.info("[NightEngine] Opened dashboard in browser: " + htmlPath);
      }
    } catch (Exception e) {
      LOG.warning("[NightEngine] Could not open HTML dashboard: " + e.getMessage());
    }
  }

  @Override
  protected Import createImporter() {
    return new NightImport();
  }

  @Override
  protected void prepareEnvironment() {
    NightEnvironment.prepare();
  }

  @Override
  protected void clearStaticData() {
    super.clearStaticData();
    PedSimCityNight.clearNightStaticData();
  }

  @Override
  protected void onJobFinished(int job, PedSimCity state, ScenarioConfig scenarioConfig) {
    if (NightPars.enableLightABTesting) {
      logger.warning(
          "Night A/B twin testing was enabled: vulnerability outputs are experimental twin "
              + "comparisons, not census-sampled shares.");
      NightDataExporter.saveABTestComparison(
          TripDiagnostic.jobFilename("ab_test_comparison.csv", job),
          state.tripRecorder.getRecords());
    }
  }

  /** Writes the night run's plain-data files (hourly edge volumes + trips) for this job. */
  @Override
  protected void onJobExport(
      int job,
      PedSimCity state,
      int day,
      java.util.Map<Integer, java.util.Map<String, Integer>> volumes) {
    NightDataExporter.export(
        job, state.tripRecorder.getRecords(), state.flowHandler.lastVolumesFile());
  }

  @Override
  protected Engine createWorkerEngine() {
    return new NightEngine(stateFactory, baseSeed);
  }
}
