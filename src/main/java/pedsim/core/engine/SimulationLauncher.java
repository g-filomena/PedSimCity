package pedsim.core.engine;

import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.logging.Logger;

import pedsim.core.parameters.ParameterManager;
import pedsim.core.parameters.Pars;
import pedsim.core.utilities.LoggerUtil;
import pedsim.core.website.GeoJsonExporter;
import pedsim.core.website.SimulationRestApi;

/**
 * Captures the identity of a simulation mode (core or night) and provides the
 * bootstrap sequences for headless, website-preload, and REST-callback launches
 * so all three paths stay in sync.
 *
 * <p>No imports from pedsim.night.* — night-specific cleanup and engine
 * creation are supplied as constructor arguments.
 */
public final class SimulationLauncher {

  private static final Logger logger = LoggerUtil.getLogger();

  private final boolean isNight;
  private final Supplier<Engine> engineFactory;
  private final ScenarioConfig scenarioConfig;

  /**
   * Optional extra cleanup run before preload and before each REST-triggered
   * run. Pass {@code PedSimCityNight::clearNightStaticData} for night mode,
   * {@code null} for core mode.
   */
  private final Runnable extraClear;

  public SimulationLauncher(
      boolean isNight,
      Supplier<Engine> engineFactory,
      ScenarioConfig scenarioConfig,
      Runnable extraClear) {
    this.isNight = isNight;
    this.engineFactory = engineFactory;
    this.scenarioConfig = scenarioConfig;
    this.extraClear = extraClear;
  }

  /** Clears core static data and any module-specific static data. */
  public void clearAll() {
    PedSimCity.clearStaticData();
    if (extraClear != null) extraClear.run();
  }

  /** Sets {@code Pars.isNight} to match this launcher's mode. */
  public void applyMode() {
    Pars.isNight = isNight;
  }

  /**
   * Applies standard REST request body parameters to {@link Pars}.
   * Shared by core and night REST callbacks to prevent drift.
   */
  public void applyRestParams(Map<String, Object> params) {
    if (params.containsKey("cityName"))
      Pars.cityName = (String) params.get("cityName");
    if (params.containsKey("days"))
      Pars.durationDays = Integer.parseInt(params.get("days").toString());
    if (params.containsKey("actualPopulation"))
      Pars.population = Integer.parseInt(params.get("actualPopulation").toString());
    if (params.containsKey("percentage"))
      Pars.percentagePopulationAgent = Double.parseDouble(params.get("percentage").toString());
    if (params.containsKey("jobs")) {
      Pars.jobs = Integer.parseInt(params.get("jobs").toString());
      Pars.parallel = (Pars.jobs > 1);
    }
  }

  /**
   * Pre-loads GIS data so the dashboard can render roads before the first run.
   * Sequence: clearAll → applyMode → setSimulationParameters → importFiles → setRoadsGeoJson.
   */
  public void preloadForDashboard() {
    try {
      clearAll();
      applyMode();
      Pars.setSimulationParameters();
      new Import().importFiles();
      SimulationStateStore.getInstance().setRoadsGeoJson(
          GeoJsonExporter.exportRoads(PedSimCity.roads));
    } catch (Exception e) {
      logger.warning("Could not pre-load default roads: " + e.getMessage());
    }
  }

  /**
   * Returns the {@link Consumer} to pass to
   * {@link SimulationRestApi#setOnStart}. Each invocation applies REST params,
   * sets mode, recomputes parameters, then runs a fresh engine.
   */
  public Consumer<Map<String, Object>> buildRestCallback() {
    return params -> {
      try {
        applyRestParams(params);
        applyMode();
        Pars.setSimulationParameters();
        engineFactory.get().runJobs(scenarioConfig, Pars.jobs > 1);
      } catch (Exception e) {
        logger.severe("Simulation failed: " + e.getMessage());
      }
    };
  }

  /**
   * Registers the REST callback and starts the API server on {@code port}.
   * Use this for website mode; do not call in GUI mode (REST is observation-only there).
   */
  public void wireAndStartRestServer(int port) {
    SimulationRestApi.setOnStart(buildRestCallback());
    SimulationRestApi.start(port);
  }

  /**
   * Full headless launch: parse CLI args, apply mode, run engine.
   * {@link Engine#runJobs} handles clearStaticData / setSimulationParameters / importFiles internally.
   */
  public void headlessRun(String[] args) throws Exception {
    ParameterManager.initFromArgs(args);
    applyMode();
    engineFactory.get().runJobs(scenarioConfig, Pars.parallel);
  }
}
