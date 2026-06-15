package pedsim.core.engine;

import java.util.Map;
import java.util.logging.Logger;
import pedsim.core.parameters.ParameterManager;
import pedsim.core.parameters.Pars;
import pedsim.core.utilities.LoggerUtil;
import pedsim.core.website.GeoJsonExporter;
import pedsim.core.website.SimulationRestApi;

/**
 * Lifecycle scaffold for one {@link SimulationModule}: pre-load, headless run, and REST wiring.
 *
 * <p>All three launch paths (GUI, headless, website/REST) route through this class so that clear →
 * applyMode → parameters → run stays consistent across modes.
 */
public final class SimulationLauncher {

  private static final Logger logger = LoggerUtil.getLogger();

  private final SimulationModule module;

  public SimulationLauncher(SimulationModule module) {
    this.module = module;
  }

  /** Returns the module this launcher wraps. */
  public SimulationModule getModule() {
    return module;
  }

  /** Clears core static data and then module-specific data. */
  public void clearAll() {
    PedSimCity.clearStaticData();
    module.clearStaticData();
  }

  /**
   * Sets mode flags via the module and records the active module in {@link SimulationStateStore}
   * so the REST state snapshot can include module-specific fields.
   */
  public void applyMode() {
    module.applyMode();
    SimulationStateStore.getInstance().setActiveModule(module);
  }

  /**
   * Applies standard REST/CLI parameters to {@link Pars}, then delegates module-specific
   * parameters to {@link SimulationModule#applyParameters(Map)}.
   */
  public void applyRestParams(Map<String, Object> params) {
    if (params.containsKey("cityName")) Pars.cityName = (String) params.get("cityName");
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
    module.applyParameters(params);
  }

  /**
   * Pre-loads GIS data so the dashboard can render roads before the first simulation run.
   *
   * <p>Ordering contract — must not be changed:
   *
   * <ol>
   *   <li>{@code clearAll()} — flushes core and module-specific static data
   *   <li>{@code applyMode()} — sets {@code Pars.isNight} and registers the active module
   *   <li>{@code Pars.setSimulationParameters()} — reads {@code Pars.isNight}; must come after
   *   <li>{@code Import.importFiles()} — reads {@code Pars.isNight}; must come after
   *   <li>{@code setRoadsGeoJson()} — exports loaded road data
   * </ol>
   */
  public void preloadForDashboard() {
    try {
      clearAll();       // 1. clear first
      applyMode();      // 2. mode flags before any parameter/import call
      Pars.setSimulationParameters();
      new Import().importFiles();
      SimulationStateStore.getInstance()
          .setRoadsGeoJson(GeoJsonExporter.exportRoads(PedSimCity.roads));
    } catch (Exception e) {
      logger.warning("Could not pre-load default roads: " + e.getMessage());
    }
  }

  /**
   * Registers this launcher's module with {@link SimulationRestApi} and starts the HTTP server on
   * {@code port}. Use in website mode only; in GUI mode the server is started without a module
   * registration so {@code /api/start} returns 503.
   */
  public void wireAndStartRestServer(int port) {
    SimulationRestApi.registerModule(module);
    SimulationRestApi.start(port);
  }

  /**
   * Full headless launch: parse CLI args, apply mode, run engine.
   *
   * <p>{@link Engine#runJobs} handles clearStaticData / setSimulationParameters / importFiles
   * internally; this method only needs to set mode flags and CLI parameters first.
   */
  public void headlessRun(String[] args) throws Exception {
    ParameterManager.initFromArgs(args);
    applyMode();
    module.createEngine().runJobs(module.scenarioConfig(), Pars.parallel);
  }
}
