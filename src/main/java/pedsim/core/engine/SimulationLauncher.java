package pedsim.core.engine;

import java.util.Map;
import java.util.logging.Logger;
import pedsim.core.parameters.ParameterManager;
import pedsim.core.parameters.Pars;
import pedsim.core.utilities.LoggerUtil;
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

  /** Clears all static data for this module (core + module-specific). Delegates to the module. */
  public void clearAll() {
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
   * Starts the REST API server on {@code port}. If the module is a concrete runnable simulation
   * (i.e. {@link SimulationModule#isConcreteRunnable()} is {@code true}), it is also registered so
   * that {@code POST /api/start} can select it. Infrastructure-only modules (e.g. {@link
   * CoreSimulationModule}) are never registered and will not appear in {@code GET /api/modules}.
   */
  public void wireAndStartRestServer(int port) {
    if (module.isConcreteRunnable()) {
      SimulationRestApi.registerModule(module);
    }
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
