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
 * <p>Both launch paths (headless and website/REST) route through this class so that clear →
 * defaults → city file → command line → run stays consistent across modules.
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

  /** Reserves the simulation before applying CLI parameters. */
  public void headlessRun(String[] args) throws Exception {
    var reservation = SimulationStateStore.getInstance().tryReserveRun();
    if (reservation == null) {
      logger.warning("Simulation is already running! Ignoring new run request.");
      return;
    }
    run(ParameterManager.parseArgs(args), reservation);
  }

  /** Shared CLI/REST lifecycle. Always releases the reservation, including setup failures. */
  public void run(Map<String, String> params, SimulationStateStore.RunReservation reservation)
      throws Exception {
    try (reservation) {
      reservation.requireActive();
      configure(params);
      SimulationStateStore.getInstance().setActiveModule(module);
      RouteTrace.openLegFile();
      try {
        module.createEngine().runJobs(module.scenarioConfig(), Pars.parallel, reservation);
      } finally {
        RouteTrace.closeLegFile();
      }
    }
  }

  /** Module defaults, city settings, explicit parameters, then derived values. */
  void configure(Map<String, String> params) {
    module.applyDefaults(params);
    if (params.containsKey("cityName")) Pars.cityName = params.get("cityName");
    module.loadCityConfig(Pars.cityName);
    ParameterManager.initFromParams(params, module.parameterClasses());
    module.applyParameters(new java.util.HashMap<>(params));
  }
}
