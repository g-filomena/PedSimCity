package pedsim.core.engine;

import java.util.Map;

/**
 * Encapsulates everything module-specific needed to launch, configure, and describe one simulation
 * mode (core, night, cityImage, empirical).
 *
 * <p>Implement this interface to add a new module. Register instances with {@link
 * pedsim.core.website.SimulationRestApi#registerModule(SimulationModule)} so the REST layer can
 * route {@code POST /api/start { "module": "…" }} requests to the right engine.
 *
 * <p>Extension points for future modules:
 *
 * <ul>
 *   <li>{@code cityImage} — {@code CityImageSimulationModule} in {@code pedsim.cityimage.engine}
 *   <li>{@code empirical} — {@code EmpiricalSimulationModule} in {@code pedsim.empirical.engine}
 * </ul>
 */
public interface SimulationModule {

  /** Unique string key used in the REST API {@code "module"} field (e.g. {@code "core"}). */
  String moduleId();

  /**
   * Sets all module-specific mode flags on shared parameter classes (e.g. {@code Pars.isNight}).
   * Must be called before {@link pedsim.core.parameters.Pars#setSimulationParameters()} and before
   * any GIS import.
   */
  void applyMode();

  /**
   * Clears module-specific static data beyond what {@link Engine#clearStaticData()} already
   * handles. Called during dashboard pre-load before the first import.
   */
  void clearStaticData();

  /** Creates a fresh {@link Engine} instance for this module. */
  Engine createEngine();

  /** Returns the {@link ScenarioConfig} governing which scenario groups this module runs. */
  ScenarioConfig scenarioConfig();

  /**
   * Applies module-specific parameters from a REST body or CLI argument map. Common parameters
   * (cityName, days, actualPopulation, percentage, jobs) are already applied by the caller before
   * this method is invoked.
   */
  void applyParameters(Map<String, Object> params);

  /**
   * Returns a live snapshot of module-specific state merged into the REST {@code /api/state}
   * response under the {@code "moduleState"} key. Called on every state poll — keep it cheap.
   *
   * <p>Example keys for the night module: {@code enableAB}, {@code crowdednessPercentile}.
   *
   * <p><b>Implementation contract:</b> all values in the returned map must be JSON-serializable by
   * Jackson (primitives, strings, booleans, numbers, nested maps/lists of the same). Do not return
   * arbitrary Java objects. If this method throws, {@link
   * pedsim.core.engine.SimulationStateStore} will catch the exception and return an empty map so
   * that {@code /api/state} never fails due to a module bug.
   */
  default Map<String, Object> extraState() {
    return Map.of();
  }

  /**
   * Describes the module-specific parameters accepted by {@link #applyParameters}. Keys are
   * parameter names; values are type strings ({@code "string"}, {@code "integer"}, {@code
   * "double"}, {@code "boolean"}). Common parameters (cityName, days, actualPopulation, percentage,
   * jobs, module) are implied and not repeated here.
   *
   * <p><b>Implementation contract:</b> all values must be JSON-serializable strings (type names
   * only). Do not put live objects here.
   */
  default Map<String, Object> parameterSchema() {
    return Map.of();
  }
}
