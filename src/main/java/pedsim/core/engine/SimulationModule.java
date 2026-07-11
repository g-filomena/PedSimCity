package pedsim.core.engine;

import java.util.Map;

/**
 * Encapsulates everything module-specific needed to launch, configure, and describe one simulation
 * mode (night, cityImage, empirical, …).
 *
 * <p>{@code core} is shared infrastructure; it does <em>not</em> implement this interface as a
 * runnable module. Concrete domain modules (night, cityImage, empirical) implement this interface
 * and are registered with {@link pedsim.core.website.SimulationRestApi#registerModule} so the REST
 * layer can route {@code POST /api/start { "module": "night" }} requests to the right engine.
 *
 * <p>Extension points for future modules:
 *
 * <ul>
 *   <li>{@code cityImage} — {@code CityImageSimulationModule} in {@code pedsim.cityimage.engine}
 *   <li>{@code empirical} — {@code EmpiricalSimulationModule} in {@code pedsim.empirical.engine}
 * </ul>
 */
public interface SimulationModule {

  /** Unique string key used in the REST API {@code "module"} field (e.g. {@code "night"}). */
  String moduleId();

  /**
   * Returns {@code true} if this module is a concrete runnable simulation that should be exposed
   * via {@code GET /api/modules} and selectable via {@code POST /api/start}. Returns {@code false}
   * for infrastructure-only or base modules that must not appear as selectable simulations.
   *
   * <p>The default is {@code true}; override to {@code false} only in non-runnable base
   * implementations (e.g. {@code CoreSimulationModule}).
   */
  default boolean isConcreteRunnable() {
    return true;
  }

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

  /**
   * The population this module would use for the given city, or {@code 0} when it has none (callers
   * then fall back to a user-supplied value). Modules may derive it from their own city data (e.g.
   * the activity module reads census resident totals); the default is {@code 0}. Lets core UIs
   * (dashboard, applet) show/lock the population field without depending on any module.
   */
  default long populationForCity(String city) {
    return 0;
  }
}
