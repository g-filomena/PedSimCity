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
 * <p>Implemented by activity, night, learning, social, cityImage and empirical. The last two
 * override {@link Engine#executeJob} and never construct an {@link AgentReleaseManager}: they walk a
 * fixed origin-destination matrix rather than releasing agents across a day, and they read no
 * per-city configuration file, because those files configure activity behaviour and these two model
 * none.
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
   * Every parameter class a run of this module may have values written into: core's, plus the
   * module's own.
   *
   * <p>This is the single list consulted by everything that writes parameters: the command line
   * ({@link pedsim.core.parameters.ParameterManager#initFromArgs}) and the per-city configuration
   * file (via {@link #loadCityConfig}). A parameter class omitted here is unreachable from both — a
   * key naming one of its fields is accepted and then ignored, with nothing in the output to say so —
   * so a module that adds a parameter class must add it here.
   *
   * <p>Core cannot name a module's class, so the module supplies the list rather than core importing
   * it.
   *
   * @return the parameter classes, core's first
   */
  default Class<?>[] parameterClasses() {
    return new Class<?>[] {
      pedsim.core.parameters.Pars.class,
      pedsim.core.parameters.TimePars.class,
      pedsim.core.parameters.RouteChoicePars.class
    };
  }

  /**
   * Reads whatever per-city configuration this module has for {@code cityName}, before the command
   * line is applied so a sweep still overrides it.
   *
   * <p>Core does nothing here. It is the machinery - graph, routing, cognitive map, movement, the
   * day loop - and has no behaviour to configure; a city configuration is a set of behavioural facts
   * (how often people walk to work, what the population is made of, how far they will go for a given
   * attraction), which belongs to a model. The activity module reads one; see
   * {@code activity.parameters.CityConfig}.
   *
   * @param cityName the city being loaded
   */
  default void loadCityConfig(String cityName) {
    // Core has no behaviour to configure.
  }

  /**
   * Sets all module-specific mode flags on shared parameter classes.
   * Must be called before {@link pedsim.core.parameters.Pars#setSimulationParameters()} and before
   * any GIS import.
   */
  /**
   * Sets this module's defaults, before the city file and before the command line.
   *
   * <p>This is the first of the four stages that write parameters, and the order between them is
   * the contract: <b>module defaults → city file → command line → derived</b>. Anything set here is
   * overwritten by a command line that names it, so a default may be stated unconditionally - there
   * is no need to ask whether the user supplied one.
   *
   * <p>{@code selectors} is the parsed command line, and may be read <b>only to choose between
   * alternative sets of defaults</b> - cityImage picks a test design from {@code stringMode} - never
   * to take a value from it. Taking values here is how a module comes to overwrite the command line
   * and then need a second pass to undo itself.
   *
   * @param selectors the parsed command line, for choosing which defaults apply
   */
  default void applyDefaults(Map<String, String> selectors) {}

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
   * <p>Example keys: {@code enableAB}, {@code crowdednessPercentile}.
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
   * then fall back to a user-supplied value). Modules may derive it from their own city data; the
   * default is {@code 0}. Lets core UIs (dashboard, applet) show/lock the population field without
   * depending on any module.
   */
  default long populationForCity(String city) {
    return 0;
  }
}
