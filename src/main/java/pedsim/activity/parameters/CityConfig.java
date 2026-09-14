package pedsim.activity.parameters;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Properties;
import java.util.logging.Logger;
import pedsim.activity.agents.ActivityPurpose;
import pedsim.core.parameters.ParameterManager;
import pedsim.core.utilities.LoggerUtil;

/**
 * Per-city parameters, read from {@code <City>/<City>.properties} beside the city's GIS layers.
 *
 * <p>The rule this replaces was "keep national defaults, the model is general", and it had stopped
 * being true: {@code walkShareCommuteWorker = 0.163} is the ISTAT commuting matrix for Turin, and it
 * sat in a class every city loads. A per-city file makes adding a city a matter of adding data
 * rather than editing Java, and makes the Turin-ness of a Turin number visible at the point of use.
 *
 * <p><b>This belongs to the activity module, and core cannot reach it.</b> Core is the machinery -
 * graph, routing, cognitive map, movement, the day loop - and has no behaviour to configure. A
 * "city", in the sense this file means it, is a set of behavioural facts: how often people walk to
 * work, what the population is made of, how far they will go for a given attraction. Those are
 * activity-model ideas. This class sat in {@code core.parameters} for half a day, called from
 * {@code SimulationLauncher}, which put a behavioural concern inside the engine; core now offers the
 * seam ({@link pedsim.core.engine.SimulationModule#loadCityConfig}) and the module does the reading.
 *
 * <p>So {@code Pars.minRouteLength}, {@code Pars.networkCircuityFactor} and the rest of core's
 * parameters are not settable from a city file. Circuity is measured from the network; the others
 * are command-line arguments.
 *
 * <h2>What may be in the file</h2>
 *
 * Four kinds of parameter, and only the second belongs here:
 *
 * <ul>
 *   <li><b>Model constants</b> — {@code explorationRho}, {@code familiarLocationCapacity},
 *       {@code RADIAL_SHELLS}, the destination-search convergence controls. Properties of the model
 *       or of human mobility in general, not of a city. Not settable here.
 *   <li><b>City parameters</b> — walk shares, persona fallbacks, distance decays, utility weights,
 *       latitude, opening windows. This file.
 *   <li><b>Measured from the city at startup</b> — {@code networkCircuityFactor}. The file may
 *       <i>pin</i> one, which turns the measurement off; it may not pretend to set one that would
 *       then be overwritten. Setting it here implies {@code measureNetworkCircuity = false}, the
 *       same rule the command line follows.
 *   <li><b>Run switches</b> — {@code useDestinationChoice}, {@code useWeather},
 *       {@code calibrateCommute}, {@code parallel}, {@code seed}. These select a model structure or
 *       an experiment, not a city. Putting one here would let two city files differ in what model
 *       they run, and a comparison between those cities would be measuring the difference in
 *       mechanism while appearing to measure the difference in city. Rejected with a warning.
 * </ul>
 *
 * <h2>Precedence</h2>
 *
 * Java defaults, then this file, then the command line — so a city file states the city and a
 * parameter sweep still overrides it. Anything derived ({@code setMinMaxTripDistance},
 * {@code recomputeAgentCount}) is computed after both, in
 * {@link Pars#setSimulationParameters()}.
 *
 * <h2>It says what it did</h2>
 *
 * Every key is reported: applied, refused as a run switch, or unplaced because no parameter class
 * has a field of that name. Nothing is applied without being reported: a misspelled key is a
 * warning, not a silent default.
 */
public final class CityConfig {

  private static final Logger logger = LoggerUtil.getLogger();

  /**
   * Names a city file may not set, because they choose a model rather than describe a place. The
   * check is by name: these live on several different classes, and what disqualifies them is what
   * they mean, not where they sit.
   */
  private static final List<String> RUN_SWITCHES =
      List.of(
          "useDestinationChoice",
          "useAgendaDepartureProfile",
          "usePersonaReleaseWeights",
          "useCensusPersonas",
          "useWeather",
          "useSeasonalDaylight",
          "calibrateCommute",
          "calibrationHomes",
          "enableLightABTesting",
          "abTestPairs",
          "measureNetworkCircuity",
          "parallel",
          "seed",
          "jobs",
          "durationDays",
          "headless",
          "exportHtmlDashboard",
          "stepDelayMs");

  private CityConfig() {}

  /**
   * Loads and applies {@code <City>/<City>.properties}, if the city has one.
   *
   * <p>A missing file is not an error: a city without one runs on the built-in defaults.
   *
   * @param cityName the city whose resource folder to look in
   * @param targets every parameter class this run may write into, from
   *     {@link pedsim.core.engine.SimulationModule#parameterClasses()}
   */
  public static void load(String cityName, Class<?>[] targets) {
    // Before anything is read: a second city in the same JVM must not inherit the first's opening
    // hours, and a city with no file must run on the built-in ones.
    ActivityPurpose.resetToDefaults();
    if (cityName == null || cityName.isBlank()) {
      return;
    }
    String resource = cityName + "/" + cityName + ".properties";
    Properties properties = new Properties();

    try (InputStream in = CityConfig.class.getClassLoader().getResourceAsStream(resource)) {
      if (in == null) {
        logger.info("No city configuration at " + resource + "; using built-in defaults.");
        return;
      }
      properties.load(in);
    } catch (IOException e) {
      logger.warning("Could not read " + resource + ": " + e.getMessage());
      return;
    }

    Map<String, String> applied = new LinkedHashMap<>();
    List<String> refused = new ArrayList<>();
    List<String> unplaced = new ArrayList<>();

    for (String key : properties.stringPropertyNames()) {
      String raw = properties.getProperty(key);
      if (raw == null || raw.isBlank()) {
        continue;
      }
      raw = raw.trim();

      if (RUN_SWITCHES.contains(key)) {
        refused.add(key);
        continue;
      }
      if (key.startsWith(PURPOSE_PREFIX)) {
        if (writePurpose(key, raw)) {
          applied.put(key, raw);
        } else {
          unplaced.add(key);
        }
        continue;
      }
      if (writeInto(targets, key, raw)) {
        applied.put(key, raw);
      } else {
        unplaced.add(key);
      }
    }

    logger.info(
        String.format(
            "%s: applied %d parameter(s) from %s", cityName, applied.size(), resource));
    for (Map.Entry<String, String> entry : applied.entrySet()) {
      logger.fine("  " + entry.getKey() + " = " + entry.getValue());
    }
    if (!refused.isEmpty()) {
      logger.warning(
          resource
              + ": ignored "
              + refused
              + " — these choose a model or an experiment, not a city, and belong on the command"
              + " line. A city file that set one would make two cities differ in mechanism.");
    }
    if (!unplaced.isEmpty()) {
      logger.warning(
          resource
              + ": no parameter field named "
              + unplaced
              + " in "
              + describe(targets)
              + ". Check the spelling — nothing was applied for these.");
    }
  }

  /** Prefix for the opening-window keys, e.g. {@code purpose.DINING.open}. */
  private static final String PURPOSE_PREFIX = "purpose.";

  /**
   * Writes one {@code purpose.<NAME>.<setting>} key.
   *
   * <p>These are the one group of city parameters that are not fields on a parameter class: they
   * live on the {@link ActivityPurpose} enum, four per purpose. The enum's own values are generic
   * defaults - dining 11:00-23:00 is not an Italian day - so a city that knows better says so here.
   *
   * @return whether the key named a purpose and a setting that exist, and parsed as a number
   */
  private static boolean writePurpose(String key, String raw) {
    String[] parts = key.split("\\.");
    if (parts.length != 3) {
      return false;
    }
    try {
      return ActivityPurpose.applyCitySetting(parts[1], parts[2], Double.parseDouble(raw));
    } catch (NumberFormatException e) {
      return false;
    }
  }

  /**
   * Writes one key into the first class that declares a field of that name.
   *
   * @return whether any class took it
   */
  private static boolean writeInto(Class<?>[] targets, String key, String raw) {
    for (Class<?> target : targets) {
      try {
        target.getDeclaredField(key);
      } catch (NoSuchFieldException e) {
        continue;
      }
      ParameterManager.setFieldValue(target, key, raw);
      return true;
    }
    return false;
  }

  private static String describe(Class<?>[] targets) {
    List<String> names = new ArrayList<>();
    for (Class<?> target : targets) {
      names.add(target.getSimpleName());
    }
    return String.join(", ", names);
  }
}
