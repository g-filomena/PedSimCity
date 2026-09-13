package pedsim.core.parameters;

import java.util.HashMap;
import pedsim.core.utilities.StringEnum.RoadType;

/**
 * The Parameters class contains global parameters and settings for the PedSimCity simulation. These
 * parameters are used to configure various aspects of the simulation, including simulation mode,
 * agent behavior, and data import options.
 */
public class Pars {

  // General parameters
  public static String cityName = "Torino";
  public static int population = 1500000;
  public static double percentagePopulationAgent = 0.001;
  public static int numAgents;

  /**
   * How often one person sets off from home on an average day, when nothing is known about who they
   * are.
   *
   * <p>Read only when the running module supplies no travel demand of its own. Core models no reason
   * for a person to leave the house, so this figure is a placeholder that lets the skeleton run
   * rather than a claim about anybody's travel: it carries no source and is not something to
   * calibrate against. A module that models activity states its own count and this is never read.
   */
  public static double departuresPerPersonPerDay = 0.25;

  /**
   * Range the desired route length is drawn from, in <b>walked</b> metres.
   *
   * <p>Applies wherever a destination is picked by distance rather than by an activity model's
   * utility: {@link pedsim.core.agents.Agent#defineRandomDestination()}, and the origin-destination
   * generators of the cityImage and empirical modules.
   *
   * <p><b>Convert before searching.</b> These are lengths of a walked route; node lookup works in
   * straight lines ({@code NodesLookup} takes a Euclidean interval). Pass them through
   * {@link pedsim.core.engine.NetworkCircuity#straightLineFor(double)} first, never raw. The gap is
   * the network's circuity, 1.17 to 1.54 across the bundled cities, and passing a walked length as a
   * Euclidean one produces routes that much too long with no error to show for it.
   *
   * <p>Both are set directly; nothing derives one from the other.
   */
  public static double minRouteLength = 900;

  public static double maxRouteLength = 2700;

  public static double networkCircuityFactor = 1.23;

  /**
   * Whether to measure {@link #networkCircuityFactor} from the city being loaded rather than use
   * the fallback. Passing {@code networkCircuityFactor} on the command line implies false; see
   * {@link ParameterManager#initFromArgs}.
   */
  public static boolean measureNetworkCircuity = true;

  /**
   * The run's base seed. Every generator in the simulation derives from it - per agent, per
   * populate pass, per release manager - so this one number decides whether two runs are the same
   * run.
   *
   * <p>Fixed by default, so that two runs of the same configuration are the same run: every A/B in
   * this project - route-choice models, parameter sweeps, the night module's light experiment - is a
   * comparison of runs. Pass {@code --seed=-1} for a clock seed when independent replicates are
   * wanted; job {@code n} uses {@code seed + n}, so a multi-job run already gives replicates from one
   * base seed.
   */
  public static long seed = 20260912L;

  public static int jobs = 1;
  public static int durationDays = 7;
  public static int stepDelayMs = 100;

  /**
   * How far the known-space skeleton reaches beyond each of an agent's anchors, in metres of
   * network distance.
   *
   * <p>Network distance, not Euclidean: {@code CognitiveMap.buildActivityBone()} accumulates edge
   * lengths walking out from each anchor. The anchors themselves come from
   * {@link pedsim.core.agents.Agent#cognitiveAnchors()}, which a module overrides to say what places
   * its people know - a retiree has no workplace - so this radius is about anchors rather than about
   * employment.
   *
   * <p>The 600 m has no source behind it.
   */
  public static double anchorRadius = 600;

  // Average pedestrian speed 1.42 m/s; moveRate (metres walked per step) is derived from
  // TimePars.STEP_DURATION in setSimulationParameters(), so changing the step size rescales
  // movement automatically (e.g. 300 s steps --> 426 m per step).

  protected static double pedestrianSpeed = 1.42;
  public static double SPEED_INCREMENT_FACTOR = 0.20;
  // meters per step;
  public static double moveRate;

  // for development/testing purposes only
  public static boolean parallel = false;

  public static boolean isNight = false;

  // Self-contained HTML dashboard export at the end of each job. It embeds every trip path, so its
  // size grows with the trip count (~29 MB at 5.8k trips); switch it off with
  // --exportHtmlDashboard=false for large runs, which also skips the trajectory snapshots that only
  // feed it. The plain data exports (volumes CSV, routes GeoPackage, module data files) are
  // unaffected.
  public static boolean exportHtmlDashboard = true;

  static String[] primary = {"primary", "primary_link"};
  static String[] secondary = {"secondary", "secondary_link"};
  static String[] tertiary = {"tertiary", "tertiary_link", "unclassified"};
  static String[] neighborhood = {"residential", "pedestrian", "living_street"};
  static String[] unknown = {
    "footway", "bridleway", "steps", "corridor", "path", "track", "service"
  };

  public static HashMap<RoadType, String[]> roadTypes = new HashMap<>();

  /** The seed to run with: the configured one, or a fresh clock seed when it is negative. */
  public static long resolvedSeed() {
    return seed >= 0 ? seed : System.currentTimeMillis();
  }

  public static void setSimulationParameters() {

    TimePars.setTemporalPars();
    moveRate = TimePars.STEP_DURATION * pedestrianSpeed;
    recomputeAgentCount();
    setRoadTypeMap();
  }

  /**
   * Recomputes the sampled agent count from {@code population * percentagePopulationAgent}. Call
   * after changing {@code population} at runtime — e.g. when a module derives it from its own city
   * data.
   */
  public static void recomputeAgentCount() {
    numAgents = (int) (population * percentagePopulationAgent);
  }

  private static void setRoadTypeMap() {
    roadTypes.put(RoadType.PRIMARY, primary);
    roadTypes.put(RoadType.SECONDARY, secondary);
    roadTypes.put(RoadType.TERTIARY, tertiary);
    roadTypes.put(RoadType.NEIGHBOURHOOD, neighborhood);
    roadTypes.put(RoadType.UNKNOWN, unknown);
  }
}
