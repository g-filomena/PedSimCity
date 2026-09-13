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
   * How often one person sets off from home on an average day, when nothing is known about who
   * they are.
   *
   * <p>Unsourced, and deliberately so: it exists so that the core skeleton runs, not as a claim
   * about anybody's travel. Core models no reason for a person to leave the house, so there is no
   * quantity here to be right about - a module that does model one states its own count and this is
   * never read.
   */
  public static double departuresPerPersonPerDay = 0.25;

  /**
   * Desired length of the route between the origin and the destination, in **walked metres**.
   *
   * <p>Walked, not straight-line, and the distinction is the whole point of the name. A route is
   * what an agent walks on the network; the straight line between its endpoints is shorter by the
   * network's circuity, which on the bundled cities runs from 1.17 (Barcelona) to 1.54 (Melbourne).
   * Anything that picks a destination works in straight lines - {@code NodesLookup} takes a
   * Euclidean interval - so a caller must convert with
   * {@link pedsim.core.engine.NetworkCircuity#straightLineFor(double)} before searching, never pass
   * these figures through raw.
   *
   * <p>That is not hypothetical. These were briefly handed straight to
   * {@code NodesLookup.randomNodeBetweenDistanceInterval} by the cityImage and empirical modules
   * while core divided by the circuity first, so one field meant walked metres in one place and
   * straight-line metres in another - on Torino, a nominal 900-2700 m band generating routes of
   * 1163-3488 m.
   *
   * <p>Set directly. They used to be derived by a {@code setMinMaxTripDistance()} that took an
   * {@code avgTripDistance} and multiplied it by 0.5 and 1.5 - two invented factors that also
   * overwrote whatever had been configured, which is how a declared 700/2500 came to run as
   * 900/2700 with nothing in the source saying so.
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
   * <p>It had no way of being set. {@code Engine(stateFactory)} seeded from
   * {@code System.currentTimeMillis()} and there was no command-line parameter, so every headless
   * run drew a different seed and no two runs could be compared. That quietly undid the seeding
   * work: each generator was faithfully derived from a base seed that was itself the clock.
   *
   * <p>Fixed by default, because a model whose runs are not repeatable cannot be compared with
   * itself, and every A/B in this project - route-choice models, parameter sweeps, the night
   * module's light experiment - is a comparison of runs. Pass {@code --seed=-1} for a clock seed
   * when independent replicates are wanted; job {@code n} uses {@code seed + n}, so a multi-job run
   * already gives replicates from one base seed.
   */
  public static long seed = 20260912L;

  public static int jobs = 1;
  public static int durationDays = 7;
  public static int stepDelayMs = 100;

  /**
   * How far the known-space skeleton reaches beyond each of an agent's anchors, in metres of
   * network distance.
   *
   * <p>It was called {@code homeWorkRadius} while home and work were the only anchors the code
   * could imagine. {@link pedsim.core.agents.Agent#cognitiveAnchors()} now lets a module say what
   * anchors its people actually have - a retiree has no workplace - so the radius is about anchors,
   * not about employment. The comment above it said "Euclidean Distance between Origin and
   * Destination", which it has never been: the walk out from each anchor in
   * {@code CognitiveMap.buildActivityBone()} accumulates edge lengths.
   *
   * <p>The 600 m itself has no source behind it.
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
   *
   * <p>It also used to set {@code metersPerDay}, the day's release budget, from a
   * {@code metersPerDayPerPerson}. Nothing spends a metres budget any more, and that figure had no
   * reader left at all once the budget went - it was a comment with a {@code double} around it. The
   * derivation it belongs to is in RELEASE_BUDGET.md, which is where a run's walked metres should be
   * compared against it.
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
