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
  public static double metersPerDayPerPerson = 4000; // 4k
  public static double metersPerDay;
  public static int numAgents;

  public static int jobs = 1;
  public static int durationDays = 7;
  public static int stepDelayMs = 100;

  // Euclidean Distance between Origin and Destination
  public static double homeWorkRadius = 600;

  // Average pedestrian speed 1.42 m/s; moveRate (metres walked per step) is derived from
  // TimePars.STEP_DURATION in setSimulationParameters(), so changing the step size rescales
  // movement automatically (e.g. 300 s steps --> 426 m per step).

  protected static double pedestrianSpeed = 1.42;
  public static double SPEED_INCREMENT_FACTOR = 0.20;
  // meters per step;
  public static double moveRate;

  // for development/testing purposes only
  public static boolean javaProject = false;
  public static String localPath = System.getProperty("pedsimcity.localPath", "");
  public static boolean parallel = false;

  public static boolean isNight = false;

  static String[] primary = {"primary", "primary_link"};
  static String[] secondary = {"secondary", "secondary_link"};
  static String[] tertiary = {"tertiary", "tertiary_link", "unclassified"};
  static String[] neighborhood = {"residential", "pedestrian", "living_street"};
  static String[] unknown = {
    "footway", "bridleway", "steps", "corridor", "path", "track", "service"
  };

  public static HashMap<RoadType, String[]> roadTypes = new HashMap<>();

  public static void setSimulationParameters() {

    TimePars.setTemporalPars();
    moveRate = TimePars.STEP_DURATION * pedestrianSpeed;
    recomputeAgentCount();
    setRoadTypeMap();
    RouteChoicePars.setMinMaxTripDistance();
  }

  /**
   * Recomputes the sampled agent count (and derived daily distance) from {@code population *
   * percentagePopulationAgent}. Call after changing {@code population} at runtime — e.g. when a
   * module derives it from its own city data.
   */
  public static void recomputeAgentCount() {
    numAgents = (int) (population * percentagePopulationAgent);
    metersPerDay = metersPerDayPerPerson * numAgents;
  }

  private static void setRoadTypeMap() {
    roadTypes.put(RoadType.PRIMARY, primary);
    roadTypes.put(RoadType.SECONDARY, secondary);
    roadTypes.put(RoadType.TERTIARY, tertiary);
    roadTypes.put(RoadType.NEIGHBOURHOOD, neighborhood);
    roadTypes.put(RoadType.UNKNOWN, unknown);
  }
}
