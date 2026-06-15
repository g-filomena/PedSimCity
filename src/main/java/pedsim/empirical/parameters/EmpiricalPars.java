package pedsim.empirical.parameters;

import java.util.ArrayList;
import java.util.List;
import pedsim.core.parameters.Pars;
import pedsim.empirical.agent.EmpiricalAgentsGroup;

/**
 * Parameters specific to the empirical ABM mode.
 *
 * Core simulation parameters remain in {@link Pars}; this class only stores
 * empirical-mode parameters that should not live in cityimage.TestPars.
 */
public final class EmpiricalPars {

  private EmpiricalPars() {}

  public static final List<EmpiricalAgentsGroup> empiricalGroups = new ArrayList<>();

  public static int numberTripsPerAgent = 3;

  public static boolean usingDMA = true;

  public static boolean includePopulationBenchmark = true;
  public static boolean includeNullBenchmark = true;

  public static int defaultNumAgents = 301;
  public static int numAgents = defaultNumAgents;

  public static int defaultJobs = 10;

  public static String defaultCityName = "Muenster";

  public static void applyDefaults() {
    Pars.cityName = defaultCityName;
    Pars.setSimulationParameters();
    Pars.numAgents = numAgents;
    Pars.jobs = defaultJobs;
  }
}
