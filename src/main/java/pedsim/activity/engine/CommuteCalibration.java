package pedsim.activity.engine;

import ec.util.MersenneTwisterFast;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.logging.Logger;
import pedsim.activity.agents.ActivityPurpose;
import pedsim.activity.agents.WorkplaceChoice;
import pedsim.activity.parameters.ActivityPars;
import pedsim.core.utilities.LoggerUtil;
import pedsim.core.parameters.Pars;
import sim.graph.GraphUtils;
import sim.graph.NodeGraph;

/**
 * What commute distribution does a given workplace decay produce?
 *
 * <p>The answer depends on the census homes, the WORK tags, the decay and the walk-share curve, and
 * on nothing else - not on agendas, not on releases, not on routes. So it is computed here directly
 * instead of by running simulated days and reading the log, which cost minutes per parameter value
 * and produced a share estimated over about a hundred workers.
 *
 * <p>Targets, from the ISTAT commuting matrix restricted to Torino residents working inside Torino
 * (see {@code COMMUTE_DISTANCE.md}): 16.3% of work commutes on foot, and among those, 76.2% under
 * 1,278 m, 19.0% to 2,556 m, 3.4% to 5,112 m, 1.3% beyond. The share and the shape are one target.
 *
 * <p>Run with {@code --calibrateCommute=true}: the environment is prepared, this runs, the process
 * exits before any simulated day.
 */
public final class CommuteCalibration {

  private static final Logger logger = LoggerUtil.getLogger();

  /** ISTAT band edges in metres: duration bands at Pars.pedestrianSpeed = 1.42 m/s. */
  private static final double[] BAND_EDGES = {1278.0, 2556.0, 5112.0};

  private static final double[] ISTAT_WORK_BANDS = {76.2, 19.0, 3.4, 1.3};

  private static final double ISTAT_WORK_SHARE = 16.3;

  /** Study commutes, same matrix, motivo = 1. Schools are more local than jobs, and it shows. */
  private static final double[] ISTAT_STUDY_BANDS = {87.5, 11.1, 0.9, 0.5};

  private static final double ISTAT_STUDY_SHARE = 38.0;

  private CommuteCalibration() {}

  /**
   * Sweeps the decay and the floor and reports the walked share and length distribution for each.
   *
   * @param homes how many home locations to draw
   * @param seed the generator seed
   */
  public static void run(int homes, long seed) {
    Map<NodeGraph, Double> work =
        PedSimCityActivity.nodesPurposeWeight.get(ActivityPurpose.WORK);
    if (work == null || work.isEmpty()) {
      logger.warning("commute calibration: this city carries no WORK tags; nothing to calibrate.");
      return;
    }

    MersenneTwisterFast random = new MersenneTwisterFast(seed);
    List<NodeGraph> homeNodes = sampleHomes(homes, random);
    if (homeNodes.isEmpty()) {
      logger.warning("commute calibration: no residential census zones with nodes.");
      return;
    }

    logger.info(
        String.format(
            "commute calibration: %d homes, %d WORK-tagged nodes, circuity %.3f%n"
                + "  target: 16.3%% walking, lengths 76.2 / 19.0 / 3.4 / 1.3",
            homeNodes.size(), work.size(), Pars.networkCircuityFactor));

    calibrate("WORK", work, ISTAT_WORK_SHARE, ISTAT_WORK_BANDS,
        ActivityPars.walkShareCommuteHalfDistance, ActivityPars.walkShareCommuteSteepness,
        homeNodes, random);

    Map<NodeGraph, Double> education =
        PedSimCityActivity.nodesPurposeWeight.get(ActivityPurpose.EDUCATION);
    if (education == null || education.isEmpty()) {
      logger.warning("commute calibration: no EDUCATION tags; skipping the student fit.");
    } else {
      calibrate("EDUCATION", education, ISTAT_STUDY_SHARE, ISTAT_STUDY_BANDS,
          ActivityPars.walkShareStudentHalfDistance, ActivityPars.walkShareStudentSteepness,
          homeNodes, random);
    }
  }

  /**
   * Sweeps decay and curve for one activity, against that activity's own ISTAT targets.
   *
   * <p>Run for work and for study separately because they are separate questions. A school
   * catchment is not a labour market: ISTAT puts 87.5% of walked study trips under fifteen minutes
   * against 76.2% of work trips, and 38.0% of study commutes on foot against 16.3%. Sharing one
   * decay between them was never a modelling decision, only an unexamined reuse of the same
   * method.
   */
  private static void calibrate(
      String label,
      Map<NodeGraph, Double> attraction,
      double targetShare,
      double[] targetBands,
      double currentHalf,
      double currentSteepness,
      List<NodeGraph> homeNodes,
      MersenneTwisterFast random) {

    double[] betas = {0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0};

    System.out.println();
    System.out.printf(
        "=== %s: target %.1f%% walking, lengths %.1f / %.1f / %.1f / %.1f (%d tagged nodes)%n",
        label, targetShare, targetBands[0], targetBands[1], targetBands[2], targetBands[3],
        attraction.size());

    System.out.printf("%nA. with this purpose's curve as it stands (half %.0f m, steepness %.5f)%n",
        currentHalf, currentSteepness);
    System.out.printf("%6s %10s   %31s   %8s%n",
        "beta", "walk share", "walked length bands (%)", "misfit");
    System.out.println("-".repeat(64));
    for (double beta : betas) {
      double[] d = commuteDistances(homeNodes, attraction, beta, 0.0, random);
      Result r = evaluate(d, currentHalf, currentSteepness, targetShare, targetBands);
      System.out.printf("%6.1f %9.1f%%   %7.1f %7.1f %7.1f %6.1f   %8.1f%n",
          beta, r.walkShare * 100.0,
          r.bands[0], r.bands[1], r.bands[2], r.bands[3], r.misfit);
    }

    System.out.println();
    System.out.println("B. best curve per decay (two parameters against five targets)");
    System.out.printf("%6s %10s %11s %10s   %31s   %8s%n",
        "beta", "half (m)", "steepness", "walk share", "walked length bands (%)", "misfit");
    System.out.println("-".repeat(88));
    for (double beta : betas) {
      double[] d = commuteDistances(homeNodes, attraction, beta, 0.0, random);
      double bestMisfit = Double.MAX_VALUE;
      double bestHalf = 0.0;
      double bestSteep = 0.0;
      Result best = null;
      for (double half = 200.0; half <= 3000.0; half += 50.0) {
        for (double steep = 0.0005; steep <= 0.0121; steep += 0.0002) {
          Result r = evaluate(d, half, steep, targetShare, targetBands);
          if (r.misfit < bestMisfit) {
            bestMisfit = r.misfit;
            bestHalf = half;
            bestSteep = steep;
            best = r;
          }
        }
      }
      System.out.printf("%6.1f %10.0f %11.5f %9.1f%%   %7.1f %7.1f %7.1f %6.1f   %8.1f%n",
          beta, bestHalf, bestSteep, best.walkShare * 100.0,
          best.bands[0], best.bands[1], best.bands[2], best.bands[3], best.misfit);
    }
    System.out.println();
  }

  /**
   * Draws one workplace per home and returns the estimated walked distance of each commute.
   *
   * <p>Separated from the curve because the workplace draw does not depend on it: the distances are
   * drawn once per decay and every candidate walk-share curve is then scored against the same
   * vector. That is what makes a joint fit over three parameters cheap enough to be worth running.
   */
  private static double[] commuteDistances(
      List<NodeGraph> homeNodes,
      Map<NodeGraph, Double> work,
      double beta,
      double floor,
      MersenneTwisterFast random) {

    List<Double> metres = new ArrayList<>(homeNodes.size());
    for (NodeGraph home : homeNodes) {
      NodeGraph workplace = WorkplaceChoice.draw(home, work, beta, floor, random);
      if (workplace == null) {
        continue;
      }
      // The same estimate decideCommuteMode uses: straight line scaled by measured circuity.
      metres.add(GraphUtils.nodesDistance(home, workplace) * Pars.networkCircuityFactor);
    }
    double[] out = new double[metres.size()];
    for (int i = 0; i < out.length; i++) {
      out[i] = metres.get(i);
    }
    return out;
  }

  /**
   * Scores a walk-share curve against a set of commute distances, in expectation.
   *
   * <p>Expectations rather than draws: the walk decision is a Bernoulli whose probability the curve
   * gives outright, so summing the probabilities is the same answer without the sampling noise, and
   * a fit that moves when the seed moves is not a fit.
   */
  private static Result evaluate(
      double[] distances,
      double halfDistance,
      double steepness,
      double targetShare,
      double[] targetBands) {
    double walked = 0.0;
    double[] counts = new double[4];
    for (double d : distances) {
      double p = 1.0 / (1.0 + Math.exp(steepness * (d - halfDistance)));
      walked += p;
      counts[band(d)] += p;
    }
    Result r = new Result();
    r.walkShare = distances.length > 0 ? walked / distances.length : 0.0;
    for (int i = 0; i < 4; i++) {
      r.bands[i] = walked > 0.0 ? 100.0 * counts[i] / walked : 0.0;
    }
    r.misfit = Math.abs(r.walkShare * 100.0 - targetShare);
    for (int i = 0; i < 4; i++) {
      r.misfit += Math.abs(r.bands[i] - targetBands[i]);
    }
    return r;
  }

  private static int band(double metres) {
    for (int i = 0; i < BAND_EDGES.length; i++) {
      if (metres <= BAND_EDGES[i]) {
        return i;
      }
    }
    return 3;
  }

  /** Draws home nodes the way {@code ActivityPopulate} does: residence-weighted census zones. */
  private static List<NodeGraph> sampleHomes(int howMany, MersenneTwisterFast random) {
    List<CensusZone> zones = new ArrayList<>();
    double total = 0.0;
    for (CensusZone zone : PedSimCityActivity.censusZones) {
      if (zone.residence > 0.0 && !zone.nodes.isEmpty()) {
        zones.add(zone);
        total += zone.residence;
      }
    }
    List<NodeGraph> homes = new ArrayList<>();
    if (zones.isEmpty() || total <= 0.0) {
      return homes;
    }
    double[] cumulative = new double[zones.size()];
    double running = 0.0;
    for (int i = 0; i < zones.size(); i++) {
      running += zones.get(i).residence;
      cumulative[i] = running;
    }
    for (int n = 0; n < howMany; n++) {
      double r = random.nextDouble() * total;
      int lo = 0;
      int hi = cumulative.length - 1;
      while (lo < hi) {
        int mid = (lo + hi) / 2;
        if (cumulative[mid] < r) {
          lo = mid + 1;
        } else {
          hi = mid;
        }
      }
      CensusZone zone = zones.get(lo);
      homes.add(zone.nodes.get(random.nextInt(zone.nodes.size())));
    }
    return homes;
  }

  private static final class Result {
    double walkShare;
    final double[] bands = new double[4];
    double misfit;
  }
}
