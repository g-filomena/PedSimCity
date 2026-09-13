package pedsim.core.engine;

import ec.util.MersenneTwisterFast;
import java.util.List;
import java.util.logging.Logger;
import pedsim.core.parameters.Pars;
import pedsim.core.utilities.LoggerUtil;
import sim.graph.Graph;
import sim.graph.NodeGraph;
import sim.graph.NodesLookup;
import sim.routing.Astar;
import sim.routing.Route;

/**
 * How much longer walking somewhere is than the straight line to it, measured on the city at hand.
 *
 * <p>The destination utility charges a cost per metre walked, and what it has to work with is the
 * straight line between two nodes. The gap between the two is this network's circuity. It is a
 * property of the street layout rather than of behaviour, so the model measures it from the streets
 * it has just loaded rather than taking a configured figure that would be right for one city and
 * wrong for the next.
 *
 * <p>Sampled rather than exhaustive: a few hundred pairs settle a mean ratio well enough, and the
 * pairs are drawn from the distances people actually walk, because circuity is not constant across
 * them - short trips are proportionally more crooked, so a figure averaged over the wrong range
 * answers the wrong question.
 *
 * <p>An explicit value set from the command line is left alone. Measuring is the default, not a
 * rule.
 */
public final class NetworkCircuity {

  private static final Logger logger = LoggerUtil.getLogger();

  /** The walking range the ratio is averaged over, in metres. */
  private static final double MIN_PAIR_METRES = 200.0;

  private static final double MAX_PAIR_METRES = 3000.0;

  /** Pairs to average over, and how many attempts to allow for finding them. */
  private static final int PAIRS = 300;

  private static final int MAX_ATTEMPTS = PAIRS * 8;

  private NetworkCircuity() {}

  /**
   * Measures the network's circuity and stores it in {@link Pars#networkCircuityFactor}.
   *
   * @param network the primal graph the agents walk
   */
  public static void measureInto(Graph network) {
    if (!Pars.measureNetworkCircuity) {
      logger.info(
          "network circuity: measurement disabled, using " + Pars.networkCircuityFactor);
      return;
    }
    if (network == null || network.getNodes().isEmpty()) {
      return;
    }
    // A fixed seed on purpose: the crookedness of a city is a property of the city, so it should
    // not come out differently because the run was seeded differently.
    MersenneTwisterFast random = new MersenneTwisterFast(20260911L);

    Astar astar = new Astar();
    double straightTotal = 0.0;
    double walkedTotal = 0.0;
    int pairs = 0;

    for (int attempt = 0; attempt < MAX_ATTEMPTS && pairs < PAIRS; attempt++) {
      NodeGraph origin = NodesLookup.randomNode(network, random);
      if (origin == null) {
        continue;
      }
      List<NodeGraph> reachable =
          NodesLookup.getNodesBetweenDistanceInterval(
              network, origin, MIN_PAIR_METRES, MAX_PAIR_METRES);
      if (reachable.isEmpty()) {
        continue;
      }
      NodeGraph destination = reachable.get(random.nextInt(reachable.size()));
      Route route = astar.astarRoute(origin, destination, network, null);
      if (route == null || route.getLength() <= 0.0) {
        continue;
      }
      straightTotal += origin.getCoordinate().distance(destination.getCoordinate());
      walkedTotal += route.getLength();
      pairs++;
    }

    if (pairs == 0 || straightTotal <= 0.0) {
      logger.info(
          "network circuity: could not be measured, keeping "
              + Pars.networkCircuityFactor);
      return;
    }

    double measured = walkedTotal / straightTotal;
    Pars.networkCircuityFactor = measured;
    logger.info(
        String.format(
            "network circuity measured on %d pairs between %.0f and %.0f m: %.3f",
            pairs, MIN_PAIR_METRES, MAX_PAIR_METRES, measured));
  }

  /**
   * The straight-line distance whose route is expected to be {@code routeMetres} long on foot.
   *
   * <p>A division, and the single place in the model that performs it. Node lookup works in
   * Euclidean distance - {@code NodesLookup.getNodesBetweenDistanceInterval} and
   * {@code randomNodeBetweenDistanceInterval} both take a straight-line interval - while
   * {@link Pars#minRouteLength} and {@link Pars#maxRouteLength} are stated in walked metres. Passing
   * one where the other is expected fails quietly: the run completes and every route is too long by
   * the city's circuity, which is 29% on Torino and 54% on Melbourne. Call this rather than dividing
   * at the call site, so that every caller means the same thing by a length.
   *
   * @param routeMetres the desired walked length of the route, in metres
   * @return the straight-line distance to search for, in metres
   */
  public static double straightLineFor(double routeMetres) {
    double factor = Pars.networkCircuityFactor;
    return factor > 0.0 ? routeMetres / factor : routeMetres;
  }
}
