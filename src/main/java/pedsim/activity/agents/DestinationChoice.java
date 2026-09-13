package pedsim.activity.agents;

import ec.util.MersenneTwisterFast;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import pedsim.core.parameters.Pars;
import pedsim.activity.parameters.ActivityPars;
import sim.graph.NodeGraph;

/**
 * Destination choice as a choice, rather than as a search at a distance.
 *
 * <p>The mechanism this replaces ran quantity → length → place: the release drew a trip length, the
 * agent looked for nodes at that radius, and attraction picked among whatever the radius happened
 * to offer. Causality in the other direction is the odd part — people go somewhere because
 * something is there, and the distance is what it turns out to be — and every defect that took a
 * day to chase followed from it: how wide the interval should be, that an annulus offers more
 * candidates the further out you look, that attraction and radius were two mechanisms deciding the
 * same thing with nothing to arbitrate between them.
 *
 * <p>Here distance is a cost inside one utility, next to attraction and habit:
 *
 * <pre>{@code
 * U(j) = sizeWeight * ln(1 + attraction_j) - distanceWeight * d_ij + habitWeight * known_j
 * P(j) = exp(U_j) / sum_k exp(U_k)
 * }</pre>
 *
 * <p>The logarithm on attraction is the standard size term: twice the opportunities is not twice
 * the pull. Habit is a term rather than a branch that skips everything else, so a familiar place
 * competes with a better or nearer one instead of overriding it.
 *
 * <p><b>The trip-length distribution is now an output.</b> Nothing here targets a length;
 * {@code distanceWeight} shapes one, and whether the shape matches what travel surveys observe is
 * a result to be checked rather than an input to be honoured. That is the whole point of the
 * change, and it is also its cost: that coefficient has to be calibrated against an observed
 * distribution, where the mechanism it replaces needed no calibration because it was handed the
 * answer.
 */
public final class DestinationChoice {

  private DestinationChoice() {}

  /**
   * Draws a destination for a purpose, from the opportunities around an origin.
   *
   * @param candidates the choice set: nodes reachable from the origin
   * @param origin where the agent is standing
   * @param attraction per-node attraction for the purpose, or null when the city carries no tags
   * @param familiar places this agent already knows for this purpose; may be null or empty
   * @param random the agent's generator
   * @return the chosen node, or null when the choice set is empty
   */
  public static NodeGraph choose(
      List<NodeGraph> candidates,
      NodeGraph origin,
      Map<NodeGraph, Double> attraction,
      Map<NodeGraph, Integer> familiar,
      MersenneTwisterFast random) {

    if (candidates == null || candidates.isEmpty()) {
      return null;
    }

    // Sampling of alternatives. Enumerating every opportunity within reach costs about five times
    // what the mechanism this replaces did, on the hottest path in the model, and buys nothing:
    // McFadden showed a choice set drawn uniformly at random gives consistent estimates, and with
    // uniform sampling the correction term is the same for every alternative, so it cancels inside
    // the softmax. What follows is therefore the same choice, computed over a sample.
    if (candidates.size() > ActivityPars.choiceSetSize) {
      List<NodeGraph> sample = new ArrayList<>(ActivityPars.choiceSetSize);
      for (int i = 0; i < ActivityPars.choiceSetSize; i++) {
        sample.add(candidates.get(random.nextInt(candidates.size())));
      }
      candidates = sample;
    }

    double[] utility = new double[candidates.size()];
    double best = Double.NEGATIVE_INFINITY;

    for (int i = 0; i < candidates.size(); i++) {
      NodeGraph candidate = candidates.get(i);
      // Straight-line, scaled to what walking it actually costs. The impedance is a cost per metre
      // of pavement, and a destination 500 m away across a river is not 500 m of walking. Measuring
      // the real path to sixty candidates would mean sixty shortest-path searches per leg, so the
      // network's mean circuity stands in for it. With a linear impedance this is the same model as
      // one calibrated on straight lines - the scaling is absorbed into the coefficient - but it
      // keeps the coefficient meaning "utility per metre walked", which is the unit anyone
      // calibrating it will have. What it still cannot see is circuity that varies from place to
      // place, which is exactly where a river or a railway is.
      double metres =
          origin.getCoordinate().distance(candidate.getCoordinate())
              * Pars.networkCircuityFactor;

      double pull = attraction == null ? 0.0 : attraction.getOrDefault(candidate, 0.0);
      double u =
          ActivityPars.sizeWeight * Math.log1p(pull)
              - ActivityPars.distanceWeight * metres;
      if (familiar != null && familiar.containsKey(candidate)) {
        u += ActivityPars.habitWeight;
      }
      utility[i] = u;
      best = Math.max(best, u);
    }

    // Softmax, shifted by the maximum so that exp() cannot overflow on a large utility spread.
    double total = 0.0;
    for (int i = 0; i < utility.length; i++) {
      utility[i] = Math.exp(utility[i] - best);
      total += utility[i];
    }
    if (!(total > 0.0) || Double.isNaN(total)) {
      return candidates.get(random.nextInt(candidates.size()));
    }

    double draw = random.nextDouble() * total;
    double cumulative = 0.0;
    for (int i = 0; i < utility.length; i++) {
      cumulative += utility[i];
      if (draw <= cumulative) {
        return candidates.get(i);
      }
    }
    return candidates.get(candidates.size() - 1);
  }
}
