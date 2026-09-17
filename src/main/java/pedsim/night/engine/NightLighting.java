package pedsim.night.engine;

import java.util.Collections;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.night.parameters.NightPars;
import sim.graph.EdgeGraph;
import sim.util.geo.AttributeValue;

/**
 * Whether a street counts as lit: one rule, for the agent's own gate and for the edges it detours
 * around alike.
 *
 * <p>A tag is data; <i>lit</i> is a judgement against a {@link NightPars} threshold, so it belongs
 * to this module rather than to the cognitive map.
 *
 * <p>The sets are derived once per network and dropped by {@link
 * PedSimCityNight#clearNightStaticData()}, so a re-imported city is not answered from the old one.
 */
public final class NightLighting {

  private static volatile Set<EdgeGraph> taggedLitEdges;

  /**
   * One unlit set per sensitivity threshold, over the edges outside the community-known network.
   *
   * <p>Which edges read as unlit depends on the threshold and nothing else, so agents sharing one
   * share the answer. Vulnerable thresholds are drawn onto the
   * {@link NightPars#lightSensitivityQuantumLux} grid, which is what makes them shareable; the
   * non-vulnerable threshold is a single value and is just another key here.
   *
   * <p>Keyed on the threshold alone, which is only safe because the candidate set is itself one
   * per network. {@code NightAgentMovement.clearCachedNetworkSets} drops both together.
   */
  private static final Map<Double, Set<EdgeGraph>> unlitByThreshold = new ConcurrentHashMap<>();

  private NightLighting() {}

  /**
   * Whether an edge reads as lit to an agent with this sensitivity threshold: bright enough on
   * average <b>and</b> with no unlit gap along it.
   *
   * <p>The second test is what an average cannot see. {@code min_lux} is the darkest 2 m sample
   * point on the edge, and a street bright at both ends and black in the middle passes on
   * {@code mean_lux} alone. It is compared against {@link NightPars#darkSpotLuxThreshold}, the
   * pipeline's own service level, not against the agent's own threshold: the minimum over a whole
   * edge is an extreme value, and asking it to clear a 15-lux threshold would fail nearly every
   * street in the city.
   *
   * <p>With no continuous measurement the binary tag decides, which is all a city without a
   * lighting pipeline has.
   */
  public static boolean isLit(EdgeGraph edge, double threshold) {
    var meanLuxAttr = edge.attributes.get("mean_lux");
    if (meanLuxAttr == null) {
      return isTaggedLit(edge);
    }
    if (meanLuxAttr.getDouble() < threshold) {
      return false;
    }
    var minLuxAttr = edge.attributes.get("min_lux");
    return minLuxAttr == null || minLuxAttr.getDouble() >= NightPars.darkSpotLuxThreshold;
  }

  /**
   * How far an edge falls below a lighting threshold, as a fraction: 0.0 at or above it, 1.0 at
   * total darkness, and 0.0 wherever no continuous {@code mean_lux} exists, so an unmeasured edge
   * is never charged for a darkness nobody measured.
   *
   * <p>One definition for both readers of it: the situated choice between turning off a street and
   * walking it faster ({@code NightBehaviour.rerouteOrIncreaseSpeed}) and the planning cost that
   * steers a route away from one before the agent sets off ({@code
   * DijkstraRoadDistanceNight.lightingCostMultiplier}). They are the same question asked at two
   * moments, and they were two copies of the same four lines.
   *
   * <p>{@code mean_lux}, not the directional entrance value: this grades the edge as a whole,
   * which is what a cost and a reroute probability both want. The direction-specific view belongs
   * to the gate that fires on arrival.
   */
  public static double darknessDepth(EdgeGraph edge, double threshold) {
    if (threshold <= 0) {
      return 0.0;
    }
    var meanLuxAttr = edge.attributes.get("mean_lux");
    if (meanLuxAttr == null) {
      return 0.0;
    }
    double lux = meanLuxAttr.getDouble();
    if (lux >= threshold) {
      return 0.0;
    }
    return Math.min(1.0, (threshold - lux) / threshold);
  }

  /**
   * Whether the OSM {@code lit} tag claims this edge is lit. A claim, not a measurement: it says a
   * lamp exists, not how much of the street it reaches.
   */
  public static boolean isTaggedLit(EdgeGraph edge) {
    Set<EdgeGraph> cached = taggedLitEdges;
    if (cached == null) {
      cached = new HashSet<>();
      for (EdgeGraph candidate : SharedCognitiveMap.getCommunityPrimalNetwork().getEdges()) {
        if (readsAsLit(candidate.attributes.get("lit"))) {
          cached.add(candidate);
        }
      }
      taggedLitEdges = cached;
    }
    return cached.contains(edge);
  }

  /**
   * Every unlit city edge outside the community-known network: the fixed half of a non-vulnerable
   * agent's avoid-set, unlit by {@link #isLit} at {@link NightPars#nonVulnerableLightSensitivity},
   * so what frightens an agent onto a detour and what it detours around are one definition.
   *
   * <p>Cached, because that threshold is one number for every non-vulnerable agent in the run. A
   * vulnerable agent draws its own and goes through {@link #unlitEdges} instead.
   *
   * @param outsideCommunityKnown the edges no one in the city is taken to know
   */
  public static Set<EdgeGraph> unlitEdgesOutsideCommunityKnown(
      Set<EdgeGraph> outsideCommunityKnown) {
    return unlitEdgesOutsideCommunityKnown(
        outsideCommunityKnown, NightPars.nonVulnerableLightSensitivity);
  }

  /**
   * Those of {@code outsideCommunityKnown} that read as unlit at {@code threshold}, computed once
   * per threshold.
   *
   * <p>The returned set is shared and must not be modified; callers copy it into their own
   * avoid-set.
   *
   * @param outsideCommunityKnown the edges no one in the city is taken to know
   * @param threshold the agent's light-sensitivity threshold, in lux
   */
  public static Set<EdgeGraph> unlitEdgesOutsideCommunityKnown(
      Set<EdgeGraph> outsideCommunityKnown, double threshold) {
    return unlitByThreshold.computeIfAbsent(
        threshold, lux -> Collections.unmodifiableSet(unlitEdges(outsideCommunityKnown, lux)));
  }

  /**
   * Those of {@code candidates} that read as unlit to an agent with this sensitivity threshold.
   *
   * <p>One {@link #isLit} test per candidate, so it is O(candidates). Callers that ask repeatedly
   * over the whole network go through
   * {@link #unlitEdgesOutsideCommunityKnown(Set, double)}, which keeps one answer per threshold.
   */
  public static Set<EdgeGraph> unlitEdges(Set<EdgeGraph> candidates, double threshold) {
    Set<EdgeGraph> unlit = new HashSet<>();
    for (EdgeGraph edge : candidates) {
      if (!isLit(edge, threshold)) {
        unlit.add(edge);
      }
    }
    return unlit;
  }

  /**
   * Reads the raw {@code lit} attribute, which arrives as a boolean from some layers and as 1/0
   * from others, and anything else is not a claim of light.
   */
  private static boolean readsAsLit(AttributeValue lit) {
    if (lit == null) {
      return false;
    }
    Object value = lit.getValue();
    if (value instanceof Boolean flag) {
      return flag;
    }
    return value instanceof Integer number && number != 0;
  }

  /** Drops the derived sets, so a re-imported network is not answered from the old one. */
  public static void clearCaches() {
    taggedLitEdges = null;
    unlitByThreshold.clear();
  }
}
