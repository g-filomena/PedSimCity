package pedsim.night.engine;

import java.util.HashSet;
import java.util.Set;
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

  private static volatile Set<EdgeGraph> unlitEdgesOutsideCommunityKnown;

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
    Set<EdgeGraph> cached = unlitEdgesOutsideCommunityKnown;
    if (cached == null) {
      cached = unlitEdges(outsideCommunityKnown, NightPars.nonVulnerableLightSensitivity);
      unlitEdgesOutsideCommunityKnown = cached;
    }
    return cached;
  }

  /**
   * Those of {@code candidates} that read as unlit to an agent with this sensitivity threshold.
   *
   * <p>Not cached: a vulnerable agent's threshold is drawn per agent, so there is no one answer to
   * keep. The cost is one {@link #isLit} test per candidate, against the {@code addAll} of the same
   * set that the caller was already paying.
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
    unlitEdgesOutsideCommunityKnown = null;
  }
}
