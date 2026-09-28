package pedsim.night.engine;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.night.parameters.NightPars;
import sim.graph.EdgeGraph;
import sim.util.geo.AttributeValue;

/**
 * How a street's lighting reads to a night agent: whether it counts as lit, how dark it is, and
 * what an agent expects of a street it has not walked.
 *
 * <p>A tag is data; <i>lit</i> is a judgement against a {@link NightPars} threshold, so it belongs
 * to this module rather than to the cognitive map.
 *
 * <p>The sets are derived once per network and dropped by {@link
 * PedSimCityNight#clearNightStaticData()}, so a re-imported city is not answered from the old one.
 */
public final class NightLighting {

  private static volatile Set<EdgeGraph> taggedLitEdges;

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
   * The illuminance of an edge as the model knows it: the measured {@code mean_lux} where the
   * lighting pipeline wrote one, {@link NightPars#litEdgeNominalLux} for an edge known lit only
   * through the OSM tag, and 0 for an edge that is neither.
   */
  public static double measuredLux(EdgeGraph edge) {
    var meanLuxAttr = edge.attributes.get("mean_lux");
    if (meanLuxAttr != null) {
      return meanLuxAttr.getDouble();
    }
    return isTaggedLit(edge) ? NightPars.litEdgeNominalLux : 0.0;
  }

  /**
   * How dark an illuminance reads, from 1.0 at 0 lux to 0.0 at {@link NightPars#reassuranceLux}
   * and above, falling as {@code 1 - ln(1 + lux) / ln(1 + reassuranceLux)}.
   *
   * <p>Concave, because reassurance gains most from the first few lux and little past the
   * plateau (Fotios, Unwin and Farrall 2015; Portnov, Fotios et al. 2024): a street at 2 lux reads
   * as 0.54 dark, one at 5 lux as 0.25.
   */
  public static double darkness(double lux) {
    double reference = NightPars.reassuranceLux;
    if (reference <= 0.0 || lux >= reference) {
      return 0.0;
    }
    if (lux <= 0.0) {
      return 1.0;
    }
    return 1.0 - Math.log1p(lux) / Math.log1p(reference);
  }

  /**
   * The illuminance an agent expects of a street it does not know: the median {@link #measuredLux}
   * of the city's streets of the same OSM {@code highway} class, or of all streets for a class
   * with none. What anyone would guess of a residential street or a main road they have not
   * walked, the same guess for every agent.
   */
  public static double typicalLux(EdgeGraph edge) {
    Map<String, Double> byClass = typicalLuxByClass;
    if (byClass == null) {
      byClass = computeTypicalLuxByClass();
      typicalLuxByClass = byClass;
    }
    Double lux = byClass.get(highwayClass(edge));
    return lux != null ? lux : byClass.getOrDefault(ALL_CLASSES, 0.0);
  }

  private static final String ALL_CLASSES = "";

  private static volatile Map<String, Double> typicalLuxByClass;

  private static Map<String, Double> computeTypicalLuxByClass() {
    Map<String, List<Double>> samples = new HashMap<>();
    for (EdgeGraph edge : SharedCognitiveMap.getCommunityPrimalNetwork().getEdges()) {
      double lux = measuredLux(edge);
      samples.computeIfAbsent(highwayClass(edge), key -> new ArrayList<>()).add(lux);
      samples.computeIfAbsent(ALL_CLASSES, key -> new ArrayList<>()).add(lux);
    }
    Map<String, Double> medians = new HashMap<>();
    samples.forEach(
        (key, values) -> {
          Collections.sort(values);
          int n = values.size();
          medians.put(
              key,
              n % 2 == 1 ? values.get(n / 2) : (values.get(n / 2 - 1) + values.get(n / 2)) / 2.0);
        });
    return medians;
  }

  private static String highwayClass(EdgeGraph edge) {
    var highway = edge.attributes.get("highway");
    if (highway == null) {
      return ALL_CLASSES;
    }
    String value = highway.getString();
    return value == null ? ALL_CLASSES : value.trim();
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
    typicalLuxByClass = null;
  }
}
