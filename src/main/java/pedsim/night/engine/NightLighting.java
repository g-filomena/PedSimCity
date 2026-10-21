package pedsim.night.engine;

import java.util.ArrayList;
import java.util.Arrays;
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
   * How dark an illuminance reads to an agent with a given reassurance level, from 1.0 at 0 lux to
   * 0.0 at that level and above, falling as {@code 1 - ln(1 + lux) / ln(1 + reassuranceLux)}.
   *
   * <p>Concave, because reassurance gains most from the first few lux and little past the
   * plateau (Fotios, Unwin and Farrall 2015; Portnov, Fotios et al. 2024): at a 10 lx level a
   * street at 2 lux reads as 0.54 dark, one at 5 lux as 0.25.
   *
   * @param lux the street's illuminance
   * @param reassuranceLux the illuminance at and above which the agent is not troubled by the dark
   */
  public static double darkness(double lux, double reassuranceLux) {
    if (reassuranceLux <= 0.0 || lux >= reassuranceLux) {
      return 0.0;
    }
    if (lux <= 0.0) {
      return 1.0;
    }
    return 1.0 - Math.log1p(lux) / Math.log1p(reassuranceLux);
  }

  /**
   * How dark an agent with a given reassurance level expects a street it does not know to be: the
   * {@link #darkness} of the city's streets of the same OSM {@code highway} class, averaged over
   * their length, or of all streets for a class with none.
   *
   * <p>The expected darkness, not the darkness of the typical illuminance: a class whose median
   * street is lit, as residential streets are in Turin, would otherwise read as entirely lit
   * although a quarter of its length is not.
   *
   * @param edge the street
   * @param reassuranceLux the agent's reassurance level
   */
  public static double expectedDarkness(EdgeGraph edge, double reassuranceLux) {
    Map<String, ClassLighting> byClass = lightingByClass;
    if (byClass == null) {
      byClass = computeLightingByClass();
      lightingByClass = byClass;
    }
    ClassLighting lighting = byClass.get(highwayClass(edge));
    if (lighting == null) {
      lighting = byClass.get(ALL_CLASSES);
    }
    return lighting == null ? 0.0 : lighting.expectedDarkness(reassuranceLux);
  }

  /**
   * The length-weighted mean {@link #darkness} of a set of streets.
   *
   * @param lengths each street's length
   * @param lux each street's illuminance, in the same order
   * @param reassuranceLux the reassurance level darkness is read at
   * @return the expected darkness, 0 for no length
   */
  static double expectedDarkness(double[] lengths, double[] lux, double reassuranceLux) {
    return new ClassLighting(lengths, lux).expectedDarkness(reassuranceLux);
  }

  /**
   * The lighting of one street class, held so that its expected darkness can be read at any
   * reassurance level without a pass over the streets. With the streets sorted by illuminance, the
   * streets below a level {@code R} contribute {@code length - length * ln(1 + lux) / ln(1 + R)},
   * so two running sums over the sorted streets give the expectation by binary search.
   */
  static final class ClassLighting {
    private final double[] sortedLux;
    private final double[] lengthBelow;
    private final double[] logWeightedLengthBelow;
    private final double totalLength;

    ClassLighting(double[] lengths, double[] lux) {
      Integer[] order = new Integer[lux.length];
      for (int i = 0; i < order.length; i++) {
        order[i] = i;
      }
      Arrays.sort(order, (a, b) -> Double.compare(lux[a], lux[b]));
      sortedLux = new double[lux.length];
      lengthBelow = new double[lux.length + 1];
      logWeightedLengthBelow = new double[lux.length + 1];
      for (int k = 0; k < order.length; k++) {
        int i = order[k];
        double streetLux = Math.max(0.0, lux[i]);
        sortedLux[k] = streetLux;
        lengthBelow[k + 1] = lengthBelow[k] + lengths[i];
        logWeightedLengthBelow[k + 1] =
            logWeightedLengthBelow[k] + lengths[i] * Math.log1p(streetLux);
      }
      totalLength = lengthBelow[lux.length];
    }

    double expectedDarkness(double reassuranceLux) {
      if (totalLength <= 0.0 || reassuranceLux <= 0.0) {
        return 0.0;
      }
      int below = firstAtOrAbove(reassuranceLux);
      double dark = lengthBelow[below] - logWeightedLengthBelow[below] / Math.log1p(reassuranceLux);
      return dark / totalLength;
    }

    /** The number of streets whose illuminance is below the given level. */
    private int firstAtOrAbove(double level) {
      int low = 0;
      int high = sortedLux.length;
      while (low < high) {
        int mid = (low + high) >>> 1;
        if (sortedLux[mid] < level) {
          low = mid + 1;
        } else {
          high = mid;
        }
      }
      return low;
    }
  }

  private static final String ALL_CLASSES = "";

  private static volatile Map<String, ClassLighting> lightingByClass;

  private static Map<String, ClassLighting> computeLightingByClass() {
    Map<String, List<double[]>> streets = new HashMap<>();
    for (EdgeGraph edge : SharedCognitiveMap.getCommunityPrimalNetwork().getEdges()) {
      double[] street = {edge.getLength(), measuredLux(edge)};
      for (String key : new String[] {highwayClass(edge), ALL_CLASSES}) {
        streets.computeIfAbsent(key, k -> new ArrayList<>()).add(street);
      }
    }
    Map<String, ClassLighting> byClass = new HashMap<>();
    streets.forEach(
        (key, list) -> {
          double[] lengths = new double[list.size()];
          double[] lux = new double[list.size()];
          for (int i = 0; i < lengths.length; i++) {
            lengths[i] = list.get(i)[0];
            lux[i] = list.get(i)[1];
          }
          byClass.put(key, new ClassLighting(lengths, lux));
        });
    return byClass;
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
    lightingByClass = null;
  }
}
