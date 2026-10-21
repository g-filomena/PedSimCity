package pedsim.night.engine;

import pedsim.night.parameters.NightPars;
import sim.graph.EdgeGraph;

/**
 * How enclosed a street is, from how little can be seen from it: 0 for a street whose smallest
 * isovist is at least {@link NightPars#enclosedAreaM2}, rising linearly to 1 as that area falls to
 * nothing. Low prospect is what makes a place feel unsafe after dark (Fisher and Nasar 1992); the
 * isovist is its measure (Benedikt 1979).
 *
 * <p>The smallest isovist, not the mean: a street that narrows once is enclosed where it narrows,
 * however open the rest of it is. Isovists are cast against building footprints only, so a street
 * through trees or along walls reads as open; parks have their own term.
 */
public final class Enclosure {

  private Enclosure() {}

  /** How enclosed an edge is, in [0, 1]; 0 for an edge with no isovist measured. */
  public static double of(EdgeGraph edge) {
    Double areaMin = PedSimCityNight.streetIsovistAreaMin.get(edge.getID());
    return areaMin == null ? 0.0 : enclosure(areaMin);
  }

  /**
   * How enclosed a street with this smallest isovist area is.
   *
   * @param areaMin the smallest isovist area along the street, in m2
   */
  static double enclosure(double areaMin) {
    double open = NightPars.enclosedAreaM2;
    if (open <= 0.0 || areaMin >= open) {
      return 0.0;
    }
    return 1.0 - Math.max(areaMin, 0.0) / open;
  }
}
