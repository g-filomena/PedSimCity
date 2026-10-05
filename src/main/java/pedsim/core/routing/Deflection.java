package pedsim.core.routing;

import org.locationtech.jts.geom.Coordinate;
import sim.graph.NodeGraph;

/**
 * The deflection angle of a turn: the angle between walking from one point to a junction and
 * walking on from the junction to the next point. Each street is taken as its chord, from one end
 * to the other - cityImage's {@code deflection}, the {@code deg} it writes on a dual graph.
 */
public final class Deflection {

  private Deflection() {}

  /**
   * The deflection in degrees: 0 straight on, 180 straight back; 0 when either leg has no length.
   *
   * @param from Where the walk comes from.
   * @param junction Where it turns.
   * @param to Where it goes on to.
   * @return The deflection in degrees, in [0, 180].
   */
  public static double degrees(Coordinate from, Coordinate junction, Coordinate to) {
    double ax = junction.x - from.x;
    double ay = junction.y - from.y;
    double bx = to.x - junction.x;
    double by = to.y - junction.y;
    double magnitudeA = Math.sqrt(ax * ax + ay * ay);
    double magnitudeB = Math.sqrt(bx * bx + by * by);
    if (magnitudeA == 0 || magnitudeB == 0) {
      return 0.0;
    }
    double cosine = Math.max(-1.0, Math.min(1.0, (ax * bx + ay * by) / magnitudeA / magnitudeB));
    return Math.toDegrees(Math.acos(cosine));
  }

  /** The deflection in degrees of the turn at {@code junction}, between three nodes. */
  public static double degrees(NodeGraph from, NodeGraph junction, NodeGraph to) {
    return degrees(from.getCoordinate(), junction.getCoordinate(), to.getCoordinate());
  }
}
