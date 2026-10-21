package pedsim.night.engine;

import java.util.EnumSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.logging.Logger;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.index.strtree.STRtree;
import pedsim.activity.agents.ActivityPurpose;
import pedsim.activity.engine.PedSimCityActivity;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.engine.PedSimCity;
import pedsim.core.utilities.LoggerUtil;
import pedsim.night.parameters.NightPars;
import sim.graph.EdgeGraph;
import sim.util.geo.MasonGeometry;

/**
 * How many eyes a street has on it at a given hour: the venues open along it, after Jacobs (1961)
 * - shops, cafés, restaurants and bars give a street people with a reason to be there and to watch
 * it.
 *
 * <p>Each venue of a purpose with an opening window ({@link #PURPOSES}) is placed on the nearest
 * street within {@link NightPars#eyesSnapMetres}, and counts at the hours its purpose is open
 * ({@link ActivityPurpose#isOpenAt}, taken at the middle of the hour). A street's eyes are {@code 1 -
 * exp(-density / NightPars.eyesSaturationPer100m)}, with density the open venues per 100 m of
 * street and the street's length floored at {@link #MIN_LENGTH_METRES}, so a short stretch at a
 * corner does not read as lively on the strength of one bar.
 *
 * <p>The venues come from the POI layer, or from the buildings layer when a city has no POIs: a
 * building's use list names the venues inside it, so reading both would count them twice.
 *
 * <p>An agent knows these as it knows the typical lighting of a class of street: everyone knows
 * where the bars are, so the same expectation serves every agent, at planning and on the way.
 */
public final class EyesOnStreet {

  private static final Logger logger = LoggerUtil.getLogger();

  /** Purposes whose venues are open to the street and have opening hours. */
  static final Set<ActivityPurpose> PURPOSES =
      EnumSet.of(
          ActivityPurpose.SHOPPING,
          ActivityPurpose.ERRANDS,
          ActivityPurpose.DINING,
          ActivityPurpose.NIGHTLIFE);

  /** A street shorter than this is measured as if it were this long. */
  static final double MIN_LENGTH_METRES = 50.0;

  private static volatile Map<EdgeGraph, double[]> eyesByHour = Map.of();

  private EyesOnStreet() {}

  /**
   * The eyes on a street at an hour of the day, in [0, 1]; 0 for a street with no venue.
   *
   * @param hour the hour of day, 0-23
   */
  public static double at(EdgeGraph edge, int hour) {
    double[] byHour = eyesByHour.get(edge);
    return byHour == null ? 0.0 : byHour[Math.floorMod(hour, 24)];
  }

  /**
   * The eyes a number of open venues gives a street of a given length.
   *
   * @param openVenues venues open along the street
   * @param lengthMetres the street's length
   */
  static double eyes(double openVenues, double lengthMetres) {
    if (openVenues <= 0.0 || NightPars.eyesSaturationPer100m <= 0.0) {
      return 0.0;
    }
    double density = 100.0 * openVenues / Math.max(lengthMetres, MIN_LENGTH_METRES);
    return 1.0 - Math.exp(-density / NightPars.eyesSaturationPer100m);
  }

  /** Places the city's venues on its streets and derives each street's eyes for every hour. */
  public static void prepare() {
    List<EdgeGraph> edges = SharedCognitiveMap.getCommunityPrimalNetwork().getEdges();
    List<MasonGeometry> venues = PedSimCityActivity.poisLayer.getGeometries();
    String source = "POIs";
    if (venues.isEmpty()) {
      venues = PedSimCity.buildings.getGeometries();
      source = "buildings";
    }
    if (edges.isEmpty() || venues.isEmpty()) {
      eyesByHour = Map.of();
      logger.info("eyes on the street: no venues, so no street has any");
      return;
    }

    STRtree index = new STRtree();
    for (EdgeGraph edge : edges) {
      index.insert(edge.getLine().getEnvelopeInternal(), edge);
    }

    // LinkedHashMap: edges are visited in network order, so the map iterates the same way on
    // every machine (EdgeGraph's hash is its centroid, but insertion order is the safer promise).
    Map<EdgeGraph, double[]> openByHour = new LinkedHashMap<>();
    int placed = 0;
    for (MasonGeometry venue : venues) {
      Geometry geometry = venue.getGeometry();
      if (geometry == null) {
        continue;
      }
      Set<ActivityPurpose> purposes = ActivityPurpose.classifyAll(venue.getAttributes());
      purposes.retainAll(PURPOSES);
      if (purposes.isEmpty()) {
        continue;
      }
      EdgeGraph street = nearestStreet(geometry, index);
      if (street == null) {
        continue;
      }
      double[] open = openByHour.computeIfAbsent(street, e -> new double[24]);
      for (int hour = 0; hour < 24; hour++) {
        for (ActivityPurpose purpose : purposes) {
          if (purpose.isOpenAt(hour + 0.5)) {
            open[hour] += 1.0;
            break;
          }
        }
      }
      placed++;
    }

    Map<EdgeGraph, double[]> result = new LinkedHashMap<>();
    openByHour.forEach(
        (edge, open) -> {
          double[] eyes = new double[24];
          for (int hour = 0; hour < 24; hour++) {
            eyes[hour] = eyes(open[hour], edge.getLength());
          }
          result.put(edge, eyes);
        });
    eyesByHour = result;

    long lively = result.values().stream().filter(eyes -> eyes[21] >= 0.5).count();
    logger.info(
        String.format(
            "eyes on the street: %d venues from the %s layer on %d of %d streets; at 21:00, %d"
                + " streets have eyes >= 0.5",
            placed, source, result.size(), edges.size(), lively));
  }

  private static EdgeGraph nearestStreet(Geometry geometry, STRtree index) {
    Envelope envelope = new Envelope(geometry.getEnvelopeInternal());
    envelope.expandBy(NightPars.eyesSnapMetres);
    @SuppressWarnings("unchecked")
    List<EdgeGraph> candidates = index.query(envelope);
    EdgeGraph best = null;
    double bestDistance = NightPars.eyesSnapMetres;
    for (EdgeGraph candidate : candidates) {
      double distance = geometry.distance(candidate.getLine());
      if (distance < bestDistance
          || (distance == bestDistance && best != null && candidate.getID() < best.getID())) {
        bestDistance = distance;
        best = candidate;
      }
    }
    return best;
  }

  /** Drops the derived eyes, so a re-imported city is not answered from the old one. */
  public static void clearCaches() {
    eyesByHour = Map.of();
  }
}
