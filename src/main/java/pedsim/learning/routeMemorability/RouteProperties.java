package pedsim.learning.routeMemorability;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.stream.IntStream;
import org.javatuples.Pair;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.MultiPolygon;
import org.locationtech.jts.geom.Polygon;
import org.locationtech.jts.geom.util.GeometryCombiner;
import org.locationtech.jts.operation.union.UnaryUnionOp;
import pedsim.core.engine.PedSimCity;
import pedsim.core.parameters.RouteChoicePars;
import sim.field.geo.VectorLayer;
import sim.graph.NodeGraph;
import sim.routing.Route;

/**
 * The space a walked route lets its walker see: what an agent remembers of a route is what it saw
 * along it, not the polyline alone.
 *
 * <p>The visibility space is the union of the route's buffer, a buffer around each of its nodes and,
 * at each node, a forward cone clipped by the buildings in the way.
 */
public class RouteProperties {

  /** Angular step, in degrees, between the rays that bound a visibility cone. */
  static final int DISTANCE_ALONG_VISIBILITY = 10;

  /** Aperture, in degrees, of the forward visibility cone at each node. */
  static final double VISIBILITY_ANGLE = 100.0;

  /** Radius, in metres, of the buffer around the route and around each of its nodes. */
  static final double BUFFER_RADIUS = 20.0;

  private final Route route;
  private Geometry routeBuffer;
  private Polygon visibilitySpace;

  protected static GeometryFactory GEOMETRY_FACTORY = new GeometryFactory();

  /** Per-process cache of visibility cones, keyed by node-coordinate pair. */
  private static final Map<Pair<Coordinate, Coordinate>, Polygon> visibilityPolygonsCache =
      new ConcurrentHashMap<>();

  /**
   * Clear the static visibility-cone cache. Called from the simulation module's {@code
   * clearStaticData()} so cones (which depend on the run's obstruction layer) are not reused across
   * jobs and the map does not grow unbounded over a process's lifetime.
   */
  public static void clearVisibilityCache() {
    visibilityPolygonsCache.clear();
  }

  public RouteProperties(Route route) {
    this.route = route;
    route.attributes.put("properties", this);
  }

  public void computeRouteProperties() {
    routeBuffer = route.getLineString().buffer(BUFFER_RADIUS);
    computeVisibilitySpace();
  }

  private void computeVisibilitySpace() {
    List<Geometry> visibleSpaces = new ArrayList<>();
    List<NodeGraph> nodes = route.nodesSequence;
    for (int i = 0; i < nodes.size(); i++) {
      NodeGraph node = nodes.get(i);
      if (i < nodes.size() - 1) {
        Polygon cone =
            createVisibilityCone(
                node,
                nodes.get(i + 1),
                VISIBILITY_ANGLE,
                PedSimCity.buildings,
                RouteChoicePars.threshold3dVisibility);
        if (cone != null) visibleSpaces.add(cone);
      }
      visibleSpaces.add(node.getMasonGeometry().geometry.buffer(BUFFER_RADIUS));
    }

    visibleSpaces.add(routeBuffer);
    Geometry combined = GeometryCombiner.combine(visibleSpaces);

    if (combined instanceof MultiPolygon mp && mp.getNumGeometries() == 1)
      visibilitySpace = (Polygon) mp.getGeometryN(0);
    else if (combined instanceof Polygon p) visibilitySpace = p;
    else visibilitySpace = (Polygon) combined.buffer(0);
  }

  /**
   * Creates a visibility polygon from one node to another considering obstructions and a maximum
   * expansion distance.
   *
   * @param fromNode the starting node of the visibility polygon
   * @param toNode the destination node of the visibility polygon
   * @param visibilityAngle the angle of visibility in degrees
   * @param obstructions a layer containing potential obstructions to visibility
   * @param maxExpansionDistance the maximum distance the visibility polygon can expand
   * @return a Polygon representing the visibility area from the starting node to the destination
   *     node
   */
  private static Polygon createVisibilityCone(
      NodeGraph fromNode,
      NodeGraph toNode,
      Double visibilityAngle,
      VectorLayer obstructions,
      double maxExpansionDistance) {

    Coordinate from = fromNode.getCoordinate();
    Coordinate to = toNode.getCoordinate();
    Pair<Coordinate, Coordinate> pair = new Pair<>(from, to);

    if (visibilityPolygonsCache.containsKey(pair)) return visibilityPolygonsCache.get(pair);

    double edgeAngle = Math.toDegrees(Math.atan2(to.y - from.y, to.x - from.x));
    int limit = (int) (visibilityAngle / 2.0);

    List<LineString> rays =
        IntStream.iterate(-limit, i -> i <= limit, i -> i + DISTANCE_ALONG_VISIBILITY)
            .mapToObj(
                i -> {
                  double angle = Math.toRadians(edgeAngle + i);
                  Coordinate c =
                      new Coordinate(
                          to.x + maxExpansionDistance * Math.cos(angle),
                          to.y + maxExpansionDistance * Math.sin(angle));
                  return GEOMETRY_FACTORY.createLineString(new Coordinate[] {to, c});
                })
            .toList();

    Geometry toGeom = toNode.getMasonGeometry().geometry;
    Geometry buffer = toGeom.buffer(maxExpansionDistance);

    List<Geometry> obstacles =
        obstructions.containedFeatures(buffer).stream()
            .map(m -> m.geometry)
            .filter(g -> rays.stream().anyMatch(r -> g.crosses(r) && !g.touches(toGeom)))
            .toList();

    Geometry obstacleUnion = obstacles.isEmpty() ? null : UnaryUnionOp.union(obstacles);
    List<LineString> clipped =
        obstacles.isEmpty()
            ? rays
            : rays.stream()
                .map(
                    r -> {
                      Geometry inter = r.intersection(obstacleUnion);
                      return inter.isEmpty()
                          ? r
                          : GEOMETRY_FACTORY.createLineString(
                              new Coordinate[] {to, inter.getCoordinates()[0]});
                    })
                .toList();

    List<Coordinate> polyCoords = new ArrayList<>();
    polyCoords.add(to);
    clipped.forEach(r -> polyCoords.add(r.getCoordinateN(1)));
    polyCoords.add(to);

    Polygon cone = GEOMETRY_FACTORY.createPolygon(polyCoords.toArray(new Coordinate[0]));
    visibilityPolygonsCache.put(pair, cone);
    return cone;
  }

  public static RouteProperties getProperties(Route route) {
    return (RouteProperties) route.attributes.get("properties");
  }

  public Polygon getVisibilitySpace() {
    return this.visibilitySpace;
  }

  /** The share of the route's nodes from which a landmark is in view, in [0, 1]. */
  public double landmarkExposure() {
    List<NodeGraph> nodes = route.nodesSequence;
    if (nodes.isEmpty()) {
      return 0.0;
    }
    long seeing = nodes.stream().filter(node -> !node.visibleBuildings3d.isEmpty()).count();
    return seeing / (double) nodes.size();
  }
}
