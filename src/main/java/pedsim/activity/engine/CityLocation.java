package pedsim.activity.engine;

import java.io.File;
import java.io.InputStream;
import java.net.URL;
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;
import java.util.List;
import java.util.logging.Logger;
import mil.nga.geopackage.GeoPackage;
import mil.nga.geopackage.GeoPackageManager;
import mil.nga.geopackage.features.user.FeatureDao;
import mil.nga.geopackage.srs.SpatialReferenceSystem;
import org.locationtech.jts.algorithm.MinimumBoundingCircle;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.proj4j.BasicCoordinateTransform;
import org.locationtech.proj4j.CRSFactory;
import org.locationtech.proj4j.CoordinateReferenceSystem;
import org.locationtech.proj4j.ProjCoordinate;
import pedsim.activity.parameters.ActivityPars;
import pedsim.core.parameters.ParameterManager;
import pedsim.core.parameters.Pars;
import pedsim.core.utilities.LoggerUtil;
import sim.graph.Graph;
import sim.graph.NodeGraph;

/**
 * Where the city is, in degrees, measured from the street network it has just loaded.
 *
 * <p>The centre is the centre of the network's <b>minimum bounding circle</b>, not the mean of its
 * nodes: a mean is pulled toward wherever the network happens to be dense, so a city with a finely
 * gridded centre and coarse suburbs reports a centre that drifts with how the graph was built. The
 * smallest circle containing every node depends only on the extent, which is what "where the city
 * is" means.
 *
 * <p>The network is projected, so the centre is transformed to WGS84 through the coordinate system
 * the layer itself declares - read from the GeoPackage that supplied the nodes, not configured
 * anywhere. A layer that declares no EPSG code leaves the position unknown rather than guessed.
 *
 * <p>The measurement is geometric, but the only question it answers - when is it dark - is this
 * tier's, so the reading happens in {@link ActivityEnvironment#prepare()}, once core has built the
 * network. A module with a clock inherits it; cityImage and empirical, which have none, never ask.
 *
 * <p>An explicit {@code --cityLatitude} / {@code --cityLongitude} is left alone. Measuring is the
 * default, not a rule.
 */
public final class CityLocation {

  private static final Logger logger = LoggerUtil.getLogger();

  private static final GeometryFactory GEOMETRY_FACTORY = new GeometryFactory();

  private CityLocation() {}

  /**
   * Measures the city's centre and stores it in {@link ActivityPars#cityLatitude} and
   * {@link ActivityPars#cityLongitude}. Both are left as {@code NaN} when the position cannot be
   * established, which is a state callers must answer for.
   *
   * @param network the primal graph the agents walk
   */
  public static void measureInto(Graph network) {
    // Cleared before measuring for the same reason the coordinates below are: a second city in one
    // JVM must not inherit the first one's centre.
    centre = null;
    if (network != null && !network.getNodes().isEmpty()) {
      centre = networkCentre(network);
    }
    boolean latitudeGiven = ParameterManager.wasGivenOnCommandLine("cityLatitude");
    boolean longitudeGiven = ParameterManager.wasGivenOnCommandLine("cityLongitude");
    if (latitudeGiven && longitudeGiven) {
      logger.info(
          String.format(
              "city position given: %.4f, %.4f degrees",
              ActivityPars.cityLatitude, ActivityPars.cityLongitude));
      return;
    }
    // Cleared before measuring, not after failing: a second city loaded into the same JVM must not
    // inherit the first one's position when its own cannot be established.
    if (!latitudeGiven) {
      ActivityPars.cityLatitude = Double.NaN;
    }
    if (!longitudeGiven) {
      ActivityPars.cityLongitude = Double.NaN;
    }
    if (centre == null) {
      return;
    }

    String epsg = declaredEpsgCode();
    if (epsg == null) {
      logger.warning(
          "the network layer declares no EPSG code, so the city's position is unknown: "
              + "darkness cannot follow the sun. Re-declare the CRS on the _nodes layer.");
      return;
    }

    try {
      CRSFactory factory = new CRSFactory();
      CoordinateReferenceSystem source = factory.createFromName(epsg);
      CoordinateReferenceSystem wgs84 = factory.createFromName("EPSG:4326");
      ProjCoordinate degrees = new ProjCoordinate();
      new BasicCoordinateTransform(source, wgs84)
          .transform(new ProjCoordinate(centre.x, centre.y), degrees);
      if (!latitudeGiven) {
        ActivityPars.cityLatitude = degrees.y;
      }
      if (!longitudeGiven) {
        ActivityPars.cityLongitude = degrees.x;
      }
      logger.info(
          String.format(
              "city position from the network (%s): %.4f, %.4f degrees",
              epsg, ActivityPars.cityLatitude, ActivityPars.cityLongitude));
    } catch (Exception e) {
      logger.warning("could not transform the network centre from " + epsg + ": " + e.getMessage());
    }
  }

  /** The projected city centre, held from the measurement so a ring can be taken off it. */
  private static Coordinate centre;

  /**
   * Distance in metres from the city centre to a node, in the network's own projected coordinates,
   * or {@code NaN} before the centre has been measured or for a null node.
   *
   * @param node the node to measure
   */
  public static double distanceFromCentre(NodeGraph node) {
    if (centre == null || node == null) {
      return Double.NaN;
    }
    return centre.distance(node.getCoordinate());
  }

  /**
   * The centre of the smallest circle containing every node of the network. A mean of the nodes
   * would drift toward wherever the graph is dense; the smallest enclosing circle depends only on
   * the extent.
   */
  private static Coordinate networkCentre(Graph network) {
    List<NodeGraph> nodes = network.getNodes();
    Coordinate[] coordinates = new Coordinate[nodes.size()];
    for (int i = 0; i < coordinates.length; i++) {
      coordinates[i] = nodes.get(i).getCoordinate();
    }
    Geometry points = GEOMETRY_FACTORY.createMultiPointFromCoords(coordinates);
    return new MinimumBoundingCircle(points).getCentre();
  }

  /**
   * The EPSG code the nodes layer declares, as {@code "EPSG:<code>"}, or null when it declares none
   * this can use. Read from the GeoPackage the graph was imported from, so it is the coordinate
   * system of the coordinates in hand rather than one stated a second time somewhere else.
   */
  private static String declaredEpsgCode() {
    String resource = Pars.cityName + "/" + Pars.cityName + "_nodes.gpkg";
    URL url = CityLocation.class.getClassLoader().getResource(resource);
    if (url == null) {
      return null;
    }
    File file = null;
    try {
      if ("file".equals(url.getProtocol())) {
        file = new File(url.toURI());
      } else {
        // Inside a jar: the GeoPackage reader needs a real file, as the importer does too.
        try (InputStream input = url.openStream()) {
          file = File.createTempFile("srs_", ".gpkg");
          file.deleteOnExit();
          Files.copy(input, file.toPath(), StandardCopyOption.REPLACE_EXISTING);
        }
      }
      try (GeoPackage geoPackage = GeoPackageManager.open(file)) {
        List<String> tables = geoPackage.getFeatureTables();
        if (tables.isEmpty()) {
          return null;
        }
        FeatureDao featureDao = geoPackage.getFeatureDao(tables.get(0));
        SpatialReferenceSystem srs = featureDao.getSrs();
        if (srs == null || !"EPSG".equalsIgnoreCase(srs.getOrganization())) {
          return null;
        }
        return "EPSG:" + srs.getOrganizationCoordsysId();
      }
    } catch (Exception e) {
      logger.warning("could not read the CRS of " + resource + ": " + e.getMessage());
      return null;
    }
  }
}
