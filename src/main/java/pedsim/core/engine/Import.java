package pedsim.core.engine;

import java.net.URL;
import java.util.List;
import java.util.logging.Logger;
import pedsim.core.parameters.Pars;
import pedsim.core.utilities.LoggerUtil;
import sim.field.geo.VectorLayer;
import sim.util.geo.MasonGeometry;

/**
 * This class is responsible for importing various data files required for the
 * simulation based on
 * the selected simulation parameters. It includes methods for importing
 * distances, barriers,
 * buildings and sight lines, road network graphs, and empirical agent groups
 * data.
 */
public class Import {

  protected static final Logger logger = LoggerUtil.getLogger();
  protected final ClassLoader CLASSLOADER = getClass().getClassLoader();

  /**
   * Imports various data files required for the simulation based on the selected
   * simulation
   * parameters.
   *
   * @throws Exception If an error occurs during the import process.
   */
  public void importFiles() throws Exception {
    readBuildings();
    readSightLines();
    readBarriers();
    readGraphs();
  }

  /**
   * Reads and imports road network graphs required for the simulation.
   *
   * @throws Exception If an error occurs during the import process.
   */
  protected void readGraphs() throws Exception {
    try {
      // Primal graph is required.
      readRequiredGraphLayer("_edges", PedSimCity.roads);
      readRequiredGraphLayer("_nodes", PedSimCity.junctions);
      PedSimCity.network.fromStreetJunctionsSegments(PedSimCity.junctions, PedSimCity.roads);

      // Dual graph is optional: angular-change (simplest-path) routing needs it, but primal-only
      // cities run with shortest-path routing only.
      boolean edgesDual = readOptionalGraphLayer("_edgesDual", PedSimCity.intersectionsDual);
      boolean nodesDual = readOptionalGraphLayer("_nodesDual", PedSimCity.centroids);
      PedSimCity.dualGraphLoaded = edgesDual && nodesDual;

      if (PedSimCity.dualGraphLoaded) {
        PedSimCity.dualNetwork.fromStreetJunctionsSegments(
            PedSimCity.centroids, PedSimCity.intersectionsDual);
        logger.info("Graphs successfully imported (primal + dual).");
      } else {
        logger.info("Graphs imported (primal only); angular-change routing disabled.");
      }
    } catch (Exception e) {
      handleImportError("Importing Graphs failed", e);
    }
  }

  /** Reads a required graph layer; throws if the resource is missing. */
  private void readRequiredGraphLayer(String suffix, VectorLayer target) throws Exception {
    String resourceName = Pars.cityName + "/" + Pars.cityName + suffix + ".gpkg";
    URL fileUrl = CLASSLOADER.getResource(resourceName);
    if (fileUrl == null) {
      throw new IllegalStateException("Required graph layer not found: " + resourceName);
    }
    VectorLayer.readGPKG(fileUrl, target);
  }

  /** Reads an optional graph layer; returns false (leaving the layer empty) when missing. */
  private boolean readOptionalGraphLayer(String suffix, VectorLayer target) throws Exception {
    String resourceName = Pars.cityName + "/" + Pars.cityName + suffix + ".gpkg";
    URL fileUrl = CLASSLOADER.getResource(resourceName);
    if (fileUrl == null) {
      logger.info("Optional graph layer not found: " + resourceName);
      return false;
    }
    VectorLayer.readGPKG(fileUrl, target);
    return true;
  }

  /**
   * Reads and imports the buildings layer for the simulation.
   *
   * <p>The buildings layer (&lt;City&gt;_buildings.gpkg) holds every footprint; landmark scores
   * ({@code gScore_sc}, {@code lScore_sc}), {@code land_use}/{@code DMA} and OSM-like use tags are
   * optional per-building attributes on it. Landmarks are not a separate layer: they are the subset
   * of buildings whose scores pass the runtime thresholds (see {@code SharedCognitiveMap}). Some
   * city folders ship the same layer as &lt;City&gt;_landmarks.gpkg; both names are read.
   */
  protected void readBuildings() throws Exception {
    try {
      boolean loaded = readBuildingsLayer("_buildings");
      if (!loaded) {
        loaded = readBuildingsLayer("_landmarks");
        if (loaded) {
          logger.info(
              "Buildings layer loaded from " + Pars.cityName + "_landmarks.gpkg.");
        }
      }
      if (loaded) {
        PedSimCity.buildings.setID("buildingID");
        logger.info("Buildings successfully imported.");
      } else {
        logger.info("No usable buildings layer (" + Pars.cityName + "_buildings.gpkg); "
            + "landmark navigation and building-based destination choice disabled.");
      }
    } catch (Exception e) {
      handleImportError("Importing Buildings Failed", e);
    }
  }

  /**
   * Reads and imports the sight-lines layer, which only serves landmark navigation
   * ({@code SharedCognitiveMap.integrateLandmarks}): it is skipped — not just left unused — when
   * the buildings carry no landmark scores, since the file can be large.
   */
  protected void readSightLines() throws Exception {
    try {
      if (!buildingsCarryLandmarkScores()) {
        logger.info("Sight lines not loaded: buildings carry no landmark scores"
            + " (landmark navigation disabled).");
        return;
      }
      URL sightLinesUrl = findResource("_sight_lines2D");
      if (sightLinesUrl == null) {
        logger.info("Optional resource not found: " + Pars.cityName + "_sight_lines2D.gpkg");
      } else {
        VectorLayer.readGPKG(sightLinesUrl, PedSimCity.sightLines);
        logger.info("Sight lines successfully imported.");
      }
    } catch (Exception e) {
      handleImportError("Importing Sight Lines Failed", e);
    }
  }

  /** Whether the loaded buildings layer carries landmark scores (checked on the first feature). */
  private static boolean buildingsCarryLandmarkScores() {
    List<MasonGeometry> geometries = PedSimCity.buildings.getGeometries();
    if (geometries.isEmpty()) {
      return false;
    }
    var attributes = geometries.get(0).getAttributes();
    return attributes.containsKey("lScore_sc") || attributes.containsKey("gScore_sc");
  }

  /**
   * Reads a candidate buildings layer; rejects (and clears) it when it is absent or carries no
   * {@code buildingID}, so an unusable file cannot crash the environment preparation.
   */
  private boolean readBuildingsLayer(String suffix) throws Exception {
    URL fileUrl = findResource(suffix);
    if (fileUrl == null) {
      return false;
    }
    // clear() the layer itself; getGeometries() returns a defensive copy, so the geometries
    // must be read back *after* the load to reflect what was actually imported.
    PedSimCity.buildings.clear();
    VectorLayer.readGPKG(fileUrl, PedSimCity.buildings);
    List<MasonGeometry> geometries = PedSimCity.buildings.getGeometries();
    if (geometries.isEmpty()) {
      return false;
    }
    if (!geometries.get(0).getAttributes().containsKey("buildingID")) {
      logger.warning(Pars.cityName + suffix + ".gpkg has no buildingID column; ignoring it.");
      PedSimCity.buildings.clear();
      return false;
    }
    return true;
  }

  /** Resolves an optional city resource by suffix, or null when absent. */
  private URL findResource(String suffix) {
    return CLASSLOADER.getResource(Pars.cityName + "/" + Pars.cityName + suffix + ".gpkg");
  }

  /**
   * Reads and imports barriers data for the simulation.
   */
  protected void readBarriers() throws Exception {
    try {
      String resourceName = Pars.cityName + "/" + Pars.cityName + "_barriers.gpkg";

      URL fileUrl = CLASSLOADER.getResource(resourceName);
      if (fileUrl == null) {
        logger.info(
            "Optional layer not found: " + resourceName + "; barrier-based navigation disabled.");
        return;
      }

      VectorLayer.readGPKG(fileUrl, PedSimCity.barriers);
      PedSimCity.barriers.setID("barrierID");
      logger.info("Barriers successfully imported.");
    } catch (Exception e) {
      handleImportError("Importing Barriers Failed", e);
    }
  }

  /**
   * Reads an optional GeoPackage layer into the given target layer using a robust manual reader.
   */
  protected void readOptionalLayer(String layerName, VectorLayer targetLayer) throws Exception {
    try {
      String resourceName = Pars.cityName + "/" + Pars.cityName + "_" + layerName + ".gpkg";
      URL fileUrl = CLASSLOADER.getResource(resourceName);

      if (fileUrl == null) {
        throw new IllegalStateException("Resource not found: " + resourceName);
      }

      // clear() the layer itself: getGeometries() returns a defensive copy, so clearing that
      // copy would leave the layer untouched and re-reads would accumulate duplicate features.
      targetLayer.clear();

      // Use the standard reader but catch specific data-integrity crashes
      try {
        VectorLayer.readGPKG(fileUrl, targetLayer);
      } catch (Exception e) {
        if (e.getMessage() != null && e.getMessage().contains("getEnvelopeInternal")) {
          logger.warning(
              "Layer " + layerName + " contains records with null geometries. Skipping corrupt"
                  + " records and continuing...");
        } else {
          throw e;
        }
      }

      if (targetLayer.isEmpty()) {
        logger.warning("Layer " + layerName + " was loaded but is empty.");
      } else {
        targetLayer
            .getGeometries()
            .removeIf(
                obj -> {
                  boolean isNull = obj.getGeometry() == null;
                  if (isNull) {
                    logger.warning("Skipped null geometry in " + layerName);
                  }
                  return isNull;
                });
      }
      logger.info(layerName + " successfully imported.");
    } catch (Exception e) {
      logger.warning("Optional layer " + layerName + " failed to load: " + e.getMessage());
      e.printStackTrace();
    }
  }

  /**
   * Logs import errors.
   */
  protected static void handleImportError(String layerName, Exception e) {
    logger.severe(layerName + " | " + e.getClass().getSimpleName() + ": " + e.getMessage());
    e.printStackTrace();
  }
}
