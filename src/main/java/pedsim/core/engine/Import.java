package pedsim.core.engine;

import java.net.URL;
import java.util.logging.Logger;
import pedsim.core.parameters.Pars;
import pedsim.core.utilities.LoggerUtil;
import sim.field.geo.VectorLayer;

/**
 * This class is responsible for importing various data files required for the simulation based on
 * the selected simulation parameters. It includes methods for importing distances, barriers,
 * buildings and sight lines, road network graphs, and empirical agent groups data.
 */
public class Import {

  private static final Logger logger = LoggerUtil.getLogger();
  private final ClassLoader CLASSLOADER = getClass().getClassLoader();

  /**
   * Imports various data files required for the simulation based on the selected simulation
   * parameters.
   *
   * @throws Exception If an error occurs during the import process.
   */
  public void importFiles() throws Exception {
    readLandmarksAndSightLines();
    readBarriers();
    readGraphs();
  }

  /**
   * Reads and imports road network graphs required for the simulation.
   *
   * @throws Exception If an error occurs during the import process.
   */
  private void readGraphs() throws Exception {
    try {
      String[] layerSuffixes = {"_edges", "_nodes", "_edgesDual_graph", "_nodesDual_graph"};
      VectorLayer[] vectorLayers = {PedSimCity.roads, PedSimCity.junctions,
          PedSimCity.intersectionsDual, PedSimCity.centroids};

      for (int i = 0; i < layerSuffixes.length; i++) {
        String resourceName = Pars.cityName + "/" + Pars.cityName + layerSuffixes[i] + ".gpkg";

        URL fileUrl = CLASSLOADER.getResource(resourceName);
        if (fileUrl == null) {
          throw new IllegalStateException("Resource not found: " + resourceName);
        }

        VectorLayer.readGPKG(fileUrl, vectorLayers[i]);
      }

      PedSimCity.network.fromStreetJunctionsSegments(PedSimCity.junctions, PedSimCity.roads);
      PedSimCity.dualNetwork.fromStreetJunctionsSegments(PedSimCity.centroids,
          PedSimCity.intersectionsDual);

      logger.info("Graphs successfully imported.");
    } catch (Exception e) {
      handleImportError("Importing Graphs failed", e);
    }
  }

  /**
   * Reads and imports landmarks and sight lines data for the simulation.
   */
  private void readLandmarksAndSightLines() throws Exception {
    try {
      String[] layerSuffixes = {"_landmarks", "_sight_lines"};
      VectorLayer[] vectorLayers = {PedSimCity.buildings, PedSimCity.sightLines};

      for (int i = 0; i < layerSuffixes.length; i++) {
        String resourceName = Pars.cityName + "/" + Pars.cityName + layerSuffixes[i] + ".gpkg";

        URL fileUrl = CLASSLOADER.getResource(resourceName);
        if (fileUrl == null) {
          throw new IllegalStateException("Resource not found: " + resourceName);
        }

        VectorLayer.readGPKG(fileUrl, vectorLayers[i]);
      }

      PedSimCity.buildings.setID("buildingID");
      logger.info("Buildings successfully imported.");
    } catch (Exception e) {
      handleImportError("Importing Buildings Failed", e);
    }
  }

  /**
   * Reads and imports barriers data for the simulation.
   */
  private void readBarriers() throws Exception {
    try {
      String resourceName = Pars.cityName + "/" + Pars.cityName + "_barriers.gpkg";

      URL fileUrl = CLASSLOADER.getResource(resourceName);
      if (fileUrl == null) {
        throw new IllegalStateException("Resource not found: " + resourceName);
      }

      VectorLayer.readGPKG(fileUrl, PedSimCity.barriers);
      PedSimCity.barriers.setID("barrierID");
      logger.info("Barriers successfully imported.");
    } catch (Exception e) {
      handleImportError("Importing Barriers Failed", e);
    }
  }

  /**
   * Logs import errors.
   */
  private static void handleImportError(String layerName, Exception e) {
    logger.severe(layerName + " | " + e.getClass().getSimpleName() + ": " + e.getMessage());
  }
}
