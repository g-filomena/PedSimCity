package pedsim.core.engine;

import java.net.URL;
import java.util.logging.Logger;
import pedsim.core.parameters.Pars;
import pedsim.core.utilities.LoggerUtil;
import sim.field.geo.VectorLayer;

/**
 * This class is responsible for importing various data files required for the
 * simulation based on
 * the selected simulation parameters. It includes methods for importing
 * distances, barriers,
 * buildings and sight lines, road network graphs, and empirical agent groups
 * data.
 */
public class Import {

  private static final Logger logger = LoggerUtil.getLogger();
  protected final ClassLoader CLASSLOADER = getClass().getClassLoader();

  /**
   * Imports various data files required for the simulation based on the selected
   * simulation
   * parameters.
   *
   * @throws Exception If an error occurs during the import process.
   */
  public void importFiles() throws Exception {
    readLandmarksAndSightLines();
    readBarriers();
    readGraphs();
    readCensusZones();
    readPoiWeights();
  }

  /**
   * Reads and imports road network graphs required for the simulation.
   *
   * @throws Exception If an error occurs during the import process.
   */
  protected void readGraphs() throws Exception {
    try {
      String[] layerSuffixes = {"_edges", "_nodes", "_edgesDual", "_nodesDual"};
      VectorLayer[] vectorLayers = {
        PedSimCity.roads, PedSimCity.junctions, PedSimCity.intersectionsDual, PedSimCity.centroids
      };

      for (int i = 0; i < layerSuffixes.length; i++) {
        String resourceName = Pars.cityName + "/" + Pars.cityName + layerSuffixes[i] + ".gpkg";

        URL fileUrl = CLASSLOADER.getResource(resourceName);
        if (fileUrl == null) {
          throw new IllegalStateException("Resource not found: " + resourceName);
        }

        VectorLayer.readGPKG(fileUrl, vectorLayers[i]);
      }

      PedSimCity.network.fromStreetJunctionsSegments(PedSimCity.junctions, PedSimCity.roads);
      PedSimCity.dualNetwork.fromStreetJunctionsSegments(
          PedSimCity.centroids, PedSimCity.intersectionsDual);

      logger.info("Graphs successfully imported.");
    } catch (Exception e) {
      handleImportError("Importing Graphs failed", e);
    }
  }

  /**
   * Reads and imports landmarks and sight lines data for the simulation.
   */
  protected void readLandmarksAndSightLines() throws Exception {
    try {
      String[] layerSuffixes = {"_landmarks", "_sight_lines2D"};
      VectorLayer[] vectorLayers = {PedSimCity.buildings, PedSimCity.sightLines};

      for (int i = 0; i < layerSuffixes.length; i++) {
        String resourceName = Pars.cityName + "/" + Pars.cityName + layerSuffixes[i] + ".gpkg";

        URL fileUrl = CLASSLOADER.getResource(resourceName);
        if (fileUrl == null) {
          System.out.println("Optional resource not found: " + resourceName);
          continue;
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
  protected void readBarriers() throws Exception {
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

  protected void readCensusZones() throws Exception {
    readOptionalLayer("censusData", PedSimCity.censusZones);
    if (Pars.isNight) {
      readOptionalLayer("censusData_vulnerability", PedSimCity.censusZonesVulnerability);
    }
  }

  protected void readPoiWeights() throws Exception {
    if (Pars.isNight) {
      readOptionalLayer("Night_POI_densities", PedSimCity.nightPoiDensities);
      readOptionalLayer("Workplace_POI_densities", PedSimCity.workplacePoiDensities);
      readIlluminatedEdges();
    }
  }

  /**
   * Loads the illuminated edges dataset (edges_illuminated_continuous.gpkg) which provides
   * mean_lux values per street edge for the night simulation.
   */
  protected void readIlluminatedEdges() {
    try {
      String resourceName =
          Pars.cityName + "/" + Pars.cityName + "_edges_illuminated_continuous.gpkg";
      URL fileUrl = CLASSLOADER.getResource(resourceName);
      if (fileUrl == null) {
        logger.warning(
            "Illuminated edges dataset not found at: "
                + resourceName
                + " — mean_lux will default to 0.");
        return;
      }
      PedSimCity.illuminatedEdges.getGeometries().clear();
      VectorLayer.readGPKG(fileUrl, PedSimCity.illuminatedEdges);
      logger.info(
          "Illuminated edges loaded: "
              + PedSimCity.illuminatedEdges.getGeometries().size()
              + " features.");
    } catch (Exception e) {
      logger.warning("Failed to load illuminated edges: " + e.getMessage());
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

      targetLayer.getGeometries().clear();

      // Use the standard reader but catch specific data-integrity crashes
      try {
        VectorLayer.readGPKG(fileUrl, targetLayer);
      } catch (Exception e) {
        if (e.getMessage() != null && e.getMessage().contains("getEnvelopeInternal")) {
          logger.warning(
              "Census dataset contains records with null geometries. Skipping corrupt records and"
                  + " continuing...");
        } else {
          throw e;
        }
      }

      if (targetLayer.getGeometries().isEmpty()) {
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
