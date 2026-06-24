package pedsim.night.engine;

import java.net.URL;

import pedsim.activity.engine.ActivityImport;
import sim.field.geo.VectorLayer;

/**
 * Import strategy for the night module. Adds the night-specific perception/safety datasets
 * (vulnerability census, illuminated edges) on top of the activity census + activity-POI imports.
 */
public class NightImport extends ActivityImport {

  /**
   * Imports the activity data (core + census + workplace + night POI) plus the night-specific
   * vulnerability and lighting datasets.
   *
   * @throws Exception If an error occurs during the import process.
   */
  @Override
  public void importFiles() throws Exception {
    super.importFiles();
    readIlluminatedEdges();
  }

  /**
   * Loads the illuminated edges dataset (edges_illuminated_continuous.gpkg) which provides
   * mean_lux values per street edge for the night simulation.
   */
  protected void readIlluminatedEdges() {
    try {
      String resourceName =
          pedsim.core.parameters.Pars.cityName
              + "/"
              + pedsim.core.parameters.Pars.cityName
              + "_edges_illuminated_continuous.gpkg";
      URL fileUrl = CLASSLOADER.getResource(resourceName);
      if (fileUrl == null) {
        logger.warning(
            "Illuminated edges dataset not found at: "
                + resourceName
                + " — mean_lux will default to 0.");
        return;
      }
      PedSimCityNight.illuminatedEdges.getGeometries().clear();
      VectorLayer.readGPKG(fileUrl, PedSimCityNight.illuminatedEdges);
      logger.info(
          "Illuminated edges loaded: "
              + PedSimCityNight.illuminatedEdges.getGeometries().size()
              + " features.");
    } catch (Exception e) {
      logger.warning("Failed to load illuminated edges: " + e.getMessage());
    }
  }
}
