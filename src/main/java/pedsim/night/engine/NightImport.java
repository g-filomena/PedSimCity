package pedsim.night.engine;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.List;
import pedsim.activity.engine.ActivityImport;
import pedsim.night.parameters.NightPars;
import sim.field.geo.VectorLayer;

/**
 * Import strategy for the night module. Adds the night-specific perception/safety datasets
 * (illuminated edges and directional lighting) on top of the activity census + activity-POI imports.
 */
public class NightImport extends ActivityImport {

  @Override
  public void importFiles() throws Exception {
    super.importFiles();
    readIlluminatedEdges();
    readDirectionalLighting();
  }

  /**
   * Loads the illuminated edges dataset ({@code <City>_edges_illuminated_continuous.gpkg}) which
   * provides {@code mean_lux} per street edge for the night simulation.
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

  /**
   * Loads the directional entrance-lighting lookup ({@code <City>_directional_lighting_lookup.csv})
   * into {@link PedSimCityNight#directionalLuxMap}, keyed by (current_node_id, target_node_id) with
   * the min or mean lux per {@link NightPars#directionalLuxStatistic}. Reads by header name; on a
   * missing/empty/malformed file it logs a warning and leaves the map empty (the model then falls
   * back to the binary lit flag), consistent with {@link #readIlluminatedEdges()}.
   */
  protected void readDirectionalLighting() {
    PedSimCityNight.directionalLuxMap.clear();
    String resourceName =
        pedsim.core.parameters.Pars.cityName
            + "/"
            + pedsim.core.parameters.Pars.cityName
            + "_directional_lighting_lookup.csv";
    URL fileUrl = CLASSLOADER.getResource(resourceName);
    if (fileUrl == null) {
      logger.info("Directional lighting lookup not found at: " + resourceName);
      return;
    }

    String luxColumn =
        NightPars.directionalLuxStatistic == NightPars.DirectionalLuxStatistic.MEAN
            ? "visibility_mean_lux"
            : "visibility_min_lux";
    int loaded = 0;
    int skipped = 0;

    try (BufferedReader br =
        new BufferedReader(new InputStreamReader(fileUrl.openStream(), StandardCharsets.UTF_8))) {
      String headerLine = br.readLine();
      if (headerLine == null) {
        logger.warning("Directional lighting CSV is empty: " + resourceName);
        return;
      }
      List<String> header = Arrays.asList(headerLine.split(",", -1));
      int fromIdx = header.indexOf("current_node_id");
      int toIdx = header.indexOf("target_node_id");
      int luxIdx = header.indexOf(luxColumn);
      if (fromIdx < 0 || toIdx < 0 || luxIdx < 0) {
        logger.warning(
            "Directional lighting CSV missing a required column (current_node_id, target_node_id, "
                + luxColumn
                + "); header="
                + header
                + " — skipping directional lighting.");
        return;
      }
      int maxIdx = Math.max(fromIdx, Math.max(toIdx, luxIdx));

      String line;
      while ((line = br.readLine()) != null) {
        String[] parts = line.split(",", -1);
        if (parts.length <= maxIdx) {
          skipped++;
          continue;
        }
        try {
          long key =
              PedSimCityNight.luxKey(
                  Integer.parseInt(parts[fromIdx].trim()), Integer.parseInt(parts[toIdx].trim()));
          PedSimCityNight.directionalLuxMap.put(key, Double.parseDouble(parts[luxIdx].trim()));
          loaded++;
        } catch (NumberFormatException e) {
          skipped++;
        }
      }
    } catch (Exception e) {
      logger.warning("Failed to load directional lighting: " + e.getMessage());
      return;
    }

    logger.info(
        "Directional lighting: loaded "
            + loaded
            + " rows ("
            + skipped
            + " skipped) using "
            + NightPars.directionalLuxStatistic
            + ".");
    if (loaded == 0) {
      logger.warning("Directional lighting loaded 0 rows; check node IDs vs NodeGraph.getID().");
    }
  }
}
