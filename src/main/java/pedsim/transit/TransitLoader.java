package pedsim.transit;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.logging.Logger;
import pedsim.activity.engine.PedSimCityActivity;
import pedsim.core.engine.PedSimCity;
import pedsim.core.parameters.Pars;
import sim.graph.NodeGraph;

/**
 * High-performance runtime loader for PedSimCity multi-modal transit stations.
 * Reads {@code transit_stops.csv} during environment preparation and links each station to its
 * corresponding MASON street network {@link NodeGraph} via {@code snappedNodeId}.
 */
public class TransitLoader {

  private static final Logger logger = Logger.getLogger(TransitLoader.class.getName());

  /**
   * Loads transit stops from transit_stops.csv into static data structures inside
   * {@link PedSimCityActivity}.
   */
  public static void loadStops(PedSimCity state) {
    long startTime = System.currentTimeMillis();
    int loadedCount = 0;
    int linkedCount = 0;
    int skippedCount = 0;

    String resourcePath = Pars.cityName + "/transit_stops.csv";
    InputStream is = TransitLoader.class.getClassLoader().getResourceAsStream(resourcePath);
    BufferedReader reader = null;

    try {
      if (is != null) {
        reader = new BufferedReader(new InputStreamReader(is, StandardCharsets.UTF_8));
      } else {
        File diskFile = new File("src/main/resources/" + resourcePath);
        if (diskFile.exists()) {
          reader = new BufferedReader(new FileReader(diskFile, StandardCharsets.UTF_8));
        } else {
          // Also try checking directly under Torino if using Torino_simplified
          diskFile = new File("src/main/resources/Torino_simplified/transit_stops.csv");
          if (diskFile.exists()) {
            reader = new BufferedReader(new FileReader(diskFile, StandardCharsets.UTF_8));
          } else {
            diskFile = new File("src/main/resources/Torino/transit_stops.csv");
            if (diskFile.exists()) {
              reader = new BufferedReader(new FileReader(diskFile, StandardCharsets.UTF_8));
            }
          }
        }
      }

      if (reader == null) {
        logger.warning("transit_stops.csv not found for city " + Pars.cityName + ". Transit disabled.");
        return;
      }

      String headerLine = reader.readLine();
      if (headerLine == null) {
        reader.close();
        return;
      }

      String[] headers = headerLine.split(",");
      int idxStopId = -1, idxStopName = -1, idxX = -1, idxY = -1, idxNodeId = -1, idxModes = -1, idxRoutes = -1;
      for (int i = 0; i < headers.length; i++) {
        String h = headers[i].trim().toLowerCase();
        if (h.equals("stop_id")) idxStopId = i;
        else if (h.equals("stop_name")) idxStopName = i;
        else if (h.equals("x")) idxX = i;
        else if (h.equals("y")) idxY = i;
        else if (h.equals("snapped_node_id")) idxNodeId = i;
        else if (h.equals("modes_served")) idxModes = i;
        else if (h.equals("routes_served")) idxRoutes = i;
      }

      String line;
      while ((line = reader.readLine()) != null) {
        if (line.trim().isEmpty()) continue;
        // Parse CSV respecting potential quotes
        String[] parts = line.split(",(?=(?:[^\"]*\"[^\"]*\")*[^\"]*$)");
        if (parts.length <= Math.max(idxStopId, idxNodeId)) continue;

        try {
          String stopId = parts[idxStopId].replaceAll("\"", "").trim();
          String stopName = idxStopName >= 0 && idxStopName < parts.length ? parts[idxStopName].replaceAll("\"", "").trim() : stopId;
          double x = idxX >= 0 && idxX < parts.length ? Double.parseDouble(parts[idxX].trim()) : 0.0;
          double y = idxY >= 0 && idxY < parts.length ? Double.parseDouble(parts[idxY].trim()) : 0.0;
          int snappedNodeId = Integer.parseInt(parts[idxNodeId].trim());
          String modes = idxModes >= 0 && idxModes < parts.length ? parts[idxModes].replaceAll("\"", "").trim() : "BUS";
          String routes = idxRoutes >= 0 && idxRoutes < parts.length ? parts[idxRoutes].replaceAll("\"", "").trim() : "N/A";

          TransitStop stop = new TransitStop(stopId, stopName, snappedNodeId, x, y, modes, routes);
          
          // Link to physical street graph node
          NodeGraph nodeGraph = PedSimCity.nodesMap.get(snappedNodeId);
          if (nodeGraph != null) {
            stop.snappedNodeGraph = nodeGraph;
            linkedCount++;
          } else {
            skippedCount++;
          }

          PedSimCityActivity.allTransitStops.add(stop);
          PedSimCityActivity.transitStopsByNodeId.put(snappedNodeId, stop);

          if (stop.servesMode("METRO")) {
            PedSimCityActivity.metroStops.add(stop);
          } else if (stop.servesMode("TRAM")) {
            PedSimCityActivity.tramStops.add(stop);
          } else {
            PedSimCityActivity.busStops.add(stop);
          }
          loadedCount++;

        } catch (Exception ex) {
          skippedCount++;
        }
      }
      reader.close();

      long elapsed = System.currentTimeMillis() - startTime;
      logger.info(
          String.format(
              "Multi-Modal Transit: Loaded %d stations (%d linked to graph nodes, %d skipped) in %d ms. -> Metro: %d, Tram: %d, Bus: %d",
              loadedCount,
              linkedCount,
              skippedCount,
              elapsed,
              PedSimCityActivity.metroStops.size(),
              PedSimCityActivity.tramStops.size(),
              PedSimCityActivity.busStops.size()));

    } catch (Exception e) {
      logger.severe("Error reading transit_stops.csv: " + e.getMessage());
    }
  }
}
