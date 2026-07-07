package pedsim.activity.engine;

import java.net.URL;
import pedsim.core.utilities.LoggerUtil;
import sim.field.geo.VectorLayer;
import sim.util.geo.AttributeValue;
import sim.util.geo.MasonGeometry;

/**
 * Reads the absolute resident total (sum of the {@code residents} / census P1 column) from a city's
 * enriched census layer {@code <City>_censusData.gpkg} on the classpath.
 *
 * <p>Exposed to core UIs via {@link pedsim.core.engine.SimulationModule#populationForCity(String)} so
 * the dashboard and Swing applets can decide, before a run starts, whether the population is
 * census-driven (and lock the population input). Returns 0 when the city, file, or column is absent —
 * callers then fall back to a user-set population. This is a lightweight data probe; the actual
 * population override lives in {@link ActivityEnvironment}.
 */
public final class CensusPopulation {

  private CensusPopulation() {}

  /** Sum of the {@code residents} column for {@code city}, or 0 if unavailable. */
  public static long residentTotal(String city) {
    if (city == null || city.isBlank()) {
      return 0;
    }
    URL url =
        CensusPopulation.class.getClassLoader().getResource(city + "/" + city + "_censusData.gpkg");
    if (url == null) {
      return 0; // no census for this city (normal, e.g. cities without a census layer)
    }

    long total = 0;
    try {
      VectorLayer layer = new VectorLayer();
      VectorLayer.readGPKG(url, layer);
      for (MasonGeometry geom : layer.getGeometries()) {
        AttributeValue value = geom.getAttributes().get("residents");
        if (value != null) {
          try {
            total += Math.round(value.getDouble());
          } catch (Exception ignore) {
            // non-numeric residents value; skip
          }
        }
      }
    } catch (Exception e) {
      LoggerUtil.getLogger()
          .warning("Census population probe failed for " + city + ": " + e.getMessage());
      return 0;
    }

    LoggerUtil.getLogger().fine("Census population for " + city + ": " + total);
    return total;
  }
}
