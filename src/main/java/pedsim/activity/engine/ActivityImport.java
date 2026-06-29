package pedsim.activity.engine;

import pedsim.core.engine.Import;

/**
 * Import strategy for activity-based modules. Adds the unified census layer on top of the core
 * {@link Import} (graphs, landmarks, barriers).
 *
 * <p>The census layer is a single GeoPackage (&lt;City&gt;_censusData.gpkg) whose polygons carry the full
 * 24h activity pattern as columns: {@code residence_pct} (home spawning), {@code workplace_poi}
 * (daytime destinations) and {@code night_poi} (evening destinations).
 *
 * <p>Night-specific datasets (vulnerability, illuminated edges) are added by the night module's
 * {@code NightImport}, which extends this class.
 */
public class ActivityImport extends Import {

  /**
   * Imports the core data plus the unified census layer.
   *
   * @throws Exception If an error occurs during the import process.
   */
  @Override
  public void importFiles() throws Exception {
    super.importFiles();
    readCensusZones();
  }

  /** Reads the unified census layer (residence_pct, workplace_poi, night_poi). */
  protected void readCensusZones() throws Exception {
    readOptionalLayer("censusData", PedSimCityActivity.censusLayer);
  }
}
