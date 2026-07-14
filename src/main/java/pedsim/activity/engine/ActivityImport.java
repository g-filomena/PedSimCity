package pedsim.activity.engine;

import pedsim.core.engine.Import;

/**
 * Import strategy for activity-based modules. Adds the unified census layer on top of the core
 * {@link Import} (graphs, landmarks, barriers).
 *
 * <p>The census layer is a single GeoPackage (&lt;City&gt;_censusData.gpkg) whose polygons carry
 * population structure only: {@code residence_pct} (home spawning) and {@code residents} (absolute
 * headcount driving the population size). Destination attraction comes from OSM-like use tags on
 * the buildings/POI layers, not from the census.
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
    readPois();
  }

  /** Reads the census layer (residence_pct, residents, plus module columns like vulnerability_pct). */
  protected void readCensusZones() throws Exception {
    readOptionalLayer("censusData", PedSimCityActivity.censusLayer);
  }

  /**
   * Reads the optional POI layer (&lt;City&gt;_POIs.gpkg) whose features carry OSM-like use tags
   * ({@code amenity}, {@code shop}, {@code leisure}, …). Buildings may carry the same tags; both
   * are classified into activity purposes by {@code PoiClassifier}.
   */
  protected void readPois() throws Exception {
    readOptionalLayer("POIs", PedSimCityActivity.poisLayer);
  }
}
