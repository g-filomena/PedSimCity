package pedsim.activity.engine;

import java.util.ArrayList;
import java.util.List;
import sim.graph.NodeGraph;
import sim.util.geo.MasonGeometry;

/**
 * A census zone: one polygon carrying the per-zone population attributes and the street-network
 * nodes associated with it by proximity.
 *
 * <p>The census is population structure only: {@code residence} is a share of the city population
 * (used to sample home zones); module-specific population columns (e.g. the night module's
 * {@code vulnerability_pct}) are read from {@code geometry} via
 * {@link ActivityEnvironment#zoneValue}. Destination attraction is not a census matter — it comes
 * from the OSM-tag purpose weights built by {@link PoiClassifier}.
 *
 * <p>Nodes are assigned by proximity (see {@link ActivityEnvironment}), so a junction may belong to
 * several zones (e.g. one bordering several residential parcels). Non-residential zones (streets,
 * commercial) simply have {@code residence == 0} and are never used for home spawning.
 */
public class CensusZone {

  public final MasonGeometry geometry;
  public final List<NodeGraph> nodes = new ArrayList<>();

  public double residence; // share of total city residents (0 for non-residential zones)

  // Age-structure shares of the zone's adult residents (NaN when the census lacks them);
  // used to condition persona sampling on the home zone.
  public double retireeShare = Double.NaN; // residents aged 65+
  public double studentShare = Double.NaN; // residents aged 15-24

  public CensusZone(MasonGeometry geometry) {
    this.geometry = geometry;
  }
}
