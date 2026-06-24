package pedsim.activity.engine;

import java.util.ArrayList;
import java.util.List;
import sim.graph.NodeGraph;
import sim.util.geo.MasonGeometry;

/**
 * A census zone: one polygon carrying the per-zone activity attributes and the street-network nodes
 * associated with it by proximity.
 *
 * <p>Weights are zone-level totals. {@code residence} is a share of the city population (used to
 * sample home zones); {@code workplace} and {@code night} are raw POI counts — numbers of
 * opportunities — used to sample daytime / evening destinations.
 *
 * <p>Nodes are assigned by proximity (see {@link ActivityEnvironment}), so a junction may belong to
 * several zones (e.g. one bordering several residential parcels). Non-residential zones (streets,
 * commercial) simply have {@code residence == 0} and are never used for home spawning, but remain
 * valid work/night destinations.
 */
public class CensusZone {

  public final MasonGeometry geometry;
  public final List<NodeGraph> nodes = new ArrayList<>();

  public double residence; // share of total city residents (0 for non-residential zones)
  public double workplace; // daytime workplace POI count
  public double night; // evening / leisure POI count

  public CensusZone(MasonGeometry geometry) {
    this.geometry = geometry;
  }
}
