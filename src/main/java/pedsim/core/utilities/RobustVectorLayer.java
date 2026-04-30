package pedsim.core.utilities;

import sim.field.geo.VectorLayer;

/**
 * A thin extension of {@link VectorLayer} that provides a more resilient
 * interface for optional data layers (e.g. census zones, POI densities).
 *
 * <p>This class exists to allow optional simulation data layers to be declared
 * with a distinct type, making it clear at the declaration site that the layer
 * may not always be populated. All core functionality is inherited from
 * {@link VectorLayer}.
 */
public class RobustVectorLayer extends VectorLayer {

  private static final long serialVersionUID = 1L;

  /**
   * Returns whether this layer has been populated with any geometries.
   *
   * @return {@code true} if the layer contains at least one geometry.
   */
  public boolean isPopulated() {
    return !getGeometries().isEmpty();
  }
}
