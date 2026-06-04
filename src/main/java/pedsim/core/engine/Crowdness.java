package pedsim.core.engine;

import java.util.List;
import java.util.stream.Collectors;
import sim.graph.EdgeGraph;

public class Crowdness {

  private static long lastStep = -1;
  private static double cachedThreshold = Double.MAX_VALUE;

  /**
   * Checks whether the edge is overcrowded based on the agent count.
   *
   * @param edge The edge to check.
   * @return true if the edge is overcrowded, false otherwise.
   */
  public static boolean isEdgeCrowded(EdgeGraph edge) {
    long currentStep = 0;
    if (PedSimCity.currentInstance != null && PedSimCity.currentInstance.schedule != null) {
      currentStep = PedSimCity.currentInstance.schedule.getSteps();
    }

    if (currentStep != lastStep) {
      double percentile = 80.0; // Default sensible value: top 20% of occupied edges
      if (PedSimCity.currentInstance instanceof pedsim.night.engine.PedSimCityNight) {
        percentile = ((pedsim.night.engine.PedSimCityNight) PedSimCity.currentInstance).getCrowdednessPercentile();
      }
      cachedThreshold = calculateVolumesPercentile(percentile);
      lastStep = currentStep;
    }

    // A valid threshold must be > 0. If it's 0 (all occupied edges have 0?), clamp to 1.
    if (cachedThreshold <= 0) cachedThreshold = 1;

    return edge.getAgentCount() >= cachedThreshold;
  }

  /**
   * Calculates the volume percentile for a given percentile.
   *
   * @param percentile The percentile to calculate (e.g. 80.0 means top 20%).
   * @return The volume at the given percentile.
   */
  protected static double calculateVolumesPercentile(double percentile) {
    List<Integer> volumes = PedSimCity.edges.stream()
        .map(EdgeGraph::getAgentCount)
        .filter(count -> count > 0)
        .sorted()
        .collect(Collectors.toList());

    if (volumes.isEmpty()) return Double.MAX_VALUE;

    int index = (int) Math.ceil((percentile / 100.0) * volumes.size()) - 1;
    index = Math.max(0, Math.min(index, volumes.size() - 1));

    return volumes.get(index);
  }


}
