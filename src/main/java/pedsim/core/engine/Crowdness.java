package pedsim.core.engine;

import java.util.List;
import java.util.stream.Collectors;
import sim.graph.EdgeGraph;

public class Crowdness {

  /**
   * Checks whether the edge is overcrowded based on the agent count.
   *
   * @param edge The edge to check.
   * @return true if the edge is overcrowded, false otherwise.
   */
  public static boolean isEdgeCrowded(EdgeGraph edge) {
    double volumePercentile = calculateVolumesPercentile(20);
    return edge.getAgentCount() >= volumePercentile;
  }

  /**
   * Calculates the volume percentile for a given percentile.
   *
   * @param percentile The percentile to calculate.
   * @return The volume at the given percentile.
   */
  protected static double calculateVolumesPercentile(int percentile) {
    // Collect volumes from edges (Set to List)
    List<Integer> volumes = PedSimCity.edges.stream().map(EdgeGraph::getAgentCount) // Map each edge
                                                                                    // to its
                                                                                    // agentCount
        .filter(agentCount -> agentCount > 0) // Only keep agent counts greater than 0
        .sorted() // Sort the agent counts
        .collect(Collectors.toList()); // Collect to a List

    // Calculate the index for the percentile
    int index = (int) Math.ceil(percentile / 100.0 * volumes.size()) - 1;
    index = Math.max(0, index); // Ensure the index is within bounds

    // Return the value at the calculated index

    if (!volumes.isEmpty())
      return volumes.get(index);
    return Double.MAX_VALUE;
  }


}
