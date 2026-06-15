package pedsim.core.engine;

import java.util.List;
import java.util.stream.Collectors;

import sim.graph.EdgeGraph;

public class Crowdness {

	private static final double DEFAULT_CROWDEDNESS_PERCENTILE = 80.0;

	private static final ThreadLocal<CrowdnessCache> CACHE = ThreadLocal.withInitial(CrowdnessCache::new);

	private static final class CrowdnessCache {
		private PedSimCity state = null;
		private long step = -1;
		private double threshold = Double.MAX_VALUE;
	}

	/**
	 * Checks whether the edge is overcrowded based on the agent count.
	 *
	 * @param edge the edge to check.
	 * @return true if the edge is overcrowded, false otherwise.
	 */
	public static boolean isEdgeCrowded(EdgeGraph edge) {
		if (edge == null) {
			return false;
		}

		PedSimCity state = PedSimCity.currentInstance;

		long currentStep = 0;
		if (state != null && state.schedule != null) {
			currentStep = state.schedule.getSteps();
		}

		CrowdnessCache cache = CACHE.get();

		if (cache.state != state || cache.step != currentStep) {
			double percentile = DEFAULT_CROWDEDNESS_PERCENTILE;

			if (state instanceof pedsim.night.engine.PedSimCityNight) {
				percentile = ((pedsim.night.engine.PedSimCityNight) state).getCrowdednessPercentile();
			}

			cache.threshold = validThreshold(calculateVolumesPercentile(percentile));
			cache.step = currentStep;
			cache.state = state;
		}

		return edge.getAgentCount() >= cache.threshold;
	}

	/**
	 * Ensures the threshold remains usable as a minimum positive crowdness value.
	 *
	 * @param threshold the calculated threshold.
	 * @return a valid threshold.
	 */
	private static double validThreshold(double threshold) {
		if (threshold <= 0) {
			return 1;
		}

		return threshold;
	}

	/**
	 * Calculates the volume percentile for a given percentile.
	 *
	 * @param percentile the percentile to calculate, e.g. 80.0 means top 20%.
	 * @return the volume at the given percentile.
	 */
	protected static double calculateVolumesPercentile(double percentile) {
		if (PedSimCity.edges == null || PedSimCity.edges.isEmpty()) {
			return Double.MAX_VALUE;
		}

		List<Integer> volumes = PedSimCity.edges.stream().map(EdgeGraph::getAgentCount).filter(count -> count > 0)
				.sorted().collect(Collectors.toList());

		if (volumes.isEmpty()) {
			return Double.MAX_VALUE;
		}

		int index = (int) Math.ceil((percentile / 100.0) * volumes.size()) - 1;
		index = Math.max(0, Math.min(index, volumes.size() - 1));

		return volumes.get(index);
	}
}