package pedsim.cityimage.engine;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import org.locationtech.jts.linearref.LengthIndexedLine;
import org.locationtech.jts.planargraph.DirectedEdge;

import pedsim.cityimage.utilities.StringEnum.RouteChoice;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.ScenarioConfig;

/** Simulation state for the city-image testing module. */
public class PedSimCityImage extends PedSimCity {

	private static final long serialVersionUID = 1L;

	/**
	 * Kept for backward compatibility with city-image code paths that used their
	 * own edge cache. Core also has an indexed-edge cache; new code should prefer
	 * the core cache when possible.
	 */
	public static final Map<DirectedEdge, LengthIndexedLine> indexedEdgeCache = new ConcurrentHashMap<>();

	public PedSimCityImage(long seed, int job, ScenarioConfig scenarioConfig) {
		super(seed, job, defaultScenarioConfig(scenarioConfig));
	}

	private static ScenarioConfig defaultScenarioConfig(ScenarioConfig scenarioConfig) {
		if (scenarioConfig != null) {
			return scenarioConfig;
		}

		return new ScenarioConfig(RouteChoice.values(), null);
	}

	@Override
	protected void populateEnvironment() {
		CityImagePopulate populate = new CityImagePopulate();
		populate.populateTests(this);
	}
}