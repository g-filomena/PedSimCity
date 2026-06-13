package pedsim.cityimage.engine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import org.locationtech.jts.linearref.LengthIndexedLine;
import org.locationtech.jts.planargraph.DirectedEdge;

import pedsim.cityimage.utilities.StringEnum.RouteChoice;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.ScenarioConfig;

/** Simulation state for the city-image testing module. */
public class PedSimCityImage extends PedSimCity {

	private static final long serialVersionUID = 1L;

	/**
	 * Distance bins used by the landmark testing mode. They are populated by the
	 * import/preparation pipeline when the relevant city resources are loaded.
	 */
	public static final ArrayList<Float> distances = new ArrayList<>();

	/**
	 * Kept for backward compatibility with city-image code paths that used their
	 * own edge cache. Core also has an indexed-edge cache; new code should prefer
	 * the core cache when possible.
	 */
	public static final Map<DirectedEdge, LengthIndexedLine> indexedEdgeCache = new HashMap<>();

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
		Populate populate = new Populate();
		populate.populateTests(this);
	}
}