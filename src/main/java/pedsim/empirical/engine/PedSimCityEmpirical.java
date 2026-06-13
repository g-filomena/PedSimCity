package pedsim.empirical.engine;

import java.util.ArrayList;
import java.util.List;

import pedsim.core.engine.Environment;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.parameters.Pars;
import pedsim.empirical.agent.EmpiricalAgentsGroup;
import pedsim.empirical.agent.EmpiricalGroup;
import pedsim.empirical.parameters.EmpiricalPars;
import sim.engine.SimState;

/** Empirical ABM simulation state. */
public class PedSimCityEmpirical extends PedSimCity {

	private static final long serialVersionUID = 1L;

	public static final List<EmpiricalAgentsGroup> empiricalGroups = new ArrayList<>();

	public PedSimCityEmpirical(long seed, int job, ScenarioConfig scenarioConfig) {
		super(seed, job, defaultScenarioConfig(scenarioConfig));
	}

	private static ScenarioConfig defaultScenarioConfig(ScenarioConfig scenarioConfig) {
		if (scenarioConfig != null) {
			return scenarioConfig;
		}

		return new ScenarioConfig(EmpiricalGroup.values(), null);
	}

	@Override
	protected void populateEnvironment() {
		Populate populate = new Populate();
		populate.populateEmpiricalGroups(this);
	}

	public static void main(String[] args) throws Exception {
		EmpiricalPars.applyDefaults();

		if (args.length > 0 && args[0] != null && !args[0].isBlank()) {
			Pars.cityName = args[0].trim();
		}

		PedSimCity.clearStaticData();

		EmpiricalImport importer = new EmpiricalImport();
		importer.importFiles();

		Environment.prepare();

		ScenarioConfig scenarioConfig = new ScenarioConfig(EmpiricalGroup.values(), null);

		for (int job = 0; job < Pars.jobs; job++) {
			System.out.println("Run nr.. " + job);

			final SimState state = new PedSimCityEmpirical(System.currentTimeMillis(), job, scenarioConfig);

			state.start();

			while (state.schedule.step(state)) {
				// Simulation loop.
			}

			state.finish();
		}

		System.exit(0);
	}
}