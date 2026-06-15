package pedsim.empirical.engine;

import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.parameters.Pars;
import pedsim.empirical.agent.EmpiricalGroup;
import pedsim.empirical.parameters.EmpiricalPars;

/** Empirical ABM simulation state. */
public class PedSimCityEmpirical extends PedSimCity {

	private static final long serialVersionUID = 1L;

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
		EmpiricalPopulate populate = new EmpiricalPopulate();
		populate.populateEmpiricalGroups(this);
	}

	public static void main(String[] args) throws Exception {
		EmpiricalPars.applyDefaults();

		if (args.length > 0 && args[0] != null && !args[0].isBlank()) {
			Pars.cityName = args[0].trim();
		}

		ScenarioConfig scenarioConfig = new ScenarioConfig(EmpiricalGroup.values(), null);
		new EmpiricalEngine(PedSimCityEmpirical::new).runJobs(scenarioConfig, Pars.parallel);

		System.exit(0);
	}
}