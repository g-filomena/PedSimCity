package pedsim.empirical.engine;

import pedsim.core.engine.Engine;
import pedsim.core.engine.Import;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.engine.SimulationStateStore;
import pedsim.core.parameters.Pars;
import pedsim.empirical.parameters.EmpiricalPars;

/** Engine for the empirical ABM module. */
public class EmpiricalEngine extends Engine {

	public EmpiricalEngine(StateFactory stateFactory) {
		super(stateFactory);
	}

	public EmpiricalEngine(StateFactory stateFactory, long baseSeed) {
		super(stateFactory, baseSeed);
	}

	@Override
	protected void afterSetParameters() {
		Pars.numAgents = EmpiricalPars.numAgents;
	}

	@Override
	protected Import createImporter() {
		return new EmpiricalImport();
	}

	@Override
	protected Engine createWorkerEngine() {
		return new EmpiricalEngine(stateFactory, baseSeed);
	}

	@Override
	public void executeJob(int job, ScenarioConfig scenarioConfig) throws Exception {
		long seed = seedForJob(job);
		PedSimCityEmpirical state = (PedSimCityEmpirical) stateFactory.create(seed, job, scenarioConfig);

		state.start();

		onJobStarted(job, state, scenarioConfig);

		while (state.schedule.step(state)) {
			onJobStep(job, state, scenarioConfig);

			if (SimulationStateStore.getInstance().stopRequested) {
				break;
			}
		}

		onJobFinished(job, state, scenarioConfig);

		state.finish();
	}
}
