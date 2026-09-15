package pedsim.empirical.engine;

import pedsim.core.engine.Engine;
import pedsim.core.engine.Import;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.engine.SimulationStateStore;

/** Engine for the empirical ABM module. */
public class EmpiricalEngine extends Engine {

  public EmpiricalEngine(StateFactory stateFactory) {
    super(stateFactory);
  }

  public EmpiricalEngine(StateFactory stateFactory, long baseSeed) {
    super(stateFactory, baseSeed);
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
    PedSimCityEmpirical state =
        (PedSimCityEmpirical) stateFactory.create(seed, job, scenarioConfig);

    state.start();

    onJobStarted(job, state, scenarioConfig);

    while (state.schedule.step(state)) {
      onJobStep(job, state, scenarioConfig);

      if (SimulationStateStore.getInstance().stopRequested) {
        break;
      }
    }

    onJobFinished(job, state, scenarioConfig);

    // Same gap as cityImage had: this engine overrides executeJob, so it never reached the export
    // core's Engine performs, and a run produced no volumes and no routes.
    state.flowHandler.exportFlowsData(1);

    state.finish();
  }
}
