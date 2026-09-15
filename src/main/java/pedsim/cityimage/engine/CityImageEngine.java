package pedsim.cityimage.engine;

import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.logging.Logger;
import pedsim.cityimage.parameters.TestPars;
import pedsim.core.agents.Agent;
import pedsim.core.engine.Engine;
import pedsim.core.engine.Import;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.engine.SimulationStateStore;
import pedsim.core.parameters.Pars;
import pedsim.core.utilities.LoggerUtil;

/** Engine specialised for the city-image testing module. */
public class CityImageEngine extends Engine {

  private static final Logger LOGGER = LoggerUtil.getLogger();

  private final ConcurrentHashMap<Integer, Integer> remainingTripsByJob = new ConcurrentHashMap<>();
  private final AtomicInteger totalRemainingTrips = new AtomicInteger(0);

  public CityImageEngine(StateFactory stateFactory) {
    super(stateFactory);
  }

  public CityImageEngine(StateFactory stateFactory, long baseSeed) {
    super(stateFactory, baseSeed);
  }

  @Override
  protected Import createImporter() {
    return new CityImageImport();
  }

  @Override
  protected void clearStaticData() {
    super.clearStaticData();
    TestPars.distances.clear();
  }

  /**
   * The agent count is the number of route-choice models being compared: this module runs one agent
   * per model over a shared OD matrix.
   *
   * <p>Set here rather than in {@code CityImageSimulationModule.applyMode()} only because
   * {@code TestPars.defineMode()} may have changed the model list since. The test design itself is
   * resolved in {@code applyMode()}, which runs before the command line is re-applied; resolving it
   * here would put it after, and the design's own defaults would override whatever was asked for.
   */
  @Override
  protected void afterSetParameters() {
    Pars.numAgents = TestPars.scenarios.length;
  }

  @Override
  protected Engine createWorkerEngine() {
    return new CityImageEngine(stateFactory, baseSeed);
  }

  @Override
  public void executeJob(int job, ScenarioConfig scenarioConfig) throws Exception {
    long seed = seedForJob(job);
    PedSimCityImage state = (PedSimCityImage) stateFactory.create(seed, job, scenarioConfig);

    state.start();

    onJobStarted(job, state, scenarioConfig);

    Set<?> agentList = state.getAgentsList();

    while (state.schedule.step(state)) {
      updateRemainingTrips(job, agentList);
      onJobStep(job, state, scenarioConfig);

      if (SimulationStateStore.getInstance().stopRequested) {
        LOGGER.info("Stop requested - ending CityImage job.");
        break;
      }
    }

    updateRemainingTrips(job, agentList);
    onJobFinished(job, state, scenarioConfig);

    // The module's output: the per-edge volumes each route-choice model produced. This engine
    // overrides executeJob, so it does not reach the export core's Engine performs per day and must
    // do it here.
    state.flowHandler.exportFlowsData(1);
    LOGGER.info("[cityImage] job " + job + ": pedestrian volumes and routes exported.");

    state.finish();
  }

  @Override
  protected void onJobStarted(int job, PedSimCity state, ScenarioConfig scenarioConfig) {
    remainingTripsByJob.put(job, 0);
  }

  @Override
  protected void onJobFinished(int job, PedSimCity state, ScenarioConfig scenarioConfig) {
    remainingTripsByJob.remove(job);
  }

  private void updateRemainingTrips(int job, Set<?> agentList) {
    int currentRemaining = 0;

    for (Object object : agentList) {
      if (object instanceof Agent agent) {
        currentRemaining += Math.max(0, agent.OD.size() - agent.getTripsDone());
      }
    }

    Integer previous = remainingTripsByJob.put(job, currentRemaining);
    int previousValue = previous == null ? 0 : previous;

    int total = totalRemainingTrips.addAndGet(currentRemaining - previousValue);

    LOGGER.fine("[cityImage] job " + job + ": " + total + " trips remaining across all jobs");
  }
}
