package pedsim.cityimage.engine;

import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.logging.Logger;
import pedsim.cityimage.parameters.TestPars;
import pedsim.core.parameters.Pars;
import pedsim.core.agents.Agent;
import pedsim.core.engine.Engine;
import pedsim.core.engine.Import;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.engine.SimulationStateStore;
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
   * Restores the agent count after {@code Pars.setSimulationParameters()} has overwritten it.
   *
   * <p>This module runs one agent per route-choice model over a shared OD matrix, so its agent count
   * is the number of models being compared. {@code setSimulationParameters()} calls
   * {@code recomputeAgentCount()}, which replaces that with
   * {@code population * percentagePopulationAgent}. The empirical engine already had this hook for
   * the same reason; city-image did not, so whatever {@code TestPars.defineMode()} worked out was
   * discarded.
   *
   * <p>Only the count. The mode itself is resolved in {@code CityImageSimulationModule.applyMode()},
   * before the command-line overrides are re-applied — putting {@code defineMode()} here instead
   * made it run <i>after</i> the command line and silently reset {@code numberTripsPerAgent} to the
   * mode's own default, which is how {@code --numberTripsPerAgent=8} became 2,000 and ran out of
   * heap.
   */
  @Override
  protected void afterSetParameters() {
    Pars.numAgents = TestPars.routeChoiceModels.length;
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

    // Pushed onto the AWT event queue and into a label on PedSimCityImageApplet until the GUI was
    // removed, behind a GraphicsEnvironment.isHeadless() guard that made it a no-op on exactly the
    // runs anyone watches. It goes to the log instead, where a headless run can see it.
    LOGGER.fine("[cityImage] job " + job + ": " + total + " trips remaining across all jobs");
  }
}
