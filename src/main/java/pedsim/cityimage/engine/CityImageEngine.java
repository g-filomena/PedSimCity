package pedsim.cityimage.engine;

import java.awt.EventQueue;
import java.awt.GraphicsEnvironment;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicInteger;
import pedsim.cityimage.applet.PedSimCityImageApplet;
import pedsim.core.agents.Agent;
import pedsim.core.engine.Engine;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.ScenarioConfig;

public class CityImageEngine extends Engine {

  private final ConcurrentHashMap<Integer, Integer> remainingTripsByJob = new ConcurrentHashMap<>();
  private final AtomicInteger totalRemainingTrips = new AtomicInteger(0);

  public CityImageEngine(StateFactory stateFactory) {
    super(stateFactory);
  }

  public CityImageEngine(StateFactory stateFactory, long baseSeed) {
    super(stateFactory, baseSeed);
  }

  @Override
  public void executeJob(int job, ScenarioConfig scenarioConfig) throws Exception {

    long seed = seedForJob(job);
    PedSimCityImage state = (PedSimCityImage) stateFactory.create(seed, job, scenarioConfig);
    state.start();
    onJobStarted(job, state, scenarioConfig);
    Set<Agent> agentList = state.getAgentsList();

    while (state.schedule.step(state)) {
      updateRemainingTrips(job, agentList);
      onJobStep(job, state, scenarioConfig);
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

  private void updateRemainingTrips(int job, Set<Agent> agentList) {
    int currentRemaining =
        agentList.stream().mapToInt(agent -> agent.OD.size() - agent.getTripsDone()).sum();

    Integer previous = remainingTripsByJob.put(job, currentRemaining);
    int previousValue = previous == null ? 0 : previous;
    int total = totalRemainingTrips.addAndGet(currentRemaining - previousValue);

    if (GraphicsEnvironment.isHeadless()) {
      return;
    }

    EventQueue.invokeLater(() -> {
      PedSimCityImageApplet.setRemainingTripsCount(total);
      PedSimCityImageApplet.updateRemainingTripsLabel(true);
    });
  }
}
