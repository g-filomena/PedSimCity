package pedsim.learning.engine;

import pedsim.activity.engine.ActivityEngine;
import pedsim.core.engine.Engine;
import pedsim.core.engine.PedSimCity;

/**
 * Learning-simulation engine: a thin extension of {@link ActivityEngine}.
 *
 * <p>The activity engine provides everything the learning model needs — the census/POI import and
 * environment preparation, the day-based loop, {@code AgentReleaseManager} releases, and
 * {@code FlowHandler} cognitive-map/flow export ({@code updateCognitiveMapsData} /
 * {@code exportCognitiveMapsData}).
 *
 * <p>This subclass exists only to supply the {@link PedSimCityLearning} state factory (and to keep
 * parallel worker engines the same type). It needs no static-data clearing of its own — learning
 * adds no static caches — and inherits core + activity clearing. Time-based memory decay is applied
 * per agent per step in {@link pedsim.learning.agents.LearningAgent#step}, since the decay factor is
 * calibrated per {@code STEP_DURATION}.
 */
public class LearningEngine extends ActivityEngine {

  public LearningEngine(StateFactory stateFactory) {
    super(stateFactory);
  }

  public LearningEngine(StateFactory stateFactory, long baseSeed) {
    super(stateFactory, baseSeed);
  }

  @Override
  protected Engine createWorkerEngine() {
    return new LearningEngine(stateFactory, baseSeed);
  }

  /**
   * Learning runs its jobs sequentially: it has shared mutable state that is not thread-safe — a
   * static visibility-polygon cache in {@code RouteProperties} and internal {@code parallelStream}
   * usage — so concurrent jobs would race or oversubscribe the common pool.
   */
  @Override
  protected boolean supportsParallel() {
    return false;
  }

  /** The learning model is the only one that persists agent cognitive maps at each day boundary. */
  @Override
  protected void exportCognitiveData(PedSimCity state, int day) throws Exception {
    LearningDayReport.log(state, day);
    state.flowHandler.updateCognitiveMapsData(null);
    state.flowHandler.exportCognitiveMapsData(day);
  }
}
