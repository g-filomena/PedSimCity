package pedsim.night.engine;

import pedsim.core.engine.Engine;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.engine.TripDiagnostic;
import pedsim.core.parameters.TimePars;
import pedsim.night.parameters.NightPars;

/** Night-simulation engine: extends core Engine with night-specific data clearing. */
public class NightEngine extends Engine {

  public NightEngine(StateFactory stateFactory) {
    super(stateFactory);
  }

  public NightEngine(StateFactory stateFactory, long baseSeed) {
    super(stateFactory, baseSeed);
  }

  @Override
  protected void clearStaticData() {
    super.clearStaticData();
    PedSimCityNight.clearNightStaticData();
  }

  @Override
  protected void onStepUpdate(PedSimCity state, double steps) {
    if (state instanceof PedSimCityNight nightState) {
      java.time.LocalTime time = TimePars.getTime(steps).toLocalTime();
      nightState.isDark = time.isAfter(java.time.LocalTime.of(19, 59))
          || time.isBefore(java.time.LocalTime.of(6, 0));
    }
  }

  @Override
  protected void onJobFinished(int job, PedSimCity state, ScenarioConfig scenarioConfig) {
    if (NightPars.enableLightABTesting) {
      TripDiagnostic.saveABTestComparison("ab_test_comparison.csv");
    }
  }

  @Override
  protected Engine createWorkerEngine() {
    return new NightEngine(stateFactory, baseSeed);
  }
}
