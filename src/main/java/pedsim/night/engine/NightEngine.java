package pedsim.night.engine;

import pedsim.activity.engine.ActivityEngine;
import pedsim.core.engine.Engine;
import pedsim.core.engine.Import;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.engine.TripDiagnostic;
import pedsim.night.parameters.NightPars;

/**
 * Night-simulation engine: extends {@link ActivityEngine} with the night importer, night
 * environment preparation, night-specific data clearing and the day/night step update.
 */
public class NightEngine extends ActivityEngine {

  public NightEngine(StateFactory stateFactory) {
    super(stateFactory);
  }

  public NightEngine(StateFactory stateFactory, long baseSeed) {
    super(stateFactory, baseSeed);
  }

  @Override
  protected Import createImporter() {
    return new NightImport();
  }

  @Override
  protected void prepareEnvironment() {
    NightEnvironment.prepare();
  }

  @Override
  protected void clearStaticData() {
    super.clearStaticData();
    PedSimCityNight.clearNightStaticData();
  }

  @Override
  protected void onJobFinished(int job, PedSimCity state, ScenarioConfig scenarioConfig) {
    if (NightPars.enableLightABTesting) {
      System.out.println(
          "Night A/B twin testing was enabled: vulnerability outputs are experimental twin "
              + "comparisons, not census-sampled shares.");
      TripDiagnostic.saveABTestComparison("ab_test_comparison.csv");
    }
  }

  @Override
  protected Engine createWorkerEngine() {
    return new NightEngine(stateFactory, baseSeed);
  }
}
