package pedsim.activity.engine;

import pedsim.core.engine.Engine;
import pedsim.core.engine.Import;
import pedsim.core.engine.PedSimCity;
import pedsim.core.parameters.TimePars;

/**
 * Engine for activity-based modules. Wires the activity {@link Import} and
 * {@link ActivityEnvironment} preparation, and clears the activity census/workplace static data in
 * addition to core's.
 *
 * <p>It is the base for the night and learning engines, mirroring the
 * {@code PedSimCity → PedSimCityActivity} and {@code Populate → ActivityPopulate} hierarchies.
 */
public class ActivityEngine extends Engine {

  public ActivityEngine(StateFactory stateFactory) {
    super(stateFactory);
  }

  public ActivityEngine(StateFactory stateFactory, long baseSeed) {
    super(stateFactory, baseSeed);
  }

  @Override
  protected Import createImporter() {
    return new ActivityImport();
  }

  @Override
  protected void prepareEnvironment() {
    ActivityEnvironment.prepare();
  }

  @Override
  protected boolean runDiagnosticsInstead() {
    if (!pedsim.activity.parameters.ActivityPars.calibrateCommute) {
      return false;
    }
    CommuteCalibration.run(pedsim.activity.parameters.ActivityPars.calibrationHomes, baseSeed);
    return true;
  }

  @Override
  protected void clearStaticData() {
    super.clearStaticData();
    PedSimCityActivity.clearStaticData();
  }

  /**
   * Advances the 24h activity clock. Behavioural darkness follows seasonal sunrise and sunset
   * for the configured date and city latitude.
   */
  @Override
  protected void onStepUpdate(PedSimCity state, double steps) {
    if (state instanceof PedSimCityActivity activityState) {
      java.time.LocalDateTime now = TimePars.getTime(steps);
      activityState.isDark = Daylight.isDark(now);
    }
  }

  /** Every module on this tier gets a row per simulated day. */
  @Override
  protected void onDayFinished(PedSimCity state, int job, int day) {
    DaySummary.append(state, state.appName(), job, day);
  }

  @Override
  protected Engine createWorkerEngine() {
    return new ActivityEngine(stateFactory, baseSeed);
  }
}
