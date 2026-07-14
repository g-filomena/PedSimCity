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
  protected void clearStaticData() {
    super.clearStaticData();
    PedSimCityActivity.clearStaticData();
  }

  /**
   * Advances the 24h activity clock. Behavioural darkness follows the seasonal sunrise/sunset
   * model when enabled (so agents switch to evening behaviour at actual dusk, which in Liverpool
   * ranges from ~16:00 in December to ~21:45 in June), the fixed
   * {@code DAY_START_HOUR}/{@code NIGHT_START_HOUR} window otherwise. The exporter's day/night
   * volume aggregation always uses the fixed window so outputs stay comparable.
   */
  @Override
  protected void onStepUpdate(PedSimCity state, double steps) {
    if (state instanceof PedSimCityActivity activityState) {
      java.time.LocalDateTime now = TimePars.getTime(steps);
      activityState.isDark =
          pedsim.activity.parameters.ActivityPars.useSeasonalDaylight
              ? Daylight.isDark(now)
              : TimePars.isNight(now.toLocalTime());
    }
  }

  @Override
  protected Engine createWorkerEngine() {
    return new ActivityEngine(stateFactory, baseSeed);
  }
}
