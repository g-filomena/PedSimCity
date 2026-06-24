package pedsim.activity.engine;

import java.time.LocalTime;
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
   * Advances the 24h activity clock. Sets {@code isDark} between ~20:00 and ~06:00 so that
   * activity-based agents switch from daytime (workplace) to evening (night) destinations.
   */
  @Override
  protected void onStepUpdate(PedSimCity state, double steps) {
    if (state instanceof PedSimCityActivity activityState) {
      LocalTime time = TimePars.getTime(steps).toLocalTime();
      activityState.isDark =
          time.isAfter(LocalTime.of(19, 59)) || time.isBefore(LocalTime.of(6, 0));
    }
  }

  @Override
  protected Engine createWorkerEngine() {
    return new ActivityEngine(stateFactory, baseSeed);
  }
}
