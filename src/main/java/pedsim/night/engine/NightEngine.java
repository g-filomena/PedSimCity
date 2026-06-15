package pedsim.night.engine;

/** Night-simulation engine: extends core Engine with night-specific data clearing. */
public class NightEngine extends pedsim.core.engine.Engine {

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
  protected pedsim.core.engine.Engine createWorkerEngine() {
    return new NightEngine(stateFactory, baseSeed);
  }
}
