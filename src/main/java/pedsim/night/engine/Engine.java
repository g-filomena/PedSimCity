package pedsim.night.engine;

/** Engine specialisation for night simulations. */
public class Engine extends pedsim.core.engine.Engine {

  public Engine(StateFactory stateFactory) {
    super(stateFactory);
  }

  public Engine(StateFactory stateFactory, long baseSeed) {
    super(stateFactory, baseSeed);
  }

  @Override
  protected void clearStaticData() {
    pedsim.core.engine.PedSimCity.clearStaticData();
    PedSimCityNight.clearNightStaticData();
  }

  @Override
  protected Engine createWorkerEngine() {
    return new Engine(stateFactory, baseSeed);
  }
}
