package pedsim.activity.engine;

import pedsim.core.engine.Engine;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.engine.SimulationModule;
import java.util.Map;

public final class ActivitySimulationModule implements SimulationModule {

    public static final ActivitySimulationModule INSTANCE = new ActivitySimulationModule();

    private ActivitySimulationModule() {}

    @Override
    public String moduleId() {
        return "activity";
    }

    @Override
    public void applyMode() {
        // neutral model: no night, no empirical, no learning flags
        pedsim.core.parameters.Pars.isNight = false;
    }

    @Override
    public Engine createEngine() {
        return new ActivityEngine(PedSimCityActivity::new);
    }

    @Override
    public ScenarioConfig scenarioConfig() {
        return ScenarioConfig.singleRun();
    }

    @Override
    public void clearStaticData() {
        PedSimCity.clearStaticData();
        PedSimCityActivity.clearStaticData();
    }

    @Override
    public void applyParameters(Map<String, Object> params) {
        // later: numAgents, days, cityName, trips, etc.
    }
}