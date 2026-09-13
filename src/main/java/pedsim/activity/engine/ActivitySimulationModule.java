package pedsim.activity.engine;

import java.util.Map;
import pedsim.core.engine.Engine;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.engine.SimulationModule;
import pedsim.core.utilities.StringEnum;

public final class ActivitySimulationModule implements SimulationModule {

  public static final ActivitySimulationModule INSTANCE = new ActivitySimulationModule();

  private ActivitySimulationModule() {}

  @Override
  public long populationForCity(String city) {
    return CensusPopulation.residentTotal(city);
  }

  @Override
  public String moduleId() {
    return "activity";
  }

  /** The activity tier's per-city file; see {@link pedsim.activity.parameters.CityConfig}. */
  @Override
  public void loadCityConfig(String cityName) {
    pedsim.activity.parameters.CityConfig.load(cityName, parameterClasses());
  }

  @Override
  public Class<?>[] parameterClasses() {
    return new Class<?>[] {
      pedsim.core.parameters.Pars.class,
      pedsim.core.parameters.TimePars.class,
      pedsim.core.parameters.RouteChoicePars.class,
      pedsim.activity.parameters.ActivityPars.class
    };
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
    // Hourly volumes (h01–h24) for a single DEFAULT agent type — no vulnerability split.
    return new ScenarioConfig(StringEnum.Default.values(), StringEnum.Hour.values());
  }

  @Override
  public void clearStaticData() {
    PedSimCity.clearStaticData();
    PedSimCityActivity.clearStaticData();
  }

  @Override
  public void applyParameters(Map<String, Object> params) {
    if (params.containsKey("usePublicTransport")) {
      pedsim.core.parameters.RouteChoicePars.usePublicTransport =
          Boolean.parseBoolean(params.get("usePublicTransport").toString());
    }
    if (params.containsKey("useGravityModel")) {
      pedsim.core.parameters.RouteChoicePars.useGravityModel =
          Boolean.parseBoolean(params.get("useGravityModel").toString());
    }
    // The activity module's own parameters. Core applies command-line overrides to Pars, TimePars
    // and RouteChoicePars only, so anything in ActivityPars has to be picked up here or it is
    // silently ignored - which is exactly what happened to useDestinationChoice, quietly turning
    // every comparison run into two runs of the same code.
    if (params.containsKey("useDestinationChoice")) {
      pedsim.activity.parameters.ActivityPars.useDestinationChoice =
          Boolean.parseBoolean(params.get("useDestinationChoice").toString());
    }
    if (params.containsKey("useAgendaDepartureProfile")) {
      pedsim.activity.parameters.ActivityPars.useAgendaDepartureProfile =
          Boolean.parseBoolean(params.get("useAgendaDepartureProfile").toString());
    }
    if (params.containsKey("calibrateCommute")) {
      pedsim.activity.parameters.ActivityPars.calibrateCommute =
          Boolean.parseBoolean(params.get("calibrateCommute").toString());
    }
    if (params.containsKey("calibrationHomes")) {
      pedsim.core.parameters.ParameterManager.setFieldValue(
          pedsim.activity.parameters.ActivityPars.class, "calibrationHomes",
          params.get("calibrationHomes").toString());
    }
    for (String key :
        new String[] {
          "distanceWeight", "sizeWeight", "habitWeight", "choiceSetRadiusMetres",
          "walkedTripsPerPersonPerDay",
          "workplaceDistanceDecay", "workplaceMinDistanceMetres",
          "walkShareCommuteWorker", "walkShareCommuteStudent",
          "walkShareCommuteHalfDistance", "walkShareCommuteSteepness",
          "walkShareStudentHalfDistance", "walkShareStudentSteepness",
          "educationDistanceDecay"
        }) {
      if (params.containsKey(key)) {
        pedsim.core.parameters.ParameterManager.setFieldValue(
            pedsim.activity.parameters.ActivityPars.class, key, params.get(key).toString());
      }
    }
    if (params.containsKey("choiceSetSize")) {
      pedsim.core.parameters.ParameterManager.setFieldValue(
          pedsim.activity.parameters.ActivityPars.class, "choiceSetSize",
          params.get("choiceSetSize").toString());
    }
  }

  @Override
  public Map<String, Object> extraState() {
    return Map.of(
        "usePublicTransport", pedsim.core.parameters.RouteChoicePars.usePublicTransport,
        "useGravityModel", pedsim.core.parameters.RouteChoicePars.useGravityModel);
  }

  @Override
  public Map<String, Object> parameterSchema() {
    return Map.of(
        "usePublicTransport", "boolean",
        "useGravityModel", "boolean");
  }
}
