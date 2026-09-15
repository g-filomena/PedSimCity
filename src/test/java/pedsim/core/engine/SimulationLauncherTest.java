package pedsim.core.engine;

import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;
import pedsim.core.parameters.ParameterManager;
import pedsim.core.parameters.Pars;
import pedsim.testing.ParameterSnapshot;

class SimulationLauncherTest {
  @Test
  void explicitParametersOverrideCitySettingsBeforeEngineCreation() throws Exception {
    try (var saved = new ParameterSnapshot(Pars.class, ParameterManager.class)) {
      List<String> stages = new ArrayList<>();
      SimulationModule module =
          new ProbeModule() {
            public void applyDefaults(Map<String, String> params) {
              stages.add("defaults");
              Pars.population = 100;
              Pars.percentagePopulationAgent = 0.1;
            }

            public void loadCityConfig(String city) {
              assertEquals("Example", city);
              stages.add("city");
              Pars.population = 200;
              Pars.percentagePopulationAgent = 0.2;
            }

            public void applyParameters(Map<String, Object> params) {
              stages.add("explicit");
              assertEquals(300, Pars.population);
              assertEquals(60, Pars.numAgents);
            }

            public Engine createEngine() {
              stages.add("engine");
              assertEquals(9007199254740993L, Pars.seed);
              return new Engine(PedSimCity::new) {
                public void runJobs(
                    ScenarioConfig config,
                    boolean parallel,
                    SimulationStateStore.RunReservation reservation) {
                  reservation.requireActive();
                }
              };
            }
          };
      new SimulationLauncher(module)
          .headlessRun(
              new String[] {
                "--cityName=Example", "--actualPopulation=300", "--seed=9007199254740993"
              });
      assertEquals(List.of("defaults", "city", "explicit", "engine"), stages);
      assertFalse(SimulationStateStore.getInstance().running);
    }
  }

  @Test
  void rejectedHeadlessLaunchCannotChangeParameters() throws Exception {
    try (var saved = new ParameterSnapshot(Pars.class);
        var reservation = SimulationStateStore.getInstance().tryReserveRun()) {
      assertNotNull(reservation);
      Pars.cityName = "ActiveCity";
      new SimulationLauncher(new ProbeModule()).headlessRun(new String[] {"--cityName=OtherCity"});
      assertEquals("ActiveCity", Pars.cityName);
      reservation.requireActive();
    }
  }

  @Test
  void setupFailureReleasesReservation() {
    SimulationModule module =
        new ProbeModule() {
          public void applyDefaults(Map<String, String> params) {
            throw new IllegalArgumentException("bad setup");
          }
        };
    assertThrows(
        IllegalArgumentException.class,
        () -> new SimulationLauncher(module).headlessRun(new String[0]));
    try (var next = SimulationStateStore.getInstance().tryReserveRun()) {
      assertNotNull(next);
    }
  }

  @Test
  void negativeAndMaximumSeedsAreAcceptedExactly() throws Exception {
    try (var saved = new ParameterSnapshot(Pars.class, ParameterManager.class)) {
      for (long seed : new long[] {-1, 123, Long.MAX_VALUE}) {
        ParameterManager.initFromArgs(new String[] {"--seed=" + seed}, new Class<?>[] {Pars.class});
        assertEquals(seed, Pars.seed);
        if (seed >= 0) assertEquals(seed, Pars.resolvedSeed());
        else assertTrue(Pars.resolvedSeed() > 0);
      }
    }
  }

  private static class ProbeModule implements SimulationModule {
    public String moduleId() {
      return "probe";
    }

    public void clearStaticData() {}

    public Engine createEngine() {
      throw new AssertionError("Engine should not be created");
    }

    public ScenarioConfig scenarioConfig() {
      return new ScenarioConfig(null, null);
    }

    public void applyParameters(Map<String, Object> params) {}
  }
}
