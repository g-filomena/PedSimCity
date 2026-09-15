package pedsim.activity.engine;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Arrays;
import org.junit.jupiter.api.Test;
import pedsim.activity.agents.ActivityPurpose;
import pedsim.activity.parameters.ActivityPars;
import pedsim.core.engine.SimulationModule;
import pedsim.core.parameters.ParameterManager;
import pedsim.testing.ParameterSnapshot;

public class CityConfigContractTest {
  protected SimulationModule module() {
    return ActivitySimulationModule.INSTANCE;
  }

  @Test
  void cityConfigLoadsAndExplicitParametersWin() throws Exception {
    SimulationModule module = module();
    Class<?>[] classes =
        Arrays.copyOf(module.parameterClasses(), module.parameterClasses().length + 1);
    classes[classes.length - 1] = ParameterManager.class;
    try (var saved = new ParameterSnapshot(classes)) {
      ActivityPars.walkShareCommuteWorker = -1;
      module.loadCityConfig("Torino");
      assertEquals(0.163, ActivityPars.walkShareCommuteWorker, 1e-12);
      ParameterManager.initFromArgs(
          new String[] {"--walkShareCommuteWorker=0.42"}, module.parameterClasses());
      assertEquals(0.42, ActivityPars.walkShareCommuteWorker, 1e-12);
    } finally {
      ActivityPurpose.resetToDefaults();
    }
  }
}
