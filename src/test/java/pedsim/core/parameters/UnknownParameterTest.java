package pedsim.core.parameters;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Set;
import org.junit.jupiter.api.Test;
import pedsim.activity.parameters.ActivityPars;

/** A parameter no class of the running module declares stops the run instead of being ignored. */
class UnknownParameterTest {

  private static final Class<?>[] CORE = {Pars.class, TimePars.class, RouteChoicePars.class};
  private static final Class<?>[] ACTIVITY = {
    Pars.class, TimePars.class, RouteChoicePars.class, ActivityPars.class
  };

  @Test
  void a_misspelt_key_is_rejected_with_the_name_it_meant() {
    IllegalArgumentException error =
        assertThrows(
            IllegalArgumentException.class,
            () -> ParameterManager.rejectUnknownKeys(Set.of("perceptionErorSD"), CORE));
    assertTrue(error.getMessage().contains("--perceptionErorSD"), error.getMessage());
    assertTrue(
        error.getMessage().contains("did you mean --perceptionErrorSD?"), error.getMessage());
  }

  @Test
  void a_removed_parameter_is_rejected() {
    assertThrows(
        IllegalArgumentException.class,
        () -> ParameterManager.rejectUnknownKeys(Set.of("maxReroutesPerLeg"), ACTIVITY));
  }

  @Test
  void aliases_and_launcher_keys_are_accepted() {
    assertDoesNotThrow(
        () ->
            ParameterManager.rejectUnknownKeys(
                Set.of("percentage", "days", "headless", "website", "module", "seed"), CORE));
  }

  @Test
  void a_module_key_is_known_only_to_a_module_that_declares_it() {
    assertThrows(
        IllegalArgumentException.class,
        () -> ParameterManager.rejectUnknownKeys(Set.of("useDestinationChoice"), CORE));
    assertDoesNotThrow(
        () -> ParameterManager.rejectUnknownKeys(Set.of("useDestinationChoice"), ACTIVITY));
  }
}
