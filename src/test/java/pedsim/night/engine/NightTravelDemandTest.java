package pedsim.night.engine;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class NightTravelDemandTest {

  @Test
  void theRequestedPairsAreSpreadOverTheDarkEventsAndRoundedUp() {
    assertEquals(182, NightTravelDemand.pairsPerEvent(4000, 22));
    assertEquals(1, NightTravelDemand.pairsPerEvent(5, 5));
    assertEquals(2, NightTravelDemand.pairsPerEvent(6, 5));
  }

  @Test
  void aDateWithNoDarkEventSendsEverythingAtTheFirstOneThatComes() {
    assertEquals(4000, NightTravelDemand.pairsPerEvent(4000, 0));
  }

  @Test
  void theEventsTogetherHoldTheWholeExperiment() {
    for (int requested = 1; requested <= 600; requested += 7) {
      for (int events = 1; events <= 40; events++) {
        int perEvent = NightTravelDemand.pairsPerEvent(requested, events);
        assertTrue(perEvent * events >= requested, requested + " pairs over " + events + " events");
        assertTrue(perEvent <= requested, requested + " pairs over " + events + " events");
      }
    }
  }
}
