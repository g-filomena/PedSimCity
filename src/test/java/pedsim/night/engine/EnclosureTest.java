package pedsim.night.engine;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import pedsim.night.parameters.NightPars;

class EnclosureTest {

  private final double open = NightPars.enclosedAreaM2;

  @AfterEach
  void restore() {
    NightPars.enclosedAreaM2 = open;
  }

  @Test
  void aStreetSeeingAtLeastTheOpenAreaIsNotEnclosed() {
    NightPars.enclosedAreaM2 = 5000.0;
    assertEquals(0.0, Enclosure.enclosure(5000.0));
    assertEquals(0.0, Enclosure.enclosure(20000.0));
  }

  @Test
  void enclosureRisesLinearlyToOneAsTheViewCloses() {
    NightPars.enclosedAreaM2 = 5000.0;
    assertEquals(0.5, Enclosure.enclosure(2500.0), 1e-12);
    assertEquals(0.8, Enclosure.enclosure(1000.0), 1e-12);
    assertEquals(1.0, Enclosure.enclosure(0.0), 1e-12);
  }

  @Test
  void aZeroOpenAreaSwitchesEnclosureOff() {
    NightPars.enclosedAreaM2 = 0.0;
    assertEquals(0.0, Enclosure.enclosure(100.0));
  }
}
