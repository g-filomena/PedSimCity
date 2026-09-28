package pedsim.night.engine;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import pedsim.night.parameters.NightPars;

class NightLightingTest {

  private final double reassuranceLux = NightPars.reassuranceLux;

  @AfterEach
  void restore() {
    NightPars.reassuranceLux = reassuranceLux;
  }

  @Test
  void darknessRunsFromOneAtZeroLuxToZeroAtTheReassuranceLevel() {
    NightPars.reassuranceLux = 10.0;
    assertEquals(1.0, NightLighting.darkness(0.0));
    assertEquals(0.0, NightLighting.darkness(10.0));
    assertEquals(0.0, NightLighting.darkness(40.0));
  }

  @Test
  void darknessFallsFastestInTheFirstFewLux() {
    NightPars.reassuranceLux = 10.0;
    assertEquals(1.0 - Math.log1p(2.0) / Math.log1p(10.0), NightLighting.darkness(2.0), 1e-12);
    assertEquals(0.2527, NightLighting.darkness(5.0), 1e-4);
    double previous = 1.0;
    double previousDrop = Double.MAX_VALUE;
    for (double lux = 1.0; lux <= 10.0; lux += 1.0) {
      double darkness = NightLighting.darkness(lux);
      double drop = previous - darkness;
      assertTrue(darkness < previous, "falls with light at " + lux);
      assertTrue(drop < previousDrop, "falls less with each further lux at " + lux);
      previous = darkness;
      previousDrop = drop;
    }
  }

  @Test
  void aZeroReassuranceLevelMakesNothingDark() {
    NightPars.reassuranceLux = 0.0;
    assertEquals(0.0, NightLighting.darkness(0.0));
  }
}
