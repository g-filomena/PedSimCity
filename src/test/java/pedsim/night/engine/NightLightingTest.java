package pedsim.night.engine;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class NightLightingTest {

  @Test
  void darknessRunsFromOneAtZeroLuxToZeroAtTheReassuranceLevel() {
    assertEquals(1.0, NightLighting.darkness(0.0, 10.0));
    assertEquals(0.0, NightLighting.darkness(10.0, 10.0));
    assertEquals(0.0, NightLighting.darkness(40.0, 10.0));
  }

  @Test
  void darknessFallsFastestInTheFirstFewLux() {
    assertEquals(
        1.0 - Math.log1p(2.0) / Math.log1p(10.0), NightLighting.darkness(2.0, 10.0), 1e-12);
    assertEquals(0.2527, NightLighting.darkness(5.0, 10.0), 1e-4);
    double previous = 1.0;
    double previousDrop = Double.MAX_VALUE;
    for (double lux = 1.0; lux <= 10.0; lux += 1.0) {
      double darkness = NightLighting.darkness(lux, 10.0);
      double drop = previous - darkness;
      assertTrue(darkness < previous, "falls with light at " + lux);
      assertTrue(drop < previousDrop, "falls less with each further lux at " + lux);
      previous = darkness;
      previousDrop = drop;
    }
  }

  @Test
  void anAgentNeedingMoreLightFindsTheSameStreetDarker() {
    assertEquals(0.0, NightLighting.darkness(15.0, 10.0));
    assertTrue(NightLighting.darkness(15.0, 26.0) > 0.0);
    assertTrue(NightLighting.darkness(5.0, 26.0) > NightLighting.darkness(5.0, 8.9));
  }

  /**
   * A class whose median street is lit but a quarter of whose length is at 2 lux is expected to be
   * a quarter as dark as those streets, not lit: the darkness of the median would be 0.
   */
  @Test
  void aMostlyLitClassIsExpectedToBeAsDarkAsItsDarkShare() {
    double[] lengths = {100.0, 100.0, 100.0, 100.0};
    double[] lux = {20.0, 20.0, 20.0, 2.0};
    assertEquals(
        0.25 * NightLighting.darkness(2.0, 10.0),
        NightLighting.expectedDarkness(lengths, lux, 10.0),
        1e-12);
  }

  @Test
  void expectedDarknessWeighsStreetsByLength() {
    double[] lengths = {300.0, 100.0};
    double[] lux = {0.0, 20.0};
    assertEquals(0.75, NightLighting.expectedDarkness(lengths, lux, 10.0), 1e-12);
  }

  /** The running-sum table answers every reassurance level as a direct pass over the streets. */
  @Test
  void expectedDarknessAtAnyLevelMatchesTheDirectMean() {
    double[] lengths = {40.0, 120.0, 75.0, 300.0, 10.0, 55.0, 200.0};
    double[] lux = {0.0, 3.5, 8.9, 12.0, 12.0, 25.0, 60.0};
    for (double level : new double[] {0.5, 3.5, 8.9, 10.0, 12.0, 17.3, 26.0, 100.0}) {
      double weighted = 0.0;
      double total = 0.0;
      for (int i = 0; i < lengths.length; i++) {
        weighted += lengths[i] * NightLighting.darkness(lux[i], level);
        total += lengths[i];
      }
      assertEquals(
          weighted / total,
          NightLighting.expectedDarkness(lengths, lux, level),
          1e-12,
          "at " + level + " lx");
    }
  }

  @Test
  void aZeroReassuranceLevelMakesNothingDark() {
    assertEquals(0.0, NightLighting.darkness(0.0, 0.0));
    assertEquals(0.0, NightLighting.expectedDarkness(new double[] {1.0}, new double[] {0.0}, 0.0));
  }
}
