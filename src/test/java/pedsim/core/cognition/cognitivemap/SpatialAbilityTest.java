package pedsim.core.cognition.cognitivemap;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * Spatial ability is the Santa Barbara Sense-of-Direction scale (1-7) rescaled to [0, 1]: a normal
 * draw around the pooled sample mean of 3.83, standard deviation 1.06, cut at the scale's ends.
 */
class SpatialAbilityTest {

  @Test
  void theMedianAgentSitsAtTheScaleMean() {
    assertEquals((3.83 - 1.0) / 6.0, CognitiveMap.spatialAbilityAt(0.5), 1e-9);
  }

  @Test
  void theExtremesAreCutAtTheScaleEnds() {
    assertEquals(0.0, CognitiveMap.spatialAbilityAt(0.0), 0.0);
    assertEquals(1.0, CognitiveMap.spatialAbilityAt(1.0 - 1e-12), 0.0);
  }

  @Test
  void abilityRisesWithTheQuantile() {
    double previous = -1.0;
    for (int i = 0; i < 1000; i++) {
      double ability = CognitiveMap.spatialAbilityAt(i / 1000.0);
      assertTrue(ability >= previous, "ability fell at quantile " + i / 1000.0);
      previous = ability;
    }
  }

  @Test
  void aUniformPopulationHasTheScalesSpread() {
    int n = 100_000;
    double sum = 0.0;
    double sumSquares = 0.0;
    for (int i = 0; i < n; i++) {
      double ability = CognitiveMap.spatialAbilityAt((i + 0.5) / n);
      sum += ability;
      sumSquares += ability * ability;
    }
    double mean = sum / n;
    double sd = Math.sqrt(sumSquares / n - mean * mean);
    assertEquals((3.83 - 1.0) / 6.0, mean, 0.005);
    assertEquals(1.06 / 6.0, sd, 0.005);
  }
}
