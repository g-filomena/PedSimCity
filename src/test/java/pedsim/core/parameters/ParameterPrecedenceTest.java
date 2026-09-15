package pedsim.core.parameters;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * The order in which parameters are written: <b>module defaults → city file → command line →
 * derived</b>.
 *
 * <p>Every parameter defect this project has had was a later stage silently beating an earlier one
 * — a module default overwriting the command line, a design's own figures overwriting it, a
 * derivation overwriting it. These pin the rule rather than the individual cases.
 */
class ParameterPrecedenceTest {

  private static final Class<?>[] CORE = {Pars.class, TimePars.class, RouteChoicePars.class};

  @Test
  void commandLineNamesThatDifferFromTheirFieldStillLand() {
    Pars.percentagePopulationAgent = 0.001;
    Pars.population = 1;
    Pars.durationDays = 1;

    ParameterManager.initFromArgs(
        new String[] {"--percentage=0.004", "--actualPopulation=846567", "--days=7"}, CORE);

    assertEquals(0.004, Pars.percentagePopulationAgent, 1e-12);
    assertEquals(846567, Pars.population);
    assertEquals(7, Pars.durationDays);
  }

  /** The agent count is derived from population and percentage when it is not given. */
  @Test
  void theAgentCountIsDerivedWhenItIsNotGiven() {
    ParameterManager.initFromArgs(
        new String[] {"--actualPopulation=100000", "--percentage=0.001"}, CORE);

    assertEquals(100, Pars.numAgents);
    assertFalse(ParameterManager.wasGivenOnCommandLine("numAgents"));
  }

  /** A count that was asked for is not an input to the derivation. */
  @Test
  void anExplicitAgentCountSurvivesTheDerivation() {
    ParameterManager.initFromArgs(
        new String[] {"--actualPopulation=100000", "--percentage=0.001", "--numAgents=42"}, CORE);

    assertEquals(42, Pars.numAgents);
    assertTrue(ParameterManager.wasGivenOnCommandLine("numAgents"));
  }

  /**
   * Asking for a circuity factor means asking for that factor: the startup measurement writes the
   * same field, so supplying one without the other must switch the measurement off.
   */
  @Test
  void aGivenCircuityFactorTurnsTheMeasurementOff() {
    Pars.measureNetworkCircuity = true;
    ParameterManager.initFromArgs(new String[] {"--networkCircuityFactor=1.29"}, CORE);
    assertFalse(Pars.measureNetworkCircuity);

    Pars.measureNetworkCircuity = false;
    ParameterManager.initFromArgs(
        new String[] {"--networkCircuityFactor=1.29", "--measureNetworkCircuity=true"}, CORE);
    assertTrue(Pars.measureNetworkCircuity, "an explicit flag on the same line should win");
  }

  /** Enum-valued parameters are set by the one writer, not hand-parsed per module. */
  @Test
  void enumValuedParametersAreSetByReflection() {
    ParameterManager.setFieldValue(Probe.class, "mode", "second");
    assertEquals(Probe.Mode.SECOND, Probe.mode);
  }

  /** A key naming no field is ignored rather than throwing. */
  @Test
  void anUnknownKeyIsHarmless() {
    ParameterManager.initFromArgs(new String[] {"--notAParameterOfAnything=3"}, CORE);
  }

  static class Probe {
    enum Mode {
      FIRST,
      SECOND
    }

    static Mode mode = Mode.FIRST;
  }
}
