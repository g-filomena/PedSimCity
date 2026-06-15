package pedsim.core.agents;

import pedsim.core.parameters.PopulationPars;

public final class BarrierPreference {

  // Natural barriers are intentionally inverted:
  // higher population value -> lower routing cost threshold
  private static final double NATURAL_OUTPUT_MIN = 0.00;
  private static final double NATURAL_OUTPUT_MAX = 1.00;

  // Severing barriers keep the original direction:
  // higher population value -> higher routing cost threshold
  private static final double SEVERING_OUTPUT_MIN = 1.00;
  private static final double SEVERING_OUTPUT_MAX = 2.00;

  private static final double NATURAL_PREFERENCE_THRESHOLD = 0.95;
  private static final double SEVERING_AVERSION_THRESHOLD = 1.05;

  public void assign(AgentProperties ap) {
    double naturalMean =
        rescale(0.0, 1.0, NATURAL_OUTPUT_MAX, NATURAL_OUTPUT_MIN, PopulationPars.naturalBarriers);

    double severingMean =
        rescale(
            0.0, 1.0, SEVERING_OUTPUT_MIN, SEVERING_OUTPUT_MAX, PopulationPars.severingBarriers);

    ap.setNaturalBarriersMean(naturalMean);
    ap.setNaturalBarriersSD(PopulationPars.naturalBarriersSD);

    ap.setSeveringBarriersMean(severingMean);
    ap.setSeveringBarriersSD(PopulationPars.severingBarriersSD);

    ap.setPreferenceNaturalBarriers(naturalMean < NATURAL_PREFERENCE_THRESHOLD);
    ap.setAversionSeveringBarriers(severingMean > SEVERING_AVERSION_THRESHOLD);
  }

  private double rescale(double oldMin, double oldMax, double newMin, double newMax, double value) {

    double oldRange = oldMax - oldMin;
    if (oldRange == 0.0) {
      throw new IllegalArgumentException("Old range cannot be zero.");
    }

    double newRange = newMax - newMin;
    return (value - oldMin) * newRange / oldRange + newMin;
  }
}
