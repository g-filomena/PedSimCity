package pedsim.core.agents;

import java.util.Random;
import pedsim.core.parameters.PopulationPars;
import pedsim.core.utilities.StringEnum.AgentBarrierType;
import pedsim.core.utilities.StringEnum.LandmarkType;

public class AgentProperties {

  public Agent agent;

  // routing flags
  public boolean minimisingDistance = false;
  public boolean minimisingAngular = false;
  public boolean localHeuristicDistance = false;
  public boolean localHeuristicAngular = false;

  public boolean usingLocalLandmarks = false;
  public boolean usingDistantLandmarks = false;
  public boolean regionBasedNavigation = false;
  public boolean barrierBasedNavigation = false;
  public boolean preferenceNaturalBarriers = false;
  public boolean aversionSeveringBarriers = false;

  public double naturalBarriersMean = 0.0;
  public double naturalBarriersSD = 0.10;
  public double severingBarriersMean = 0.0;
  public double severingBarriersSD = 0.10;

  public AgentBarrierType barrierType;
  public LandmarkType landmarkType;

  boolean usingElements = false;
  boolean elementsActivated = false;

  private static final double MIN_NATURAL_BARRIERS = 1.00;
  private static final double MAX_NATURAL_BARRIERS = 0.00;
  private static final double MIN_SEVERING_BARRIERS = 1.00;
  private static final double MAX_SEVERING_BARRIERS = 2.00;

  final Random random = new Random();

  public AgentProperties(Agent agent) {
    this.agent = agent;
  }

  protected void setMinimisationOnly() {
    minimisingDistance = true;
  }

  protected void setBarriersEffect() {
    // Barriers
    naturalBarriersMean = rescale(0.0, 1.0, MAX_NATURAL_BARRIERS, MIN_NATURAL_BARRIERS,
        PopulationPars.naturalBarriers);
    naturalBarriersSD = PopulationPars.naturalBarriersSD;

    severingBarriersMean = rescale(0.0, 1.0, MIN_SEVERING_BARRIERS, MAX_SEVERING_BARRIERS,
        PopulationPars.severingBarriers);
    severingBarriersSD = PopulationPars.severingBarriersSD;


    // Preferences
    preferenceNaturalBarriers = naturalBarriersMean < 0.95;
    aversionSeveringBarriers = severingBarriersMean > 1.05;
  }

  public void reset() {
    usingElements = false;
    elementsActivated = false;
    minimisingDistance = false;
    minimisingAngular = false;
    localHeuristicDistance = false;
    localHeuristicAngular = false;
    barrierType = null;
    usingLocalLandmarks = false;
    barrierBasedNavigation = false;
    regionBasedNavigation = false;
    usingDistantLandmarks = false;
    preferenceNaturalBarriers = false;
    aversionSeveringBarriers = false;
  }

  private double rescale(double oldMin, double oldMax, double newMin, double newMax, double value) {
    double oldRange = oldMax - oldMin;
    double newRange = newMax - newMin;
    return (value - oldMin) * newRange / oldRange + newMin;
  }

  /**
   * Checks if the agent should use minimization for route planning.
   *
   * @return True if the agent should use minimization, otherwise false.
   */
  public boolean shouldOnlyUseMinimization() {
    return minimisingDistance || minimisingAngular;
  }

  /**
   * Checks if the agent should use local heuristics for route planning.
   *
   * @return True if the agent should use local heuristics, otherwise false.
   */
  public boolean shouldUseLocalHeuristic() {
    return localHeuristicDistance || localHeuristicAngular;
  }
}
