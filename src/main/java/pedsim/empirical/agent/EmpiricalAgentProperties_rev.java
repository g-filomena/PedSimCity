package pedsim.empirical.agent;

import java.util.List;
import java.util.Map;
import org.javatuples.Pair;
import pedsim.core.utilities.StringEnum.AgentBarrierType;
import sim.util.geo.Utilities;

/**
 * `EmpiricalAgentProperties` is a subclass of `AgentProperties` that represents the properties of
 * an agent in a pedestrian simulation with empirical-based parameters. It extends the base
 * `AgentProperties` class to incorporate additional parameters.
 */
public class EmpiricalAgentProperties_rev extends AgentProperties {

  // =====================
  // prob UTILITIES
  // =====================

  /**
   * Corrected version of updateProbabilities: - Uses index-based loop instead of indexOf(d), which
   * was buggy.
   */
  public void updateProbabilities(List<Double> probs, List<Pair<Double, Double>> pDistribution) {
    for (int i = 0; i < probs.size(); i++) {
      double p = Utilities.fromDistribution(pDistribution.get(i).getValue0(),
          pDistribution.get(i).getValue1(), null);
      probs.set(i, p);
    }
  }

  /**
   * Randomly picks one RouteChoiceProperty based on its probability weight.
   *
   * @param properties The ordered list of candidate properties. The order here determines how ties
   *        are resolved and ensures deterministic iteration.
   * @param weights A map giving each property its probability weight.
   * @return The chosen RouteChoiceProperty.
   */
  protected RouteChoiceProperty pickProperty(List<RouteChoiceProperty> properties,
      Map<RouteChoiceProperty, Double> weights) {
    // Sum all weights
    double totalWeight = 0.0;
    for (RouteChoiceProperty prop : properties) {
      totalWeight += weights.getOrDefault(prop, 0.0);
    }

    // Draw a random value in [0, totalWeight)
    double r = random.nextDouble() * totalWeight;

    // Walk through the properties in the given order until threshold is passed
    double cumulative = 0.0;
    for (RouteChoiceProperty prop : properties) {
      cumulative += weights.getOrDefault(prop, 0.0);
      if (r <= cumulative) {
        return prop;
      }
    }

    // Fallback: return the first property if something goes wrong
    return properties.get(0);
  }


  private void mapGroupProbs(List<RouteChoiceProperty> props, List<Pair<Double, Double>> pars,
      Map<RouteChoiceProperty, Double> outMap) {
    // 1:1 mapping
    if (pars.size() == props.size()) {
      for (int i = 0; i < props.size(); i++) {
        double w =
            Utilities.fromDistribution(pars.get(i).getValue0(), pars.get(i).getValue1(), null);
        outMap.put(props.get(i), w);
      }
      return;
    }
    // binary group, single param -> complement
    if (props.size() == 2 && pars.size() == 1) {
      double p = Utilities.fromDistribution(pars.get(0).getValue0(), pars.get(0).getValue1(), null);
      outMap.put(props.get(0), p);
      outMap.put(props.get(1), 1.0 - p);
      return;
    }
    // ternary (subGoals)
    if (props.size() == 3 && pars.size() == 2) {
      double p0 =
          Utilities.fromDistribution(pars.get(0).getValue0(), pars.get(0).getValue1(), null);
      double p1 =
          Utilities.fromDistribution(pars.get(1).getValue0(), pars.get(1).getValue1(), null);
      outMap.put(props.get(0), p0);
      outMap.put(props.get(1), p1);
      outMap.put(props.get(2), 1.0 - (p0 + p1));
      return;
    }
  }

  // ===========================
  // prob SETUP PER GROUP
  // ===========================

  public void setRouteChoiceParameters() {
    // Elements
    mapGroupProbs(
        List.of(RouteChoiceProperty.USING_ELEMENTS, RouteChoiceProperty.NOT_USING_ELEMENTS),
        List.of(new Pair<>(RouteChoicePars.probUsingElements, RouteChoicePars.probUsingElementsSD),
            new Pair<>(RouteChoicePars.probNotUsingElements,
                RouteChoicePars.probNotUsingElementsSD)),
        elementsMap);

    // Minimisation
    mapGroupProbs(List.of(RouteChoiceProperty.ROAD_DISTANCE, RouteChoiceProperty.ANGULAR_CHANGE),
        List.of(new Pair<>(RouteChoicePars.probRoadDistance, RouteChoicePars.probRoadDistanceSD),
            new Pair<>(RouteChoicePars.probAngularChange, RouteChoicePars.probAngularChangeSD)),
        minimisationMap);

    // Local heuristics
    mapGroupProbs(
        List.of(RouteChoiceProperty.ROAD_DISTANCE_LOCAL, RouteChoiceProperty.ANGULAR_CHANGE_LOCAL),
        List.of(
            new Pair<>(RouteChoicePars.probLocalRoadDistance,
                RouteChoicePars.probLocalRoadDistanceSD),
            new Pair<>(RouteChoicePars.probLocalAngularChange,
                RouteChoicePars.probLocalAngularChangeSD)),
        localHeuristicsMap);

    // Region-based
    mapGroupProbs(List.of(RouteChoiceProperty.REGION_BASED, RouteChoiceProperty.NOT_REGION_BASED),
        List.of(new Pair<>(RouteChoicePars.probRegionBasedNavigation,
            RouteChoicePars.probRegionBasedNavigationSD)),
        regionBasedMap);

    // Subgoals
    mapGroupProbs(
        List.of(RouteChoiceProperty.LOCAL_LANDMARKS, RouteChoiceProperty.BARRIER_SUBGOALS,
            RouteChoiceProperty.NO_SUBGOALS),
        List.of(
            new Pair<>(RouteChoicePars.probLocalLandmarks, RouteChoicePars.probLocalLandmarksSD),
            new Pair<>(RouteChoicePars.probBarrierSubGoals, RouteChoicePars.probBarrierSubGoalsSD)),
        subGoalsMap);

    // Distant landmarks
    mapGroupProbs(List.of(RouteChoiceProperty.USING_DISTANT, RouteChoiceProperty.NOT_USING_DISTANT),
        List.of(new Pair<>(RouteChoicePars.probDistantLandmarks,
            RouteChoicePars.probDistantLandmarksSD)),
        distantLandmarksMap);

    // Composite random elements
    randomElementsMap.put(RouteChoiceProperty.REGION_BASED,
        regionBasedMap.get(RouteChoiceProperty.REGION_BASED));
    randomElementsMap.put(RouteChoiceProperty.LOCAL_LANDMARKS,
        subGoalsMap.get(RouteChoiceProperty.LOCAL_LANDMARKS));
    randomElementsMap.put(RouteChoiceProperty.BARRIER_SUBGOALS,
        subGoalsMap.get(RouteChoiceProperty.BARRIER_SUBGOALS));
    randomElementsMap.put(RouteChoiceProperty.USING_DISTANT,
        distantLandmarksMap.get(RouteChoiceProperty.USING_DISTANT));

    Barriers();
  }


  // =====================
  // RANDOMISATION
  // =====================

  public void randomizeRouteChoiceParametersProbs() {
    ap.reset();
    ap.setRouteChoiceParameters();

    // Elements choice
    RouteChoiceProperty elementsChoice = ap.pickProperty(
        List.of(RouteChoiceProperty.USING_ELEMENTS, RouteChoiceProperty.NOT_USING_ELEMENTS),
        ap.elementsMap);
    ap.usingElements = (elementsChoice == RouteChoiceProperty.USING_ELEMENTS);

    // If not using elements, only minimisation and STOP
    if (!ap.usingElements) {
      RouteChoiceProperty minChoice = ap.pickProperty(
          List.of(RouteChoiceProperty.ROAD_DISTANCE, RouteChoiceProperty.ANGULAR_CHANGE),
          ap.minimisationMap);
      ap.minimisingDistance = (minChoice == RouteChoiceProperty.ROAD_DISTANCE);
      ap.minimisingAngular = !ap.minimisingDistance;

      // TEMPORARY override
      if (ap.minimisingAngular) {
        ap.minimisingDistance = true;
        ap.minimisingAngular = false;
      }
      return;
    }

    // Preferences
    ap.preferenceNaturalBarriers = ap.naturalBarriers < 0.95;
    ap.aversionSeveringBarriers = ap.severingBarriers > 1.05;

    // Local heuristic
    RouteChoiceProperty localChoice = ap.pickProperty(
        List.of(RouteChoiceProperty.ROAD_DISTANCE_LOCAL, RouteChoiceProperty.ANGULAR_CHANGE_LOCAL),
        ap.localHeuristicsMap);
    ap.localHeuristicDistance = (localChoice == RouteChoiceProperty.ROAD_DISTANCE_LOCAL);
    ap.localHeuristicAngular = !ap.localHeuristicDistance;
    if (ap.localHeuristicAngular) {
      ap.localHeuristicDistance = true;
      ap.localHeuristicAngular = false;
    }

    // Activation loop (region / subgoals / distant only)
    while (ap.usingElements && !ap.elementsActivated) {
      // region
      RouteChoiceProperty regionChoice = ap.pickProperty(
          List.of(RouteChoiceProperty.REGION_BASED, RouteChoiceProperty.NOT_REGION_BASED),
          ap.regionBasedMap);
      ap.regionBasedNavigation = (regionChoice == RouteChoiceProperty.REGION_BASED);
      ap.elementsActivated = true;

      // subgoals
      RouteChoiceProperty subChoice = ap.pickProperty(List.of(RouteChoiceProperty.LOCAL_LANDMARKS,
          RouteChoiceProperty.BARRIER_SUBGOALS, RouteChoiceProperty.NO_SUBGOALS), ap.subGoalsMap);
      ap.usingLocalLandmarks = (subChoice == RouteChoiceProperty.LOCAL_LANDMARKS);
      ap.barrierBasedNavigation = (subChoice == RouteChoiceProperty.BARRIER_SUBGOALS);
      if (ap.usingLocalLandmarks || ap.barrierBasedNavigation) {
        ap.elementsActivated = true;
        ap.barrierType = AgentBarrierType.SEPARATING;
        ap.landmarkType = LandmarkType.LOCAL;
      }

      // distant
      RouteChoiceProperty distChoice = ap.pickProperty(
          List.of(RouteChoiceProperty.USING_DISTANT, RouteChoiceProperty.NOT_USING_DISTANT),
          ap.distantLandmarksMap);
      ap.usingDistantLandmarks = (distChoice == RouteChoiceProperty.USING_DISTANT);
      if (ap.usingDistantLandmarks)
        ap.elementsActivated = true;
    }
  }

}
