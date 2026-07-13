package pedsim.core.agents;

import java.util.Collections;
import java.util.EnumSet;
import java.util.Set;
import pedsim.core.utilities.StringEnum.AgentBarrierType;
import pedsim.core.utilities.StringEnum.LandmarkType;
import pedsim.core.utilities.StringEnum.LocalHeuristicMode;
import pedsim.core.utilities.StringEnum.MinimisationMode;
import pedsim.core.utilities.StringEnum.RouteChoiceElement;

/**
 * An agent's route-choice configuration: the minimisation mode (shortest vs least-turn), the local
 * heuristic, the active route-choice elements (landmarks, regions, barriers) and the barrier
 * cost/preference factors. A plain state holder — {@code Heuristics} and the module-specific
 * subclasses decide the values; the routing classes read them.
 */
public class AgentProperties {

  private MinimisationMode minimisationMode;
  private LocalHeuristicMode localHeuristicMode;
  private final EnumSet<RouteChoiceElement> elements;

  protected boolean preferenceNaturalBarriers;
  protected boolean aversionSeveringBarriers;
  protected double naturalBarriersMean;
  protected double naturalBarriersSD;
  protected double severingBarriersMean;
  protected double severingBarriersSD;

  private AgentBarrierType barrierType;
  private LandmarkType landmarkType;

  public AgentProperties() {
    this.elements = EnumSet.noneOf(RouteChoiceElement.class);
    reset();
  }

  public void reset() {
    minimisationMode = MinimisationMode.NONE;
    localHeuristicMode = LocalHeuristicMode.NONE;
    elements.clear();

    preferenceNaturalBarriers = false;
    aversionSeveringBarriers = false;

    naturalBarriersMean = 0.0;
    naturalBarriersSD = 0.0;
    severingBarriersMean = 0.0;
    severingBarriersSD = 0.0;

    barrierType = null;
    landmarkType = null;
  }

  public boolean shouldOnlyUseMinimization() {
    return minimisationMode != MinimisationMode.NONE;
  }

  public boolean shouldUseLocalHeuristic() {
    return localHeuristicMode != LocalHeuristicMode.NONE;
  }

  public boolean hasElement(RouteChoiceElement element) {
    return elements.contains(element);
  }

  public void addElement(RouteChoiceElement element) {
    elements.add(element);
  }

  public void removeElement(RouteChoiceElement element) {
    elements.remove(element);
  }

  public void setElement(RouteChoiceElement element, boolean enabled) {
    if (enabled) {
      elements.add(element);
    } else {
      elements.remove(element);
    }
  }

  public void clearElements() {
    elements.clear();
  }

  public Set<RouteChoiceElement> getElements() {
    return Collections.unmodifiableSet(elements);
  }

  public MinimisationMode getMinimisationMode() {
    return minimisationMode;
  }

  public void setMinimisationMode(MinimisationMode minimisationMode) {
    this.minimisationMode = minimisationMode;
  }

  public LocalHeuristicMode getLocalHeuristicMode() {
    return localHeuristicMode;
  }

  public void setLocalHeuristicMode(LocalHeuristicMode localHeuristicMode) {
    this.localHeuristicMode = localHeuristicMode;
  }

  public boolean isMinimisingDistance() {
    return minimisationMode == MinimisationMode.DISTANCE;
  }

  public boolean isMinimisingAngular() {
    return minimisationMode == MinimisationMode.ANGULAR;
  }

  public boolean isLocalHeuristicDistance() {
    return localHeuristicMode == LocalHeuristicMode.DISTANCE;
  }

  public boolean isLocalHeuristicAngular() {
    return localHeuristicMode == LocalHeuristicMode.ANGULAR;
  }

  public boolean isUsingLocalLandmarks() {
    return hasElement(RouteChoiceElement.LOCAL_LANDMARKS);
  }

  public void setUsingLocalLandmarks(boolean enabled) {
    setElement(RouteChoiceElement.LOCAL_LANDMARKS, enabled);
  }

  public boolean isUsingDistantLandmarks() {
    return hasElement(RouteChoiceElement.DISTANT_LANDMARKS);
  }

  public void setUsingDistantLandmarks(boolean enabled) {
    setElement(RouteChoiceElement.DISTANT_LANDMARKS, enabled);
  }

  public boolean isRegionBasedNavigation() {
    return hasElement(RouteChoiceElement.REGION_BASED_NAVIGATION);
  }

  public void setRegionBasedNavigation(boolean enabled) {
    setElement(RouteChoiceElement.REGION_BASED_NAVIGATION, enabled);
  }

  public boolean isBarrierBasedNavigation() {
    return hasElement(RouteChoiceElement.BARRIER_BASED_NAVIGATION);
  }

  public void setBarrierBasedNavigation(boolean enabled) {
    setElement(RouteChoiceElement.BARRIER_BASED_NAVIGATION, enabled);
  }

  public boolean isPreferenceNaturalBarriers() {
    return preferenceNaturalBarriers;
  }

  public void setPreferenceNaturalBarriers(boolean preferenceNaturalBarriers) {
    this.preferenceNaturalBarriers = preferenceNaturalBarriers;
  }

  public boolean isAversionSeveringBarriers() {
    return aversionSeveringBarriers;
  }

  public void setAversionSeveringBarriers(boolean aversionSeveringBarriers) {
    this.aversionSeveringBarriers = aversionSeveringBarriers;
  }

  public double getNaturalBarriersMean() {
    return naturalBarriersMean;
  }

  public void setNaturalBarriersMean(double naturalBarriersMean) {
    this.naturalBarriersMean = naturalBarriersMean;
  }

  public double getNaturalBarriersSD() {
    return naturalBarriersSD;
  }

  public void setNaturalBarriersSD(double naturalBarriersSD) {
    this.naturalBarriersSD = naturalBarriersSD;
  }

  public double getSeveringBarriersMean() {
    return severingBarriersMean;
  }

  public void setSeveringBarriersMean(double severingBarriersMean) {
    this.severingBarriersMean = severingBarriersMean;
  }

  public double getSeveringBarriersSD() {
    return severingBarriersSD;
  }

  public void setSeveringBarriersSD(double severingBarriersSD) {
    this.severingBarriersSD = severingBarriersSD;
  }

  public AgentBarrierType getBarrierType() {
    return barrierType;
  }

  public void setBarrierType(AgentBarrierType barrierType) {
    this.barrierType = barrierType;
  }

  public LandmarkType getLandmarkType() {
    return landmarkType;
  }

  public void setLandmarkType(LandmarkType landmarkType) {
    this.landmarkType = landmarkType;
  }
}
