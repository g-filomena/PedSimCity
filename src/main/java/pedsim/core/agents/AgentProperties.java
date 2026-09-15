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
 * The working copy a single trip is planned against.
 *
 * <p>{@link RouteChoiceModel} is the decision - what this agent's route choice <i>is</i>. This is
 * the mutable view of it that the routing classes read while one trip is being planned, and that a
 * planner may switch parts of off when the trip cannot use them: {@code RoutePlanner} disables
 * region navigation for an origin and destination inside one region,
 * {@code RegionBasedNavigation} when no gateway sequence survives.
 *
 * <p><b>{@code RoutePlanner} reapplies the model at the start of every trip</b>, through
 * {@link #applyModel(RouteChoiceModel)}, so those decisions last exactly as long as the trip that
 * made them. Writing them to something the agent keeps is what turned "not for this trip" into "not
 * for this agent, ever" - an OD agent walks its whole matrix on one of these.
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

  /** The model this copy was last built from, or null before the first trip is planned. */
  private RouteChoiceModel model;

  public AgentProperties() {
    this.elements = EnumSet.noneOf(RouteChoiceElement.class);
    reset();
  }

  /**
   * Rebuilds this working copy from {@code model}, discarding whatever the previous trip decided.
   *
   * @param model the agent's route choice
   */
  public void applyModel(RouteChoiceModel model) {
    reset();
    this.model = model;
    if (model == null) {
      return;
    }
    minimisationMode = model.minimisation();
    localHeuristicMode = model.localHeuristic();
    elements.addAll(model.elements());
    landmarkType = model.landmarkType();

    RouteChoiceModel.BarrierPreferences barriers = model.barriers();
    barrierType = barriers.type();
    preferenceNaturalBarriers = barriers.preferenceNatural();
    aversionSeveringBarriers = barriers.aversionSevering();
    naturalBarriersMean = barriers.naturalMean();
    naturalBarriersSD = barriers.naturalSD();
    severingBarriersMean = barriers.severingMean();
    severingBarriersSD = barriers.severingSD();
  }

  /** The model this copy was built from, or null if none has been applied. */
  public RouteChoiceModel model() {
    return model;
  }

  /**
   * A model describing the values currently held here.
   *
   * <p>For a subclass that decides a route choice by writing into these fields - the empirical
   * module samples its clusters that way - this is how the result becomes the agent's model. The
   * strategy follows the same rule the fields do: a minimisation mode and nothing else is a pure
   * minimisation, anything else is element-based.
   *
   * @return the model these values describe
   */
  public RouteChoiceModel toModel() {
    RouteChoiceModel.BarrierPreferences barriers =
        new RouteChoiceModel.BarrierPreferences(
            barrierType,
            preferenceNaturalBarriers,
            aversionSeveringBarriers,
            naturalBarriersMean,
            naturalBarriersSD,
            severingBarriersMean,
            severingBarriersSD);

    if (minimisationMode != MinimisationMode.NONE) {
      return RouteChoiceModel.minimising(minimisationMode).withBarriers(barriers);
    }
    return RouteChoiceModel.usingElements(localHeuristicMode, elements, landmarkType, barriers);
  }

  /** Clears every value back to the state before a model is applied. */
  public void reset() {
    model = null;
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

  /**
   * Whether the route is one cost minimised end to end, with no sub-goals - v1.11's
   * {@code onlyMinimising}, and still the intended meaning.
   *
   * <p>Answered by the model's declared {@code Strategy} rather than inferred from "a minimisation
   * mode is set", so a model cannot fall into this case by acquiring a mode.
   */
  public boolean shouldOnlyUseMinimization() {
    return model != null && model.isPureMinimisation();
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
