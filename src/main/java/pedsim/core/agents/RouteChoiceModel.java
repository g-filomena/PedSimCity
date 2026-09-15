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
 * How an agent chooses a route: one immutable value, decided once by whoever owns the decision.
 *
 * <p>Who that is varies by tier, and the difference is the experiment. A cityImage agent is built to
 * walk one named model over a shared OD matrix; an empirical agent draws one from its survey
 * cluster; a core, activity or night agent has none of its own and asks {@link Heuristics} for one
 * each trip. All three end up here, so the routing code reads one shape.
 *
 * <p><b>The value is separate from {@link AgentProperties} on purpose.</b> Properties are the
 * working copy a single trip is planned against, and a planner may switch parts of it off when the
 * trip cannot use them — region navigation for an origin and destination inside one region, say.
 * Keeping the decision here and the working copy there is what stops "not for this trip" from
 * becoming "not for this agent, ever": {@code RoutePlanner} reapplies the model at the start of
 * every trip, so a disabled element is back by the next one.
 *
 * @param strategy whether the route is a pure minimisation or is built from route-choice elements
 * @param minimisation the cost minimised by a {@code PURE_MINIMISATION} model
 * @param localHeuristic the cost minimised between sub-goals by an {@code ELEMENT_BASED} model
 * @param elements the route-choice elements an {@code ELEMENT_BASED} model uses
 * @param landmarkType which landmarks the agent recognises, or null
 * @param barriers how the agent perceives barriers
 */
public record RouteChoiceModel(
    Strategy strategy,
    MinimisationMode minimisation,
    LocalHeuristicMode localHeuristic,
    Set<RouteChoiceElement> elements,
    LandmarkType landmarkType,
    BarrierPreferences barriers) {

  /**
   * The two kinds of route-choice model, stated rather than inferred.
   *
   * <p>A pure minimisation ignores every element - that is what it means. The strategy is declared
   * rather than inferred from "a minimisation mode is set", so acquiring a mode cannot quietly move
   * a model into that case, and {@code definePath} switches over a closed set.
   */
  public enum Strategy {
    /** One cost minimised end to end: shortest path, or least angular change. */
    PURE_MINIMISATION,
    /** Sub-goals from landmarks, regions or barriers, with legs routed by the local heuristic. */
    ELEMENT_BASED
  }

  /**
   * How an agent perceives barriers: which kinds it notices, and what they do to an edge's cost.
   *
   * @param type the barrier kinds this agent can perceive
   * @param preferenceNatural whether parks and water attract it
   * @param aversionSevering whether roads and railways repel it
   * @param naturalMean cost multiplier for an edge along a natural barrier
   * @param naturalSD spread of that multiplier
   * @param severingMean cost multiplier for an edge along a severing barrier
   * @param severingSD spread of that multiplier
   */
  public record BarrierPreferences(
      AgentBarrierType type,
      boolean preferenceNatural,
      boolean aversionSevering,
      double naturalMean,
      double naturalSD,
      double severingMean,
      double severingSD) {

    /** Barriers noticed by nothing: the agent routes as though the city had none. */
    public static final BarrierPreferences NONE =
        new BarrierPreferences(null, false, false, 0.0, 0.0, 0.0, 0.0);

    /**
     * What a model gets when it asks for barrier navigation without stating its own perception:
     * every barrier kind is a candidate, parks and water attract, severing barriers repel, at fixed
     * factors. The empirical module draws its own from its survey clusters instead.
     */
    public static final BarrierPreferences DEFAULT =
        new BarrierPreferences(AgentBarrierType.ALL, true, true, 0.70, 0.0, 1.30, 0.0);
  }

  public RouteChoiceModel {
    elements =
        elements == null || elements.isEmpty()
            ? Collections.emptySet()
            : Collections.unmodifiableSet(EnumSet.copyOf(elements));
    if (barriers == null) {
      barriers = BarrierPreferences.NONE;
    }
    if (strategy == Strategy.PURE_MINIMISATION && minimisation == MinimisationMode.NONE) {
      throw new IllegalArgumentException("a pure-minimisation model needs a minimisation mode");
    }
    if (strategy == Strategy.ELEMENT_BASED
        && localHeuristic == LocalHeuristicMode.NONE
        && elements.isEmpty()) {
      throw new IllegalArgumentException(
          "an element-based model needs a local heuristic or at least one element");
    }
  }

  /** Minimises one cost end to end. */
  public static RouteChoiceModel minimising(MinimisationMode mode) {
    return new RouteChoiceModel(
        Strategy.PURE_MINIMISATION,
        mode,
        LocalHeuristicMode.NONE,
        Collections.emptySet(),
        null,
        BarrierPreferences.NONE);
  }

  /** Routes between sub-goals drawn from {@code elements}, each leg by {@code localHeuristic}. */
  public static RouteChoiceModel usingElements(
      LocalHeuristicMode localHeuristic,
      Set<RouteChoiceElement> elements,
      LandmarkType landmarkType,
      BarrierPreferences barriers) {
    return new RouteChoiceModel(
        Strategy.ELEMENT_BASED,
        MinimisationMode.NONE,
        localHeuristic,
        elements,
        landmarkType,
        barriers);
  }

  /** Minimises distance end to end: the shortest path. */
  public static RouteChoiceModel minimisingDistance() {
    return minimising(MinimisationMode.DISTANCE);
  }

  /** Minimises angular change end to end: the simplest path. */
  public static RouteChoiceModel minimisingAngular() {
    return minimising(MinimisationMode.ANGULAR);
  }

  /** On-route marks from local landmarks, each leg routed by {@code localHeuristic}. */
  public static RouteChoiceModel localLandmarks(LocalHeuristicMode localHeuristic) {
    return landmarks(localHeuristic, true, false);
  }

  /** Landmark-weighted edge costs from distant landmarks, each leg routed by the heuristic. */
  public static RouteChoiceModel distantLandmarks(LocalHeuristicMode localHeuristic) {
    return landmarks(localHeuristic, false, true);
  }

  /** Both landmark terms together. */
  public static RouteChoiceModel localAndDistantLandmarks(LocalHeuristicMode localHeuristic) {
    return landmarks(localHeuristic, true, true);
  }

  /** Distant landmarks with no heuristic under them: landmarkness alone decides the route. */
  public static RouteChoiceModel distantLandmarksAlone() {
    return landmarks(LocalHeuristicMode.NONE, false, true);
  }

  /** Gateway sub-goals between regions, each leg routed by {@code localHeuristic}. */
  public static RouteChoiceModel regions(LocalHeuristicMode localHeuristic) {
    return usingElements(
        localHeuristic,
        EnumSet.of(RouteChoiceElement.REGION_BASED_NAVIGATION),
        null,
        BarrierPreferences.NONE);
  }

  /** Barrier sub-goals, each leg routed by {@code localHeuristic}. */
  public static RouteChoiceModel barriers(LocalHeuristicMode localHeuristic) {
    return usingElements(
        localHeuristic,
        EnumSet.of(RouteChoiceElement.BARRIER_BASED_NAVIGATION),
        null,
        BarrierPreferences.DEFAULT);
  }

  /** Region gateways and barrier sub-goals together. */
  public static RouteChoiceModel regionsAndBarriers(LocalHeuristicMode localHeuristic) {
    return usingElements(
        localHeuristic,
        EnumSet.of(
            RouteChoiceElement.REGION_BASED_NAVIGATION,
            RouteChoiceElement.BARRIER_BASED_NAVIGATION),
        null,
        BarrierPreferences.DEFAULT);
  }

  /** Recognising local landmarks is what gives a model a {@link LandmarkType}. */
  private static RouteChoiceModel landmarks(
      LocalHeuristicMode localHeuristic, boolean local, boolean distant) {
    EnumSet<RouteChoiceElement> elements = EnumSet.noneOf(RouteChoiceElement.class);
    if (local) {
      elements.add(RouteChoiceElement.LOCAL_LANDMARKS);
    }
    if (distant) {
      elements.add(RouteChoiceElement.DISTANT_LANDMARKS);
    }
    return usingElements(
        localHeuristic, elements, local ? LandmarkType.LOCAL : null, BarrierPreferences.NONE);
  }

  public boolean isPureMinimisation() {
    return strategy == Strategy.PURE_MINIMISATION;
  }

  public boolean hasElement(RouteChoiceElement element) {
    return elements.contains(element);
  }

  /** The same model with different barrier perception; everything else is unchanged. */
  public RouteChoiceModel withBarriers(BarrierPreferences newBarriers) {
    return new RouteChoiceModel(
        strategy, minimisation, localHeuristic, elements, landmarkType, newBarriers);
  }
}
