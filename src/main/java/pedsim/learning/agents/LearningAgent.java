package pedsim.learning.agents;

import pedsim.activity.agents.ActivityAgent;
import pedsim.core.agents.Heuristics;
import pedsim.core.agents.RouteChoiceActivation;
import pedsim.core.agents.RouteChoiceModel;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.engine.PedSimCity;
import pedsim.core.routing.RoutePlanner;
import pedsim.core.utilities.StringEnum.Learner;
import pedsim.learning.cognitivemap.CognitiveMap;
import pedsim.learning.cognitivemap.IncrementalLearning;
import pedsim.learning.habit.RouteHabit;
import sim.engine.SimState;
import sim.graph.EdgeGraph;
import sim.graph.NodeGraph;
import sim.routing.Astar;
import sim.routing.Route;

/**
 * Agent for the incremental-learning model.
 *
 * <p>Extends {@link ActivityAgent}, so a learner has the full activity programme - persona, daily
 * agenda, purpose-typed destinations, the working day and its commute mode - and adds only what is
 * learning-specific:
 *
 * <ul>
 *   <li>the incremental {@link CognitiveMap} (extends core's), dropped into the inherited
 *       {@code cognitiveMap} field polymorphically;
 *   <li>the {@link IncrementalLearning} route-memory engine;
 *   <li>a {@link Learner} status; only {@code LEARNER} agents reinforce memory after each trip.
 * </ul>
 *
 * <p>Home, work and persona are assigned by {@code LearningPopulate} through
 * {@code ActivityPopulate.defineHomeWorkLocations}; the cognitive map reads them from the agent
 * rather than generating its own.
 *
 * <p><b>Route choice is the survey's.</b> A learner is given activation probabilities of its own,
 * drawn once from the route-choice survey's population row ({@link RouteChoiceSurvey}), and every
 * trip samples a model from them: a pure minimisation, or sub-goals from regions, local landmarks,
 * barriers and distant landmarks, each only where the city carries the data. Landmark recognition
 * and the wayfinding-easiness threshold follow what the agent has learned: see {@link
 * CognitiveMap}.
 */
public class LearningAgent extends ActivityAgent {

  private static final long serialVersionUID = 1L;

  /** Incremental route-memory engine; built once per agent. */
  public IncrementalLearning learning;

  /** The street preferences a learner has formed; see {@link RouteHabit}. */
  private final RouteHabit habit = new RouteHabit(this);

  private Learner learner = Learner.NOT_LEARNER;

  private double efficiencySum;
  private int efficiencyTrips;

  /**
   * The agent that starts from what this one starts with, never learns and is never placed in the
   * simulation; it plans every leg this one plans. Null without {@code LearningPars.shadowPairs}.
   */
  private LearningAgent shadow;

  /** The route-choice model this shadow plans with, set for each leg by its learner. */
  private RouteChoiceModel pinnedModel;

  /** Totals over the legs the learner and its shadow both planned: {legs, learner m, shadow m,
   * shortest m, learner straight-line m}. */
  private final double[] pairTotals = new double[5];

  public LearningAgent(PedSimCity state, boolean registerSpatial) {
    super(state, registerSpatial);
    // Replace the core cognitive map with the learning (incremental) one. The learning
    // CognitiveMap extends core's, so it slots into the inherited field polymorphically.
    this.cognitiveMap = new CognitiveMap(this);
    this.learning = new IncrementalLearning(this);
  }

  /** Typed accessor for the learning cognitive map (the inherited getter returns the core type). */
  public CognitiveMap getLearningCognitiveMap() {
    return (CognitiveMap) cognitiveMap;
  }

  /**
   * Runs the core agent step, then applies time-based memory decay for learners. Decay is applied
   * every step because its factor is calibrated per {@code STEP_DURATION}; it is a no-op until the
   * agent has accumulated route memory (the vividness grid is created lazily). The agent's stable
   * {@code spatialAbility} modulates the rate (higher ability → slower decay).
   */
  @Override
  public void step(SimState state) {
    super.step(state);
    if (isLearner()) {
      learning.applyDecay(getLearningCognitiveMap().spatialAbility);
    }
  }

  @Override
  protected void handleReachedDestination() {
    // Learning hook: reinforce route memory before core nulls origin/destination/route.
    Route walked = getRoute();
    if (walked != null
        && originNode != null
        && destinationNode != null
        && originNode.getID() != destinationNode.getID()
        && walked.getLineString().getLength() > 10) {
      double efficiency = RouteHabit.efficiency(walked, originNode, destinationNode);
      if (!Double.isNaN(efficiency)) {
        efficiencySum += efficiency;
        efficiencyTrips++;
      }
      if (isLearner()) {
        learning.updateAgentMemory(walked);
        habit.walked(walked, originNode, destinationNode);
      }
    }
    super.handleReachedDestination();
  }

  /** Gives this agent the shadow that plans beside it. */
  public void setShadow(LearningAgent shadow) {
    this.shadow = shadow;
  }

  public LearningAgent getShadow() {
    return shadow;
  }

  /**
   * Hands what this agent knows to its shadow, once the map has formed: the same known space, the
   * same vividness grid, the same route-choice probabilities.
   */
  public void startShadow() {
    if (shadow == null) {
      return;
    }
    if (routeChoiceActivation == null) {
      routeChoiceActivation = RouteChoiceSurvey.draw(getRandom());
    }
    shadow.routeChoiceActivation = routeChoiceActivation;
    shadow.getLearningCognitiveMap().copyKnowledgeFrom(getLearningCognitiveMap());
    shadow.getLearningCognitiveMap().spatialAbility = getLearningCognitiveMap().spatialAbility;
    shadow.learning.copyFrom(learning);
  }

  @Override
  protected RouteChoiceModel assignedRouteChoice() {
    return pinnedModel;
  }

  /** Plans the route as the planner would, and has the shadow plan the same leg. */
  @Override
  protected void planRoute() {
    super.planRoute();
    Route own = getRoute();
    if (shadow == null
        || !isLearner()
        || own == null
        || own.getLineString() == null
        || originNode == null
        || destinationNode == null
        || originNode.getID() == destinationNode.getID()) {
      return;
    }
    Route other = shadow.planLike(originNode, destinationNode, agentProperties.model());
    Route shortest =
        new Astar()
            .astarRoute(
                originNode, destinationNode, SharedCognitiveMap.getCommunityPrimalNetwork(), null);
    double ownMetres = own.getLineString().getLength();
    if (other == null
        || other.getLineString() == null
        || shortest == null
        || ownMetres <= 10.0
        || shortest.getLength() <= 0.0) {
      return;
    }
    pairTotals[0] += 1.0;
    pairTotals[1] += ownMetres;
    pairTotals[2] += other.getLineString().getLength();
    pairTotals[3] += shortest.getLength();
    pairTotals[4] += originNode.getCoordinate().distance(destinationNode.getCoordinate());
  }

  /** Plans a leg with the model its learner used, without walking it or recording it. */
  private Route planLike(NodeGraph from, NodeGraph to, RouteChoiceModel model) {
    originNode = from;
    destinationNode = to;
    pinnedModel = model;
    initialiseHeuristics(false);
    Route route = new RoutePlanner(from, to, this).definePath();
    originNode = null;
    destinationNode = null;
    return route;
  }

  /**
   * The totals of the legs this learner and its shadow have planned since the last call, as {legs,
   * learner metres, shadow metres, shortest-path metres, straight-line metres}.
   */
  public double[] drainPairTotals() {
    double[] drained = pairTotals.clone();
    java.util.Arrays.fill(pairTotals, 0.0);
    return drained;
  }

  @Override
  public double edgeCostFactor(EdgeGraph edge) {
    return isLearner() ? habit.factor(edge) : 1.0;
  }

  /**
   * The efficiency (straight line over walked length) of the trips walked since the last call, as
   * {sum, trips}.
   */
  public double[] drainEfficiency() {
    double[] drained = {efficiencySum, efficiencyTrips};
    efficiencySum = 0.0;
    efficiencyTrips = 0;
    return drained;
  }

  /** The agent's own route-choice probabilities, drawn on the first trip it plans. */
  private RouteChoiceActivation routeChoiceActivation;

  @Override
  protected void configureHeuristics(Heuristics heuristics) {
    if (routeChoiceActivation == null) {
      routeChoiceActivation = RouteChoiceSurvey.draw(getRandom());
    }
    heuristics.setActivation(routeChoiceActivation);
  }

  public boolean isLearner() {
    return learner == Learner.LEARNER;
  }

  public void setLearner(Learner learner) {
    this.learner = learner;
  }

  public Learner getLearner() {
    return learner;
  }

  /**
   * Samples this agent's route choice, exactly as {@link #planRoute()} does before every trip.
   *
   * <p>Exposed because {@code IncrementalLearning.buildBasicMemory()} plans routes itself rather
   * than through {@code planRoute()}, which is what calls {@code initialiseHeuristics} before an
   * ordinary trip. Unconfigured properties ({@code MinimisationMode.NONE},
   * {@code LocalHeuristicMode.NONE}) would make {@code RoutePlanner.definePath()} route by angular
   * change whatever the agent's route choice.
   */
  public void sampleRouteChoice() {
    initialiseHeuristics(false);
  }

  @Override
  public Enum<?> getAgentScenario() {
    return learner;
  }
}
