package pedsim.learning.agents;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import pedsim.activity.agents.ActivityAgent;
import pedsim.activity.agents.ActivityPurpose;
import pedsim.core.agents.Heuristics;
import pedsim.core.agents.RouteChoiceActivation;
import pedsim.core.agents.RouteChoiceModel;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.engine.PedSimCity;
import pedsim.core.parameters.RouteChoicePars;
import pedsim.core.routing.RoutePlanner;
import pedsim.core.utilities.StringEnum.Learner;
import pedsim.learning.cognitivemap.CognitiveMap;
import pedsim.learning.cognitivemap.IncrementalLearning;
import pedsim.learning.cognitivemap.MemoryProfile;
import pedsim.learning.habit.RouteHabit;
import pedsim.learning.parameters.LearningPars;
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

  /** Whether this agent is the shadow of a learner. */
  private boolean isShadow;

  /** The route-choice model this shadow plans with, set for each leg by its learner. */
  private RouteChoiceModel pinnedModel;

  /**
   * Totals over the legs the learner and its shadow both planned: {legs, learner m, shadow m,
   * shortest m, learner straight-line m, learner main-road m, shadow main-road m, learner
   * familiarity-weighted m, learner m on unfamiliar streets, shadow familiarity-weighted m, shadow m
   * on unfamiliar streets, learner legs with an unfamiliar street, shadow legs with one}.
   */
  private final double[] pairTotals = new double[13];

  public LearningAgent(PedSimCity state, boolean registerSpatial) {
    super(state, registerSpatial);
    // Replace the core cognitive map with the learning (incremental) one. The learning
    // CognitiveMap extends core's, so it slots into the inherited field polymorphically.
    this.cognitiveMap = new CognitiveMap(this);
    this.learning = new IncrementalLearning(this);
  }

  /** How this agent builds and keeps what it learns. */
  public MemoryProfile memoryProfile() {
    return learning.profile();
  }

  /** Typed accessor for the learning cognitive map (the inherited getter returns the core type). */
  public CognitiveMap getLearningCognitiveMap() {
    return (CognitiveMap) cognitiveMap;
  }

  /**
   * Runs the core agent step, then applies time-based memory decay for learners. Decay is applied
   * every step because its factor is calibrated per {@code STEP_DURATION}; it is a no-op until the
   * agent has accumulated route memory (the vividness grid is created lazily). The agent's memory
   * profile sets how fast the lasting part fades.
   */
  @Override
  public void step(SimState state) {
    super.step(state);
    if (isLearner() && LearningPars.learningEnabled) {
      learning.applyDecay();
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
      if (isLearner() && LearningPars.learningEnabled) {
        learning.updateAgentMemory(walked);
        habit.walked(walked, originNode, destinationNode);
      }
    }
    super.handleReachedDestination();
  }

  /** Gives this agent the shadow that plans beside it. */
  public void setShadow(LearningAgent shadow) {
    this.shadow = shadow;
    shadow.isShadow = true;
    shadow.errorOwner = agentID;
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
    planningLeg++;
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
    Route other =
        shadow.planLike(originNode, destinationNode, agentProperties.model(), planningLeg);
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
    pairTotals[5] += mainRoadMetres(own);
    pairTotals[6] += mainRoadMetres(other);
    double[] ownFamiliarity = familiarityOf(own, this);
    double[] otherFamiliarity = familiarityOf(other, shadow);
    pairTotals[7] += ownFamiliarity[0];
    pairTotals[8] += ownFamiliarity[1];
    pairTotals[9] += otherFamiliarity[0];
    pairTotals[10] += otherFamiliarity[1];
    pairTotals[11] += ownFamiliarity[1] > 0.0 ? 1.0 : 0.0;
    pairTotals[12] += otherFamiliarity[1] > 0.0 ? 1.0 : 0.0;
  }

  /** The streets less familiar than this count as unfamiliar. */
  private static final double UNFAMILIAR_BELOW = 0.5;

  /**
   * The route's length weighted by the planner's familiarity with each street, and the length of
   * it on streets below {@link #UNFAMILIAR_BELOW}.
   */
  private static double[] familiarityOf(Route route, LearningAgent planner) {
    double weighted = 0.0;
    double unfamiliar = 0.0;
    if (route.edgesSequence != null) {
      CognitiveMap map = planner.getLearningCognitiveMap();
      for (EdgeGraph edge : route.edgesSequence) {
        double metres = edge.getLine().getLength();
        double familiarity = map.familiarity(edge);
        weighted += familiarity * metres;
        if (familiarity < UNFAMILIAR_BELOW) {
          unfamiliar += metres;
        }
      }
    }
    return new double[] {weighted, unfamiliar};
  }

  /** The length of the route that runs along primary and secondary roads. */
  private static double mainRoadMetres(Route route) {
    double metres = 0.0;
    if (route.edgesSequence != null) {
      for (EdgeGraph edge : route.edgesSequence) {
        if (MainRoads.contains(edge)) {
          metres += edge.getLine().getLength();
        }
      }
    }
    return metres;
  }

  /** Plans a leg with the model its learner used, without walking it or recording it. */
  private Route planLike(NodeGraph from, NodeGraph to, RouteChoiceModel model, long leg) {
    originNode = from;
    destinationNode = to;
    pinnedModel = model;
    planningLeg = leg;
    initialiseHeuristics(false);
    Route route = new RoutePlanner(from, to, this).definePath();
    originNode = null;
    destinationNode = null;
    return route;
  }

  /**
   * The totals of the legs this learner and its shadow have planned since the last call, as {legs,
   * learner metres, shadow metres, shortest-path metres, straight-line metres, learner main-road
   * metres, shadow main-road metres, and the unknown-street totals listed on {@code pairTotals}}.
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

  /** Whether this learner or shadow plans over the whole network, not only what it knows. */
  @Override
  public boolean plansBeyondKnownNetwork() {
    return LearningPars.familiarityPlanning && (isLearner() || isShadow);
  }

  /** The planning leg this agent, or the learner whose leg it shadows, is on. */
  private long planningLeg;

  /** The agent whose number the errors of this agent's estimates are drawn from. */
  private int errorOwner = -1;

  private long perceivedFor = -1;
  private final java.util.Map<Integer, Double> perceived = new java.util.HashMap<>();

  /**
   * With {@code LearningPars.familiarityPlanning}, the cost factor of a street for the whole leg
   * being planned: {@code (1 + uncertaintyPremium * (1 - familiarity))} times an error with a
   * standard deviation of {@code perceptionErrorSD * (1 + (unfamiliarErrorMultiple - 1) * (1 -
   * familiarity))}. The error is drawn from the learner's number, the leg and the street, so the
   * learner and its shadow, and every search of one leg, see the same error for the same street.
   */
  @Override
  public double perceivedCostFactor(EdgeGraph edge, double drawn) {
    if (!LearningPars.familiarityPlanning || (!isLearner() && !isShadow)) {
      return drawn;
    }
    if (perceivedFor != planningLeg) {
      perceived.clear();
      perceivedFor = planningLeg;
    }
    return perceived.computeIfAbsent(edge.getID(), id -> perceivedFactor(edge));
  }

  private double perceivedFactor(EdgeGraph edge) {
    int owner = errorOwner >= 0 ? errorOwner : agentID;
    double unfamiliarity = 1.0 - getLearningCognitiveMap().familiarity(edge);
    double sd =
        RouteChoicePars.perceptionErrorSD
            * (1.0 + (LearningPars.unfamiliarErrorMultiple - 1.0) * unfamiliarity);
    double error = Math.max(0.05, 1.0 + sd * standardNormal(owner, planningLeg, edge.getID()));
    return (1.0 + LearningPars.uncertaintyPremium * unfamiliarity) * error;
  }

  /** A standard normal number determined by the three keys. */
  private static double standardNormal(long owner, long leg, long street) {
    long h = mix(mix(mix(0x9E3779B97F4A7C15L ^ owner) ^ leg) ^ street);
    double u1 = ((h >>> 11) + 0.5) / (double) (1L << 53);
    double u2 = ((mix(h) >>> 11) + 0.5) / (double) (1L << 53);
    return Math.sqrt(-2.0 * Math.log(u1)) * Math.cos(2.0 * Math.PI * u2);
  }

  private static long mix(long x) {
    x = (x ^ (x >>> 30)) * 0xBF58476D1CE4E5B9L;
    x = (x ^ (x >>> 27)) * 0x94D049BB133111EBL;
    return x ^ (x >>> 31);
  }

  /**
   * With {@code LearningPars.awareDestinations}, the places the learner knows of among those
   * within reach, and with probability {@code discoveryProbability} one more it has not seen. With
   * none known, the places within reach.
   */
  @Override
  protected List<NodeGraph> destinationCandidates(
      List<NodeGraph> candidates, Map<NodeGraph, Double> attraction) {
    if (!LearningPars.awareDestinations || !isLearner() || !getCognitiveMap().individualised) {
      return candidates;
    }
    CognitiveMap map = getLearningCognitiveMap();
    List<NodeGraph> aware = new ArrayList<>();
    List<NodeGraph> unseen = new ArrayList<>();
    for (NodeGraph node : candidates) {
      if (map.isAwareOf(node)) {
        aware.add(node);
      } else if (attraction == null || attraction.getOrDefault(node, 0.0) > 0.0) {
        unseen.add(node);
      }
    }
    if (!unseen.isEmpty() && getRandom().nextDouble() < LearningPars.discoveryProbability) {
      aware.add(unseen.get(getRandom().nextInt(unseen.size())));
    }
    return aware.isEmpty() ? candidates : aware;
  }

  private final Map<ActivityPurpose, Set<NodeGraph>> destinationsChosen =
      new EnumMap<>(ActivityPurpose.class);

  @Override
  protected void chooseDestination() {
    super.chooseDestination();
    if (isLearner() && destinationNode != null && currentPurpose != null) {
      destinationsChosen.computeIfAbsent(currentPurpose, p -> new HashSet<>()).add(destinationNode);
    }
  }

  /** How many different places the agent has chosen for a purpose so far. */
  public int destinationsChosen(ActivityPurpose purpose) {
    Set<NodeGraph> chosen = destinationsChosen.get(purpose);
    return chosen == null ? 0 : chosen.size();
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
