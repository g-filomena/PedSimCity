package pedsim.learning.cognitivemap;

import java.util.List;
import org.locationtech.jts.geom.Polygon;
import pedsim.core.engine.Engine;
import pedsim.core.parameters.TimePars;
import pedsim.core.routing.RoutePlanner;
import pedsim.learning.agents.LearningAgent;
import pedsim.learning.parameters.LearningPars;
import pedsim.learning.routeMemorability.RouteMeaningfulness;
import pedsim.learning.routeMemorability.RouteProperties;
import sim.graph.NodeGraph;
import sim.routing.Route;

public class IncrementalLearning {

  private LearningAgent agent;
  private CognitiveMap cognitiveMap;

  VividnessGrid vividnessGrid;

  /** The day the consolidated store was last faded to; negative before the first step. */
  private long consolidatedFadedTo = -1;

  /** The day the cognitive map was last derived from the grid. */
  private long rebuiltOn = Long.MIN_VALUE / 2;

  /** Whether fading has changed the grid since the map was last derived from it. */
  private boolean fadedSinceRebuild = false;

  public IncrementalLearning(LearningAgent agent) {
    this.agent = agent;
    this.cognitiveMap = agent.getLearningCognitiveMap();
  }

  /** Starts from a copy of what {@code other} has accumulated. */
  public void copyFrom(IncrementalLearning other) {
    vividnessGrid = other.vividnessGrid == null ? null : other.vividnessGrid.copy();
    consolidatedFadedTo = other.consolidatedFadedTo;
    rebuiltOn = other.rebuiltOn;
    fadedSinceRebuild = other.fadedSinceRebuild;
  }

  /** The agent's accumulated vividness grid, or {@code null} before any route memory is built. */
  public VividnessGrid getVividnessGrid() {
    return vividnessGrid;
  }

  /**
   * Seeds the agent's route memory with the routes between its cognitive anchors: from the first
   * anchor, home, to each of the others.
   *
   * <p>The anchors are the ones the activity bone was built around ({@code
   * Agent.cognitiveAnchors()}): home and the workplace for someone who has one, home and the
   * places the persona goes for someone who has not. Each route is planned under a freshly sampled
   * route choice and adds its walks to the cells it lets the agent see.
   */
  public void buildBasicMemory() {
    List<NodeGraph> anchors = cognitiveMap.getAnchors();
    if (anchors.size() < 2) {
      return;
    }
    NodeGraph home = anchors.get(0);
    for (int i = 1; i < anchors.size(); i++) {
      NodeGraph anchor = anchors.get(i);
      if (anchor.getID() == home.getID()) {
        continue;
      }
      agent.sampleRouteChoice();
      Route route = new RoutePlanner(home, anchor, agent).definePath();
      if (route == null || route.getLineString() == null) {
        continue;
      }
      new RouteProperties(route).computeRouteProperties();
      accumulateRouteMemory(route);
    }
    // One rebuild for all of them: readjustCognitiveMap() clears agentKnownNodes/agentKnownEdges
    // and derives them afresh from whatever the grid holds, and the grid only accumulates.
    rebuildCognitiveMap();
  }

  public void updateAgentMemory(Route route) {

    RouteProperties routeProperties = RouteProperties.getProperties(route);
    if (routeProperties == null) {
      routeProperties = new RouteProperties(route);
      routeProperties.computeRouteProperties();
    }
    accumulateRouteMemory(route);
    rebuildCognitiveMap();
  }

  /**
   * Adds one route's visibility space to the vividness grid, without touching the cognitive map.
   * The route counts as one walk plus what its meaningfulness adds.
   */
  private void accumulateRouteMemory(Route route) {

    RouteProperties routeProperties = RouteProperties.getProperties(route);
    if (routeProperties == null) {
      return;
    }
    Polygon routeVisibilitySpace = routeProperties.getVisibilitySpace();
    if (routeVisibilitySpace == null || routeVisibilitySpace.isEmpty()) {
      return;
    }
    if (vividnessGrid == null)
      vividnessGrid =
          new VividnessGrid(routeVisibilitySpace.getEnvelopeInternal(), LearningPars.cellSize);

    double exposure = routeProperties.landmarkExposure();
    vividnessGrid.setDay(currentDay());
    vividnessGrid.stamp(
        routeVisibilitySpace, novelty -> RouteMeaningfulness.walks(novelty, exposure));
  }

  /** Re-derives the agent's known network from the vividness grid as it currently stands. */
  private void rebuildCognitiveMap() {
    if (vividnessGrid == null) {
      return;
    }
    cognitiveMap.readjustCognitiveMap(vividnessGrid.updateCollage(LearningPars.memoryThreshold));
    rebuiltOn = currentDay();
    fadedSinceRebuild = false;
  }

  /**
   * Fades the vividness grid by one simulation step, by the agent's own rate.
   *
   * <p>The grid fades as one scale factor, in constant time. The known space is the cells holding at
   * least {@code LearningPars.memoryThreshold} walks, so the cognitive map is rebuilt when the
   * fade takes the weakest of them below it, or the consolidated store fades, at most once every
   * {@code LearningPars.rebuildEveryDays}.
   *
   * @param memoryAbility value between 0 and 1 (higher = better memory, slower decay)
   */
  public void applyDecay(double memoryAbility) {
    if (vividnessGrid == null) return;
    vividnessGrid.decay(computeDecayFactor(memoryAbility));
    fadedSinceRebuild |= vividnessGrid.collageHasFaded();

    long today = currentDay();
    if (consolidatedFadedTo < 0) {
      consolidatedFadedTo = today;
    } else if (today > consolidatedFadedTo) {
      double factor = consolidatedDecayFactor(memoryAbility, today - consolidatedFadedTo);
      consolidatedFadedTo = today;
      if (factor < 1.0) {
        vividnessGrid.decayConsolidated(factor);
        fadedSinceRebuild = true;
      }
    }
    if (fadedSinceRebuild && today - rebuiltOn >= LearningPars.rebuildEveryDays) {
      rebuildCognitiveMap();
    }
  }

  private long currentDay() {
    return Engine.getDays(agent.getState().schedule.getSteps());
  }

  /**
   * The factor the consolidated store fades by over some days: the labile rate's ability scaling
   * with {@code LearningPars.consolidatedHalfLife} as the half-life. 1 when that is 0.
   */
  public static double consolidatedDecayFactor(double memoryAbility, long days) {
    if (LearningPars.consolidatedHalfLife <= 0.0) {
      return 1.0;
    }
    double rate =
        Math.log(2.0) / LearningPars.consolidatedHalfLife * Math.pow(2.0, 1.0 - memoryAbility);
    return Math.exp(-rate * days);
  }

  /**
   * Compute the per-step decay factor given the base decay rate and individual memory ability.
   *
   * <p>Memory ability is a value between 0 and 1: 1.0 decays at the base rate, 0.0 at twice it.
   *
   * @param memoryAbility between 0 and 1
   * @return per-step decay factor (multiplier per tick)
   */
  public static double computeDecayFactor(double memoryAbility) {
    double lambda = computeBaseDecayRate();
    double adjustedLambda = lambda * Math.pow(2.0, 1.0 - memoryAbility);
    return Math.exp(-adjustedLambda * TimePars.STEP_DURATION);
  }

  /**
   * Compute the base decay rate (λ) from the half-life, in per-second units: λ = ln(2) /
   * halfLifeSeconds.
   *
   * @return base decay rate (λ) in per-second units
   */
  public static double computeBaseDecayRate() {
    double halfLifeSeconds = LearningPars.halfLife * 60 * 60 * 24;
    return Math.log(2.0) / halfLifeSeconds;
  }
}
