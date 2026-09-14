package pedsim.night.agents;

import ec.util.MersenneTwisterFast;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.engine.Crowdness;
import pedsim.night.parameters.NightPars;
import sim.graph.EdgeGraph;
import sim.graph.NodeGraph;

public class NightBehaviour {

  NightAgent agent;
  NightAgentMovement nightMovement;

  protected boolean increaseSpeedAtNight = false;
  protected boolean avoidParksWater;

  protected MersenneTwisterFast random;

  // P(reroute) at or above the agent's sensitivity threshold, and the fallback used whenever no
  // continuous lux value is available for the current edge. Equal to the value this whole method
  // used unconditionally before this fix (see rerouteOrIncreaseSpeed below), so anywhere the
  // darkness of the edge can't actually be graded, behaviour is unchanged from the original.
  private static final double BASELINE_REROUTE_PROBABILITY = 0.5;

  NightBehaviour(NightAgent agent, NightAgentMovement nightMovement) {
    this.agent = agent;
    this.nightMovement = nightMovement;
    this.random = agent.getRandom();
  }

  /**
   * Handles the case when the current edge is lit.
   *
   * <p>Important: non-vulnerable agents should not be processed by the vulnerable-agent rule-set
   * simply because the edge is lit. They only trigger night behaviour here when the edge is close
   * to parks/water and runtime rerouting is allowed.
   *
   * @param edge the edge to be approached
   */
  protected void whenLit(EdgeGraph edge) {
    if (isParkWaterNonVulnerable(edge)) {
      whenParkWater(edge);
      return;
    }

    if (agent.isVulnerable()) {
      whenLitVulnerable(edge);
    }
  }

  /**
   * Checks if the edge is next to a park/water and the agent is non-vulnerable.
   *
   * @param edge the edge to check
   * @return true if the edge is next to a park/water and the agent is non-vulnerable
   */
  protected boolean isParkWaterNonVulnerable(EdgeGraph edge) {
    return SharedCognitiveMap.isEdgeNextToParkOrWater(edge) && !agent.isVulnerable();
  }

  /**
   * Handles the case when an agent approaches an edge in proximity to parks or water.
   *
   * @param edge the edge to be approached
   */
  protected void whenParkWater(EdgeGraph edge) {
    if (nightMovement.canReroute()) {
      avoidParksWater = true;
      nightMovement.computeAlternativeRoute();
    } else {
      increaseSpeedAtNight = true;
    }
  }

  /**
   * Handles the case when a vulnerable agent is approaching a lit edge.
   *
   * @param edge the lit edge to be approached
   */
  protected void whenLitVulnerable(EdgeGraph edge) {
    // Unknown, not main road, not busy -> recompute.
    if (!agent.getCognitiveMap().isEdgeKnown(edge)
        && !SharedCognitiveMap.isEdgeMainRoad(edge)
        && !Crowdness.isEdgeCrowded(edge)) {
      if (nightMovement.canReroute()) {
        nightMovement.computeAlternativeRoute();
      } else {
        increaseSpeedAtNight = true;
      }
      return;
    }

    // Not main road and not crowded -> reroute or increase speed.
    if (!SharedCognitiveMap.isEdgeMainRoad(edge) && !Crowdness.isEdgeCrowded(edge)) {
      rerouteOrIncreaseSpeed();
    }
  }

  /**
   * Determines what to do when an agent approaches a non-lit edge.
   *
   * @param edge the non-lit edge to be approached
   */
  protected void whenNonLit(EdgeGraph edge) {
    if (agent.isVulnerable()) {
      nonLitVulnerable(edge);
    } else {
      nonLit(edge);
    }
  }

  /**
   * Handles the case when a non-vulnerable agent is approaching a non-lit edge.
   *
   * @param edge the non-lit edge to be approached
   */
  private void nonLit(EdgeGraph edge) {
    if (isParkWaterNonVulnerable(edge)) {
      whenParkWater(edge);
      return;
    }

    // Crowded -> ok.
    if (Crowdness.isEdgeCrowded(edge)) {
      return;
    }

    // Unknown and not main road -> reroute or increase speed. Otherwise (main road or known) proceed.
    if (!agent.getCognitiveMap().isEdgeKnown(edge) && !SharedCognitiveMap.isEdgeMainRoad(edge)) {
      rerouteOrIncreaseSpeed();
    }
  }

  /**
   * Handles the case when a vulnerable agent is approaching a non-lit edge.
   *
   * @param edge the non-lit edge to be approached
   */
  protected void nonLitVulnerable(EdgeGraph edge) {
    // Unknown, not main road, not crowded -> reroute.
    if (!agent.getCognitiveMap().isEdgeKnown(edge)
        && !SharedCognitiveMap.isEdgeMainRoad(edge)
        && !Crowdness.isEdgeCrowded(edge)) {
      if (nightMovement.canReroute()) {
        nightMovement.computeAlternativeRoute();
      } else {
        increaseSpeedAtNight = true;
      }
      return;
    }

    // Unknown but crowded -> ok.
    if (!agent.getCognitiveMap().isEdgeKnown(edge) && Crowdness.isEdgeCrowded(edge)) {
      return;
    }

    // Main road but not crowded -> increase speed.
    if (SharedCognitiveMap.isEdgeMainRoad(edge) && !Crowdness.isEdgeCrowded(edge)) {
      increaseSpeedAtNight = true;
      return;
    }

    // Known, not main road, not crowded -> reroute or increase speed.
    if (agent.getCognitiveMap().isEdgeKnown(edge) && !Crowdness.isEdgeCrowded(edge)) {
      rerouteOrIncreaseSpeed();
    }
  }

  /**
     * Checks the current edge using explicit lighting pass/fail semantics.
     *
     * mean-light passes if measured mean_lux AND min_lux both exist and clear the threshold (or,
     * where min_lux is absent, mean_lux alone — see {@link #meanLightPasses}), or if the binary
     * lit fallback says the edge is lit. Entrance-light passes if directional lux exists and is
     * above threshold, or if the binary lit fallback says the edge is lit. Missing data without
     * a binary-lit fallback fails closed.
     */
    protected void checkLightLevel() {
        final double threshold = agent.lightSensitivityThreshold;

        boolean meanLightPasses = meanLightPasses(threshold);
        boolean entranceLightPasses = entranceLightPasses(threshold);

        if (meanLightPasses && entranceLightPasses) {
            whenLit(nightMovement.currentEdge);
        } else {
            whenNonLit(nightMovement.currentEdge);
        }
    }

    /**
     * Mean-light gate for the current edge: passes only when the edge is lit on average AND has
     * no sampled point below threshold, so an edge bright at both ends and dark in the middle
     * (mean passes, min doesn't) now correctly fails instead of reading as lit &mdash; register
     * finding C1. min_lux and mean_lux are written by the same pipeline aggregation over the same
     * sample points ({@code 03_street_lights.py}), so wherever one is present the other normally
     * is too; where min_lux is absent (a lighting dataset from before that column existed) this
     * falls back to the mean-only check that ran here before this fix, so older datasets keep
     * working exactly as they did.
     *
     * <p>{@code pct_unlit} is the other statistic C1 flagged as computed and unused; left out of
     * this gate deliberately &mdash; it is the percentage of the edge below the pipeline's fixed
     * 5 lux service threshold, not this agent's personal {@code threshold}, so it isn't directly
     * comparable the way min_lux and mean_lux are. A candidate for a separate, explicitly-scoped
     * use, not one folded silently in here.
     */
    private boolean meanLightPasses(double threshold) {
        var meanLuxAttr = nightMovement.currentEdge.attributes.get("mean_lux");
        if (meanLuxAttr == null) {
            // No measured lux: pass only if the binary lit flag says the edge is lit.
            return SharedCognitiveMap.getLitEdges().contains(nightMovement.currentEdge);
        }
        if (meanLuxAttr.getDouble() < threshold) {
            return false;
        }
        var minLuxAttr = nightMovement.currentEdge.attributes.get("min_lux");
        return minLuxAttr == null || minLuxAttr.getDouble() >= threshold;
    }

    private boolean entranceLightPasses(double threshold) {
        Double entranceLux = directionalEntranceLuxOrNull();
        if (entranceLux != null) {
            return entranceLux >= threshold;
        }
        // No directional value: pass only if the binary lit flag says the edge is lit.
        return SharedCognitiveMap.getLitEdges().contains(nightMovement.currentEdge);
    }

  /**
     * Directional entrance illuminance for the current directed edge, or null when no directional
     * value exists for this edge/direction.
     */
    private Double directionalEntranceLuxOrNull() {
        if (nightMovement.currentDirectedEdge == null) {
            return null;
        }

        NodeGraph fromNode = (NodeGraph) nightMovement.currentDirectedEdge.getFromNode();
        NodeGraph toNode = (NodeGraph) nightMovement.currentDirectedEdge.getToNode();
        return pedsim.night.engine.PedSimCityNight.directionalLuxMap.get(
            pedsim.night.engine.PedSimCityNight.luxKey(fromNode.getID(), toNode.getID()));
    }

  /**
   * Best available continuous illuminance reading for the current edge/direction, preferring the
   * directional entrance value (what the agent actually sees approaching) over the edge's mean,
   * or null when neither exists. Deliberately does not fall back to the binary "lit" flag: that
   * flag carries no magnitude, so it cannot inform how much darker than threshold the edge is,
   * only whether it clears the bar at all (already handled by meanLightPasses/entranceLightPasses
   * upstream in checkLightLevel).
   */
  private Double currentLuxOrNull() {
    Double entranceLux = directionalEntranceLuxOrNull();
    if (entranceLux != null) {
      return entranceLux;
    }
    var meanLuxAttr = nightMovement.currentEdge.attributes.get("mean_lux");
    return meanLuxAttr != null ? meanLuxAttr.getDouble() : null;
  }

  /**
   * How far the current edge's illuminance sits below the agent's sensitivity threshold,
   * normalised to [0, 1] (0 = at or above threshold or no lux data available, 1 = 0 lux / pitch
   * black). Used by rerouteOrIncreaseSpeed to let the reroute probability actually respond to how
   * dark the edge is, instead of the fixed, unmotivated 50/50 split this replaces (register
   * finding C5).
   */
  private double darknessDepth() {
    double threshold = agent.lightSensitivityThreshold;
    if (threshold <= 0) {
      return 0.0;
    }
    Double lux = currentLuxOrNull();
    if (lux == null || lux >= threshold) {
      return 0.0;
    }
    return Math.min(1.0, (threshold - lux) / threshold);
  }

  /**
   * Determines whether to reroute the agent or increase its speed.
   *
   * <p>P(reroute) is {@link #BASELINE_REROUTE_PROBABILITY} (0.5, matching the original fixed
   * split exactly) at or above the agent's threshold and wherever no continuous lux reading is
   * available for the edge, and rises linearly toward {@link NightPars#maxRerouteProbabilityInDarkness}
   * as measured illuminance falls toward zero. This only changes behaviour where the register's
   * C5 finding actually applies &mdash; edges with a real, below-threshold lux value; every call
   * site that previously saw the constant 0.5 (e.g. the still-lit branch in whenLitVulnerable, or
   * any edge with no continuous lux data) is unaffected.
   */
  protected void rerouteOrIncreaseSpeed() {
    double rerouteProbability =
        BASELINE_REROUTE_PROBABILITY
            + (NightPars.maxRerouteProbabilityInDarkness - BASELINE_REROUTE_PROBABILITY)
                * darknessDepth();
    if (random.nextDouble() < rerouteProbability && nightMovement.canReroute()) {
      nightMovement.computeAlternativeRoute();
    } else {
      increaseSpeedAtNight = true;
    }
  }
}
