package pedsim.activity.engine;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import pedsim.activity.agents.ActivityAgent;
import pedsim.activity.agents.ActivityPurpose;
import pedsim.activity.agents.Persona;
import pedsim.activity.parameters.ActivityPars;
import pedsim.core.agents.Agent;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.Populate;
import pedsim.core.parameters.RouteChoicePars;
import sim.graph.GraphUtils;
import sim.graph.NodeGraph;

/**
 * Populate strategy for activity-based modules. Adds census-zone-based residence selection and
 * OSM-tag-based workplace selection on top of core {@link Populate}'s DMA / uniform-random
 * fallbacks.
 *
 * <p>Home nodes are drawn from residence-weighted census zones (the census is population structure
 * only); work nodes are drawn from the WORK-purpose attraction weights the {@link PoiClassifier}
 * derives from OSM-like use tags, with gravity decay from home. When either dataset is missing the
 * behaviour degrades gracefully to the core DMA / random path.
 */
public class ActivityPopulate extends Populate {

  // Residence-weighted sampling table over the residential zones (residence > 0 and ≥1 node).
  private List<CensusZone> residentialZones;
  private double[] cumulativeResidence;
  private double totalResidence = 0.0;

  @Override
  public void populate(PedSimCity state) {
    this.state = state;
    if (!PedSimCityActivity.censusZones.isEmpty()) {
      buildResidenceProbabilities();
    }
    super.populate(state);
  }

  @Override
  protected Agent createAgent(int agentID) {
    ActivityAgent agent = new ActivityAgent(this.state, false);
    agent.agentID = agentID;
    defineHomeWorkLocations(agent);
    return agent;
  }

  // Zone the current agent's home was drawn from; null when home did not come from the census.
  private CensusZone homeZone;

  /**
   * Assigns home/work, then samples the persona — conditioned on the home zone's age structure
   * when the census provides it — and applies its employment status: personas without a mandatory
   * activity get no work node (no daily commute); students are re-targeted to an education-tagged
   * node when the city provides them. Runs for every activity-based agent, including night agents
   * (their populate goes through this method too).
   */
  @Override
  protected void defineHomeWorkLocations(Agent agent) {
    homeZone = null;
    super.defineHomeWorkLocations(agent);
    if (agent instanceof ActivityAgent activityAgent) {
      activityAgent.setPersona(samplePersona());
      applyPersonaEmployment(activityAgent);
    }
  }

  /** Persona draw: zone-conditioned when enabled and the home zone carries age shares. */
  private Persona samplePersona() {
    if (ActivityPars.useCensusPersonas && homeZone != null) {
      return Persona.sample(random, homeZone.retireeShare, homeZone.studentShare);
    }
    return Persona.sample(random);
  }

  private void applyPersonaEmployment(ActivityAgent agent) {
    Persona persona = agent.getPersona();
    if (persona == null) {
      return;
    }
    if (!persona.hasMandatoryActivity()) {
      // Retirees / flex adults have no daily commute; all their trips are discretionary.
      agent.setHomeWorkLoctations(agent.getHome(), null);
      return;
    }
    if (persona == Persona.STUDENT) {
      NodeGraph studyNode = sampleEducationNode(agent.getHome());
      if (studyNode != null) {
        agent.setHomeWorkLoctations(agent.getHome(), studyNode);
      }
    }
  }

  /**
   * Weighted draw among education-tagged nodes within the trip-distance band of home, or
   * {@code null} when the city has no education tags (the workplace assignment then stands in).
   */
  private NodeGraph sampleEducationNode(NodeGraph homeNode) {
    Map<NodeGraph, Double> education =
        PedSimCityActivity.nodesPurposeWeight.get(ActivityPurpose.EDUCATION);
    if (education == null || education.isEmpty() || homeNode == null) {
      return null;
    }

    List<NodeGraph> candidates = new ArrayList<>();
    List<Double> weights = new ArrayList<>();
    double totalWeight = 0.0;
    for (Map.Entry<NodeGraph, Double> entry : education.entrySet()) {
      double distance = GraphUtils.nodesDistance(homeNode, entry.getKey());
      if (distance >= RouteChoicePars.minTripDistance * 0.5
          && distance <= RouteChoicePars.maxTripDistance) {
        candidates.add(entry.getKey());
        weights.add(entry.getValue());
        totalWeight += entry.getValue();
      }
    }
    if (candidates.isEmpty() || totalWeight <= 0.0) {
      return null;
    }

    double r = random.nextDouble() * totalWeight;
    double cumulative = 0.0;
    for (int i = 0; i < candidates.size(); i++) {
      cumulative += weights.get(i);
      if (r <= cumulative) {
        return candidates.get(i);
      }
    }
    return candidates.get(candidates.size() - 1);
  }

  @Override
  protected void assignHomeNode() {
    if (hasUsableCensusZones()) {
      CensusZone zone = sampleResidentialZone();
      homeNode = randomNodeIn(zone);
      if (homeNode != null) {
        homeZone = zone; // remembered so the persona can be conditioned on the home zone
      } else {
        // DMA fallback only when census data was attempted but zone lookup returned no node.
        homeNode = selectHomeNodeWithDMA();
      }
    }
    // When no census data is available, distribute uniformly across all network nodes.
    if (homeNode == null) homeNode = selectRandomNode();
  }

  @Override
  protected void assignWorkNode() {
    if (homeNode == null) return;

    workNode = selectWorkNodeFromPurposeWeights(homeNode);

    // When the home node came from the census it stays fixed (true = keep home).
    if (workNode == null) workNode = selectWorkNodeWithDMA(hasUsableCensusZones());

    if (workNode == null) workNode = selectWorkNodeWithDistanceFallback(homeNode);

    if (workNode == null) {
      workNode = selectRandomNode();
      if (workNode != null) randomFallbackCount.incrementAndGet();
    }
  }

  /** Builds the cumulative residence-weight table over the residential zones (residence &gt; 0). */
  private void buildResidenceProbabilities() {
    residentialZones = new ArrayList<>();
    for (CensusZone zone : PedSimCityActivity.censusZones) {
      if (zone.residence > 0.0 && !zone.nodes.isEmpty()) {
        residentialZones.add(zone);
      }
    }
    cumulativeResidence = new double[residentialZones.size()];
    double cumulative = 0.0;
    totalResidence = 0.0;
    for (int i = 0; i < residentialZones.size(); i++) {
      cumulative += residentialZones.get(i).residence;
      cumulativeResidence[i] = cumulative;
      totalResidence += residentialZones.get(i).residence;
    }
  }

  private boolean hasUsableCensusZones() {
    return residentialZones != null && !residentialZones.isEmpty() && totalResidence > 0.0;
  }

  private CensusZone sampleResidentialZone() {
    double r = random.nextDouble() * totalResidence;
    int idx = Arrays.binarySearch(cumulativeResidence, r);
    if (idx < 0) idx = -idx - 1; // insertion point = first zone whose cumulative weight exceeds r
    if (idx >= residentialZones.size()) idx = residentialZones.size() - 1;
    return residentialZones.get(idx);
  }

  private NodeGraph randomNodeIn(CensusZone zone) {
    if (zone == null || zone.nodes.isEmpty()) return null;
    return zone.nodes.get(random.nextInt(zone.nodes.size()));
  }

  /**
   * Weighted draw among WORK-tagged nodes (offices, commercial, industrial… from the OSM-tag
   * purpose weights) within the trip-distance band of home, with gravity decay when
   * {@link RouteChoicePars#useGravityModel} is set. Returns {@code null} when the city carries no
   * work tags — the DMA / distance fallbacks then stand in.
   */
  private NodeGraph selectWorkNodeFromPurposeWeights(NodeGraph homeNode) {
    Map<NodeGraph, Double> work =
        PedSimCityActivity.nodesPurposeWeight.get(ActivityPurpose.WORK);
    if (work == null || work.isEmpty()) return null;

    // Beta = 2.0 is a standard gravity model decay parameter
    final double BETA = 2.0;

    List<NodeGraph> candidates = new ArrayList<>();
    List<Double> weights = new ArrayList<>();
    double totalWeight = 0.0;
    for (Map.Entry<NodeGraph, Double> entry : work.entrySet()) {
      double distance = GraphUtils.nodesDistance(homeNode, entry.getKey());
      if (distance < RouteChoicePars.minTripDistance * 0.6
          || distance > RouteChoicePars.maxTripDistance) {
        continue;
      }
      double weight = entry.getValue();
      if (RouteChoicePars.useGravityModel) {
        weight /= Math.pow(Math.max(10.0, distance), BETA);
      }
      candidates.add(entry.getKey());
      weights.add(weight);
      totalWeight += weight;
    }
    if (candidates.isEmpty() || totalWeight <= 0.0) return null;

    double r = random.nextDouble() * totalWeight;
    double cumulative = 0.0;
    for (int i = 0; i < candidates.size(); i++) {
      cumulative += weights.get(i);
      if (r <= cumulative) {
        spatialJumpSuccessCount.incrementAndGet();
        return candidates.get(i);
      }
    }
    return candidates.get(candidates.size() - 1);
  }
}
