package pedsim.activity.engine;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.Point;
import pedsim.activity.agents.ActivityAgent;
import pedsim.core.agents.Agent;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.Populate;
import pedsim.core.parameters.RouteChoicePars;
import sim.graph.NodeGraph;

/**
 * Populate strategy for activity-based modules. Adds census-zone-based residence and workplace
 * selection on top of core {@link Populate}'s DMA / uniform-random fallbacks.
 *
 * <p>Home nodes are drawn from residence-weighted census zones; work nodes are drawn from nearby
 * zones weighted by workplace POI count. When census data is not loaded the behaviour degrades
 * gracefully to the core DMA / random path.
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

  @Override
  protected void assignHomeNode() {
    if (hasUsableCensusZones()) {
      CensusZone zone = sampleResidentialZone();
      homeNode = randomNodeIn(zone);
      // DMA fallback only when census data was attempted but zone lookup returned no node.
      if (homeNode == null) homeNode = selectHomeNodeWithDMA();
    }
    // When no census data is available, distribute uniformly across all network nodes.
    if (homeNode == null) homeNode = selectRandomNode();
  }

  @Override
  protected void assignWorkNode() {
    if (homeNode == null) return;

    boolean useCensusZones = hasUsableCensusZones();

    if (useCensusZones) workNode = selectWorkNodeFromZones(homeNode);

    // In the census workflow the home node stays fixed (true = keep home).
    if (workNode == null) workNode = selectWorkNodeWithDMA(useCensusZones);

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

  private NodeGraph selectWorkNodeFromZones(NodeGraph homeNode) {
    Envelope envelope = new Envelope(homeNode.getCoordinate());
    envelope.expandBy(RouteChoicePars.maxTripDistance);

    @SuppressWarnings("unchecked")
    List<CensusZone> candidates = PedSimCityActivity.censusZonesIndex.query(envelope);
    if (candidates == null || candidates.isEmpty()) return null;

    Point homePoint = GEOMETRY_FACTORY.createPoint(homeNode.getCoordinate());
    List<CensusZone> valid = new ArrayList<>();
    double totalWeight = 0.0;
    
    // Beta = 2.0 is a standard gravity model decay parameter
    final double BETA = 2.0;

    for (CensusZone zone : candidates) {
      if (zone.workplace <= 0.0 || zone.nodes.isEmpty()) continue;

      double distanceToCentroid = zone.geometry.getGeometry().getCentroid().distance(homePoint);
      if (distanceToCentroid >= RouteChoicePars.minTripDistance * 0.6
          && distanceToCentroid <= RouteChoicePars.maxTripDistance) {
        valid.add(zone);
        
        if (pedsim.core.parameters.RouteChoicePars.useGravityModel) {
          // Apply distance decay to the workplace POI count
          double dist = Math.max(10.0, distanceToCentroid);
          double weight = zone.workplace / Math.pow(dist, BETA);
          totalWeight += weight;
        } else {
          // Legacy uniform weight
          totalWeight += zone.workplace;
        }
      }
    }
    if (valid.isEmpty() || totalWeight <= 0.0) return null;

    double r = random.nextDouble() * totalWeight;
    double cumulative = 0.0;
    for (CensusZone zone : valid) {
      double weight;
      if (pedsim.core.parameters.RouteChoicePars.useGravityModel) {
        double distanceToCentroid = zone.geometry.getGeometry().getCentroid().distance(homePoint);
        double dist = Math.max(10.0, distanceToCentroid);
        weight = zone.workplace / Math.pow(dist, BETA);
      } else {
        weight = zone.workplace;
      }
      
      cumulative += weight;
      if (r <= cumulative) {
        spatialJumpSuccessCount.incrementAndGet();
        return randomNodeIn(zone);
      }
    }
    return null;
  }
}
