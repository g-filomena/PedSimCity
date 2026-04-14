package pedsim.core.engine;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.logging.Logger;
import java.util.stream.Collectors;
import java.util.stream.IntStream;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.Point;
import pedsim.core.agents.Agent;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.parameters.Pars;
import pedsim.core.parameters.RouteChoicePars;
import pedsim.core.utilities.LoggerUtil;
import sim.graph.NodeGraph;
import sim.graph.NodesLookup;
import sim.util.geo.MasonGeometry;

/**
 * The Populate class is responsible for generating test agents, building the OD matrix, and
 * populating empirical groups for pedestrian simulation.
 */
public class Populate {

  protected PedSimCity state;
  protected static final Logger logger = LoggerUtil.getLogger();
  final GeometryFactory GEOMETRY_FACTORY = new GeometryFactory();
  protected Random random = new Random();

  private double[] cumulativeProbabilities;
  private double totalProbability = 0.0;

  private NodeGraph homeNode;
  private NodeGraph workNode;

  // Counters to test Spatial Jump vs Fallback performance
  public static AtomicInteger spatialJumpSuccessCount = new AtomicInteger(0);
  public static AtomicInteger randomFallbackCount = new AtomicInteger(0);

  /**
   * Populates agents, OD matrix, for the simulation. It creates a set of agents with the learner
   * status and updates their cognitive maps. The agents are then added to the simulation state.
   *
   * @param state The PedSimCity simulation state.
   */
  public void populate(PedSimCity state) {

    this.state = state;

    // Step 1: Create agents in sequence (Fast with spatial index)
    // Create agents with parameter true
    int totalAgents = Pars.numAgents;
    logger.info("Creating " + totalAgents + " Agents. Building Their Cognitive Maps");
    List<Agent> newAgents =
        IntStream.range(0, totalAgents).mapToObj(this::createAgent).collect(Collectors.toList());

    // Step 2: Register agents sequentially (Thread-safe state update)
    for (Agent agent : newAgents) {
      state.agentsList.add(agent);

      // Update agent position to its homeNode before adding to the layer
      if (agent.homeNode != null) {
        agent.currentLocation.geometry =
            new GeometryFactory().createPoint(agent.homeNode.getCoordinate());
      }

      state.agents.addGeometry(agent.getLocation());
      agent.updateAgentLists(false, true);
    }

    logger.info("Agent Routing Stats -> Spatial Jump Successes: " + spatialJumpSuccessCount.get()
        + " | Instant Random Fallbacks: " + randomFallbackCount.get());
    logger.info(state.agentsList.size() + " agents created");
  }

<<<<<<< Updated upstream
  private void prepareVulnerabilityZones() {
    if (PedSimCity.censusZones == null
        || PedSimCity.censusZones.getGeometries().isEmpty()) {
      return;
    }

    logger.info("Mapping nodes to vulnerability zones...");

    zonesList.clear();
    zoneToNodesMap.clear();
    zoneToWorkplaceWeight.clear();
    zoneToNightWeight.clear();
    nodeToZoneMap.clear();
    totalProbability = 0.0;

    // Load zones and cache attributes
    for (MasonGeometry zone : PedSimCity.censusZones.getGeometries()) {
      if (zone == null || zone.getGeometry() == null) {
        continue;
      }

      zonesList.add(zone);
      zoneToNodesMap.put(zone, new ArrayList<>());

      double residencePct = parseDoubleAttribute(zone, "zone_residence_pct");
      double workWeight = parseDoubleAttribute(zone, "workplace_count");
      double nightWeight = parseDoubleAttribute(zone, "night_dest_count");

      totalProbability += residencePct;
      zoneToWorkplaceWeight.put(zone, workWeight);
      zoneToNightWeight.put(zone, nightWeight);
    }

    // Build cumulative probabilities
    cumulativeProbabilities = new double[zonesList.size()];
    double currentSum = 0.0;

    for (int i = 0; i < zonesList.size(); i++) {
      MasonGeometry zone = zonesList.get(i);
      double residencePct = parseDoubleAttribute(zone, "zone_residence_pct");
      currentSum += residencePct;
      cumulativeProbabilities[i] = currentSum;
    }

    // Build spatial index
    zoneIndex = new STRtree();
    for (MasonGeometry zone : zonesList) {
      zoneIndex.insert(zone.getGeometry().getEnvelopeInternal(), zone);
    }
    zoneIndex.build();

    // Map nodes to zones
    GeometryFactory gf = new GeometryFactory();
    List<NodeGraph> allNodes = SharedCognitiveMap.getCommunityPrimalNetwork().getNodes();

    for (NodeGraph node : allNodes) {
      Point pt = gf.createPoint(node.getCoordinate());

      @SuppressWarnings("unchecked")
      List<MasonGeometry> candidates =
          (List<MasonGeometry>) zoneIndex.query(pt.getEnvelopeInternal());

      for (MasonGeometry zone : candidates) {
        if (zone.getGeometry().contains(pt) || zone.getGeometry().distance(pt) < 1e-6) {
          zoneToNodesMap.get(zone).add(node);
          nodeToZoneMap.put(node, zone);
          break; // assign node to first matching zone
        }
      }
    }
  }

  protected double parseDoubleAttribute(MasonGeometry geometry, String attributeName) {
    try {
      Object attr = geometry.getAttribute(attributeName);
      if (attr == null) {
        return 0.0;
      }

      Object value = ((sim.util.geo.AttributeValue) attr).getValue();
      if (value == null) {
        return 0.0;
      }

      return Double.parseDouble(value.toString().replace(",", "."));
    } catch (Exception e) {
      logger.warning("Failed to parse attribute '" + attributeName + "': " + e.getMessage());
      return 0.0;
    }
  }

  /**
   * - [x] Update `Agent.java` (Core) with `hasWorkedToday` and 6-9 hour stay logic. - [x] Ensure
   * `planTrip()` in `Agent.java` targets `workNode` during the day. - [x] Update
   * `pedsim.night.agents.Agent.java` to stay consistent with core changes. - [x] Reset
   * `hasWorkedToday` in `handleReachedHome()`. - [x] Verify the simulation boot and check the logs
   * for agent walking patterns.
   * 
   * @param node The candidate destination node.
   * @param isDark Whether the simulation currently considers it "Night".
   * @return The weight (number of POIs) for that node's zone.
   */
  public static double getPOIWeight(NodeGraph node, boolean isDark) {
    MasonGeometry zone = nodeToZoneMap.get(node);
    if (zone == null)
      return 0.1; // Baseline for nodes outside any defined zone

    Double weight = isDark ? zoneToNightWeight.get(zone) : zoneToWorkplaceWeight.get(zone);
    return (weight != null && weight > 0) ? weight : 0.1; // Return weight or baseline
  }

=======
>>>>>>> Stashed changes
  /**
   * Creates a new agent but does NOT register it with simulation fields (VectorLayer, etc). This is
   * intended to be called in parallel threads.
   *
   * @param agentID The identifier of the agent.
   * @return The created agent.
   */
  protected Agent createAgent(int agentID) {
    Agent agent = new Agent(this.state, false);
    agent.agentID = agentID;
    defineHomeWorkLocations(agent);
    agent.updateAgentLists(false, true);
    return agent;
  }

  /**
   * - [x] Update `Agent.java` (Core) with `hasWorkedToday` and 6-9 hour stay logic. - [x] Ensure
   * `planTrip()` in `Agent.java` targets `workNode` during the day. - [x] Update
   * `pedsim.night.agents.Agent.java` to stay consistent with core changes. - [x] Reset
   * `hasWorkedToday` in `handleReachedHome()`. - [x] Verify the simulation boot and check the logs
   * for agent walking patterns.
   * 
   * @param node The candidate destination node.
   * @param isDark Whether the simulation currently considers it "Night".
   * @return The weight (number of POIs) for that node's zone.
   */
  public static double getPOIWeight(NodeGraph node, boolean isDark) {
    MasonGeometry zone = PedSimCity.nodesCensusZonesMap.get(node);
    if (zone == null)
      return 0.1; // Baseline for nodes outside any defined zone

    Double weight = isDark ? PedSimCity.censusZonesNightWeight.get(zone)
        : PedSimCity.censusZonesWorkplaceWeight.get(zone);
    return (weight != null && weight > 0) ? weight : 0.1; // Return weight or baseline
  }

  protected void defineHomeWorkLocations(Agent agent) {

    boolean useCensusZones = hasUsableCensusZones();
    assignHomeNode(useCensusZones);
    assignWorkNode(useCensusZones);
    agent.setHomeWorkLoctations(homeNode, workNode);
  }

  private void assignHomeNode(boolean useCensusZones) {

    if (useCensusZones) {
      buildResidenceProbabilities();
      MasonGeometry selectedZone = selectHomeZoneByResidenceWeight();
      homeNode = selectRandomNodeFromCensusZone(selectedZone);
    }

    if (homeNode == null)
      homeNode = selectHomeNodeWithDMA();

    if (homeNode == null)
      homeNode = selectRandomNode();
  }

  // Load census zones and build cumulative residence probabilities
  private void buildResidenceProbabilities() {

    cumulativeProbabilities = new double[PedSimCity.censusZonesList.size()];
    double cumulative = 0.0;

    for (MasonGeometry zone : PedSimCity.censusZonesList) {
      double residencePct = zone.getDoubleAttribute("residence_pct");
      totalProbability += residencePct;
      cumulative += residencePct;
      cumulativeProbabilities[PedSimCity.censusZonesList.indexOf(zone)] = cumulative;
    }
  }

  private boolean hasUsableCensusZones() {
    return !PedSimCity.censusZonesList.isEmpty() && cumulativeProbabilities != null
        && cumulativeProbabilities.length == PedSimCity.censusZonesList.size()
        && totalProbability > 0;
  }

  private MasonGeometry selectHomeZoneByResidenceWeight() {
    double r = random.nextDouble() * totalProbability;

    // Default to the last bucket to avoid edge cases caused by floating-point rounding.
    int selectedZoneIndex = cumulativeProbabilities.length - 1;

    for (int i = 0; i < cumulativeProbabilities.length; i++) {
      if (r <= cumulativeProbabilities[i]) {
        selectedZoneIndex = i;
        break;
      }
    }
    return PedSimCity.censusZonesList.get(selectedZoneIndex);
  }

  private NodeGraph selectRandomNodeFromCensusZone(MasonGeometry zone) {

    List<NodeGraph> availableNodes = PedSimCity.censusZonesNodesMap.get(zone);
    if (availableNodes == null || availableNodes.isEmpty())
      return null;
    return availableNodes.get(random.nextInt(availableNodes.size()));
  }

  private NodeGraph selectHomeNodeWithDMA() {
    try {
      return NodesLookup.randomNodeDMA(SharedCognitiveMap.getCommunityPrimalNetwork(), "live");
    } catch (Exception e) {
      return null;
    }
  }

  private void assignWorkNode(boolean useCensusZones) {
    if (homeNode == null)
      return;

    if (useCensusZones)
      workNode = selectWorkNodeFromZones(homeNode);

    if (workNode == null)
      workNode = selectWorkNodeWithDMA(useCensusZones);

    if (workNode == null)
      workNode = selectWorkNodeWithDistanceFallback(homeNode);


    if (workNode == null) {
      workNode = selectRandomNode();
      if (workNode != null) {
        randomFallbackCount.incrementAndGet();
      }
    }
  }

  private NodeGraph selectWorkNodeFromZones(NodeGraph homeNode) {

    Envelope envelope = new Envelope(homeNode.getCoordinate());
    // TODO probably to improve?
    envelope.expandBy(RouteChoicePars.maxTripDistance);

    @SuppressWarnings("unchecked")
    List<MasonGeometry> candidates = PedSimCity.censusZonesSpatialIndex.query(envelope);

    if (candidates == null || candidates.isEmpty()) {
      return null;
    }

    List<MasonGeometry> validCensusZones = new ArrayList<>();
    double totalWeight = 0.0;
    Point homePoint = GEOMETRY_FACTORY.createPoint(homeNode.getCoordinate());

    for (MasonGeometry candidateZone : candidates) {

      // We use centroid distance as a cheap spatial pre-filter before picking a node in the zone.
      double distanceToCentroid = candidateZone.getGeometry().getCentroid().distance(homePoint);

      // The 0.6 relaxation is intentional: Euclidean distance is usually shorter than network
      // distance.
      if (distanceToCentroid >= (RouteChoicePars.minTripDistance * 0.6)
          && distanceToCentroid <= RouteChoicePars.maxTripDistance) {
        validCensusZones.add(candidateZone);
        totalWeight += getZoneWorkplaceWeight(candidateZone);
      }
    }

    if (validCensusZones.isEmpty() || totalWeight <= 0.0)
      return null;

    double r = random.nextDouble() * totalWeight;
    double cumulative = 0.0;

    for (MasonGeometry validCensusZone : validCensusZones) {
      cumulative += getZoneWorkplaceWeight(validCensusZone);

      if (r <= cumulative) {
<<<<<<< Updated upstream
        NodeGraph node = selectRandomNodeFromCensusZone(zone);
        if (node != null) {
          spatialJumpSuccessCount.incrementAndGet();
        }
=======
        NodeGraph node = selectRandomNodeFromCensusZone(validCensusZone);
        spatialJumpSuccessCount.incrementAndGet();
>>>>>>> Stashed changes
        return node;
      }
    }
    return null;
  }

  private double getZoneWorkplaceWeight(MasonGeometry zone) {
    Double weight = PedSimCity.censusZonesWorkplaceWeight.get(zone);
    return weight != null ? weight : 0.0;
  }

  // TODO, include the vulnerability in a overriding function in night.Populate
  private NodeGraph selectWorkNodeWithDMA(boolean useVulnerability) {
    int attempts = 0;

    while (attempts < 20) {
      try {
        NodeGraph node = NodesLookup.randomNodeBetweenDistanceIntervalDMA(
            SharedCognitiveMap.getCommunityPrimalNetwork(), homeNode,
            RouteChoicePars.minTripDistance, RouteChoicePars.maxTripDistance, "work");

        if (node != null) {
          return node;
        }
      } catch (Exception e) {
        return null;
      }

      // In the non-zone workflow, the legacy logic retries with a different home node.
      if (!useVulnerability) {
        NodeGraph replacementHome = selectHomeNodeWithDMA();
        if (replacementHome != null) {
          homeNode = replacementHome;
        }
      }

      attempts++;
    }
    return null;
  }

  private NodeGraph selectWorkNodeWithDistanceFallback(NodeGraph homeNode) {
    try {
      return NodesLookup.randomNodeBetweenDistanceInterval(
          SharedCognitiveMap.getCommunityPrimalNetwork(), homeNode, RouteChoicePars.minTripDistance,
          RouteChoicePars.maxTripDistance);
    } catch (Exception e) {
      return null;
    }
  }

  private NodeGraph selectRandomNode() {
    List<NodeGraph> nodes = SharedCognitiveMap.getCommunityPrimalNetwork().getNodes();
    return nodes.get(random.nextInt(nodes.size()));
  }
}
