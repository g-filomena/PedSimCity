package pedsim.core.engine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.logging.Logger;
import java.util.stream.Collectors;
import java.util.stream.IntStream;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.Point;
import org.locationtech.jts.index.strtree.STRtree;
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

  private List<MasonGeometry> zonesList = new ArrayList<>();
  private double[] cumulativeProbabilities;
  private Map<MasonGeometry, List<NodeGraph>> zoneToNodesMap = new HashMap<>();
  private double totalProbability = 0.0;

  // Caches for POI-based destination selection
  private static Map<MasonGeometry, Double> zoneToWorkplaceWeight = new HashMap<>();
  private static Map<MasonGeometry, Double> zoneToNightWeight = new HashMap<>();
  private static Map<NodeGraph, MasonGeometry> nodeToZoneMap = new HashMap<>();
  private static STRtree zoneIndex;

  // Counters to test Spatial Jump vs Fallback performance
  public static java.util.concurrent.atomic.AtomicInteger spatialJumpSuccessCount =
      new java.util.concurrent.atomic.AtomicInteger(0);
  public static java.util.concurrent.atomic.AtomicInteger randomFallbackCount =
      new java.util.concurrent.atomic.AtomicInteger(0);

  protected Random random = new Random();

  /**
   * Populates agents, OD matrix, for the simulation. It creates a set of agents with the learner
   * status and updates their cognitive maps. The agents are then added to the simulation state.
   *
   * @param state The PedSimCity simulation state.
   */
  public void populate(PedSimCity state) {

    this.state = state;

    prepareVulnerabilityZones();

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

  private void prepareVulnerabilityZones() {
    if (PedSimCity.vulnerabilityZones == null
        || PedSimCity.vulnerabilityZones.getGeometries().isEmpty()) {
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
    for (MasonGeometry zone : PedSimCity.vulnerabilityZones.getGeometries()) {
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

  private double parseDoubleAttribute(MasonGeometry geometry, String attributeName) {
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

  private static final GeometryFactory GEOMETRY_FACTORY = new GeometryFactory();

  protected void defineHomeWorkLocations(Agent agent) {
    LocationSelection selection = new LocationSelection();
    boolean useCensusZones = hasUsableCensusZones();

    if (useCensusZones) {
      MasonGeometry selectedZone = selectHomeZoneByResidenceWeight();
      selection.homeNode = selectRandomNodeFromCensusZone(selectedZone);
    }

    if (selection.homeNode == null)
      selection.homeNode = selectHomeNodeWithDMA();

    if (selection.homeNode == null)
      selection.homeNode = selectRandomNetworkNode();

    assignWorkNode(selection, useCensusZones);
    agent.setHomeWorkLoctations(selection.homeNode, selection.workNode);
  }

  private boolean hasUsableCensusZones() {
    return zonesList != null && !zonesList.isEmpty() && cumulativeProbabilities != null
        && cumulativeProbabilities.length == zonesList.size() && totalProbability > 0;
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
    return zonesList.get(selectedZoneIndex);
  }

  private NodeGraph selectRandomNodeFromCensusZone(MasonGeometry zone) {
    if (zone == null) {
      return null;
    }

    List<NodeGraph> availableNodes = zoneToNodesMap.get(zone);
    if (availableNodes == null || availableNodes.isEmpty()) {
      return null;
    }

    return availableNodes.get(random.nextInt(availableNodes.size()));
  }

  private NodeGraph selectHomeNodeWithDMA() {
    try {
      return NodesLookup.randomNodeDMA(SharedCognitiveMap.getCommunityPrimalNetwork(), "live");
    } catch (Exception e) {
      return null;
    }
  }

  private NodeGraph selectRandomNetworkNode() {
    List<NodeGraph> allNodes = SharedCognitiveMap.getCommunityPrimalNetwork().getNodes();
    if (allNodes == null || allNodes.isEmpty()) {
      return null;
    }

    return allNodes.get(random.nextInt(allNodes.size()));
  }

  private void assignWorkNode(LocationSelection selection, boolean useCensusZones) {
    if (selection == null || selection.homeNode == null) {
      return;
    }

    if (useCensusZones && zoneIndex != null) {
      selection.workNode = selectWorkNodeFromZones(selection.homeNode);
    }

    if (selection.workNode == null) {
      selection.workNode = selectWorkNodeWithDMA(selection, useCensusZones);
    }

    if (selection.workNode == null) {
      selection.workNode = selectWorkNodeWithDistanceFallback(selection.homeNode);
    }

    if (selection.workNode == null) {
      selection.workNode = selectRandomNetworkNode();
      if (selection.workNode != null) {
        randomFallbackCount.incrementAndGet();
      }
    }
  }

  private NodeGraph selectWorkNodeFromZones(NodeGraph homeNode) {
    Envelope env = new Envelope(homeNode.getCoordinate());
    env.expandBy(RouteChoicePars.maxTripDistance);

    @SuppressWarnings("unchecked")
    List<MasonGeometry> candidates = (List<MasonGeometry>) zoneIndex.query(env);

    if (candidates == null || candidates.isEmpty()) {
      return null;
    }

    List<MasonGeometry> validZones = new ArrayList<>();
    double totalWeight = 0.0;
    Point homePoint = GEOMETRY_FACTORY.createPoint(homeNode.getCoordinate());

    for (MasonGeometry zone : candidates) {
      if (zone == null || zone.getGeometry() == null) {
        continue;
      }

      // We use centroid distance as a cheap spatial pre-filter before picking a node in the zone.
      double distanceToCentroid = zone.getGeometry().getCentroid().distance(homePoint);

      // The 0.6 relaxation is intentional: Euclidean distance is usually shorter than network
      // distance.
      if (distanceToCentroid >= (RouteChoicePars.minTripDistance * 0.6)
          && distanceToCentroid <= RouteChoicePars.maxTripDistance) {
        validZones.add(zone);
        totalWeight += getZoneWorkplaceWeight(zone);
      }
    }

    if (validZones.isEmpty() || totalWeight <= 0.0) {
      return null;
    }

    double r = random.nextDouble() * totalWeight;
    double cumulative = 0.0;

    for (MasonGeometry zone : validZones) {
      cumulative += getZoneWorkplaceWeight(zone);

      if (r <= cumulative) {
        NodeGraph node = selectRandomNodeFromZone(zone);
        if (node != null) {
          spatialJumpSuccessCount.incrementAndGet();
        }
        return node;
      }
    }

    return null;
  }

  private double getZoneWorkplaceWeight(MasonGeometry zone) {
    Double weight = zoneToWorkplaceWeight.get(zone);
    return weight != null ? weight : 0.0;
  }

  private NodeGraph selectWorkNodeWithDMA(LocationSelection selection,
      boolean useVulnerabilityZones) {
    int attempts = 0;

    while (attempts < 20) {
      try {
        NodeGraph node = NodesLookup.randomNodeBetweenDistanceIntervalDMA(
            SharedCognitiveMap.getCommunityPrimalNetwork(), selection.homeNode,
            RouteChoicePars.minTripDistance, RouteChoicePars.maxTripDistance, "work");

        if (node != null) {
          return node;
        }
      } catch (Exception e) {
        return null;
      }

      // In the non-zone workflow, the legacy logic retries with a different home node.
      if (!useVulnerabilityZones) {
        NodeGraph replacementHome = selectHomeNodeWithDMA();
        if (replacementHome != null) {
          selection.homeNode = replacementHome;
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

  private static final class LocationSelection {
    private NodeGraph homeNode;
    private NodeGraph workNode;
  }
}
