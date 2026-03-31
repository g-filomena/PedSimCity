package pedsim.core.engine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.logging.Logger;
import java.util.stream.Collectors;
import java.util.stream.IntStream;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.Point;
import org.locationtech.jts.geom.Envelope;
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
  public static java.util.concurrent.atomic.AtomicInteger spatialJumpSuccessCount = new java.util.concurrent.atomic.AtomicInteger(0);
  public static java.util.concurrent.atomic.AtomicInteger randomFallbackCount = new java.util.concurrent.atomic.AtomicInteger(0);

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
    int totalAgents = Pars.numAgents;
    logger.info("Creating " + totalAgents + " Agents. Building Their Cognitive Maps");
    List<Agent> newAgents = IntStream.range(0, totalAgents)
      .mapToObj(this::createAgent)
      .collect(Collectors.toList());

    // Step 2: Register agents sequentially (Thread-safe state update)
    for (Agent agent : newAgents) {
      state.agentsList.add(agent);
      
      // Update agent position to its homeNode before adding to the layer
      if (agent.homeNode != null) {
          agent.currentLocation.geometry = new GeometryFactory().createPoint(agent.homeNode.getCoordinate());
      }
      
      state.agents.addGeometry(agent.getLocation());
      agent.updateAgentLists(false, true);
    }
    
    logger.info("Agent Routing Stats -> Spatial Jump Successes: " + spatialJumpSuccessCount.get() + 
                " | Instant Random Fallbacks: " + randomFallbackCount.get());
    logger.info(state.agentsList.size() + " agents created");
  }

  private void prepareVulnerabilityZones() {
    if (PedSimCity.vulnerabilityZones == null || PedSimCity.vulnerabilityZones.getGeometries().isEmpty()) {
      return;
    }

    logger.info("Mapping nodes to vulnerability zones...");
    for (Object obj : PedSimCity.vulnerabilityZones.getGeometries()) {
      MasonGeometry zone = (MasonGeometry) obj;
      if (zone.getGeometry() == null) continue;

      zonesList.add(zone);
      zoneToNodesMap.put(zone, new ArrayList<>());
      
      // Parse residence percentage for spawning
      Object attr = zone.getAttribute("zone_residence_pct");
      double pct = 0.0;
      if (attr != null) {
          try {
              Object val = ((sim.util.geo.AttributeValue) attr).getValue();
              if (val != null) {
                  pct = Double.parseDouble(val.toString().replace(",", "."));
              }
          } catch (Exception e) {
              logger.warning("Failed to parse zone_residence_pct: " + e.getMessage());
          }
      }
      totalProbability += pct;

      // Parse and cache POI weights for destination selection
      try {
          Object workAttr = zone.getAttribute("workplace_count");
          double workWeight = (workAttr != null) ? 
              Double.parseDouble(((sim.util.geo.AttributeValue) workAttr).getValue().toString()) : 0.0;
          zoneToWorkplaceWeight.put(zone, workWeight);

          Object nightAttr = zone.getAttribute("night_dest_count");
          double nightWeight = (nightAttr != null) ? 
              Double.parseDouble(((sim.util.geo.AttributeValue) nightAttr).getValue().toString()) : 0.0;
          zoneToNightWeight.put(zone, nightWeight);
      } catch (Exception e) {
          logger.warning("Failed to parse POI counts for zone: " + e.getMessage());
      }
    }

    cumulativeProbabilities = new double[zonesList.size()];
    double currentSum = 0.0;
    for (int i = 0; i < zonesList.size(); i++) {
        Object attr = zonesList.get(i).getAttribute("zone_residence_pct");
        double pct = 0.0;
        if (attr != null) {
            try {
                Object val = ((sim.util.geo.AttributeValue) attr).getValue();
                if (val != null) {
                    pct = Double.parseDouble(val.toString().replace(",", "."));
                }
            } catch (Exception e) {
                logger.warning("Failed to parse cumulative pct: " + e.getMessage());
            }
        }
        currentSum += pct;
        cumulativeProbabilities[i] = currentSum;
    }

    // Build a spatial index for the zones
    zoneIndex = new STRtree();
    for (MasonGeometry zone : zonesList) {
        zoneIndex.insert(zone.getGeometry().getEnvelopeInternal(), zone);
    }
    zoneIndex.build();

    // Map all nodes to zones using the spatial index
    GeometryFactory gf = new GeometryFactory();
    List<NodeGraph> allNodes = SharedCognitiveMap.getCommunityPrimalNetwork().getNodes();
    nodeToZoneMap.clear();
    
    for (NodeGraph node : allNodes) {
      Point pt = gf.createPoint(node.getCoordinate());
      
      // Query the index for candidate zones (ones whose bounding box contains the point)
      @SuppressWarnings("unchecked")
      List<MasonGeometry> candidates = (List<MasonGeometry>) zoneIndex.query(pt.getEnvelopeInternal());
      
      for (MasonGeometry zone : candidates) {
        if (zone.getGeometry().contains(pt) || zone.getGeometry().distance(pt) < 1e-6) {
          zoneToNodesMap.get(zone).add(node);
          nodeToZoneMap.put(node, zone);
          break; // Assign node to first matching zone
        }
      }
    }
  }

  /**
   * Retrieves the POI weight for a given node based on the time of day.
   * @param node The candidate destination node.
   * @param isDark Whether the simulation currently considers it "Night".
   * @return The weight (number of POIs) for that node's zone.
   */
  public static double getPOIWeight(NodeGraph node, boolean isDark) {
      MasonGeometry zone = nodeToZoneMap.get(node);
      if (zone == null) return 0.1; // Baseline for nodes outside any defined zone
      
      Double weight = isDark ? zoneToNightWeight.get(zone) : zoneToWorkplaceWeight.get(zone);
      return (weight != null && weight > 0) ? weight : 0.1; // Return weight or baseline
  }

  /**
   * Creates a new agent but does NOT register it with simulation fields (VectorLayer, etc).
   * This is intended to be called in parallel threads.
   *
   * @param agentID The identifier of the agent.
   * @return The created agent.
   */
  protected Agent createAgent(int agentID) {
    Agent agent = new Agent(this.state, false);
    agent.agentID = agentID;
    defineHomeWorkLocations(agent);
    return agent;
  }

  protected void defineHomeWorkLocations(Agent agent) {

    // in the community network
    NodeGraph homeNode = null;
    NodeGraph workNode = null;
    java.util.Random rnd = new java.util.Random();
    
    boolean useVulnerabilityZones = (zonesList != null && !zonesList.isEmpty() && totalProbability > 0);

    if (useVulnerabilityZones) {
      double r = rnd.nextDouble() * totalProbability;
      int selectedZoneIndex = 0;
      for (int i = 0; i < cumulativeProbabilities.length; i++) {
        if (r <= cumulativeProbabilities[i]) {
          selectedZoneIndex = i;
          break;
        }
      }
      
      MasonGeometry selectedZone = zonesList.get(selectedZoneIndex);
      List<NodeGraph> availableNodes = zoneToNodesMap.get(selectedZone);
      
      if (availableNodes != null && !availableNodes.isEmpty()) {
        homeNode = availableNodes.get(rnd.nextInt(availableNodes.size()));
      }
      
      Object vulnAttr = selectedZone.getAttribute("vulnerability_pct");
      double vulnProb = 0.0;
      if (vulnAttr != null) {
          try {
              Object val = ((sim.util.geo.AttributeValue) vulnAttr).getValue();
              if (val != null) {
                  vulnProb = Double.parseDouble(val.toString().replace(",", "."));
              }
          } catch (Exception e) {
              logger.warning("Failed to parse vuln attribute: " + e.getMessage());
          }
      }
      if (vulnProb > 1.0) vulnProb /= 100.0; // Assume 0-100 scale if value > 1
      
      double rVuln = rnd.nextDouble();
      boolean isVuln = rVuln < vulnProb;
      agent.setVulnerable(isVuln);
    }

    if (homeNode == null) {
      int count = 0;
      while (homeNode == null && count < 100) {
        try {
          homeNode = NodesLookup.randomNodeDMA(SharedCognitiveMap.getCommunityPrimalNetwork(), "live");
        } catch (Exception e) {
          homeNode = null;
          break;
        }
        count++;
      }
    }
    
    // Fallback if no DMA found for homeNode
    if (homeNode == null) {
      java.util.List<NodeGraph> allNodes = SharedCognitiveMap.getCommunityPrimalNetwork().getNodes();
      if (!allNodes.isEmpty()) {
          homeNode = allNodes.get(rnd.nextInt(allNodes.size()));
      }
    }

    int count = 0;
    while (workNode == null && count < 20) {
      if (useVulnerabilityZones) {
          // Spatial Jump Optimization
          Envelope env = new Envelope(homeNode.getCoordinate());
          env.expandBy(RouteChoicePars.maxTripDistance);
          @SuppressWarnings("unchecked")
          List<MasonGeometry> candidates = (List<MasonGeometry>) zoneIndex.query(env);
          
          List<MasonGeometry> validZones = new ArrayList<>();
          double totalW = 0.0;
          Geometry pt = new GeometryFactory().createPoint(homeNode.getCoordinate());
          
          for(MasonGeometry z : candidates) {
             // Calculate distance to centroid, not to nearest edge of the polygon
             double d = z.getGeometry().getCentroid().distance(pt);
             
             // Euclidean distance is shorter than network distance, so relax the minimum constraint (e.g. 60%)
             if (d >= (RouteChoicePars.minTripDistance * 0.6) && d <= RouteChoicePars.maxTripDistance) {
                 validZones.add(z);
                 Double rw = zoneToWorkplaceWeight.get(z);
                 totalW += (rw != null ? rw : 0.0);
             }
          }
          
          if (!validZones.isEmpty() && totalW > 0) {
              double rw = rnd.nextDouble() * totalW;
              double cur = 0;
              for(MasonGeometry z : validZones) {
                  Double w = zoneToWorkplaceWeight.get(z);
                  cur += (w != null ? w : 0.0);
                  if (rw <= cur) {
                      List<NodeGraph> znodes = zoneToNodesMap.get(z);
                      if (znodes != null && !znodes.isEmpty()) {
                          workNode = znodes.get(rnd.nextInt(znodes.size()));
                          spatialJumpSuccessCount.incrementAndGet();
                      }
                      break;
                  }
              }
          }
          // Spatial jump is fast, but if it fails for this homeNode, do not loop 20 times!
          // Break immediately to use the fast, random fallback node at the bottom.
          break;
      }
      
      try {
        workNode = NodesLookup.randomNodeBetweenDistanceIntervalDMA(
            SharedCognitiveMap.getCommunityPrimalNetwork(), homeNode, RouteChoicePars.minTripDistance,
            RouteChoicePars.maxTripDistance, "work");
      } catch (Exception e) {
        workNode = null;
        break;
      }
      
      // If we are not using vulnerability zones and workNode is still null, 
      // the original logic would pick a new homeNode.
      if (workNode == null && !useVulnerabilityZones) {
          try {
            homeNode = NodesLookup.randomNodeDMA(SharedCognitiveMap.getCommunityPrimalNetwork(), "live");
          } catch (Exception e) {}
      }
      count++;
    }

    // Final fallback for workNode
    if (workNode == null) {
      try {
        workNode = NodesLookup.randomNodeBetweenDistanceInterval(
            SharedCognitiveMap.getCommunityPrimalNetwork(), homeNode, RouteChoicePars.minTripDistance,
            RouteChoicePars.maxTripDistance);
      } catch (Exception e) {
        workNode = null;
      }
    }
    if (workNode == null) {
      java.util.List<NodeGraph> allNodes = SharedCognitiveMap.getCommunityPrimalNetwork().getNodes();
      if (!allNodes.isEmpty()) {
          workNode = allNodes.get(rnd.nextInt(allNodes.size()));
          randomFallbackCount.incrementAndGet();
      }
    }

    agent.setHomeWorkLoctations(homeNode, workNode);
  }
}
