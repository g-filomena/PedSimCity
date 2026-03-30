package pedsim.core.engine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.logging.Logger;
import java.util.stream.IntStream;
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

  private List<MasonGeometry> zonesList = new ArrayList<>();
  private double[] cumulativeProbabilities;
  private Map<MasonGeometry, List<NodeGraph>> zoneToNodesMap = new HashMap<>();
  private double totalProbability = 0.0;

  /**
   * Populates agents, OD matrix, for the simulation. It creates a set of agents with the learner
   * status and updates their cognitive maps. The agents are then added to the simulation state.
   *
   * @param state The PedSimCity simulation state.
   */
  public void populate(PedSimCity state) {

    this.state = state;

    prepareVulnerabilityZones();

    // Create agents with parameter true
    int totalAgents = Pars.numAgents;
    logger.info("Creating " + totalAgents + " Agents. Building Their Cognitive Maps");
    IntStream.range(0, totalAgents).parallel().forEach(agentID -> {
      addAgent(agentID); // Must be thread-safe!
    });

    for (Agent agent : state.agentsList) {
      state.agents.addGeometry(agent.getLocation());
    }
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
      
      Object attr = zone.getAttribute("zone_residence_pct");
      double pct = 0.0;
      if (attr != null) {
          try {
              Object val = ((sim.util.geo.AttributeValue) attr).getValue();
              if (val != null) {
                  pct = Double.parseDouble(val.toString().replace(",", "."));
              }
          } catch (Exception e) {
              logger.warning("Failed to parse zone_residence_pct for zone: " + e.getMessage());
          }
      }
      totalProbability += pct;
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

    // Map all nodes to zones to speed up agent spawning
    GeometryFactory gf = new GeometryFactory();
    List<NodeGraph> allNodes = SharedCognitiveMap.getCommunityPrimalNetwork().getNodes();
    for (NodeGraph node : allNodes) {
      Point pt = gf.createPoint(node.getCoordinate());
      for (MasonGeometry zone : zonesList) {
        if (zone.getGeometry().contains(pt) || zone.getGeometry().distance(pt) < 1e-6) {
          zoneToNodesMap.get(zone).add(node);
          break; // Assign node to first matching zone
        }
      }
    }
  }

  /**
   * Adds a new agent to the simulation with a randomly assigned vulnerability status. The agent is
   * added to the list of agents and its cognitive map is Initialised.
   *
   * @param agentID The identifier of the agent to be added.
   */
  protected void addAgent(int agentID) {

    Agent agent = new Agent(this.state);
    agent.agentID = agentID;
    defineHomeWorkLocations(agent);
    state.agentsList.add(agent);
    agent.updateAgentLists(false, true);
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

    int count = 0;
    while (workNode == null && count < 20) {
      if (useVulnerabilityZones && count > 0) {
          // If we are using vulnerability zones, homeNode is fixed. If the first try fails,
          // don't search from the same homeNode again 100 times. Break and use fallback.
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

    // Fallback if no DMA found
    if (homeNode == null) {
      java.util.List<NodeGraph> allNodes = SharedCognitiveMap.getCommunityPrimalNetwork().getNodes();
      homeNode = allNodes.get(rnd.nextInt(allNodes.size()));
    }
    if (workNode == null) {
      try {
        workNode = NodesLookup.randomNodeBetweenDistanceInterval(
            SharedCognitiveMap.getCommunityPrimalNetwork(), homeNode, RouteChoicePars.minTripDistance,
            RouteChoicePars.maxTripDistance);
      } catch (Exception e) {
        workNode = null;
      }
    }
    // Final fallback
    if (workNode == null) {
      java.util.List<NodeGraph> allNodes = SharedCognitiveMap.getCommunityPrimalNetwork().getNodes();
      workNode = allNodes.get(rnd.nextInt(allNodes.size()));
    }

    agent.setHomeWorkLoctations(homeNode, workNode);
  }
}
