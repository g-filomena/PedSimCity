package pedsim.core.engine;

import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.logging.Logger;
import java.util.stream.Collectors;
import java.util.stream.IntStream;
import org.locationtech.jts.geom.GeometryFactory;
import pedsim.core.agents.Agent;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.parameters.Pars;
import pedsim.core.parameters.RouteChoicePars;
import pedsim.core.utilities.LoggerUtil;
import sim.graph.NodeGraph;
import sim.graph.NodesLookup;

/**
 * The Populate class is responsible for generating test agents, building the OD matrix, and
 * populating empirical groups for pedestrian simulation.
 *
 * <p>Core populate provides only the census-agnostic home/work assignment (DMA → uniform-random
 * fallback). Census-zone-based residence and workplace selection lives in
 * {@link pedsim.activity.engine.ActivityPopulate}, the base for activity-based modules.
 */
public class Populate {

  protected PedSimCity state;
  protected static final Logger logger = LoggerUtil.getLogger();
  protected final GeometryFactory GEOMETRY_FACTORY = new GeometryFactory();
  protected Random random = new Random();

  protected NodeGraph homeNode;
  protected NodeGraph workNode;

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
    int totalAgents = Pars.numAgents;
    logger.info("Creating " + totalAgents + " Agents. Building Their Cognitive Maps");
    List<Agent> newAgents =
        IntStream.range(0, totalAgents).mapToObj(this::createAgent).collect(Collectors.toList());

    // Step 2: Register agents sequentially (Thread-safe state update)
    for (Agent agent : newAgents) {
      // Update agent position to its homeNode before adding to the layer
      if (agent.homeNode != null) {
        agent.currentLocation.geometry =
            new GeometryFactory().createPoint(agent.homeNode.getCoordinate());
      }

      state.agents.addGeometry(agent.getLocation());
      agent.updateAgentLists(false, true); // adds to agentsList + agentsAtHome
    }

    logger.info(
        "Agent Routing Stats -> Spatial Jump Successes: "
            + spatialJumpSuccessCount.get()
            + " | Instant Random Fallbacks: "
            + randomFallbackCount.get());
    logger.info(state.agentsList.size() + " agents created");
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
    return agent;
  }

  protected void defineHomeWorkLocations(Agent agent) {
    this.homeNode = null;
    this.workNode = null;

    assignHomeNode();
    assignWorkNode();
    agent.setHomeWorkLoctations(homeNode, workNode);
  }

  /**
   * Assigns a home node. Core: DMA selection where available, otherwise uniform random.
   * Census-based modules override this to draw from residence-weighted census zones first.
   */
  protected void assignHomeNode() {
    if (homeNode == null) homeNode = selectHomeNodeWithDMA();
    // When no DMA/census data is available, distribute uniformly across all network nodes.
    if (homeNode == null) homeNode = selectRandomNode();
  }

  /**
   * Assigns a work node. Core: DMA selection, then distance-interval fallback, then uniform random.
   * Census-based modules override this to draw from workplace-weighted census zones first.
   */
  protected void assignWorkNode() {
    if (homeNode == null) return;

    if (workNode == null) workNode = selectWorkNodeWithDMA(false);

    if (workNode == null) workNode = selectWorkNodeWithDistanceFallback(homeNode);

    if (workNode == null) {
      workNode = selectRandomNode();
      if (workNode != null) {
        randomFallbackCount.incrementAndGet();
      }
    }
  }

  protected NodeGraph selectHomeNodeWithDMA() {
    // DMA attributes are only assigned when the landmarks/buildings layer is loaded.
    // If it's empty, every node has dma="" and randomNodeDMA would spin forever.
    if (PedSimCity.buildings.getGeometries().isEmpty()) {
      return null;
    }
    try {
      return NodesLookup.randomNodeDMA(SharedCognitiveMap.getCommunityPrimalNetwork(), "live");
    } catch (Exception e) {
      return null;
    }
  }

  protected NodeGraph selectWorkNodeWithDMA(boolean keepHomeNode) {
    // DMA attributes are only assigned when the landmarks/buildings layer is loaded.
    // If it's empty, every node has dma="" and randomNodeBetweenDistanceIntervalDMA would spin
    // forever.
    if (PedSimCity.buildings.getGeometries().isEmpty()) {
      return null;
    }

    int attempts = 0;

    while (attempts < 20) {
      try {
        NodeGraph node =
            NodesLookup.randomNodeBetweenDistanceIntervalDMA(
                SharedCognitiveMap.getCommunityPrimalNetwork(),
                homeNode,
                RouteChoicePars.minTripDistance,
                RouteChoicePars.maxTripDistance,
                "work");

        if (node != null) {
          return node;
        }
      } catch (Exception e) {
        return null;
      }

      // In the non-zone workflow, the legacy logic retries with a different home node.
      if (!keepHomeNode) {
        NodeGraph replacementHome = selectHomeNodeWithDMA();
        if (replacementHome != null) {
          homeNode = replacementHome;
        }
      }

      attempts++;
    }
    return null;
  }

  protected NodeGraph selectWorkNodeWithDistanceFallback(NodeGraph homeNode) {
    try {
      return NodesLookup.randomNodeBetweenDistanceInterval(
          SharedCognitiveMap.getCommunityPrimalNetwork(),
          homeNode,
          RouteChoicePars.minTripDistance,
          RouteChoicePars.maxTripDistance);
    } catch (Exception e) {
      return null;
    }
  }

  protected NodeGraph selectRandomNode() {
    List<NodeGraph> nodes = SharedCognitiveMap.getCommunityPrimalNetwork().getNodes();
    return nodes.get(random.nextInt(nodes.size()));
  }
}
