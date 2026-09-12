package pedsim.core.engine;

import ec.util.MersenneTwisterFast;
import java.util.List;
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
 * <p>Core populate provides only the data-agnostic home/work assignment (DMA → uniform-random
 * fallback). Modules override the assignment seams to draw from their own data.
 */
public class Populate {

  protected PedSimCity state;
  protected static final Logger logger = LoggerUtil.getLogger();
  protected final GeometryFactory GEOMETRY_FACTORY = new GeometryFactory();
  /**
   * Populate-time generator. Seeded from the model's seed by {@link #seedFrom}, because home and
   * work assignment decides where every agent lives and therefore how far it walks: left on the
   * clock, as it was, no run could be replayed however carefully the agents themselves were seeded.
   */
  protected MersenneTwisterFast random = new MersenneTwisterFast();

  /** Seeds the populate-time generator from the model seed. Call before drawing anything. */
  protected void seedFrom(PedSimCity state) {
    random = new MersenneTwisterFast(state.seed() * 104729L);
  }

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
    seedFrom(state);

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
            GEOMETRY_FACTORY.createPoint(agent.homeNode.getCoordinate());
      }

      state.agents.addGeometry(agent.getLocation());
      agent.updateAgentLists(false, true); // adds to agentsList + agentsAtHome
    }

    logger.info(
        "Agent Routing Stats -> Spatial Jump Successes: "
            + spatialJumpSuccessCount.get()
            + " | Instant MersenneTwisterFast Fallbacks: "
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
   * Data-driven modules override this to draw from residence-weighted zones first.
   */
  protected void assignHomeNode() {
    if (homeNode == null) homeNode = selectHomeNodeWithDMA();
    // When no DMA data is available, distribute uniformly across all network nodes.
    if (homeNode == null) homeNode = selectRandomNode();
  }

  /**
   * Assigns a work node. Core: DMA selection, then distance-interval fallback, then uniform random.
   * Data-driven modules override this to draw from workplace-weighted nodes first.
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
    if (PedSimCity.buildings.isEmpty()) {
      return null;
    }
    // No try/catch: the lookup returns null on an empty candidate set, it does not throw.
    return NodesLookup.randomNodeDMA(
        SharedCognitiveMap.getCommunityPrimalNetwork(), "live", random);
  }

  /**
   * A workplace, chosen among the work-tagged nodes of the whole city.
   *
   * <p>There is deliberately no distance interval. It used to be placed inside
   * {@code [minTripDistance, maxTripDistance]} - the range meant for discretionary walking trips -
   * which capped every commute in the model at 2,700 m. ISTAT 2017 puts 50.6% of Piedmont's
   * commuters outside their own municipality, so the cap did not bound a detail: it deleted half
   * the phenomenon, and left a model in which the only possible commute was a walkable one.
   *
   * <p>What stops a five-kilometre commute from being walked is no longer where the workplace is,
   * but {@link pedsim.activity.agents.ActivityAgent#walksToWork} deciding it is not walked. The
   * distance became a reason, instead of being prevented.
   *
   * <p>What is still missing is distance decay: workplaces are drawn with equal weight wherever
   * they are, so commutes come out longer than they should. The proper form is an attraction term
   * against an impedance term, which is the destination-choice model this is a step towards.
   */
  protected NodeGraph selectWorkNodeWithDMA(boolean keepHomeNode) {
    // DMA attributes are only assigned when the landmarks/buildings layer is loaded.
    // If it's empty, every node has dma="" and the lookup has nothing to filter on.
    if (PedSimCity.buildings.isEmpty()) {
      return null;
    }
    return NodesLookup.randomNodeDMA(
        SharedCognitiveMap.getCommunityPrimalNetwork(), "work", random);
  }

  /** Any node at all, for cities whose data carries no work tags. */
  protected NodeGraph selectWorkNodeWithDistanceFallback(NodeGraph homeNode) {
    return NodesLookup.randomNode(SharedCognitiveMap.getCommunityPrimalNetwork(), random);
  }

  protected NodeGraph selectRandomNode() {
    List<NodeGraph> nodes = SharedCognitiveMap.getCommunityPrimalNetwork().getNodes();
    return nodes.get(random.nextInt(nodes.size()));
  }
}
