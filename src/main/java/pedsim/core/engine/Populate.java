package pedsim.core.engine;

import ec.util.MersenneTwisterFast;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;
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

  /**
   * Agents whose workplace no dataset could place, and which therefore sit on a node drawn
   * uniformly from the whole city.
   *
   * <p>It sat beside a {@code spatialJumpSuccessCount} that nothing ever incremented, and both
   * were logged together as "Agent Routing Stats" - a line reporting two numbers, one of which was
   * structurally zero and the other unreachable.
   */
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
        randomFallbackCount.get() + " workplaces drawn uniformly, no dataset having placed them");
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
    agent.setHomeAndWorkplace(homeNode, workNode);
  }

  /**
   * Assigns a home node: the best-informed way of placing one that yields a node, else a uniform
   * draw over the network.
   */
  protected void assignHomeNode() {
    homeNode = firstNodeFrom(residenceLadder());
    // When nothing better could place a home, distribute uniformly across all network nodes.
    if (homeNode == null) homeNode = selectRandomNode();
  }

  /**
   * Ordered ways of choosing where someone lives, best-informed first. Core knows one: the DMA
   * tags on the buildings layer. Modules prepend their own data and inherit what follows.
   */
  protected List<Supplier<NodeGraph>> residenceLadder() {
    return List.of(this::selectHomeNodeWithDMA);
  }

  /**
   * Assigns a work node: the best-informed way of placing one that yields a node, else a uniform
   * draw over the network.
   */
  protected void assignWorkNode() {
    if (homeNode == null) return;

    workNode = firstNodeFrom(workplaceLadder());
    if (workNode == null) {
      workNode = selectRandomNode();
      randomFallbackCount.incrementAndGet();
    }
  }

  /**
   * Ordered ways of choosing a workplace, best-informed first. Core knows one: the DMA tags on the
   * buildings layer. Modules prepend their own data and inherit what follows.
   *
   * <p>This is the seam, rather than {@link #assignWorkNode()}, because a module overriding the
   * assignment restated the whole ladder to add one rung to the front of it - and the two copies
   * then drifted. The uniform draw that ends the ladder, and the counter that records it, exist
   * once.
   */
  protected List<Supplier<NodeGraph>> workplaceLadder() {
    return List.of(this::selectWorkNodeWithDMA);
  }

  /** The first rung of a ladder that yields a node, or null when none of them does. */
  protected NodeGraph firstNodeFrom(List<Supplier<NodeGraph>> ladder) {
    for (Supplier<NodeGraph> rung : ladder) {
      NodeGraph node = rung.get();
      if (node != null) {
        return node;
      }
    }
    return null;
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
   * <p>There is deliberately no distance interval, and none should be added. Bounding the workplace
   * by the discretionary walking range would cap every commute in the model at that range, while
   * ISTAT 2017 puts 50.6% of Piedmont's commuters outside their own municipality - half the
   * phenomenon. What stops a five-kilometre commute from being walked is
   * {@link pedsim.activity.agents.ActivityAgent#walksToWork} deciding it is not: distance is a
   * reason, not a prevention.
   *
   * <p>What is still missing is distance decay: workplaces are drawn with equal weight wherever
   * they are, so commutes come out longer than they should. The proper form is an attraction term
   * against an impedance term, which is the destination-choice model this is a step towards.
   *
   */
  protected NodeGraph selectWorkNodeWithDMA() {
    // DMA attributes are only assigned when the landmarks/buildings layer is loaded.
    // If it's empty, every node has dma="" and the lookup has nothing to filter on.
    if (PedSimCity.buildings.isEmpty()) {
      return null;
    }
    return NodesLookup.randomNodeDMA(
        SharedCognitiveMap.getCommunityPrimalNetwork(), "work", random);
  }

  /**
   * Any node at all: the end of every ladder, for a city whose data cannot say more.
   *
   * <p>Every use is counted in {@link #randomFallbackCount} and reported at the end of the populate
   * pass, so a city whose data placed nobody says so. Any rung added above this one must be able to
   * fail, or this one stops being reached and the count stops meaning anything.
   */
  protected NodeGraph selectRandomNode() {
    List<NodeGraph> nodes = SharedCognitiveMap.getCommunityPrimalNetwork().getNodes();
    return nodes.get(random.nextInt(nodes.size()));
  }
}
