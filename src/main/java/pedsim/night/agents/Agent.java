package pedsim.night.agents;

import java.util.List;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.utilities.StringEnum;
import pedsim.core.utilities.StringEnum.Vulnerable;
import pedsim.night.engine.PedSimCityNight;
import sim.engine.SimState;
import sim.engine.Steppable;
import sim.graph.Graph;
import sim.graph.NodeGraph;
import sim.graph.NodesLookup;

/**
 * This class represents an agent in the pedestrian simulation. Agents move along paths between
 * origin and destination nodes.
 */
public class Agent extends pedsim.core.agents.Agent implements Steppable {

  private static final long serialVersionUID = 1L;
  public StringEnum.Vulnerable vulnerable;
  private Graph agentNetwork;
  protected pedsim.night.agents.AgentMovement nightMovement;
  protected PedSimCityNight state;
  private boolean nightRoute = false;

  /**
   * Constructor Function. Creates a new agent with the specified agent properties.
   *
   * @param state the PedSimCity simulation state.
   */
  public Agent(PedSimCityNight state) {

    super();
    this.agentNetwork = SharedCognitiveMap.getCommunityPrimalNetwork();
  }

  /**
   * This is called every tick by the scheduler. It moves the agent along the path.
   *
   * @param state the simulation state.
   */
  @Override
  public void step(SimState state) {

    if (isWaiting())
      return;

    if (isWalkingAlone() && destinationNode == null) {
      // simple cognitive map in this sim
      if (!cognitiveMap.formed) {
        getCognitiveMap().buildSimpleActivityBone();
        cognitiveMap.formed = true;
      }
      if (this.state.isDark) {
        planNightTrip();
        nightRoute = true;
      } else
        planTrip();
    } else if (reachedDestination.get()) {
      nightRoute = false;
      handleReachedDestination();
    } else if (isAtDestination() && timeAtDestination <= state.schedule.getSteps())
      goHome();
    else if (isAtDestination())
      ;
    else if (nightRoute)
      nightMovement.keepWalking();
    else
      agentMovement.keepWalking();
  }

  private synchronized void planNightTrip() {
    defineOrigin();
    if (isGoingHome()) {
      destinationNode = homeNode;
    } else {
      defineRandomDestination();
    }
    // safety check
    if (destinationNode.getID() == originNode.getID()) {
      reachedDestination.set(true);
      return;
    }
    planRoute();
    nightMovement = new AgentMovement(this);
    nightMovement.initialisePath(getRoute());
  }

  /**
   * Randomly selects a destination node within a specified distance range.
   */
  private void defineRandomDestination() {

    // Initialise limits for distance calculation
    double lowerLimit = distanceNextDestination * 0.90;
    double upperLimit = distanceNextDestination * 1.10;

    // Get the network graph from the cognitive map

    // Loop until a valid destination is found
    while (destinationNode == null) {

      // Get candidate nodes between the current distance range
      List<NodeGraph> destinationCandidates = NodesLookup
          .getNodesBetweenDistanceInterval(agentNetwork, originNode, lowerLimit, upperLimit);

      if (destinationCandidates.isEmpty()) {
        // Skip this iteration and adjust the limits if no candidates found
        lowerLimit *= 0.90;
        upperLimit *= 1.10;
        continue; // Continue with the next loop iteration
      }

      // Select a random destination node from the list of candidates
      destinationNode = NodesLookup.randomNodeFromList(destinationCandidates);

      // If it's dark, filter out destination nodes that lie in parks or along rivers
      if (state.isDark && destinationNode.getEdges().stream()
          .anyMatch(SharedCognitiveMap.getEdgesWithinParksOrAlongWater()::contains)) {
        destinationNode = null; // Set destination to null and try again

        // Adjust the distance range for the next iteration
        lowerLimit *= 0.90;
        upperLimit *= 1.10;
      }
    }
  }

  /**
   * Checks if the agent is vulnerable.
   *
   * @return true if the agent is vulnerable, false otherwise.
   */
  public boolean isVulnerable() {
    return vulnerable.equals(Vulnerable.VULNERABLE);
  }

  /**
   * Gets the simulation state of the agent.
   *
   * @return The PedSimCity simulation state.
   */
  @Override
  public PedSimCityNight getState() {
    return state;
  }
}
