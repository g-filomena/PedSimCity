package pedsim.cityimage.agents;

import java.util.logging.Logger;
import org.locationtech.jts.geom.Coordinate;
import pedsim.cityimage.engine.PedSimCityImage;
import pedsim.cityimage.parameters.TestPars;
import pedsim.core.agents.AgentMovement;
import pedsim.core.routing.RoutePlanner;
import pedsim.core.utilities.LoggerUtil;
import sim.engine.SimState;
import sim.graph.NodeGraph;

/**
 * This class represents an agent in the pedestrian simulation. Agents move along paths between
 * origin and destination nodes.
 */
public final class Agent extends pedsim.core.agents.Agent {

  private static final long serialVersionUID = 1L;
  protected static final Logger logger = LoggerUtil.getLogger();
  protected AgentProperties agentProperties;


  /**
   * Constructor Function. Creates a new agent with the specified agent properties.
   *
   * @param state the PedSimCity simulation state.
   */
  public Agent(PedSimCityImage state) {

    this.state = state;
    agentMovement = new AgentMovement(this);
    placeAgent();

    if (!OD.isEmpty()) {
      originNode = (NodeGraph) OD.get(getTripsDone()).getValue(0);
      Coordinate startCoord = null;
      startCoord = originNode.getCoordinate();
      updateAgentPosition(startCoord);
    }

  }

  /**
   * Performs agent's stepping action in the simulation.
   *
   * @param state The simulation state.
   */
  @Override
  public void step(SimState state) {

    if (reachedDestination.get() == true || destinationNode == null)
      try {
        handleReachedDestination();
      } catch (Exception e) {
        e.printStackTrace();
      }
    else
      agentMovement.keepWalking();
  }

  /**
   * Handles the agent's behaviour when reaching its destination.
   */
  @Override
  protected void handleReachedDestination() {

    reachedDestination.set(false);
    if (getTripsDone() == OD.size()) {
      removeAgent();
      return;
    }
    selectNodesFromOD();
    updateAgentPosition(originNode.getCoordinate());
    planRoute();
    agentMovement.initialisePath(route);
    return;
  }

  /**
   * Selects origin and destination nodes for the agent.
   */
  private void selectNodesFromOD() {
    originNode = (NodeGraph) OD.get(getTripsDone()).getValue(0);
    destinationNode = (NodeGraph) OD.get(getTripsDone()).getValue(1);
  }

  // /**
  // * Updates data related to the volumes on the segments traversed.
  // */
  // public void updateData() {
  // tripsDone += 1;
  // state.flowHandler.updateFlowsData(this, route);
  // }

  /**
   * Plans the route for the agent.
   */
  protected void planRoute() {

    if (TestPars.verboseMode) {
      if (agentProperties.routeChoice != null) {
        logger.info(String.format("Agent %s", agentProperties.routeChoice));
      }

      logger.info(String.format(" - origin %s destination %s", originNode.getID(),
          destinationNode.getID()));
      RoutePlanner planner = new RoutePlanner(originNode, destinationNode, this);
      route = planner.definePath();
    }
  }


}
