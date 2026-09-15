package pedsim.cityimage.agents;

import static pedsim.core.utilities.StringEnum.LocalHeuristicMode.ANGULAR;
import static pedsim.core.utilities.StringEnum.LocalHeuristicMode.DISTANCE;

import java.util.List;
import java.util.logging.Logger;
import org.javatuples.Pair;
import pedsim.cityimage.engine.PedSimCityImage;
import pedsim.cityimage.parameters.TestPars;
import pedsim.cityimage.utilities.StringEnum.Scenario;
import pedsim.core.agents.AgentMovement;
import pedsim.core.agents.AgentProperties;
import pedsim.core.agents.OdAgent;
import pedsim.core.agents.RouteChoiceModel;
import pedsim.core.routing.RoutePlanner;
import pedsim.core.utilities.LoggerUtil;
import sim.graph.NodeGraph;

public final class CityImageAgent extends OdAgent {

  private static final long serialVersionUID = 1L;

  private static final Logger LOGGER = LoggerUtil.getLogger();

  private final Scenario scenario;

  public CityImageAgent(
      PedSimCityImage state, Scenario scenario, List<Pair<NodeGraph, NodeGraph>> odPairs) {
    super(state);
    this.scenario = scenario;
    this.agentProperties = new AgentProperties();
    this.agentMovement = new AgentMovement(this);
    setOD(odPairs);
  }

  @Override
  protected void planRoute() {
    if (TestPars.verboseMode) {
      LOGGER.info(
          String.format(
              "CityImage agent %d | model=%s | trip=%d | origin=%s | destination=%s",
              agentID, scenario, getTripsDone(), originNode, destinationNode));
    }

    initialiseHeuristics(false);
    RoutePlanner planner = new RoutePlanner(originNode, destinationNode, this);
    initialiseRoute(planner.definePath());
  }

  /** The scenario is the experiment: this agent walks its model and nothing re-decides it. */
  @Override
  protected RouteChoiceModel assignedRouteChoice() {
    return modelFor(scenario);
  }

  /**
   * What each scenario means.
   *
   * <p>Exhaustive and without a {@code default}, so adding a {@link Scenario} constant stops
   * compiling until it is given a model here. Reading the meaning off the constant's name instead
   * would let two scenarios resolve to one configuration unnoticed.
   *
   * @param scenario the scenario this agent was built for
   * @return the route-choice model it stands for
   */
  static RouteChoiceModel modelFor(Scenario scenario) {
    return switch (scenario) {
      case ROAD_DISTANCE -> RouteChoiceModel.minimisingDistance();
      case ANGULAR_CHANGE -> RouteChoiceModel.minimisingAngular();

      case LANDMARKS_DISTANCE -> RouteChoiceModel.localAndDistantLandmarks(DISTANCE);
      case LANDMARKS_ANGULAR -> RouteChoiceModel.localAndDistantLandmarks(ANGULAR);

      case LOCAL_LANDMARKS_DISTANCE -> RouteChoiceModel.localLandmarks(DISTANCE);
      case LOCAL_LANDMARKS_ANGULAR -> RouteChoiceModel.localLandmarks(ANGULAR);

      case DISTANT_LANDMARKS_DISTANCE -> RouteChoiceModel.distantLandmarks(DISTANCE);
      case DISTANT_LANDMARKS_ANGULAR -> RouteChoiceModel.distantLandmarks(ANGULAR);
      case DISTANT_LANDMARKS -> RouteChoiceModel.distantLandmarksAlone();

      case REGION_DISTANCE -> RouteChoiceModel.regions(DISTANCE);
      case REGION_ANGULAR -> RouteChoiceModel.regions(ANGULAR);

      case BARRIER_DISTANCE -> RouteChoiceModel.barriers(DISTANCE);
      case BARRIER_ANGULAR -> RouteChoiceModel.barriers(ANGULAR);

      case REGION_BARRIER_DISTANCE -> RouteChoiceModel.regionsAndBarriers(DISTANCE);
      case REGION_BARRIER_ANGULAR -> RouteChoiceModel.regionsAndBarriers(ANGULAR);
    };
  }

  @Override
  public Enum<?> getAgentScenario() {
    return scenario;
  }
}
