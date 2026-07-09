package pedsim.activity.agents;

import java.util.Map;
import pedsim.activity.engine.PedSimCityActivity;
import pedsim.core.agents.Agent;
import pedsim.core.engine.PedSimCity;
import pedsim.core.parameters.TimePars;
import pedsim.core.utilities.StringEnum;
import pedsim.core.utilities.StringEnum.AgentStatus;
import pedsim.transit.TransitStop;
import sim.engine.SimState;
import sim.graph.NodeGraph;

/**
 * Pedestrian agent for the activity-based model. Behaves like a core {@link Agent} but
 * follows a 24h activity pattern: it weights daytime destinations by workplace POI density and
 * evening destinations (when {@code isDark}) by night POI density. Also integrates multi-modal
 * transit routing (METRO, TRAM, BUS) for long-distance city trips (> 800m).
 */
public class ActivityAgent extends Agent {

  private static final long serialVersionUID = 1L;

  // Multi-modal transit tracking fields
  public NodeGraph ultimateDestinationNode;
  public TransitStop boardingStop;
  public TransitStop egressStop;

  public ActivityAgent(PedSimCity state, boolean registerSpatial) {
    super(state, registerSpatial);
  }

  @Override
  public void step(SimState state) {
    if (isWaiting()) {
      return;
    }

    // Check if agent arrived at the transit boarding platform (Leg 1 completed)
    if (boardingStop != null && status == AgentStatus.WALKING_ALONE) {
      boolean atPlatform = reachedDestination.get() 
          || (destinationNode != null && destinationNode.getID() == boardingStop.snappedNodeId)
          || (currentLocation != null && currentLocation.getGeometry().getCoordinate().distance(boardingStop.snappedNodeGraph.getCoordinate()) < 20.0);

      if (atPlatform) {
        reachedDestination.set(false);
        destinationNode = ultimateDestinationNode;
        TransitStop platformStop = boardingStop;
        boardingStop = null;
        
        setStatus(AgentStatus.WAITING);
        platformStop.waitingPassengers.add(this);
        return;
      }
    }

    super.step(state);
  }

  @Override
  protected synchronized void planTrip() {
    super.planTrip();

    if (reachedDestination.get() || destinationNode == null || originNode == null) {
      return;
    }

    // Evaluate multi-modal transit for trips longer than ~800 meters when transit network is active
    if (pedsim.core.parameters.RouteChoicePars.usePublicTransport && !PedSimCityActivity.allTransitStops.isEmpty() && boardingStop == null && egressStop == null) {
      double tripDist = originNode.getCoordinate().distance(destinationNode.getCoordinate());
      if (tripDist > 800.0) {
        TransitStop bStop = findNearestStop(originNode, 600.0);
        TransitStop eStop = findNearestStop(destinationNode, 600.0);

        if (bStop != null && eStop != null && bStop != eStop 
            && bStop.snappedNodeGraph != null && eStop.snappedNodeGraph != null) {
          
          double transitDist = bStop.snappedNodeGraph.getCoordinate().distance(eStop.snappedNodeGraph.getCoordinate());
          if (transitDist > 400.0) {
            String sharedMode = null;
            double splitProbability = 0.0;

            if (bStop.servesMode("METRO") && eStop.servesMode("METRO")) {
              sharedMode = "METRO";
              splitProbability = 0.45; // 45% Metro capture rate along M1 corridor
            } else if (bStop.servesMode("TRAM") && eStop.servesMode("TRAM")) {
              sharedMode = "TRAM";
              splitProbability = 0.35; // 35% Tram capture rate along tram lines
            } else if (bStop.servesMode("BUS") && eStop.servesMode("BUS")) {
              sharedMode = "BUS";
              splitProbability = 0.30; // 30% Bus capture rate across urban bus network
            }

            if (sharedMode != null && state.random.nextDouble() < splitProbability) {
              // Agent chooses transit -> Execute Leg 1 (Walk to boarding station)
              ultimateDestinationNode = destinationNode;
              boardingStop = bStop;
              egressStop = eStop;
              destinationNode = bStop.snappedNodeGraph;
              PedSimCityActivity.agentTransitDestinations.put(this, eStop);
              if (originNode != null && destinationNode != null && originNode.getID() == destinationNode.getID()) {
                reachedDestination.set(true);
              } else {
                reinitializeMovementPath();
              }
              return;

            }
          }
        }
      }
    }

    int walkCount = PedSimCityActivity.tripsByMode.getOrDefault("WALK", 0);
    PedSimCityActivity.tripsByMode.put("WALK", walkCount + 1);
  }


  private TransitStop findNearestStop(NodeGraph node, double maxRadius) {
    TransitStop bestStop = null;
    double bestDist = maxRadius;
    for (TransitStop stop : PedSimCityActivity.allTransitStops) {
      if (stop.snappedNodeGraph != null) {
        double d = node.getCoordinate().distance(stop.snappedNodeGraph.getCoordinate());
        if (d < bestDist) {
          bestDist = d;
          bestStop = stop;
        }
      }
    }
    return bestStop;
  }

  /** Reads the activity 24h clock so destination selection and work-targeting follow time of day. */
  @Override
  protected boolean isDark() {
    return state instanceof PedSimCityActivity activityState && activityState.isDark;
  }

  /**
   * Activity volumes are tallied per hour of day (h01–h24); the agent type stays {@code DEFAULT}
   * (no vulnerable / learner split). The hour is read when a trip is recorded, so a trip counts
   * towards the hour it completes in.
   */
  @Override
  public Enum<?> getSimulationScenario() {
    return StringEnum.Hour.of(TimePars.getTime(state.schedule.getSteps()).toLocalTime().getHour());
  }

  /**
   * Weights candidate destinations by activity POI density: night POI when dark, workplace POI
   * during the day. Returns 0.0 (uniform) when the relevant dataset was not loaded.
   */
  @Override
  protected double getPOIWeight(NodeGraph node, boolean isDark) {
    Map<NodeGraph, Double> weightMap =
        isDark ? PedSimCityActivity.nodesNightWeight : PedSimCityActivity.nodesWorkplaceWeight;
    if (weightMap.isEmpty()) {
      return 0.0;
    }
    return weightMap.getOrDefault(node, 0.0);
  }
}
