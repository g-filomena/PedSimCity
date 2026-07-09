package pedsim.transit;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import pedsim.activity.engine.PedSimCityActivity;
import pedsim.core.agents.Agent;
import pedsim.core.utilities.StringEnum.AgentStatus;
import sim.engine.SimState;

import sim.engine.Steppable;

/**
 * Represents a moving public transport vehicle (Metro train, Tram, or Bus) stepping along a
 * sequence of physical {@link TransitStop} stations.
 */
public class TransitVehicle implements Steppable {

  private static final long serialVersionUID = 1L;

  public final String vehicleId;
  public final String mode; // "METRO", "TRAM", or "BUS"
  public final String routeId; // e.g. "M1", "4", "16CD"
  public final List<TransitStop> stopSequence = new ArrayList<>();
  public int currentStopIndex = 0;
  public final int capacity;

  public final List<Agent> onboardPassengers = new ArrayList<>();

  public TransitVehicle(String vehicleId, String mode, String routeId, int capacity) {
    this.vehicleId = vehicleId;
    this.mode = mode;
    this.routeId = routeId;
    this.capacity = capacity;
  }

  /**
   * Advances the vehicle to the next stop along its sequence:
   * 1. Alights onboard passengers whose egress station is this stop.
   * 2. Boards waiting passengers on the platform up to vehicle capacity.
   * 3. Steps to the next station index.
   */
  @Override
  public void step(SimState state) {
    if (stopSequence.isEmpty()) return;

    TransitStop currentStop = stopSequence.get(currentStopIndex);

    // 1. Alight onboard passengers reaching their transit destination
    Iterator<Agent> it = onboardPassengers.iterator();
    while (it.hasNext()) {
      Agent agent = it.next();
      TransitStop destStop = PedSimCityActivity.agentTransitDestinations.get(agent);
      if (destStop != null && destStop.stopId.equals(currentStop.stopId)) {
        it.remove();
        PedSimCityActivity.agentTransitDestinations.remove(agent);
        
        // Re-inject agent onto the physical walkable street graph at this station
        if (currentStop.snappedNodeGraph != null) {
          agent.originNode = currentStop.snappedNodeGraph;
        }
        agent.reinitializeMovementPath();
        agent.setStatus(AgentStatus.WALKING_ALONE);
      }

    }

    // 2. Board waiting passengers on the platform
    Iterator<Agent> waitIt = currentStop.waitingPassengers.iterator();
    while (waitIt.hasNext() && onboardPassengers.size() < capacity) {
      Agent waitingAgent = waitIt.next();
      TransitStop destStop = PedSimCityActivity.agentTransitDestinations.get(waitingAgent);
      
      // Check if this vehicle can take the agent closer to their destination
      if (destStop != null && destStop.servesMode(this.mode)) {
        waitIt.remove();
        onboardPassengers.add(waitingAgent);
        
        // Update live ridership stats
        int count = PedSimCityActivity.tripsByMode.getOrDefault(this.mode, 0);
        PedSimCityActivity.tripsByMode.put(this.mode, count + 1);
      }
    }

    // 3. Move to next stop in schedule loop
    currentStopIndex = (currentStopIndex + 1) % stopSequence.size();
  }
}
